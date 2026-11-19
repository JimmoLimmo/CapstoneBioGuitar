"""Camera hand tracking shared by the mimic scripts and the sim.

Turns MediaPipe's 21 hand landmarks into finger curls (0 = straight, 1 = curled)
for the left hand (one servo per finger) and the right hand (base + tip per
finger). No pybullet and no hardware here, so it also runs on a laptop.
"""

import math
import os
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(REPO_ROOT, "MediaPipe", "hand_landmarker.task")

SIDES = ("left", "right")


# Each servo follows the bend of the joints listed, added up. A joint (a, b, c)
# is three MediaPipe landmarks and its bend is how far b is from straight.
# straight_deg / curled_deg = that sum with the finger straight / fully curled.
# These are guesses; hand_mimic.py --calibrate measures real ones on your hand
# and saves them to Hardware/calibration.json, which apply_ranges() puts in here.

def _whole(mcp, pip, dip, tip):
    return {"joints": ((0, mcp, pip), (mcp, pip, dip), (pip, dip, tip)), "straight_deg": 20.0, "curled_deg": 200.0}


LEFT_TRACKING = {
    "left_index": _whole(5, 6, 7, 8),
    "left_middle": _whole(9, 10, 11, 12),
    "left_ring": _whole(13, 14, 15, 16),
    "left_pinky": _whole(17, 18, 19, 20),
}


def _split(mcp, pip, dip, tip):
    return {"base": {"joints": ((0, mcp, pip),), "straight_deg": 15.0, "curled_deg": 80.0},
            "tip": {"joints": ((mcp, pip, dip), (pip, dip, tip)), "straight_deg": 10.0, "curled_deg": 150.0}}


_RIGHT_BY_FINGER = {
    "thumb": {"base": {"joints": ((1, 2, 3),), "straight_deg": 10.0, "curled_deg": 50.0},
              "tip": {"joints": ((2, 3, 4),), "straight_deg": 10.0, "curled_deg": 70.0}},
    "index": _split(5, 6, 7, 8),
    "middle": _split(9, 10, 11, 12),
    "ring": _split(13, 14, 15, 16),
    "pinky": _split(17, 18, 19, 20),
}
RIGHT_TRACKING = {f"{finger}_{segment}": t
                  for finger, segments in _RIGHT_BY_FINGER.items() for segment, t in segments.items()}

TRACKING = {"left": LEFT_TRACKING, "right": RIGHT_TRACKING}

MAX_CURL_PER_S = 4.0   # fastest a finger is told to move: full travel in 0.25 s
FOLLOW_TAU_S = 0.08    # servos ease toward the latest camera reading with this time constant
DEADBAND = 0.03        # a reading that moves less than this is camera noise and is ignored
MIN_SPAN_DEG = 10.0    # a measured straight-to-curled range smaller than this is rejected

# Wrist from the camera: how far the right hand is tilted from the neutral pose saved by --calibrate.
WRIST_DEV_SIGN = 1.0       # flip to -1.0 if tilting your hand toward the pinky moves the robot toward the thumb
WRIST_FLEX_SIGN = 1.0      # flip to -1.0 if bending your hand back moves the robot toward the palm
WRIST_SCALE_DEG = 45.0     # degrees treated as one unit by the curl filter and deadband
WRIST_MAX_DEG_PER_S = 120.0

# Smoothing (a One Euro filter on each curl). Lower MIN_CUTOFF_HZ = steadier when
# the hand is still; higher BETA = less lag when it moves fast.
MIN_CUTOFF_HZ = 0.5
BETA = 1.0
SPEED_CUTOFF_HZ = 1.0
RESET_AFTER_S = 0.5    # a hand gone this long starts fresh instead of sliding from the old value


# Degrees the joint at b is bent away from straight, in 3-D; None if two landmarks coincide.
def bend(points, a, b, c):
    u = (points[b].x - points[a].x, points[b].y - points[a].y, points[b].z - points[a].z)
    v = (points[c].x - points[b].x, points[c].y - points[b].y, points[c].z - points[b].z)
    norm = math.sqrt(sum(x * x for x in u) * sum(x * x for x in v))
    if norm == 0:
        return None
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(u, v)) / norm))))


# ({servo: curl}, {servo: summed bend in degrees}) for one side, from 21 landmarks.
def curls(points, side):
    out, bends = {}, {}
    for name, t in TRACKING[side].items():
        parts = [bend(points, *joint) for joint in t["joints"]]
        if None in parts:
            continue
        bends[name] = sum(parts)
        span = t["curled_deg"] - t["straight_deg"]
        out[name] = max(0.0, min(1.0, (bends[name] - t["straight_deg"]) / span))
    return out, bends


# Move each curl toward its target by at most `limit`; servos with no target hold.
def step(pose, target, limit):
    return {n: v + max(-limit, min(limit, target.get(n, v) - v)) for n, v in pose.items()}


# Like step(), but eases in: covers most of the gap quickly and slows near the target,
# so updates arriving at the camera's frame rate turn into smooth motion.
def follow(pose, target, dt, max_rate=MAX_CURL_PER_S):
    k = 1.0 - math.exp(-dt / FOLLOW_TAU_S)
    limit = max_rate * dt
    return {n: v + max(-limit, min(limit, (target.get(n, v) - v) * k)) for n, v in pose.items()}


# New readings, except those within DEADBAND of the last kept value, which keep it.
def deadband(kept, new):
    return {n: (kept[n] if n in kept and abs(x - kept[n]) <= DEADBAND else x) for n, x in new.items()}


# Put measured ranges in place of the guesses:
# saved = {"right": {servo: {"straight_deg": a, "curled_deg": b}}, "left": {...}}.
# Returns the servos that were changed.
def apply_ranges(saved):
    changed = []
    for side, servos in (saved or {}).items():
        table = TRACKING.get(side)
        if table is None or not isinstance(servos, dict):
            continue
        for name, r in servos.items():
            if name not in table or not isinstance(r, dict):
                continue
            a, b = r.get("straight_deg"), r.get("curled_deg")
            if isinstance(a, (int, float)) and isinstance(b, (int, float)) and b - a >= MIN_SPAN_DEG:
                table[name]["straight_deg"], table[name]["curled_deg"] = float(a), float(b)
                changed.append(name)
    return changed


def _vec(p):
    return (p.x, p.y, p.z)


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = math.sqrt(_dot(a, a))
    return None if n == 0 else (a[0] / n, a[1] / n, a[2] / n)


# (up, across) unit vectors: wrist to middle knuckle, and index knuckle to pinky knuckle. None if unclear.
def hand_axes(points):
    up = _unit(_sub(_vec(points[9]), _vec(points[0])))
    across = _unit(_sub(_vec(points[17]), _vec(points[5])))
    return None if up is None or across is None else (up, across)


# (deviation, flexion) in degrees for a RIGHT hand, measured from neutral = (up, across) saved by --calibrate.
# +deviation = tilted toward the pinky, +flexion = bent toward the back of the hand. None if unclear.
def wrist_angles(points, neutral):
    axes = hand_axes(points)
    up0 = _unit(tuple(neutral[0]))
    if axes is None or up0 is None:
        return None
    a = tuple(neutral[1])
    across0 = _unit(_sub(a, tuple(x * _dot(a, up0) for x in up0)))
    if across0 is None:
        return None
    back0 = _cross(across0, up0)
    up = axes[0]
    dev = math.degrees(math.atan2(_dot(up, across0), _dot(up, up0)))
    flex = math.degrees(math.atan2(_dot(up, back0), _dot(up, up0)))
    return WRIST_DEV_SIGN * dev, WRIST_FLEX_SIGN * flex


def _alpha(dt, cutoff_hz):
    return 1.0 / (1.0 + 1.0 / (2 * math.pi * cutoff_hz * dt))


# Takes the camera jitter out of the curls without adding much lag to real moves.
class CurlFilter:

    def __init__(self):
        self.value = {}
        self.speed = {}
        self.last = None

    def __call__(self, raw, now):
        if self.last is None or now - self.last > RESET_AFTER_S:
            self.value, self.speed = dict(raw), {n: 0.0 for n in raw}
            self.last = now
            return dict(raw)
        dt = max(now - self.last, 1e-3)
        self.last = now
        for name, x in raw.items():
            if name not in self.value:
                self.value[name], self.speed[name] = x, 0.0
                continue
            speed = (x - self.value[name]) / dt
            self.speed[name] += _alpha(dt, SPEED_CUTOFF_HZ) * (speed - self.speed[name])
            cutoff = MIN_CUTOFF_HZ + BETA * abs(self.speed[name])
            self.value[name] += _alpha(dt, cutoff) * (x - self.value[name])
        return {n: self.value[n] for n in raw}


# One hand in a frame: label is "left"/"right" (or None), image = landmarks in the
# picture (for drawing), world = the same in metres around the hand (for bends).
class Seen:

    def __init__(self, label, image, world):
        self.label = label
        self.image = image
        self.world = world


# The webcam and MediaPipe's hand tracker, in video mode so it follows the hand
# from frame to frame instead of finding it again in every picture.
class Camera:

    def __init__(self, index, hands=1, swap=False):
        try:
            import cv2
            import mediapipe as mp
            from mediapipe.tasks.python import BaseOptions
            from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
        except ImportError as e:
            raise SystemExit(f"[camera] {e}. The camera part needs opencv-python and mediapipe.")
        if not os.path.isfile(MODEL_PATH):
            raise SystemExit(f"[camera] {MODEL_PATH} is missing.")
        self.cv2, self.mp = cv2, mp
        self.swap = swap
        self.stamp = 0
        self.capture = cv2.VideoCapture(index)
        if not self.capture.isOpened():
            raise SystemExit(f"[camera] can't open camera {index}. Check: ls /dev/video*")
        self.options = (BaseOptions, HandLandmarker, HandLandmarkerOptions, RunningMode)
        self.landmarker = None
        self.hands = None
        self.want_hands = hands   # set this to change how many hands it looks for; applied on the next read
        self._tracker()

    def _tracker(self):
        BaseOptions, HandLandmarker, HandLandmarkerOptions, RunningMode = self.options
        if self.landmarker is not None:
            self.landmarker.close()
        self.hands = self.want_hands
        self.landmarker = HandLandmarker.create_from_options(HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=RunningMode.VIDEO,
            num_hands=self.hands,
        ))

    # (frame, [Seen for each hand in view]); frame is None if the camera gave nothing.
    def read(self):
        if self.want_hands != self.hands:
            self._tracker()
        ok, frame = self.capture.read()
        if not ok:
            return None, []
        rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        self.stamp = max(self.stamp + 1, int(time.perf_counter() * 1000))   # video mode needs rising timestamps
        result = self.landmarker.detect_for_video(self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb),
                                                  self.stamp)
        seen = []
        for i, image in enumerate(result.hand_landmarks):
            label = None
            if i < len(result.handedness) and result.handedness[i]:
                label = result.handedness[i][0].category_name.lower()
                if self.swap and label in SIDES:
                    label = "left" if label == "right" else "right"
            world = result.hand_world_landmarks[i] if i < len(result.hand_world_landmarks) else image
            seen.append(Seen(label, image, world))
        return frame, seen

    # Draw the preview. False once q is pressed.
    def show(self, frame, seen, lines, title="hand mimic"):
        cv2 = self.cv2
        height, width = frame.shape[:2]
        for hand in seen:
            colour = (255, 120, 0) if hand.label == "left" else (0, 0, 255)
            for p in hand.image:
                cv2.circle(frame, (int(p.x * width), int(p.y * height)), 5, colour, -1)
            wrist = hand.image[0]
            cv2.putText(frame, hand.label or "?", (int(wrist.x * width), int(wrist.y * height) + 30),
                        cv2.FONT_HERSHEY_PLAIN, 2, colour, 2)
        for row, line in enumerate(lines):
            cv2.putText(frame, line, (10, 30 + 25 * row), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)
        cv2.imshow(title, cv2.resize(frame, (960, 540)))
        return cv2.waitKey(1) & 0xFF != ord("q")

    def hide(self, title="hand mimic"):
        try:
            self.cv2.destroyWindow(title)
            self.cv2.waitKey(1)
        except self.cv2.error:
            pass

    def preview_works(self, title="hand mimic"):
        frame, seen = self.read()
        if frame is None:
            return False
        try:
            self.show(frame, seen, ["starting..."], title)
        except self.cv2.error:
            return False
        return True

    def close(self):
        self.capture.release()
        self.landmarker.close()
        try:
            self.cv2.destroyAllWindows()
        except self.cv2.error:
            pass
