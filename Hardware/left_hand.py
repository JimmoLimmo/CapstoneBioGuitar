"""Left (fretting) hand: its finger servos follow a hand seen by the camera.

    python3 run.py mimic --hand left                  # from the repo root (hand_mimic.py, left hand only)
    sudo python3 Hardware/left_hand.py                # the same, on its own
    python3 Hardware/left_hand.py --dry-run --show    # camera only: shows the tracking, moves nothing

This is last year's Imitation/mimic.py moved onto the right-hand box. The servos
sit on the 5 V board ("lv", 0x41), next to the wrist servos (ch 10-13), and go through power_box.py like every other
script: Enter to arm, the E-stop stops it, signals off on every exit.

Calibrate a servo with:  sudo python3 Hardware/calibrate_servo.py left_thumb
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO_ROOT, "Sim"))
sys.path.insert(0, HERE)

import calibration  # noqa: E402
import fingers as fh  # noqa: E402
import servo_output as so  # noqa: E402
from power_box import EStop, PowerBox  # noqa: E402
from strum_motion import smooth  # noqa: E402

MODEL_PATH = os.path.join(REPO_ROOT, "MediaPipe", "hand_landmarker.task")

# One servo per finger: open_pulse = finger straight, closed_pulse = curled.
# The four fingers keep last year's pulses (never above 7000).
# The thumb is new: it stays still until it has both pulses.
LEFT_SERVOS = {
    "left_thumb":  {"board": "lv", "channel": 0,  "open_pulse": None, "closed_pulse": None},
    "left_index":  {"board": "lv", "channel": 1,  "open_pulse": 1638, "closed_pulse": 7000},
    "left_middle": {"board": "lv", "channel": 2,  "open_pulse": 1638, "closed_pulse": 7000},
    "left_ring":   {"board": "lv", "channel": 3,  "open_pulse": 1638, "closed_pulse": 7000},
    "left_pinky":  {"board": "lv", "channel": 4,  "open_pulse": 1638, "closed_pulse": 7000},
}
calibration.apply("left", LEFT_SERVOS)

# joints = MediaPipe landmarks (a, b, c); the bend is the angle at b.
# straight_deg / curled_deg = that angle with the finger straight / fully curled.
# The thumb's range is a guess: read the real angles off --dry-run --show.
TRACKING = {
    "left_thumb":  {"joints": (1, 2, 4),    "straight_deg": 160.0, "curled_deg": 110.0},
    "left_index":  {"joints": (5, 6, 8),    "straight_deg": 170.0, "curled_deg": 0.0},
    "left_middle": {"joints": (9, 10, 12),  "straight_deg": 170.0, "curled_deg": 0.0},
    "left_ring":   {"joints": (13, 14, 16), "straight_deg": 170.0, "curled_deg": 0.0},
    "left_pinky":  {"joints": (17, 18, 20), "straight_deg": 170.0, "curled_deg": 0.0},
}
assert set(TRACKING) == set(LEFT_SERVOS)

MAX_CURL_PER_S = 4.0   # fastest a finger is told to move: full travel in 0.25 s


# --- calibration -------------------------------------------------------------

def calibrated(name):
    c = LEFT_SERVOS[name]
    return None not in (c["channel"], c["open_pulse"], c["closed_pulse"])


def uncalibrated():
    return [n for n in LEFT_SERVOS if not calibrated(n)]


def problems():
    """Reasons not to move anything."""
    out = []
    if not any(calibrated(n) for n in LEFT_SERVOS):
        out.append("no left-hand servo has a channel and both pulses")
    for name, c in LEFT_SERVOS.items():
        if calibrated(name) and c["open_pulse"] == c["closed_pulse"]:
            out.append(f"{name}: open_pulse == closed_pulse")
    users = {}
    for table in (LEFT_SERVOS, fh.FINGER_SERVOS, so.WRIST_SERVOS):
        for name, c in table.items():
            if c["channel"] is not None:
                users.setdefault((so.BOARDS[c["board"]], c["channel"]), []).append(name)
    for (address, channel), names in sorted(users.items()):
        if len(names) > 1 and any(n in LEFT_SERVOS for n in names):
            out.append(f"{hex(address)} ch{channel} is assigned to {', '.join(names)}")
    return out


# --- camera angles -> pulses ---------------------------------------------------

def joint_angle(a, b, c):
    """Angle at b in degrees, measured in the image; None if two landmarks coincide."""
    ax, ay, cx, cy = a.x - b.x, a.y - b.y, c.x - b.x, c.y - b.y
    norm = math.hypot(ax, ay) * math.hypot(cx, cy)
    if norm == 0:
        return None
    return math.degrees(math.acos(max(-1.0, min(1.0, (ax * cx + ay * cy) / norm))))


def finger_angles(landmarks):
    """{servo: bend angle in degrees} from one hand's 21 MediaPipe landmarks."""
    out = {}
    for name, t in TRACKING.items():
        angle = joint_angle(*(landmarks[i] for i in t["joints"]))
        if angle is not None:
            out[name] = angle
    return out


def curl_of(name, angle):
    t = TRACKING[name]
    return max(0.0, min(1.0, (t["straight_deg"] - angle) / (t["straight_deg"] - t["curled_deg"])))


def step(pose, target, limit):
    """Move each curl toward its target by at most `limit`; fingers with no target hold."""
    return {n: v + max(-limit, min(limit, target.get(n, v) - v)) for n, v in pose.items()}


def pulses(pose):
    out = {}
    for name, curl in pose.items():
        if calibrated(name):
            c = LEFT_SERVOS[name]
            pulse = c["open_pulse"] + (c["closed_pulse"] - c["open_pulse"]) * max(0.0, min(1.0, curl))
            out[name] = int(round(so.clamp(pulse, so.PULSE_ABS_MIN, so.PULSE_ABS_MAX)))
    return out


def write(bus, pose):
    sent = pulses(pose)
    for name, pulse in sent.items():
        bus.set(LEFT_SERVOS[name]["board"], LEFT_SERVOS[name]["channel"], pulse)
    return sent


def ease_open(bus, power, pose, seconds=so.EASE_SECONDS):
    steps = max(1, int(seconds * so.CONTROL_HZ))
    for k in range(1, steps + 1):
        power.check()
        write(bus, {n: v * (1.0 - smooth(k / steps)) for n, v in pose.items()})
        if not bus.dry_run:
            time.sleep(1.0 / so.CONTROL_HZ)


# --- camera ------------------------------------------------------------------

class Camera:
    """The webcam and MediaPipe's hand tracker."""

    def __init__(self, index):
        try:
            import cv2
            import mediapipe as mp
            from mediapipe.tasks.python import BaseOptions
            from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
        except ImportError as e:
            raise SystemExit(f"[left] {e}. The camera part needs opencv-python and mediapipe.")
        if not os.path.isfile(MODEL_PATH):
            raise SystemExit(f"[left] {MODEL_PATH} is missing.")
        self.cv2, self.mp = cv2, mp
        self.label = self.world = None   # last read: "Left"/"Right", and the 3-D landmarks
        self.capture = cv2.VideoCapture(index)
        if not self.capture.isOpened():
            raise SystemExit(f"[left] can't open camera {index}. Check: ls /dev/video*")
        self.landmarker = HandLandmarker.create_from_options(HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=RunningMode.IMAGE,
            num_hands=1,
        ))

    def read(self):
        """(frame, landmarks of the hand in view or None); (None, None) if the camera gave no frame."""
        ok, frame = self.capture.read()
        if not ok:
            return None, None
        rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        result = self.landmarker.detect(self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb))
        if not result.hand_landmarks:
            self.label = self.world = None
            return frame, None
        self.label = result.handedness[0][0].category_name if result.handedness else None
        self.world = result.hand_world_landmarks[0] if result.hand_world_landmarks else None
        return frame, result.hand_landmarks[0]

    def show(self, frame, hand, lines):
        """Draw the preview. False once q is pressed."""
        cv2 = self.cv2
        if hand:
            height, width = frame.shape[:2]
            for p in hand:
                cv2.circle(frame, (int(p.x * width), int(p.y * height)), 5, (0, 0, 0), -1)
        for row, line in enumerate(lines):
            cv2.putText(frame, line, (10, 30 + 25 * row), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)
        cv2.imshow("left hand", cv2.resize(frame, (960, 540)))
        return cv2.waitKey(1) & 0xFF != ord("q")

    def preview_works(self):
        frame, hand = self.read()
        if frame is None:
            return False
        try:
            self.show(frame, hand, ["starting..."])
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


def status_lines(angles, pose, sent):
    lines = []
    for name in LEFT_SERVOS:
        angle = f"{angles[name]:5.1f} deg" if name in angles else "   --    "
        pulse = sent.get(name, "not calibrated")
        lines.append(f"{name[5:]:6} {angle}  curl {pose[name]:.2f}  pulse {pulse}")
    return lines


# --- main --------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true",
                        help="use the camera but move nothing; prints the angles and pulses")
    parser.add_argument("--show", action="store_true", help="preview window with the landmarks; q quits")
    parser.add_argument("--camera", type=int, default=0, help="camera number (default 0)")
    args = parser.parse_args()

    issues = problems()
    if issues and not args.dry_run:
        print("[left] refusing to move anything:")
        for p in issues:
            print(f"          {p}")
        return
    for p in issues:
        print(f"[left] (dry run) {p}")
    if uncalibrated():
        print(f"[left] no pulses yet, staying still: {', '.join(uncalibrated())}")

    camera = Camera(args.camera)
    # Open the window before arming: if it can't open, nothing is live yet.
    if args.show and not camera.preview_works():
        print("[left] can't open a preview window here; carrying on without it.")
        args.show = False

    boards = tuple(sorted({LEFT_SERVOS[n]["board"] for n in LEFT_SERVOS if calibrated(n)})) or ("lv",)
    bus = so.ServoBus(dry_run=args.dry_run, boards=boards)
    power = PowerBox(dry_run=args.dry_run, on_safe=bus.release)
    pose = {name: 0.0 for name in LEFT_SERVOS}
    try:
        power.arm(lambda: write(bus, pose))
        print("[left] following the camera. Ctrl-C stops" + (", so does q in the preview." if args.show else "."))
        last = time.perf_counter()
        next_print = 0.0
        while True:
            power.check()
            frame, hand = camera.read()
            if frame is None:
                print("[left] the camera stopped sending frames.")
                break
            now = time.perf_counter()
            dt, last = min(now - last, 0.2), now
            angles = finger_angles(hand) if hand else {}
            pose = step(pose, {n: curl_of(n, a) for n, a in angles.items()}, MAX_CURL_PER_S * dt)
            sent = write(bus, pose)
            lines = status_lines(angles, pose, sent)
            if args.dry_run and now >= next_print:
                print(" | ".join(lines) if hand else "[left] no hand in view")
                next_print = now + 0.5
            if args.show and not camera.show(frame, hand, lines):
                break
    except EStop:
        print("[left] stopped by the E-stop.")
    except KeyboardInterrupt:
        print("\n[left] interrupted.")
    finally:
        if power.state == "ARMED" and not power.tripped.is_set() and not power.closed:
            try:
                ease_open(bus, power, pose)
            except Exception as e:
                print(f"[left] could not open the hand ({e})")
        power.close()
        bus.release()
        camera.close()
        print("[left] stopped -- signals off, servos released.")


if __name__ == "__main__":
    main()
