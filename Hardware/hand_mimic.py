"""Camera mimic for both hands: hold a left or a right hand up to the camera and the matching robot hand follows it.

    python3 run.py mimic --show                        # from the repo root, sim alongside
    sudo python3 Hardware/hand_mimic.py                # on its own
    python3 Hardware/hand_mimic.py --dry-run --show    # camera only: shows the tracking, moves nothing
    python3 run.py mimic --hand left                   # always the left hand, whatever the camera sees
    python3 run.py mimic --swap-hands                  # if your right hand comes up as "left" in the preview

The left hand is left_hand.py's (one servo per finger). The right hand uses the
finger servos in fingers.py (base + tip per finger) and is what the sim shows.
The hand that isn't being followed eases open. The wrist is not driven.
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

import finger_motion as fm  # noqa: E402
import fingers as fh  # noqa: E402
import left_hand as lh  # noqa: E402
import servo_output as so  # noqa: E402
from motion_link import DEFAULT_PORT, PoseSender  # noqa: E402
from power_box import EStop, PowerBox  # noqa: E402
from strum_motion import smooth  # noqa: E402

SIDES = ("left", "right")
SWITCH_HOLD_S = 0.5    # the camera has to see the other hand for this long before the robot hands swap


# --- right hand: camera -> curls -----------------------------------------------
# joints = MediaPipe landmarks (a, b, c); the bend is how far b is from straight,
# added up over the joints listed. straight_deg / curled_deg = that sum with the
# finger straight / fully curled. All guesses: read the real ones off --dry-run --show.

def _finger(mcp, pip, dip, tip):
    return {"base": {"joints": ((0, mcp, pip),), "straight_deg": 15.0, "curled_deg": 80.0},
            "tip": {"joints": ((mcp, pip, dip), (pip, dip, tip)), "straight_deg": 10.0, "curled_deg": 150.0}}


_BY_FINGER = {
    "thumb": {"base": {"joints": ((1, 2, 3),), "straight_deg": 10.0, "curled_deg": 50.0},
              "tip": {"joints": ((2, 3, 4),), "straight_deg": 10.0, "curled_deg": 70.0}},
    "index": _finger(5, 6, 7, 8),
    "middle": _finger(9, 10, 11, 12),
    "ring": _finger(13, 14, 15, 16),
    "pinky": _finger(17, 18, 19, 20),
}
RIGHT_TRACKING = {f"{finger}_{segment}": t for finger, segments in _BY_FINGER.items() for segment, t in segments.items()}
assert set(RIGHT_TRACKING) == set(fm.SERVO_NAMES)


def bend(points, a, b, c):
    """Degrees the joint at b is bent away from straight; None if two landmarks coincide."""
    u = (points[b].x - points[a].x, points[b].y - points[a].y, points[b].z - points[a].z)
    v = (points[c].x - points[b].x, points[c].y - points[b].y, points[c].z - points[b].z)
    norm = math.sqrt(sum(x * x for x in u) * sum(x * x for x in v))
    if norm == 0:
        return None
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(u, v)) / norm))))


def right_bends(points):
    """{servo: summed bend in degrees} from one hand's 21 landmarks."""
    out = {}
    for name, t in RIGHT_TRACKING.items():
        bends = [bend(points, *joint) for joint in t["joints"]]
        if None not in bends:
            out[name] = sum(bends)
    return out


def right_curl(name, degrees):
    t = RIGHT_TRACKING[name]
    return max(0.0, min(1.0, (degrees - t["straight_deg"]) / (t["curled_deg"] - t["straight_deg"])))


# --- which hand ------------------------------------------------------------------

class HandPicker:
    """Turns the camera's Left/Right label into the robot hand to drive."""

    def __init__(self, forced=None, swap=False):
        self.forced = forced
        self.swap = swap
        self.active = forced
        self.seen = None
        self.candidate = None
        self.since = 0.0

    def update(self, label, now):
        """label = "Left"/"Right" from MediaPipe, or None with no hand in view. Returns the active side."""
        side = label.lower() if label else None
        if side not in SIDES:
            side = None
        elif self.swap:
            side = "left" if side == "right" else "right"
        self.seen = self.forced if (self.forced and label) else side
        if self.forced:
            return self.active
        if side is None or side == self.active:
            self.candidate = None
        elif side != self.candidate:
            self.candidate, self.since = side, now
        elif now - self.since >= SWITCH_HOLD_S:
            self.active, self.candidate = side, None
        return self.active


# --- servos ----------------------------------------------------------------------

def movable(side):
    if side == "left":
        return [n for n in lh.LEFT_SERVOS if lh.calibrated(n)]
    return [n for n in fm.SERVO_NAMES if fh.is_calibrated(n)]


def clashes():
    """Channels that two servos claim, across the left hand, the right fingers and the wrist."""
    users = {}
    for table in (lh.LEFT_SERVOS, fh.FINGER_SERVOS, so.WRIST_SERVOS):
        for name, c in table.items():
            if c["channel"] is not None:
                users.setdefault((so.BOARDS[c["board"]], c["channel"]), []).append(name)
    return [f"{hex(address)} ch{channel} is assigned to {', '.join(names)}"
            for (address, channel), names in sorted(users.items()) if len(names) > 1]


def write_hands(bus, hands, left_pose, right_pose):
    """Send both poses and tell the sim about the right one. Returns the left hand's pulses."""
    sent = lh.write(bus, left_pose) if "left" in hands else {}
    if "right" in hands:
        fh.write_fingers(bus, right_pose)
    bus.publish(0.0, 0.0, right_pose)
    return sent


def ease_open(bus, power, hands, left_pose, right_pose):
    steps = max(1, int(so.EASE_SECONDS * so.CONTROL_HZ))
    for k in range(1, steps + 1):
        power.check()
        keep = 1.0 - smooth(k / steps)
        write_hands(bus, hands, {n: v * keep for n, v in left_pose.items()},
                    {n: v * keep for n, v in right_pose.items()})
        if not bus.dry_run:
            time.sleep(1.0 / so.CONTROL_HZ)


def right_lines(bends, pose):
    lines = []
    for finger in fm.FINGERS:
        parts = []
        for segment in fm.SEGMENTS:
            name = f"{finger}_{segment}"
            angle = f"{bends[name]:5.1f}" if name in bends else "  -- "
            parts.append(f"{segment} {angle} deg curl {pose[name]:.2f}")
        lines.append(f"{finger:6} " + "  ".join(parts))
    return lines


# --- main --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true",
                        help="use the camera but move nothing; prints what would be sent")
    parser.add_argument("--show", action="store_true", help="preview window with the landmarks; q quits")
    parser.add_argument("--camera", type=int, default=0, help="camera number (default 0)")
    parser.add_argument("--hand", choices=("auto", "left", "right"), default="auto",
                        help="auto = whichever hand the camera sees; left/right = always that robot hand")
    parser.add_argument("--swap-hands", action="store_true",
                        help="flip the camera's left/right label (use it if your right hand shows as left)")
    parser.add_argument("--sim-ip", help="stream the right hand's finger commands to mvmnt_demo.py --follow")
    parser.add_argument("--sim-port", type=int, default=DEFAULT_PORT, help="UDP port for --sim-ip")
    args = parser.parse_args()

    hands = SIDES if args.hand == "auto" else (args.hand,)
    issues = clashes()
    if not args.dry_run and not any(movable(side) for side in hands):
        issues.append("no servo of " + " or ".join(f"the {s} hand" for s in hands) + " is calibrated")
    if issues and not args.dry_run:
        print("[mimic] refusing to move anything:")
        for p in issues:
            print(f"          {p}")
        print("[mimic]   pass --dry-run to check the tracking (and the sim) without the servos.")
        return
    for p in issues:
        print(f"[mimic] (dry run) {p}")
    for side in hands:
        ready = movable(side)
        total = len(lh.LEFT_SERVOS if side == "left" else fm.SERVO_NAMES)
        if len(ready) < total:
            print(f"[mimic] {side} hand: {len(ready)} of {total} servos calibrated; the rest stay still.")

    camera = lh.Camera(args.camera)
    if args.show and not camera.preview_works():
        print("[mimic] can't open a preview window here; carrying on without it.")
        args.show = False

    tables = {"left": lh.LEFT_SERVOS, "right": fh.FINGER_SERVOS}
    boards = tuple(sorted({tables[side][n]["board"] for side in hands for n in movable(side)})) or ("lv",)
    bus = so.ServoBus(dry_run=args.dry_run, boards=boards)
    power = PowerBox(dry_run=args.dry_run, on_safe=bus.release)
    if args.sim_ip:
        try:
            bus.sim = PoseSender(args.sim_ip, args.sim_port)
            print(f"[mimic] streaming the right hand to the sim at {bus.sim.addr[0]}:{args.sim_port}")
        except OSError as e:
            print(f"[mimic] can't reach sim address {args.sim_ip} ({e}); continuing without the sim.")

    picker = HandPicker(None if args.hand == "auto" else args.hand, args.swap_hands)
    left_pose = {name: 0.0 for name in lh.LEFT_SERVOS}
    right_pose = dict(fm.OPEN)
    try:
        power.arm(lambda: write_hands(bus, hands, left_pose, right_pose))
        print("[mimic] following the camera. Ctrl-C stops" + (", so does q in the preview." if args.show else "."))
        last = time.perf_counter()
        next_print = 0.0
        was_active = picker.active
        while True:
            power.check()
            frame, hand = camera.read()
            if frame is None:
                print("[mimic] the camera stopped sending frames.")
                break
            now = time.perf_counter()
            dt, last = min(now - last, 0.2), now
            active = picker.update(camera.label if hand else None, now)
            if active != was_active:
                print(f"[mimic] now following the {active} hand.")
                was_active = active
            follow = hand is not None and picker.seen == active

            # the followed hand tracks the camera (or holds if the camera lost it); the other one opens
            angles, bends = {}, {}
            left_target = {} if active == "left" else {n: 0.0 for n in left_pose}
            right_target = {} if active == "right" else dict(fm.OPEN)
            if follow and active == "left":
                angles = lh.finger_angles(hand)
                left_target = {n: lh.curl_of(n, a) for n, a in angles.items()}
            elif follow and active == "right":
                bends = right_bends(camera.world or hand)
                right_target = {n: right_curl(n, b) for n, b in bends.items()}
            limit = lh.MAX_CURL_PER_S * dt
            left_pose = lh.step(left_pose, left_target, limit)
            right_pose = lh.step(right_pose, right_target, limit)
            sent = write_hands(bus, hands, left_pose, right_pose)

            head = f"following: {active or 'nothing yet'}"
            if picker.seen and picker.seen != active:
                head += f"  (camera sees a {picker.seen} hand)"
            lines = [head] + (right_lines(bends, right_pose) if active == "right"
                              else lh.status_lines(angles, left_pose, sent) if active == "left" else [])
            if args.dry_run and now >= next_print:
                print(" | ".join(lines) if hand else "[mimic] no hand in view")
                next_print = now + 0.5
            if args.show and not camera.show(frame, hand, lines):
                break
    except EStop:
        print("[mimic] stopped by the E-stop.")
    except KeyboardInterrupt:
        print("\n[mimic] interrupted.")
    finally:
        if power.state == "ARMED" and not power.tripped.is_set() and not power.closed:
            try:
                ease_open(bus, power, hands, left_pose, right_pose)
            except Exception as e:
                print(f"[mimic] could not open the hands ({e})")
        power.close()
        bus.release()
        camera.close()
        print("[mimic] stopped -- signals off, servos released.")


if __name__ == "__main__":
    main()
