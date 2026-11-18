"""Camera mimic: hold a hand up to the camera and the matching robot hand follows it.

    python3 run.py mimic --show                        # from the repo root, sim alongside
    servo Hardware/hand_mimic.py                       # on its own
    python3 Hardware/hand_mimic.py --dry-run --show    # camera only: shows the tracking, moves nothing
    python3 run.py mimic --hand left                   # always the left hand, whatever the camera sees
    python3 run.py mimic --hand both                   # both hands at once, each robot hand follows its own
    python3 run.py mimic --hand right --wrist          # the right hand's tilt drives the robot wrist too
    python3 run.py mimic --swap-hands                  # if your right hand comes up as "left" in the preview
    python3 Hardware/hand_mimic.py --calibrate --hand both --show   # measure your hands once (nothing moves)

The left hand is left_hand.py's (one servo per finger). The right hand uses the
finger servos in fingers.py (base + tip per finger). The sim shows both. With
auto/left/right, the hand that isn't being followed eases open. With both, a
hand that leaves the camera holds for a moment, then eases open.

--wrist turns the right hand's tilt (from the straight pose saved by --calibrate)
into wrist deviation and flexion, kept inside the strum's range.

The camera runs in its own thread and the servos are updated 50 times a second,
easing toward the latest reading, so the hand moves smoothly between frames.
"""

import argparse
import os
import statistics
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO_ROOT, "Sim"))
sys.path.insert(0, HERE)

import calibration  # noqa: E402
import finger_motion as fm  # noqa: E402
import fingers as fh  # noqa: E402
import hand_tracking as ht  # noqa: E402
import left_hand as lh  # noqa: E402
import servo_output as so  # noqa: E402
from motion_link import DEFAULT_PORT, PoseSender  # noqa: E402
from power_box import EStop, PowerBox  # noqa: E402
from strum_motion import LIFT_FLEX_DEG, WRIST_YAW_DOWN_DEG, WRIST_YAW_UP_DEG, smooth  # noqa: E402

SIDES = ht.SIDES
SWITCH_HOLD_S = 0.5    # the camera has to see the other hand for this long before the robot hands swap
LOST_OPEN_S = 1.0      # with --hand both, a hand out of view this long eases open (and the wrist straightens)
assert set(ht.RIGHT_TRACKING) == set(fm.SERVO_NAMES)
SAVED = calibration.load().get("tracking", {})
MEASURED = ht.apply_ranges(SAVED)
WRIST_NEUTRAL = SAVED.get("right", {}).get("wrist_neutral")

# the wrist stays inside what the strum already uses
DEV_RANGE = (min(WRIST_YAW_DOWN_DEG, WRIST_YAW_UP_DEG), max(WRIST_YAW_DOWN_DEG, WRIST_YAW_UP_DEG))
FLEX_RANGE = (-abs(LIFT_FLEX_DEG), abs(LIFT_FLEX_DEG))
STRAIGHT = {"dev": 0.0, "flex": 0.0}

CAL_SECONDS = 2.0       # how long each pose (open, fist) is recorded for --calibrate
CAL_MIN_SAMPLES = 10    # frames a servo needs in each pose to be measured
CAL_EDGE = 0.05         # pull each measured range in by this share so a real hand reaches 0 and 1


# which hand

# Turns the camera's left/right label into the robot hand to drive.
class HandPicker:

    def __init__(self, forced=None):
        self.forced = forced
        self.active = forced
        self.seen = None
        self.candidate = None
        self.since = 0.0

    # label = "left"/"right" from the camera, or None with no hand in view. Returns the active side.
    def update(self, label, now):
        side = label if label in SIDES else None
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


# camera

# Reads the camera and runs the tracker in the background, keeping only the newest frame.
class CameraThread:

    def __init__(self, camera):
        self.camera = camera
        self.lock = threading.Lock()
        self.latest = None
        self.frames = 0
        self.failed = False
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stopping.is_set():
            frame, seen = self.camera.read()
            if frame is None:
                self.failed = True
                return
            with self.lock:
                self.latest = (frame, seen, time.perf_counter())
                self.frames += 1

    # (frame, seen, time) if a new frame arrived since the last call, else None.
    def take(self):
        with self.lock:
            item, self.latest = self.latest, None
        return item

    def close(self):
        self.stopping.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.camera.close()


def _mean_unit(vectors):
    total = [sum(v[i] for v in vectors) for i in range(3)]
    n = sum(x * x for x in total) ** 0.5
    return [round(x / n, 5) for x in total] if n else None


# Record the bends with the hand open, then in a fist, and save them as that side's ranges.
# For the right hand the open pose is also saved as the wrist's straight (neutral) pose.
def calibrate_tracking(camera, side, show):
    poses = (("open", f"Hold your {side} hand up to the camera, fingers and thumb straight"
                      + (", wrist straight in line with your forearm." if side == "right" else ".")),
             ("fist", "Now make a fist, thumb curled in too."))
    recorded, axes = {}, []
    for pose, prompt in poses:
        input(f"[calibrate] {prompt}\n[calibrate] Press Enter, then hold still for {CAL_SECONDS:.0f} s... ")
        samples, wrong = {}, 0
        end = time.perf_counter() + CAL_SECONDS
        while time.perf_counter() < end:
            frame, seen = camera.read()
            if frame is None:
                raise SystemExit("[calibrate] the camera stopped sending frames.")
            hand = next((h for h in seen if h.label == side), None)
            if hand is None and seen:
                wrong += 1
            if hand is not None:
                for name, deg in ht.curls(hand.world, side)[1].items():
                    samples.setdefault(name, []).append(deg)
                if pose == "open":
                    found = ht.hand_axes(hand.world)
                    if found:
                        axes.append(found)
            if show:
                camera.show(frame, seen, [f"{side} {pose}: hold still"], "hand mimic")
        if wrong and not samples:
            print(f"[calibrate] the camera only saw a hand it calls the other side. "
                  f"Use your {side} hand, or add --swap-hands.")
        recorded[pose] = samples

    ranges, problems = {}, []
    for name in ht.TRACKING[side]:
        straight, curled = recorded["open"].get(name, []), recorded["fist"].get(name, [])
        if len(straight) < CAL_MIN_SAMPLES or len(curled) < CAL_MIN_SAMPLES:
            problems.append(f"{name}: hand not seen long enough ({len(straight)} open / {len(curled)} fist frames)")
            continue
        a, b = statistics.median(straight), statistics.median(curled)
        span = b - a
        if span < ht.MIN_SPAN_DEG:
            problems.append(f"{name}: open {a:.0f} deg and fist {b:.0f} deg are too close to use")
            continue
        ranges[name] = {"straight_deg": round(a + CAL_EDGE * span, 1), "curled_deg": round(b - CAL_EDGE * span, 1)}

    for name, r in ranges.items():
        print(f"  {name:12} straight {r['straight_deg']:6.1f} deg   curled {r['curled_deg']:6.1f} deg")
    for p in problems:
        print(f"  {p}  (keeps its old range)")
    if side == "right":
        if len(axes) >= CAL_MIN_SAMPLES:
            ranges["wrist_neutral"] = {"up": _mean_unit([a[0] for a in axes]),
                                       "across": _mean_unit([a[1] for a in axes])}
            print("  wrist        straight pose saved (used by --wrist)")
        else:
            print("  wrist        hand not seen long enough; straight pose not saved")
    if not ranges:
        print("[calibrate] nothing measured; nothing saved.")
        return
    try:
        calibration.save("tracking", side, ranges)
    except OSError as e:
        print(f"[calibrate] could not write {calibration.path()} ({e})")
        return
    print(f"[calibrate] saved the {side} hand. Run the mimic to try it.")


# servos

def movable(side):
    if side == "left":
        return [n for n in lh.LEFT_SERVOS if lh.calibrated(n)]
    return [n for n in fm.SERVO_NAMES if fh.is_calibrated(n)]


# Channels that two servos claim, across the left hand, the right fingers and the wrist.
def clashes():
    users = {}
    for table in (lh.LEFT_SERVOS, fh.FINGER_SERVOS, so.WRIST_SERVOS):
        for name, c in table.items():
            if c["channel"] is not None:
                users.setdefault((so.BOARDS[c["board"]], c["channel"]), []).append(name)
    return [f"{hex(address)} ch{channel} is assigned to {', '.join(names)}"
            for (address, channel), names in sorted(users.items()) if len(names) > 1]


def open_pose(side):
    return {n: 0.0 for n in lh.LEFT_SERVOS} if side == "left" else dict(fm.OPEN)


def clamp(value, lo_hi):
    return max(lo_hi[0], min(lo_hi[1], value))


# Send both poses (and the wrist, if it is driven) to the servos and the sim. Returns the left hand's pulses.
def write_hands(bus, hands, left_pose, right_pose, wrist=None):
    sent = lh.write(bus, left_pose) if "left" in hands else {}
    if "right" in hands:
        fh.write_fingers(bus, right_pose)
    if wrist is not None and so.is_calibrated():
        so.write_wrist(bus, wrist["dev"], wrist["flex"])
    if bus.sim:
        w = wrist or STRAIGHT
        bus.sim.send(w["dev"], w["flex"], right_pose, left=left_pose)
    return sent


def ease_open(bus, power, hands, left_pose, right_pose, wrist=None):
    steps = max(1, int(so.EASE_SECONDS * so.CONTROL_HZ))
    for k in range(1, steps + 1):
        power.check()
        keep = 1.0 - smooth(k / steps)
        write_hands(bus, hands, {n: v * keep for n, v in left_pose.items()},
                    {n: v * keep for n, v in right_pose.items()},
                    None if wrist is None else {a: v * keep for a, v in wrist.items()})
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


# main

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true",
                        help="use the camera but move nothing; prints what would be sent")
    parser.add_argument("--show", action="store_true", help="preview window with the landmarks; q quits")
    parser.add_argument("--camera", type=int, default=0, help="camera number (default 0)")
    parser.add_argument("--hand", choices=("auto", "left", "right", "both"), default="auto",
                        help="auto = whichever hand the camera sees; left/right = always that robot hand; "
                             "both = each robot hand follows its own hand at once")
    parser.add_argument("--wrist", action="store_true",
                        help="also drive the robot wrist from the right hand's tilt")
    parser.add_argument("--swap-hands", action="store_true",
                        help="flip the camera's left/right label (use it if your right hand shows as left)")
    parser.add_argument("--sim-ip", help="stream both hands' finger commands to mvmnt_demo.py --follow")
    parser.add_argument("--sim-port", type=int, default=DEFAULT_PORT, help="UDP port for --sim-ip")
    parser.add_argument("--calibrate", action="store_true",
                        help="measure your hand open and in a fist (camera only, nothing moves), then quit")
    args = parser.parse_args()
    both = args.hand == "both"

    if args.calibrate:
        sides = SIDES[::-1] if both else ("right" if args.hand == "auto" else args.hand,)
        camera = ht.Camera(args.camera, swap=args.swap_hands)
        if args.show and not camera.preview_works():
            print("[calibrate] can't open a preview window here; carrying on without it.")
            args.show = False
        try:
            for side in sides:
                calibrate_tracking(camera, side, args.show)
        except (KeyboardInterrupt, EOFError):
            print()
        finally:
            camera.close()
        return

    hands = SIDES if args.hand in ("auto", "both") else (args.hand,)
    issues = clashes()
    if args.wrist:
        if "right" not in hands:
            issues.append("--wrist follows the right hand; use --hand right, both or auto")
        if not WRIST_NEUTRAL:
            issues.append("--wrist needs the right hand's straight pose: run --calibrate --hand right first")
        if not so.is_calibrated():
            issues.append("--wrist needs all four wrist servos calibrated (calibrate_servo.py)")
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
    if MEASURED:
        print(f"[mimic] using measured ranges for {len(MEASURED)} servos.")
    else:
        print("[mimic] using guessed ranges; run with --calibrate to measure your hand.")
    wrist_on = args.wrist and bool(WRIST_NEUTRAL)

    camera = ht.Camera(args.camera, hands=2 if both else 1, swap=args.swap_hands)
    if args.show and not camera.preview_works():
        print("[mimic] can't open a preview window here; carrying on without it.")
        args.show = False

    tables = {"left": lh.LEFT_SERVOS, "right": fh.FINGER_SERVOS}
    boards = {tables[side][n]["board"] for side in hands for n in movable(side)}
    if wrist_on:
        boards |= {c["board"] for c in so.WRIST_SERVOS.values()}
    bus = so.ServoBus(dry_run=args.dry_run, boards=tuple(sorted(boards)) or ("lv",))
    power = PowerBox(dry_run=args.dry_run, on_safe=bus.release)
    if args.sim_ip:
        try:
            bus.sim = PoseSender(args.sim_ip, args.sim_port)
            print(f"[mimic] streaming both hands to the sim at {bus.sim.addr[0]}:{args.sim_port}")
        except OSError as e:
            print(f"[mimic] can't reach sim address {args.sim_ip} ({e}); continuing without the sim.")

    picker = HandPicker(None if args.hand in ("auto", "both") else args.hand)
    filters = {side: ht.CurlFilter() for side in SIDES}
    wrist_filter = ht.CurlFilter()
    poses = {side: open_pose(side) for side in SIDES}
    targets = {side: open_pose(side) for side in SIDES}
    last_seen = {side: float("-inf") for side in SIDES}
    wrist = dict(STRAIGHT)
    wrist_target = dict(STRAIGHT)
    grabber = CameraThread(camera)
    period = 1.0 / so.CONTROL_HZ

    def output():
        return write_hands(bus, hands, poses["left"], poses["right"], wrist if wrist_on else None)

    try:
        power.arm(output)
        print("[mimic] following the camera" + (" (both hands)" if both else "")
              + (", wrist on" if wrist_on else "") + ". Ctrl-C stops"
              + (", so does q in the preview." if args.show else "."))
        grabber.start()
        last = time.perf_counter()
        next_tick = last
        next_print = 0.0
        fps, fps_frames, fps_since = 0.0, 0, last
        was_active = picker.active
        while True:
            power.check()
            if grabber.failed:
                print("[mimic] the camera stopped sending frames.")
                break
            now = time.perf_counter()
            dt, last = min(now - last, 0.2), now
            if now - fps_since >= 1.0:
                fps = (grabber.frames - fps_frames) / (now - fps_since)
                fps_frames, fps_since = grabber.frames, now

            item = grabber.take()
            if item is not None:
                frame, seen, stamp = item
                if both:
                    followed = {}
                    for h in seen:
                        if h.label in SIDES and h.label not in followed:
                            followed[h.label] = h
                    active = "both"
                    head = "following: " + (" + ".join(s for s in SIDES if s in followed) or "no hand in view")
                else:
                    hand = seen[0] if seen else None
                    active = picker.update(hand.label if hand else None, stamp)
                    if active != was_active:
                        print(f"[mimic] now following the {active} hand.")
                        was_active = active
                    followed = {active: hand} if hand is not None and picker.seen == active else {}
                    head = f"following: {active or 'nothing yet'}"
                    if picker.seen and picker.seen != active:
                        head += f"  (camera sees a {picker.seen} hand)"
                    for side in SIDES:
                        if side != active:
                            targets[side] = open_pose(side)

                bends = {}
                for side in SIDES:
                    h = followed.get(side)
                    if h is None:
                        if both and stamp - last_seen[side] > LOST_OPEN_S:
                            targets[side] = open_pose(side)
                        continue
                    last_seen[side] = stamp
                    raw, bends[side] = ht.curls(h.world, side)
                    target = filters[side](raw, stamp)
                    targets[side] = {**targets[side], **ht.deadband(targets[side], target)}

                if wrist_on:
                    h = followed.get("right")
                    angles = ht.wrist_angles(h.world, (WRIST_NEUTRAL["up"], WRIST_NEUTRAL["across"])) if h else None
                    if angles is not None:
                        scaled = {"dev": angles[0] / ht.WRIST_SCALE_DEG, "flex": angles[1] / ht.WRIST_SCALE_DEG}
                        kept = {a: v / ht.WRIST_SCALE_DEG for a, v in wrist_target.items()}
                        held = ht.deadband(kept, wrist_filter(scaled, stamp))
                        wrist_target = {"dev": clamp(held["dev"] * ht.WRIST_SCALE_DEG, DEV_RANGE),
                                        "flex": clamp(held["flex"] * ht.WRIST_SCALE_DEG, FLEX_RANGE)}
                    elif (both and stamp - last_seen["right"] > LOST_OPEN_S) or (not both and active != "right"):
                        wrist_target = dict(STRAIGHT)

                lines = [f"{head}  camera {fps:.0f} fps"]
                if wrist_on:
                    lines.append(f"wrist  deviation {wrist['dev']:+5.1f} deg  flexion {wrist['flex']:+5.1f} deg")
                if "right" in hands and (both or active == "right"):
                    lines += right_lines(bends.get("right", {}), poses["right"])
                if "left" in hands and (both or active == "left"):
                    lines += lh.status_lines(bends.get("left", {}), poses["left"], lh.pulses(poses["left"]))
                if args.dry_run and now >= next_print:
                    print(" | ".join(lines) if seen else f"[mimic] no hand in view  (camera {fps:.0f} fps)")
                    next_print = now + 0.5
                if args.show and not camera.show(frame, seen, lines):
                    break

            for side in SIDES:
                poses[side] = ht.follow(poses[side], targets[side], dt)
            wrist = ht.follow(wrist, wrist_target, dt, max_rate=ht.WRIST_MAX_DEG_PER_S)
            output()

            next_tick += period
            pause = next_tick - time.perf_counter()
            if pause > 0:
                time.sleep(pause)
            else:
                next_tick = time.perf_counter()
    except EStop:
        print("[mimic] stopped by the E-stop.")
    except KeyboardInterrupt:
        print("\n[mimic] interrupted.")
    finally:
        if power.state == "ARMED" and not power.tripped.is_set() and not power.closed:
            try:
                ease_open(bus, power, hands, poses["left"], poses["right"], wrist if wrist_on else None)
            except Exception as e:
                print(f"[mimic] could not open the hands ({e})")
        power.close()
        bus.release()
        grabber.close()
        print("[mimic] stopped -- signals off, servos released.")


if __name__ == "__main__":
    main()
