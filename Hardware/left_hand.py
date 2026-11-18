"""Left (fretting) hand: its finger servos follow a hand seen by the camera.

    python3 run.py mimic --hand left                  # from the repo root (hand_mimic.py, left hand only)
    servo Hardware/left_hand.py                # the same, on its own
    python3 Hardware/left_hand.py --dry-run --show    # camera only: shows the tracking, moves nothing

This is last year's Imitation/mimic.py moved onto the right-hand box. The servos
sit on the 5 V board ("lv", 0x41), next to the wrist servos (ch 10-13), and go through power_box.py like every other
script: Enter to arm, the E-stop stops it, signals off on every exit.

Calibrate a servo with:  servo Hardware/calibrate_servo.py left_index
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO_ROOT, "Sim"))
sys.path.insert(0, HERE)

import calibration  # noqa: E402
import fingers as fh  # noqa: E402
import hand_tracking as ht  # noqa: E402
import servo_output as so  # noqa: E402
from power_box import EStop, PowerBox  # noqa: E402
from strum_motion import smooth  # noqa: E402

# One servo per finger (no thumb): open_pulse = finger straight, closed_pulse = curled.
# The four fingers keep last year's pulses (never above 7000).
LEFT_SERVOS = {
    "left_index":  {"board": "lv", "channel": 1,  "open_pulse": 1638, "closed_pulse": 7000},
    "left_middle": {"board": "lv", "channel": 2,  "open_pulse": 1638, "closed_pulse": 7000},
    "left_ring":   {"board": "lv", "channel": 3,  "open_pulse": 1638, "closed_pulse": 7000},
    "left_pinky":  {"board": "lv", "channel": 4,  "open_pulse": 1638, "closed_pulse": 7000},
}
calibration.apply("left", LEFT_SERVOS)

# How the camera's view becomes a curl per finger is ht.LEFT_TRACKING (Sim/hand_tracking.py).
assert set(ht.LEFT_TRACKING) == set(LEFT_SERVOS)
ht.apply_ranges({"left": calibration.load().get("tracking", {}).get("left", {})})


# calibration

def calibrated(name):
    c = LEFT_SERVOS[name]
    return None not in (c["channel"], c["open_pulse"], c["closed_pulse"])


def uncalibrated():
    return [n for n in LEFT_SERVOS if not calibrated(n)]


# Reasons not to move anything.
def problems():
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


def status_lines(bends, pose, sent):
    lines = []
    for name in LEFT_SERVOS:
        angle = f"{bends[name]:5.1f} deg" if name in bends else "   --    "
        pulse = sent.get(name, "not calibrated")
        lines.append(f"{name[5:]:6} {angle}  curl {pose[name]:.2f}  pulse {pulse}")
    return lines


# main

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

    camera = ht.Camera(args.camera)
    # Open the window before arming: if it can't open, nothing is live yet.
    if args.show and not camera.preview_works("left hand"):
        print("[left] can't open a preview window here; carrying on without it.")
        args.show = False

    boards = tuple(sorted({LEFT_SERVOS[n]["board"] for n in LEFT_SERVOS if calibrated(n)})) or ("lv",)
    bus = so.ServoBus(dry_run=args.dry_run, boards=boards)
    power = PowerBox(dry_run=args.dry_run, on_safe=bus.release)
    pose = {name: 0.0 for name in LEFT_SERVOS}
    smoother = ht.CurlFilter()
    try:
        power.arm(lambda: write(bus, pose))
        print("[left] following the camera. Ctrl-C stops" + (", so does q in the preview." if args.show else "."))
        last = time.perf_counter()
        next_print = 0.0
        while True:
            power.check()
            frame, seen = camera.read()
            if frame is None:
                print("[left] the camera stopped sending frames.")
                break
            now = time.perf_counter()
            dt, last = min(now - last, 0.2), now
            target, bends = ht.curls(seen[0].world, "left") if seen else ({}, {})
            if seen:
                target = smoother(target, now)
            pose = ht.step(pose, target, ht.MAX_CURL_PER_S * dt)
            sent = write(bus, pose)
            lines = status_lines(bends, pose, sent)
            if args.dry_run and now >= next_print:
                print(" | ".join(lines) if seen else "[left] no hand in view")
                next_print = now + 0.5
            if args.show and not camera.show(frame, seen, lines, "left hand"):
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
