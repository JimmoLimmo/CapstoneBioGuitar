"""Bench calibration for the right hand's servos.

Raw mode -- jog ONE channel to typed pulse values (finding neutral_pulse,
reel_in_sign and min/max_pulse for each wrist servo):
    python Hardware/calibrate_servo.py <channel 0-15>

Wrist mode -- once all four WRIST_SERVOS entries in servo_output.py are filled
in, type wrist angles and all four servos move together through the tendon
kinematics (checking directions, limits and string tension):
    python Hardware/calibrate_servo.py --wrist
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

PULSE_ABS_MIN = 1638
PULSE_ABS_MAX = 8191


def raw_mode(channel):
    from servo_output import ServoBus

    bus = ServoBus(dry_run=False)
    print(f"[calibrate] channel {channel}. Enter a pulse ({PULSE_ABS_MIN}-{PULSE_ABS_MAX}), or 'q' to quit.")
    print("[calibrate] start near the middle (~4900) and nudge in -- don't jump to the extremes.")
    try:
        while True:
            raw = input(f"channel {channel} pulse> ").strip()
            if raw.lower() in ("q", "quit", "exit"):
                break
            try:
                pulse = int(raw)
            except ValueError:
                print("  not a number")
                continue
            if not (PULSE_ABS_MIN <= pulse <= PULSE_ABS_MAX):
                print(f"  out of the safe range ({PULSE_ABS_MIN}-{PULSE_ABS_MAX}), ignored")
                continue
            bus.set(channel, pulse)
            print(f"  -> sent {pulse}. Note it down once the servo is where you want it.")
    finally:
        bus.release()


def wrist_mode():
    import servo_output as so
    import wrist_kinematics as wk

    problems = so.missing_calibration(wrist_only=True)
    if problems:
        print("[calibrate] fill these in servo_output.py WRIST_SERVOS first (use raw mode):")
        for p in problems:
            print(f"            {p}")
        return

    bus = so.ServoBus(dry_run=False)
    pose = (0.0, 0.0)
    so.ease_wrist(bus, pose, pose)
    print("[calibrate] wrist at neutral. Type '<deviation> <flexion>' in degrees, e.g. '25 0' or '0 20'.")
    print("[calibrate]   '25 0' should be the up-strum READY side, '-20 0' the down-strum follow-through.")
    print("[calibrate]   If the wrist goes the other way, swap the dev_pos/dev_neg (or flex) channel numbers.")
    print("[calibrate]   'q' parks at neutral and quits.")
    try:
        while True:
            raw = input(f"wrist (now {pose[0]:+.0f} {pose[1]:+.0f})> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            try:
                dev, flex = (float(v) for v in raw.replace(",", " ").split())
            except ValueError:
                print("  need two numbers: deviation flexion")
                continue
            target = (dev, flex)
            so.ease_wrist(bus, pose, target, seconds=1.0)
            pose = target
            pulses, clamped = so.wrist_pulses(dev, flex)
            angles = wk.servo_angles(dev, flex)
            for name in wk.SERVO_NAMES:
                flag = "  <-- at its limit" if name in clamped else ""
                print(f"  {name:8} spool {angles[name]:+6.1f} deg  pulse {pulses[name]}{flag}")
            print("  Check: all four strings snug? none slack, none straining?")
    finally:
        so.ease_wrist(bus, pose, (0.0, 0.0))
        bus.release()


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--wrist":
        wrist_mode()
    elif len(sys.argv) == 2 and sys.argv[1].isdigit() and 0 <= int(sys.argv[1]) <= 15:
        raw_mode(int(sys.argv[1]))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
