"""Bench calibration for the right hand's servos.

Raw mode -- jog ONE channel to typed pulse values (finding neutral_pulse,
reel_in_sign and min/max_pulse for each wrist servo):
    sudo python3 Hardware/calibrate_servo.py <channel 0-15> [hv|lv]
    (hv = 8.4 V board at 0x41, the default; lv = 5 V board at 0x40)

Wrist mode -- once all four WRIST_SERVOS entries in servo_output.py are filled
in, type wrist angles and all four servos move together through the tendon
kinematics (checking directions, limits and string tension):
    sudo python3 Hardware/calibrate_servo.py --wrist

Both modes go through the electrical box (power_box.py): relays start OFF,
press Enter to arm, then press the box's RESET button for servo power.
The E-stop drops everything at any time; relays go OFF on every exit.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

PULSE_ABS_MIN = 1638
PULSE_ABS_MAX = 8191
START_PULSE = 4900   # ~1.5 ms, the middle of travel: the home pulse for raw mode


def raw_mode(channel, board):
    from power_box import EStop, PowerBox
    from servo_output import BOARDS, ServoBus

    bus = ServoBus(dry_run=False, boards=(board,))
    power = PowerBox(on_safe=bus.release)
    print(f"[calibrate] channel {channel} on the '{board}' board ({hex(BOARDS[board])}).")
    try:
        power.arm(lambda: bus.set(board, channel, START_PULSE))
        print(f"[calibrate] holding {START_PULSE}. Enter a pulse ({PULSE_ABS_MIN}-{PULSE_ABS_MAX}), or 'q' to quit.")
        print("[calibrate] nudge in steps of 50-200 -- don't jump to the extremes.")
        while True:
            raw = input(f"{board} ch{channel} pulse> ").strip()
            power.check()
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
            bus.set(board, channel, pulse)
            print(f"  -> sent {pulse}. Note it down once the servo is where you want it.")
    except EStop:
        print("[calibrate] stopped by the E-stop.")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        power.close()
        bus.release()
        print("[calibrate] relays off, servo released.")


def wrist_mode():
    import servo_output as so
    import wrist_kinematics as wk
    from power_box import EStop, PowerBox

    problems = so.missing_calibration(wrist_only=True)
    if problems:
        print("[calibrate] fill these in servo_output.py WRIST_SERVOS first (use raw mode):")
        for p in problems:
            print(f"            {p}")
        return

    boards = tuple(sorted({c["board"] for c in so.WRIST_SERVOS.values()}))
    bus = so.ServoBus(dry_run=False, boards=boards)
    power = PowerBox(on_safe=bus.release)
    pose = (0.0, 0.0)
    try:
        power.arm(lambda: so.write_wrist(bus, 0.0, 0.0))
        print("[calibrate] wrist at neutral. Type '<deviation> <flexion>' in degrees, e.g. '25 0' or '0 20'.")
        print("[calibrate]   '25 0' should be the up-strum READY side, '-20 0' the down-strum follow-through.")
        print("[calibrate]   If the wrist goes the other way, swap the dev_pos/dev_neg (or flex) channel numbers.")
        print("[calibrate]   'q' parks at neutral and quits.")
        while True:
            raw = input(f"wrist (now {pose[0]:+.0f} {pose[1]:+.0f})> ").strip().lower()
            power.check()
            if raw in ("q", "quit", "exit"):
                break
            try:
                dev, flex = (float(v) for v in raw.replace(",", " ").split())
            except ValueError:
                print("  need two numbers: deviation flexion")
                continue
            target = (dev, flex)
            so.ease_wrist(bus, pose, target, seconds=1.0, power=power)
            pose = target
            pulses, clamped = so.wrist_pulses(dev, flex)
            angles = wk.servo_angles(dev, flex)
            for name in wk.SERVO_NAMES:
                flag = "  <-- at its limit" if name in clamped else ""
                print(f"  {name:8} spool {angles[name]:+6.1f} deg  pulse {pulses[name]}{flag}")
            print("  Check: all four strings snug? none slack, none straining?")
        so.ease_wrist(bus, pose, (0.0, 0.0), power=power)
    except EStop:
        print("[calibrate] stopped by the E-stop.")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        power.close()
        bus.release()
        print("[calibrate] relays off, servos released.")


def main():
    args = sys.argv[1:]
    if args == ["--wrist"]:
        wrist_mode()
    elif 1 <= len(args) <= 2 and args[0].isdigit() and 0 <= int(args[0]) <= 15 \
            and (len(args) == 1 or args[1] in ("hv", "lv")):
        raw_mode(int(args[0]), args[1] if len(args) == 2 else "hv")
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
