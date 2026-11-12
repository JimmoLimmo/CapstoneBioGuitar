"""Bench calibration for the servos (right hand, and the left hand's fingers). Saved values go to Hardware/calibration.json.

One servo by name -- jog it, then save where it is:
    sudo python3 Hardware/calibrate_servo.py dev_pos        # wrist servo: dev_pos, dev_neg, flex_pos, flex_neg
    sudo python3 Hardware/calibrate_servo.py index_tip 3    # finger servo on channel 3 of the 8.4 V board
    sudo python3 Hardware/calibrate_servo.py left_thumb     # left-hand servo (left_thumb ... left_pinky)
  At the prompt type a pulse (4900) or a nudge (+50, -50), then
    wrist servo:   neutral | min | max | sign + | sign -
    finger servo:  open | closed     (left-hand servos too)
  to save the pulse it is holding. `show` prints the servo's entry, `q` quits.

One channel by number, nothing saved (wiring checks):
    sudo python3 Hardware/calibrate_servo.py <channel 0-15> [hv|lv]
    (hv = 8.4 V board at 0x40, the default; lv = 5 V board at 0x41)

Wrist mode -- once all four wrist servos have a neutral pulse, type wrist angles
and all four move together through the tendon kinematics (checking directions,
limits and string tension):
    sudo python3 Hardware/calibrate_servo.py --wrist

What is calibrated and what is still missing (no hardware needed):
    python3 Hardware/calibrate_servo.py --show

Add --dry-run to try any of it with no hardware.

Every mode goes through the electrical box (power_box.py): press RESET on the
box for servo power, then Enter to arm (servo signals on). The E-stop drops
everything at any time; signals go OFF on every exit.
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import calibration  # noqa: E402
import fingers as fh  # noqa: E402
import left_hand as lh  # noqa: E402
import servo_output as so  # noqa: E402
import wrist_kinematics as wk  # noqa: E402
from power_box import EStop, PowerBox  # noqa: E402

PULSE_ABS_MIN = 1638
PULSE_ABS_MAX = 8191
START_PULSE = 4900   # ~1.5 ms, the middle of travel: where a servo with nothing saved starts

TABLES = {"wrist": so.WRIST_SERVOS, "fingers": fh.FINGER_SERVOS, "left": lh.LEFT_SERVOS}
SAVE_KEYS = {
    "wrist": {"neutral": "neutral_pulse", "min": "min_pulse", "max": "max_pulse"},
    "fingers": {"open": "open_pulse", "closed": "closed_pulse"},
    "left": {"open": "open_pulse", "closed": "closed_pulse"},
}
HOME_KEY = {"wrist": "neutral_pulse", "fingers": "open_pulse", "left": "open_pulse"}


def section_of(name):
    return next((section for section, table in TABLES.items() if name in table), None)


def entry_line(name):
    cfg = TABLES[section_of(name)][name]
    return f"{name:12}" + "  ".join(f"{k}={'-' if v is None else v}" for k, v in cfg.items())


def shared_with(name, board, channel):
    """Other servos already assigned this board address and channel."""
    return [other for table in TABLES.values() for other, cfg in table.items()
            if other != name and cfg["channel"] == channel and so.BOARDS.get(cfg["board"]) == so.BOARDS[board]]


def command(section, name, board, channel, pulse, words):
    """Run a save / show command for a named servo. False if `words` isn't one."""
    values = None
    if len(words) == 1 and words[0] in SAVE_KEYS[section]:
        values = {SAVE_KEYS[section][words[0]]: pulse}
    elif section == "wrist" and len(words) == 2 and words[0] == "sign" and words[1] in ("+", "-", "+1", "-1"):
        values = {"reel_in_sign": -1 if words[1].startswith("-") else 1}
    elif words != ["show"]:
        return False
    if values:
        values = {"board": board, "channel": channel, **values}
        try:
            calibration.save(section, name, values)
        except OSError as e:
            print(f"  could not write {calibration.path()} ({e})")
            return True
        TABLES[section][name].update(values)
        print("  saved.")
    print("  " + entry_line(name))
    return True


def jog(channel, board, name=None, dry_run=False):
    section = section_of(name)
    saved_home = TABLES[section][name][HOME_KEY[section]] if section else None
    start = START_PULSE if saved_home is None else saved_home
    pulse = start

    bus = so.ServoBus(dry_run=dry_run, boards=(board,))
    power = PowerBox(dry_run=dry_run, on_safe=bus.release)
    print(f"[calibrate] {name or 'raw channel'}: channel {channel} on the '{board}' board ({hex(so.BOARDS[board])}).")
    others = shared_with(name, board, channel)
    if others:
        print(f"[calibrate] note: that channel is also assigned to {', '.join(others)}.")
    try:
        power.arm(lambda: bus.set(board, channel, start))
        print(f"[calibrate] holding {pulse}. Type a pulse ({PULSE_ABS_MIN}-{PULSE_ABS_MAX}) or a nudge (+50, -50); q quits.")
        print("[calibrate] nudge in steps of 50-200 -- don't jump to the extremes.")
        if section:
            saves = " | ".join(SAVE_KEYS[section]) + (" | sign + | sign -" if section == "wrist" else "")
            print(f"[calibrate] save the pulse it is holding with: {saves}   (show prints the entry)")
        label = name or f"{board} ch{channel}"
        while True:
            raw = input(f"{label} @ {pulse}> ").strip().lower()
            power.check()
            if raw in ("q", "quit", "exit"):
                break
            if not raw:
                continue
            if section and command(section, name, board, channel, pulse, raw.split()):
                continue
            try:
                target = pulse + int(raw) if raw[0] in "+-" else int(raw)
            except ValueError:
                print("  not a pulse" + (" or a command" if section else ""))
                continue
            if not (PULSE_ABS_MIN <= target <= PULSE_ABS_MAX):
                print(f"  {target} is out of the safe range ({PULSE_ABS_MIN}-{PULSE_ABS_MAX}), ignored")
                continue
            pulse = target
            bus.set(board, channel, pulse)
            print(f"  -> sent {pulse}." + ("" if section else " Note it down once the servo is where you want it."))
    except EStop:
        print("[calibrate] stopped by the E-stop.")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        power.close()
        bus.release()
        print("[calibrate] signals off, servo released.")


def wrist_mode(dry_run=False):
    problems = so.missing_calibration(wrist_only=True)
    if problems:
        print("[calibrate] calibrate these first (calibrate_servo.py <servo name>):")
        for p in problems:
            print(f"            {p}")
        return

    boards = tuple(sorted({c["board"] for c in so.WRIST_SERVOS.values()}))
    bus = so.ServoBus(dry_run=dry_run, boards=boards)
    power = PowerBox(dry_run=dry_run, on_safe=bus.release)
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
        print("[calibrate] signals off, servos released.")


def show():
    exists = os.path.isfile(calibration.path())
    print(f"[calibrate] file: {calibration.path()}" + ("" if exists else "  (not created yet; these are the defaults)"))
    for section, table in TABLES.items():
        print(f"\n{section}:")
        for name in table:
            print("  " + entry_line(name))
    print(f"\nposes saved: {', '.join(sorted(calibration.load().get('poses', {}))) or 'none'}")
    problems = so.missing_calibration(wrist_only=False)
    problems += [f"{name}: no pulses (left hand)" for name in lh.uncalibrated()] + lh.problems()
    print("\nstill missing:" if problems else "\nnothing is missing.")
    for p in problems:
        print(f"  {p}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("target", nargs="?", help="servo name (dev_pos, index_tip...) or channel number 0-15")
    parser.add_argument("where", nargs="*", help="channel number and/or board (hv, lv)")
    parser.add_argument("--wrist", action="store_true", help="move all four wrist servos together")
    parser.add_argument("--show", action="store_true", help="print the calibration in use")
    parser.add_argument("--dry-run", action="store_true", help="no hardware")
    args = parser.parse_args()

    if args.show:
        return show()
    if args.wrist:
        return wrist_mode(args.dry_run)
    if args.target is None:
        return parser.print_help()

    board = channel = None
    for word in args.where:
        if word in so.BOARDS and board is None:
            board = word
        elif word.isdigit() and channel is None:
            channel = int(word)
        else:
            parser.error(f"don't understand {word!r}: expected a channel number and/or hv / lv")

    name = None
    if args.target.isdigit():
        if channel is not None:
            parser.error("give one channel number")
        channel, board = int(args.target), board or "hv"
    elif section_of(args.target):
        name = args.target
        cfg = TABLES[section_of(name)][name]
        board = board or cfg["board"]
        channel = cfg["channel"] if channel is None else channel
        if channel is None:
            parser.error(f"{name} has no channel yet; give it, e.g.: calibrate_servo.py {name} 3")
    else:
        names = ", ".join([*so.WRIST_SERVOS, *fh.FINGER_SERVOS, *lh.LEFT_SERVOS])
        parser.error(f"{args.target!r} is not a channel number or a servo ({names})")
    if not 0 <= channel <= 15:
        parser.error("channels are 0-15")
    jog(channel, board, name, args.dry_run)


if __name__ == "__main__":
    main()
