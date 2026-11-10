"""Get the pick into the hand: hand-off (default) or from a holder (--holder)."""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "Sim"))
sys.path.insert(0, HERE)

import finger_motion as fm  # noqa: E402
from hand_control import Hand  # noqa: E402

READY_SECONDS = 0.8
CLOSE_SECONDS = 0.6     # slow, so the thumb presses the pick instead of knocking it off
SETTLE_SECONDS = 0.3

# --- holder pickup (TODO: placeholders until the holder is built) -------------
HOLDER_WRIST = (0.0, -15.0)   # (deviation, flexion) deg that puts the index platform under the pick
LIFT_WRIST = (0.0, 10.0)      # pull the pick up and out of the holder before going home


def grip_pick(hand, ask=True):
    """Hand-off: make the platform, wait for the pick, close the thumb on it."""
    hand.move(fingers=fm.PICK_READY, seconds=READY_SECONDS)
    if ask and not hand.dry_run:
        input("[pick] Lay the pick on the side of the index finger, tip pointing at the strings, "
              "then press Enter (keep your fingers clear of the thumb)... ")
        hand.power.check()
    hand.move(fingers=fm.PICK_GRIP, seconds=CLOSE_SECONDS)
    hand.hold(SETTLE_SECONDS)


def take_from_holder(hand):
    hand.move(fingers=fm.PICK_READY, seconds=READY_SECONDS)
    hand.move(wrist=HOLDER_WRIST, seconds=0.8)
    hand.hold(SETTLE_SECONDS)
    hand.move(fingers=fm.PICK_GRIP, seconds=CLOSE_SECONDS)
    hand.hold(SETTLE_SECONDS)
    hand.move(wrist=LIFT_WRIST, seconds=0.6)
    hand.move(wrist=(0.0, 0.0), seconds=0.6)


def release_pick(hand):
    hand.move(fingers=fm.PICK_READY, seconds=CLOSE_SECONDS)
    hand.move(fingers=fm.OPEN, seconds=READY_SECONDS)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true", help="no hardware; poses still go to the sim")
    parser.add_argument("--sim-ip", help="computer running mvmnt_demo.py --follow")
    parser.add_argument("--holder", action="store_true", help="take the pick from the holder instead of a hand-off")
    parser.add_argument("--hold", type=float, default=5.0, help="seconds to hold the pick before letting go")
    args = parser.parse_args()

    with Hand(dry_run=args.dry_run, wrist=args.holder, sim_ip=args.sim_ip) as hand:
        hand.arm()
        if args.holder:
            take_from_holder(hand)
        else:
            grip_pick(hand)
        print(f"[pick] gripping. Holding {args.hold:.0f} s -- tug the pick gently to check it.")
        hand.hold(args.hold)
        if not args.dry_run:
            input("[pick] Hold the pick, then press Enter to release it... ")
            hand.power.check()
        release_pick(hand)


if __name__ == "__main__":
    main()
