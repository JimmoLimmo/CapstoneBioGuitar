"""Type commands to move the hand; `help` lists them."""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "Sim"))
sys.path.insert(0, HERE)

import finger_motion as fm  # noqa: E402
import pick_up  # noqa: E402
from hand_control import Hand  # noqa: E402
from strum_motion import build_schedule, schedule_from_onsets, tempo_onsets  # noqa: E402


HELP = """Commands (curl is 0 = straight to 1 = fully curled):
    poses                      list the named poses in Sim/finger_motion.py
    pose NAME                  move the fingers to a named pose (open, fist, pick_ready, pick_grip...)
    finger NAME CURL [TIP]     one finger: both segments to CURL, or base to CURL and tip to TIP
    servo NAME CURL            one finger servo, e.g. servo index_tip 0.4
    wrist DEV FLEX             wrist angles in degrees
    strum [PATTERN] [N] [BPM]  N eighth-note strokes, e.g. strum DU 8 90   (default: D 4 60)
    song [SPEED]               play the song set in Sim/strum_motion.py
    pick / drop                grip the pick (hand-off) / let it go
    open / home / park         fingers open / wrist straight / both
    time SECONDS               how long each move takes (default 0.6)
    show                       current pose and the pulses being sent
    save NAME                  print the current finger pose as a line to paste into finger_motion.py
    q                          park and quit
"""


class Console:
    def __init__(self, hand):
        self.hand = hand
        self.seconds = 0.6

    def run(self, line):
        """Run one command. Returns False when it's time to quit."""
        words = line.split()
        if not words:
            return True
        cmd, args = words[0].lower(), words[1:]
        if cmd in ("q", "quit", "exit"):
            return False
        handler = getattr(self, f"do_{cmd}", None)
        if handler is None:
            print(f"  unknown command {cmd!r}; type help")
            return True
        try:
            handler(*args)
        except (TypeError, ValueError) as e:
            print(f"  {e}")
        return True

    def do_help(self):
        print(HELP)

    def do_poses(self):
        for name, p in fm.NAMED_POSES.items():
            print(f"  {name:11}" + " ".join(f"{f}={p[f + '_base']:.2f}/{p[f + '_tip']:.2f}" for f in fm.FINGERS))

    def do_pose(self, name):
        if name not in fm.NAMED_POSES:
            raise ValueError(f"no pose {name!r}; poses: {', '.join(fm.NAMED_POSES)}")
        self.hand.move(fingers=fm.NAMED_POSES[name], seconds=self.seconds)

    def do_finger(self, name, curl, tip=None):
        if name not in fm.FINGERS:
            raise ValueError(f"no finger {name!r}; fingers: {', '.join(fm.FINGERS)}")
        self._warn_undriven(name)
        base, tip = float(curl), float(curl if tip is None else tip)
        self.hand.move(fingers={f"{name}_base": base, f"{name}_tip": tip}, seconds=self.seconds)

    def do_servo(self, name, curl):
        if name not in fm.SERVO_NAMES:
            raise ValueError(f"no servo {name!r}; servos: {', '.join(fm.SERVO_NAMES)}")
        self._warn_undriven(name.split("_")[0])
        self.hand.move(fingers={name: float(curl)}, seconds=self.seconds)

    def do_wrist(self, dev, flex):
        if not self.hand.use_wrist:
            raise ValueError("the wrist isn't being driven this session (--no-wrist)")
        self.hand.move(wrist=(float(dev), float(flex)), seconds=self.seconds)
        got = self.hand.wrist_pose
        if (round(got[0], 3), round(got[1], 3)) != (float(dev), float(flex)):
            print(f"  limited to {got[0]:.1f} {got[1]:.1f}")

    def do_strum(self, pattern="D", count="4", bpm="60"):
        pattern = pattern.upper()
        if not pattern or set(pattern) - {"D", "U"}:
            raise ValueError("pattern is made of D and U, e.g. D, DU, DDU")
        count, bpm = int(count), float(bpm)
        if not (1 <= count <= 64) or not (20 <= bpm <= 240):
            raise ValueError("use 1-64 strokes and 20-240 BPM")
        schedule = schedule_from_onsets(tempo_onsets(bpm, count), list(pattern))
        self.hand.play(schedule)
        self.hand.move(wrist=(0.0, 0.0), seconds=self.seconds)

    def do_song(self, speed="1.0"):
        speed = float(speed)
        if speed <= 0:
            raise ValueError("speed must be > 0")
        schedule, source = build_schedule()
        print(f"  {len(schedule)} strokes from {source}. Ctrl-C stops (and ends the session).")
        self.hand.play(schedule, speed=speed)
        self.hand.move(wrist=(0.0, 0.0), seconds=self.seconds)

    def do_pick(self):
        pick_up.grip_pick(self.hand)

    def do_drop(self):
        pick_up.release_pick(self.hand)

    def do_open(self):
        self.hand.move(fingers=fm.OPEN, seconds=self.seconds)

    def do_home(self):
        self.hand.move(wrist=(0.0, 0.0), seconds=self.seconds)

    def do_park(self):
        self.hand.park(self.seconds)

    def do_time(self, seconds):
        seconds = float(seconds)
        if not (0.1 <= seconds <= 10):
            raise ValueError("use 0.1 to 10 seconds")
        self.seconds = seconds

    def do_show(self):
        h = self.hand
        print(f"  wrist  dev={h.wrist_pose[0]:.1f}  flex={h.wrist_pose[1]:.1f}" + ("" if h.use_wrist else "  (not driven)"))
        for f in fm.FINGERS:
            tag = "" if f in h.fingers else "  (not driven)"
            print(f"  {f:7} base={h.finger_pose[f + '_base']:.2f}  tip={h.finger_pose[f + '_tip']:.2f}{tag}")
        pulses = h.pulses()
        print("  pulses: " + (" ".join(f"{n}={p}" for n, p in pulses.items()) or "none (nothing calibrated)"))

    def do_save(self, name):
        p = self.hand.finger_pose
        parts = ", ".join(f"{f}=({p[f + '_base']:.2f}, {p[f + '_tip']:.2f})" for f in fm.FINGERS)
        print(f"  {name.upper()} = make_pose({parts})")

    def _warn_undriven(self, finger):
        if finger not in self.hand.fingers:
            print(f"  note: {finger} isn't driven this session (--fingers); only the sim will move")


def main():
    parser = argparse.ArgumentParser(description=__doc__, epilog=HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true", help="no hardware; poses still go to the sim")
    parser.add_argument("--sim-ip", help="computer running mvmnt_demo.py --follow (127.0.0.1 if it's this one)")
    parser.add_argument("--no-wrist", action="store_true", help="don't drive the wrist servos")
    parser.add_argument("--fingers", default="all",
                        help="which fingers to drive: all, none, or a list like index,thumb")
    parser.add_argument("--run", help='commands to run and then quit, separated by ";"')
    args = parser.parse_args()

    if args.fingers == "all":
        fingers = fm.FINGERS
    elif args.fingers == "none":
        fingers = ()
    else:
        fingers = tuple(f.strip() for f in args.fingers.split(",") if f.strip())

    with Hand(dry_run=args.dry_run, wrist=not args.no_wrist, fingers=fingers, sim_ip=args.sim_ip) as hand:
        hand.arm()
        console = Console(hand)
        if args.run:
            for line in args.run.split(";"):
                print(f"hand> {line.strip()}")
                if not console.run(line):
                    break
                hand.power.check()
            return
        print("[console] type help for commands, q to quit.")
        while True:
            try:
                line = input("hand> ")
            except EOFError:
                break
            hand.power.check()
            if not console.run(line):
                break


if __name__ == "__main__":
    main()
