"""Test runner for the CDR test plan: finger, strum, tempo, report. Usage is in Hardware/README.md."""

import argparse
import csv
import json
import os
import random
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO_ROOT, "Sim"))
sys.path.insert(0, os.path.join(REPO_ROOT, "Hardware"))

import finger_motion as fm  # noqa: E402
from hand_control import CONTROL_HZ, Hand  # noqa: E402
from pick_up import grip_pick  # noqa: E402
from strum_motion import schedule_from_onsets, tempo_onsets  # noqa: E402

RESULTS_DIR = os.path.join(HERE, "results")

# --- acceptance numbers from the CDR -----------------------------------------
FINGER_SPACINGS_IN = (0.25, 0.20, 0.15, 0.10)
FINGER_TOLERANCE_IN = 0.02
ATTEMPTS_PER_TARGET = 10
PASS_FRACTION = 0.90          # 9 of 10
STRUM_TIME_TOLERANCE = 0.10   # pattern completion within 10% of target
INTERVAL_TOLERANCE = 0.10     # each stroke interval within +/-10%
STROKES_PER_CYCLE = 8         # eighth notes, two per beat
TEMPO_CYCLES = 10

MAP_CURLS = [i / 10 for i in range(11)]
SETTLE_SECONDS = 0.5
FAKE_TRAVEL_IN = 1.6          # --auto only: pretend fingertip travel
FAKE_NOISE_IN = 0.006

FINGER_FIELDS = ["time", "servo", "spacing_in", "target_in", "commanded_curl", "attempt",
                 "measured_in", "error_in", "within_tolerance", "note"]
STRUM_FIELDS = ["time", "cycle", "bpm", "pattern", "strokes", "target_s", "commanded_s",
                "time_error_pct", "worst_lateness_ms", "result", "note"]
TEMPO_FIELDS = ["time", "bpm", "cycle", "stroke", "direction", "target_interval_s",
                "scheduled_s", "commanded_s", "contact_frame", "result", "note"]


class Quit(Exception):
    pass


# --- small helpers -----------------------------------------------------------

def now():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def num(value):
    """float, or None for blank / not a number (CSV cells come back as text)."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class Log:
    """CSV that is flushed after every row, so an E-stop never loses data."""

    def __init__(self, name, fields, dry_run=False):
        # Rehearsal files go in their own folder so they never mix with real data.
        folder = os.path.join(RESULTS_DIR, "dry_run") if dry_run else RESULTS_DIR
        os.makedirs(folder, exist_ok=True)
        self.path = os.path.join(folder, f"{name}_{datetime.now():%Y%m%d-%H%M%S}.csv")
        self.file = open(self.path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=fields)
        self.writer.writeheader()
        self.file.flush()
        self.rows = []

    def add(self, **row):
        row = {"time": now(), **row}
        self.writer.writerow(row)
        self.file.flush()
        self.rows.append(row)

    def close(self):
        self.file.close()
        print(f"[test] results saved to {os.path.relpath(self.path, REPO_ROOT)}")


def ask_number(prompt):
    """A number, or None if skipped (fill it in later from the video)."""
    while True:
        raw = input(prompt).strip().lower()
        if raw in ("q", "quit"):
            raise Quit()
        if raw in ("", "s", "skip"):
            return None
        try:
            return float(raw)
        except ValueError:
            print("  type a number, Enter to skip, or q to stop")


def ask_cycle_result(auto):
    """('ok' | 'missed' | 'unintended' | 'both', note)."""
    if auto:
        return "ok", "SIMULATED"
    codes = {"": "ok", "y": "ok", "m": "missed", "u": "unintended", "b": "both"}
    while True:
        raw = input("  cycle result -- Enter=good  m=missed string  u=unintended contact  b=both  q=stop: ").strip().lower()
        if raw in ("q", "quit"):
            raise Quit()
        if raw in codes:
            break
    note = input("  note (optional): ").strip() if codes[raw] != "ok" else ""
    return codes[raw], note


def interp(points, x):
    """Linear interpolation through (x, y) points sorted by x, clamped at the ends."""
    if x <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


def make_hand(args, **kwargs):
    if args.auto and not args.dry_run:
        raise SystemExit("[test] --auto makes up measurements; it only works with --dry-run.")
    return Hand(dry_run=args.dry_run, sim_ip=args.sim_ip,
                realtime=True if args.realtime else None, **kwargs)


# --- fingertip position & accuracy ---------------------------------------------

def map_path(servo):
    return os.path.join(RESULTS_DIR, f"finger_map_{servo}.json")


def go_to(hand, servo, curl):
    """Always return to the start first, so every target is approached the same way."""
    hand.move(fingers={servo: 0.0}, seconds=0.5)
    hand.hold(0.2)
    hand.move(fingers={servo: curl}, seconds=0.6)
    hand.hold(SETTLE_SECONDS)


def measure_map(hand, servo, auto):
    """Step through the servo's travel and record where the fingertip marker sits."""
    print(f"[finger] mapping {servo}: read the fingertip marker on the ruler at each step (inches).")
    points = []
    for curl in MAP_CURLS:
        go_to(hand, servo, curl)
        if auto:
            position = FAKE_TRAVEL_IN * curl + random.gauss(0, FAKE_NOISE_IN / 3)
        else:
            position = ask_number(f"  curl {curl:.1f} -> position (in): ")
            if position is None:
                continue
        points.append((curl, position))
    if len(points) < 3:
        raise SystemExit("[finger] need at least 3 measured steps to build the map.")
    steps = [b[1] - a[1] for a, b in zip(points, points[1:])]
    if not (all(s > 0 for s in steps) or all(s < 0 for s in steps)):
        raise SystemExit("[finger] the readings don't move steadily in one direction, so a position can't be "
                         "turned back into a curl. Check the ruler direction and tendon slack, then rerun with --remap.")
    os.makedirs(RESULTS_DIR, exist_ok=True)
    if not auto:
        with open(map_path(servo), "w") as f:
            json.dump({"servo": servo, "measured": now(), "curl_position_in": points}, f, indent=1)
        print(f"[finger] map saved to {os.path.relpath(map_path(servo), REPO_ROOT)}")
    return points


def load_map(servo):
    try:
        with open(map_path(servo)) as f:
            return [tuple(p) for p in json.load(f)["curl_position_in"]]
    except (OSError, ValueError, KeyError):
        return None


def make_targets(lo, hi, spacing, max_targets):
    """Evenly spaced targets centred in the reachable range, kept clear of both ends."""
    margin = FINGER_TOLERANCE_IN * 1.5
    usable = (hi - margin) - (lo + margin)
    if usable < spacing:
        return []
    count = min(max_targets, int(usable / spacing + 1e-9) + 1)
    first = (lo + hi) / 2 - (count - 1) * spacing / 2
    return [first + i * spacing for i in range(count)]


def summarize_finger(rows):
    """Print the CDR numbers per target. True if the 0.10 in spacing passed."""
    groups = {}
    for r in rows:
        groups.setdefault((num(r["spacing_in"]), num(r["target_in"])), []).append(r)
    if not groups:
        print("[finger] no attempts recorded.")
        return False
    print("\n  spacing  target   measured  avg|err|  max|err|  success")
    spacing_ok = {}
    for (spacing, target), group in sorted(groups.items(), key=lambda kv: (-kv[0][0], kv[0][1])):
        errors = [num(r["measured_in"]) - target for r in group if num(r["measured_in"]) is not None]
        good = sum(abs(e) <= FINGER_TOLERANCE_IN + 1e-9 for e in errors)
        rate = good / len(group)   # an unmeasured attempt can't count as a success
        ok = len(errors) == len(group) and rate >= PASS_FRACTION
        spacing_ok[spacing] = spacing_ok.get(spacing, True) and ok
        stats = (f"{sum(map(abs, errors)) / len(errors):.3f}    {max(map(abs, errors)):.3f}" if errors
                 else "  -        -  ")
        print(f"  {spacing:5.2f}   {target:6.3f}   {len(errors):2}/{len(group):<2}     {stats}     "
              f"{rate * 100:3.0f}%  {'ok' if ok else 'FAIL' if len(errors) == len(group) else 'not all measured'}")
    final = min(FINGER_SPACINGS_IN)
    passed = all(spacing_ok.values()) and spacing_ok.get(final, False)
    reached = min(s for s, ok in spacing_ok.items() if ok) if any(spacing_ok.values()) else None
    print(f"\n[finger] tolerance +/-{FINGER_TOLERANCE_IN} in, pass = {PASS_FRACTION:.0%} at every target.")
    print(f"[finger] smallest spacing passed: {f'{reached:.2f} in' if reached else 'none'}  ->  "
          f"{'PASS' if passed else f'not passed yet (goal {final:.2f} in)'}")
    return passed


def cmd_finger(args):
    servo = f"{args.finger}_{args.segment}"
    other = f"{args.finger}_{'tip' if args.segment == 'base' else 'base'}"
    with make_hand(args, wrist=False, fingers=(args.finger,)) as hand:
        hand.arm()
        hand.move(fingers={other: args.other}, seconds=0.6)
        points = None if (args.remap or args.auto) else load_map(servo)
        log = None
        try:
            if points is None:
                points = measure_map(hand, servo, args.auto)
            else:
                print(f"[finger] using the saved map for {servo} (--remap to measure it again).")
            by_position = sorted((p, c) for c, p in points)
            lo, hi = by_position[0][0], by_position[-1][0]
            print(f"[finger] {servo} reaches {lo:.2f} to {hi:.2f} in.")

            log = Log(f"finger_{servo}", FINGER_FIELDS, args.dry_run)
            for spacing in ([args.spacing] if args.spacing else FINGER_SPACINGS_IN):
                targets = make_targets(lo, hi, spacing, args.targets)
                if len(targets) < 2:
                    print(f"[finger] can't fit two targets {spacing:.2f} in apart in that range; skipping.")
                    continue
                print(f"\n[finger] spacing {spacing:.2f} in, targets: " + ", ".join(f"{t:.3f}" for t in targets))
                start = len(log.rows)
                for target in targets:
                    curl = interp(by_position, target)
                    for attempt in range(1, ATTEMPTS_PER_TARGET + 1):
                        go_to(hand, servo, curl)
                        if args.auto:
                            measured, note = target + random.gauss(0, FAKE_NOISE_IN), "SIMULATED"
                        else:
                            measured = ask_number(f"  target {target:.3f} in, attempt {attempt}/{ATTEMPTS_PER_TARGET} "
                                                  "-> measured (in, Enter = fill in later): ")
                            note = ""
                        error = None if measured is None else measured - target
                        log.add(servo=servo, spacing_in=f"{spacing:.2f}", target_in=f"{target:.3f}",
                                commanded_curl=f"{curl:.4f}", attempt=attempt,
                                measured_in="" if measured is None else f"{measured:.3f}",
                                error_in="" if error is None else f"{error:+.3f}",
                                within_tolerance="" if error is None else int(abs(error) <= FINGER_TOLERANCE_IN + 1e-9),
                                note=note)
                done = log.rows[start:]
                misses = sum(1 for r in done if r["within_tolerance"] == 0)
                unmeasured = sum(1 for r in done if r["within_tolerance"] == "")
                per_target_ok = all(
                    sum(1 for r in done if r["target_in"] == f"{t:.3f}" and r["within_tolerance"] == 1)
                    >= PASS_FRACTION * ATTEMPTS_PER_TARGET for t in targets)
                if unmeasured:
                    print(f"[finger] {unmeasured} attempt(s) not measured yet; fill them in and run 'report'. Stopping here.")
                    break
                if not per_target_ok:
                    print(f"[finger] spacing {spacing:.2f} in did not pass ({misses} miss(es)). "
                          "Adjust, then repeat this spacing with --spacing.")
                    break
        except Quit:
            print("[finger] stopped by the operator.")
        finally:
            if log:
                summarize_finger(log.rows)
                log.close()


# --- strumming ----------------------------------------------------------------------

def run_cycle(hand, bpm, pattern, strokes, speed=1.0):
    """Play one cycle: (target s, commanded s, worst lateness ms, per-stroke log)."""
    schedule = schedule_from_onsets(tempo_onsets(bpm, strokes), list(pattern))
    log = hand.play(schedule, speed=speed)
    target = log[-1]["scheduled_s"] - log[0]["scheduled_s"]
    commanded = log[-1]["commanded_s"] - log[0]["commanded_s"]
    lateness = max(e["commanded_s"] - e["scheduled_s"] for e in log) * 1000
    return target, commanded, lateness, log


def start_strum_hand(args):
    fingers = () if args.no_fingers else fm.FINGERS
    hand = make_hand(args, wrist=True, fingers=fingers)
    return hand, fingers


def summarize_strum(rows):
    if not rows:
        print("[strum] no cycles recorded.")
        return False
    good = sum(r["result"] == "ok" for r in rows)
    unintended = sum(r["result"] in ("unintended", "both") for r in rows)
    worst_time = max(abs(num(r["time_error_pct"])) for r in rows)
    rate = good / len(rows)
    passed = rate >= PASS_FRACTION and unintended == 0 and worst_time <= STRUM_TIME_TOLERANCE * 100
    print(f"\n[strum] {good}/{len(rows)} good cycles ({rate:.0%}, need {PASS_FRACTION:.0%}); "
          f"unintended contact in {unintended} (need 0); "
          f"worst commanded completion-time error {worst_time:.1f}% (limit {STRUM_TIME_TOLERANCE:.0%}).")
    print("[strum] completion time here is what the software commanded. Check a few cycles "
          "against the video for the real motion.")
    print(f"[strum] {'PASS' if passed else 'FAIL'}")
    return passed


def cmd_strum(args):
    pattern = check_pattern(args.pattern)
    hand, fingers = start_strum_hand(args)
    with hand:
        hand.arm()
        if fingers:
            grip_pick(hand, ask=not args.auto)
        log = Log(f"strum_{args.bpm:g}bpm_{pattern}", STRUM_FIELDS, args.dry_run)
        try:
            for cycle in range(1, args.cycles + 1):
                if not (args.auto or args.dry_run):
                    if input(f"[strum] cycle {cycle}/{args.cycles}: Enter to run, q to stop: ").strip().lower() in ("q", "quit"):
                        raise Quit()
                    hand.power.check()
                target, commanded, lateness, _ = run_cycle(hand, args.bpm, pattern, args.strokes, args.speed)
                hand.hold(args.rest)
                result, note = ask_cycle_result(args.auto or args.dry_run)
                error_pct = (commanded - target) / target * 100 if target else 0.0
                log.add(cycle=cycle, bpm=f"{args.bpm:g}", pattern=pattern, strokes=args.strokes,
                        target_s=f"{target:.3f}", commanded_s=f"{commanded:.3f}",
                        time_error_pct=f"{error_pct:+.2f}", worst_lateness_ms=f"{lateness:.1f}",
                        result=result, note=note if not args.dry_run else "SIMULATED")
        except Quit:
            print("[strum] stopped by the operator.")
        finally:
            summarize_strum(log.rows)
            log.close()


# --- tempo and speed -------------------------------------------------------------------

def summarize_tempo(rows, fps):
    """Score a tempo CSV. Needs contact_frame filled in for the real result."""
    cycles = {}
    for r in rows:
        cycles.setdefault(int(num(r["cycle"])), []).append(r)
    if not cycles:
        print("[tempo] no cycles recorded.")
        return False
    bpm = num(rows[0]["bpm"])
    target = num(rows[0]["target_interval_s"])
    period_ms = 1000.0 / CONTROL_HZ
    lateness = max((num(r["commanded_s"]) - num(r["scheduled_s"])) * 1000 for r in rows)
    print(f"\n[tempo] {bpm:g} BPM, target interval {target:.3f} s (+/-{INTERVAL_TOLERANCE:.0%} = "
          f"{target * (1 - INTERVAL_TOLERANCE):.3f}..{target * (1 + INTERVAL_TOLERANCE):.3f} s)")
    print(f"[tempo] software: worst command lateness {lateness:.1f} ms "
          f"({'ok' if lateness <= period_ms * 1.5 else 'LATE: the control loop fell behind'}; "
          f"up to {period_ms:.0f} ms is the 50 Hz update step)")

    measured = passed = 0
    clean = all(r["result"] == "ok" for r in rows)
    for cycle, group in sorted(cycles.items()):
        group.sort(key=lambda r: num(r["stroke"]))
        result = group[0]["result"]
        frames = [num(r["contact_frame"]) for r in group]
        if any(f is None for f in frames):
            print(f"  cycle {cycle:2}: observed {result:10}  contact frames not filled in")
            continue
        measured += 1
        times = [f / fps for f in frames]
        intervals = [b - a for a, b in zip(times, times[1:])]
        worst = max(intervals, key=lambda i: abs(i - target))
        in_range = all(abs(i - target) <= target * INTERVAL_TOLERANCE + 1e-9 for i in intervals)
        expected = target * len(intervals)   # first contact to last = 7 intervals, not 8
        elapsed_error = ((times[-1] - times[0]) - expected) / expected * 100
        ok = in_range and result == "ok" and len(group) == STROKES_PER_CYCLE
        passed += ok
        print(f"  cycle {cycle:2}: observed {result:10}  worst interval {worst:.3f} s "
              f"({(worst - target) / target * 100:+.1f}%)  elapsed {elapsed_error:+.1f}%  {'pass' if ok else 'FAIL'}")
    if measured < len(cycles):
        print(f"[tempo] {len(cycles) - measured} cycle(s) still need contact frames: for each stroke, type the video "
              "frame number of the first intended string contact into contact_frame, then run 'report'.")
        return False
    need = int(PASS_FRACTION * len(cycles) + 0.999)
    overall = passed >= need and clean and len(cycles) >= TEMPO_CYCLES
    print(f"[tempo] {passed}/{len(cycles)} cycles passed (need {need}); "
          f"{'no' if clean else 'SOME'} missed or unintended strums"
          + ("" if len(cycles) >= TEMPO_CYCLES else f"; only {len(cycles)} of {TEMPO_CYCLES} cycles run"))
    print(f"[tempo] {bpm:g} BPM: {'PASS' if overall else 'FAIL'}")
    return overall


def cmd_tempo(args):
    pattern = check_pattern(args.pattern)
    hand, fingers = start_strum_hand(args)
    interval = 60.0 / args.bpm / 2
    with hand:
        hand.arm()
        if fingers:
            grip_pick(hand, ask=not args.auto)
        log = Log(f"tempo_{args.bpm:g}bpm", TEMPO_FIELDS, args.dry_run)
        try:
            for cycle in range(1, args.cycles + 1):
                if not (args.auto or args.dry_run):
                    if input(f"[tempo] cycle {cycle}/{args.cycles} at {args.bpm:g} BPM -- camera rolling? "
                             "Enter to run, q to stop: ").strip().lower() in ("q", "quit"):
                        raise Quit()
                    hand.power.check()
                _, _, _, strokes = run_cycle(hand, args.bpm, pattern, STROKES_PER_CYCLE)
                hand.hold(args.rest)
                result, note = ask_cycle_result(args.auto or args.dry_run)
                frame0 = random.randint(100, 400)
                for s in strokes:
                    frame = ""
                    if args.auto:   # made-up contact frames so 'report' can be rehearsed
                        frame = frame0 + round((s["scheduled_s"] + random.gauss(0, 0.004)) * args.fps)
                    log.add(bpm=f"{args.bpm:g}", cycle=cycle, stroke=s["stroke"] + 1, direction=s["direction"],
                            target_interval_s=f"{interval:.4f}", scheduled_s=f"{s['scheduled_s']:.4f}",
                            commanded_s=f"{s['commanded_s']:.4f}", contact_frame=frame, result=result,
                            note=note if not args.dry_run else "SIMULATED")
        except Quit:
            print("[tempo] stopped by the operator.")
        finally:
            summarize_tempo(log.rows, args.fps)
            log.close()


# --- report ------------------------------------------------------------------------------

def cmd_report(args):
    try:
        with open(args.file, newline="") as f:
            rows = list(csv.DictReader(f))
    except OSError as e:
        raise SystemExit(f"[report] can't read {args.file} ({e})")
    if not rows:
        raise SystemExit("[report] that file has no rows.")
    simulated = sum("SIMULATED" in (r.get("note") or "") for r in rows)
    if simulated:
        print(f"[report] NOTE: {simulated} of {len(rows)} rows are SIMULATED (dry run), not real measurements.")
    fields = set(rows[0])
    if "contact_frame" in fields:
        summarize_tempo(rows, args.fps)
    elif "measured_in" in fields:
        summarize_finger(rows)
    elif "time_error_pct" in fields:
        summarize_strum(rows)
    else:
        raise SystemExit("[report] not a results file from this script.")


def check_pattern(pattern):
    pattern = pattern.upper()
    if not pattern or set(pattern) - {"D", "U"}:
        raise SystemExit("[test] --pattern is made of D and U, e.g. D, DU, DDU")
    return pattern


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="test", required=True)

    def common(p):
        p.add_argument("--dry-run", action="store_true", help="no hardware; poses still go to the sim")
        p.add_argument("--sim-ip", help="computer running mvmnt_demo.py --follow")
        p.add_argument("--auto", action="store_true", help="with --dry-run: answer prompts with made-up values")
        p.add_argument("--realtime", action="store_true", help="with --dry-run: run at real speed (software timing check)")

    p = sub.add_parser("finger", help="fingertip position & accuracy")
    p.add_argument("finger", choices=fm.FINGERS)
    p.add_argument("--segment", choices=fm.SEGMENTS, default="tip", help="which of the finger's two servos to test")
    p.add_argument("--other", type=float, default=0.0, help="curl to hold the finger's other servo at")
    p.add_argument("--spacing", type=float, help="run one target spacing (in) instead of 0.25 -> 0.10")
    p.add_argument("--targets", type=int, default=3, help="most targets per spacing (default 3)")
    p.add_argument("--remap", action="store_true", help="measure the curl-to-position map again")
    common(p)
    p.set_defaults(run=cmd_finger)

    def strum_options(p):
        p.add_argument("--pattern", default="DU", help="stroke directions, cycled: D, DU, DDU...")
        p.add_argument("--rest", type=float, default=1.0, help="seconds between cycles")
        p.add_argument("--no-fingers", action="store_true", help="don't drive the fingers (pick clamped or taped)")
        common(p)

    p = sub.add_parser("strum", help="strumming performance / repeatability")
    p.add_argument("--cycles", type=int, default=20)
    p.add_argument("--bpm", type=float, default=60.0)
    p.add_argument("--strokes", type=int, default=STROKES_PER_CYCLE, help="eighth-note strokes per cycle")
    p.add_argument("--speed", type=float, default=1.0, help="0.25 = quarter speed for the first runs")
    strum_options(p)
    p.set_defaults(run=cmd_strum)

    p = sub.add_parser("tempo", help="tempo and speed (60, 90, 120 BPM)")
    p.add_argument("--bpm", type=float, required=True)
    p.add_argument("--cycles", type=int, default=TEMPO_CYCLES)
    p.add_argument("--fps", type=float, default=120.0, help="camera frame rate")
    strum_options(p)
    p.set_defaults(run=cmd_tempo)

    p = sub.add_parser("report", help="re-score a results CSV")
    p.add_argument("file")
    p.add_argument("--fps", type=float, default=120.0, help="camera frame rate for tempo files")
    p.set_defaults(run=cmd_report)

    args = parser.parse_args()
    for name, lo, hi in (("bpm", 20, 240), ("cycles", 1, 200), ("strokes", 2, 64), ("speed", 0.05, 2.0),
                         ("fps", 24, 1000), ("other", 0.0, 1.0), ("targets", 2, 10)):
        value = getattr(args, name, None)
        if value is not None and not (lo <= value <= hi):
            parser.error(f"--{name} must be between {lo} and {hi}")
    args.run(args)


if __name__ == "__main__":
    main()
