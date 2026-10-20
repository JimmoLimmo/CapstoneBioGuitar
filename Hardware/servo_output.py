"""Drive the right hand's real servos from the shared strum choreography.

    python Hardware/servo_output.py --dry-run          # no hardware: prints targets + servo demand
    python Hardware/servo_output.py --wrist-only --speed 0.25   # first wrist bench run: quarter speed
    python Hardware/servo_output.py                    # full hand, real speed

Every tick (50 Hz):
    strum_motion.wrist_at(t)          -> (deviation_deg, flexion_deg)
    wrist_kinematics.servo_angles()   -> 4 spool angles, one per wrist servo
    per-servo calibration below       -> 4 PCA9685 pulses
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO_ROOT, "Sim"))
sys.path.insert(0, HERE)

from strum_motion import (  # noqa: E402  (path insert must come first)
    GRIP_CURL,
    WRIST_PRO_BASE_DEG,
    build_schedule,
    smooth,
    wrist_at,
)
import wrist_kinematics as wk  # noqa: E402

# --- wrist servos (TODO: fill in on the bench with calibrate_servo.py) -------
# The marionette wrist has four MG90S servos, two per axis. Names say which
# way that servo PULLS the wrist when it reels in (see wrist_kinematics.py).
#   channel        PCA9685 channel 0-15.
#   neutral_pulse  pulse with the wrist sitting straight (0 deg / 0 deg) and
#                  this servo's tendon just snug -- not slack, not straining.
#   reel_in_sign   +1 if a LARGER pulse winds the string in, -1 if a smaller
#                  one does. Depends on which way the servo is mounted.
#   min/max_pulse  hard limits for this servo: jog until the wrist reaches its
#                  mechanical stop, then back off a little. The defaults are
#                  deliberately narrow.
WRIST_SERVOS = {
    "dev_pos":  {"channel": None, "neutral_pulse": None, "reel_in_sign": +1, "min_pulse": 2500, "max_pulse": 7300},
    "dev_neg":  {"channel": None, "neutral_pulse": None, "reel_in_sign": +1, "min_pulse": 2500, "max_pulse": 7300},
    "flex_pos": {"channel": None, "neutral_pulse": None, "reel_in_sign": +1, "min_pulse": 2500, "max_pulse": 7300},
    "flex_neg": {"channel": None, "neutral_pulse": None, "reel_in_sign": +1, "min_pulse": 2500, "max_pulse": 7300},
}

# MG90S: 500-2500 us over ~180 deg (same range as the CDR servo test bench).
# At 50 Hz one duty-cycle count is 20000/65535 us, so 180 deg = 1638..8191.
# Check it once: command neutral +/- 1638 counts and confirm the horn turns
# about 45 deg each way. Adjust this if it doesn't.
PULSE_PER_SERVO_DEG = (8191 - 1638) / 180.0

# --- non-wrist channels ------------------------------------------------------
OTHER_CHANNELS = {
    "grip": {  # pick-grip pose, set once at start; becomes 10 finger servos later
        "channel": None,
        "curl_lo": 0.0, "pulse_lo": 1638,    # TODO: open hand
        "curl_hi": 1.0, "pulse_hi": 8191,    # TODO: full fist
    },
    "wrist_PRO": {  # forearm rotation -- not actuated in the CDR design; leave None
        "channel": None,
        "deg_lo": -140.0, "pulse_lo": 1638,
        "deg_hi": 60.0, "pulse_hi": 8191,
    },
}

PULSE_ABS_MIN = 1638   # last-resort clamp on every channel, on top of per-servo limits
PULSE_ABS_MAX = 8191

I2C_BUS = 7            # same as last year's left hand -- reconfirm on the shared Jetson
I2C_ADDRESS = 0x40
PWM_FREQUENCY = 50
CONTROL_HZ = 50        # one update per PWM frame; faster wouldn't reach the servos
EASE_SECONDS = 0.5     # time to ease into the ready pose at start and back to neutral at stop


def clamp(value, lo, hi):
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, value))


# --- conversion --------------------------------------------------------------

def servo_pulse(name, spool_deg):
    """Spool angle (deg, + = reel in) -> pulse for one wrist servo, clamped
    to its own limits and then to the absolute range. Returns (pulse, clamped?)."""
    cfg = WRIST_SERVOS[name]
    raw = cfg["neutral_pulse"] + cfg["reel_in_sign"] * spool_deg * PULSE_PER_SERVO_DEG
    pulse = clamp(clamp(raw, cfg["min_pulse"], cfg["max_pulse"]), PULSE_ABS_MIN, PULSE_ABS_MAX)
    return int(round(pulse)), abs(pulse - raw) > 1


def wrist_pulses(dev_deg, flex_deg):
    """(deviation, flexion) -> ({servo_name: pulse}, [servos that hit a limit])."""
    pulses, clamped = {}, []
    for name, deg in wk.servo_angles(dev_deg, flex_deg).items():
        pulses[name], hit = servo_pulse(name, deg)
        if hit:
            clamped.append(name)
    return pulses, clamped


def lerp_pulse(value, in_lo, in_hi, pulse_lo, pulse_hi):
    span = in_hi - in_lo
    t = clamp(0.0 if span == 0 else (value - in_lo) / span, 0.0, 1.0)
    return int(clamp(pulse_lo + (pulse_hi - pulse_lo) * t, PULSE_ABS_MIN, PULSE_ABS_MAX))


def is_calibrated():
    return all(c["channel"] is not None and c["neutral_pulse"] is not None for c in WRIST_SERVOS.values())


# --- output ------------------------------------------------------------------

class ServoBus:
    """PCA9685, or a stand-in that only remembers what it was told (--dry-run)."""

    def __init__(self, dry_run):
        self.dry_run = dry_run
        self.pca = None
        self.last = {}
        if not dry_run:
            from adafruit_extended_bus import ExtendedI2C as I2C
            from adafruit_pca9685 import PCA9685

            self.pca = PCA9685(I2C(I2C_BUS), address=I2C_ADDRESS)
            self.pca.frequency = PWM_FREQUENCY

    def set(self, channel, pulse):
        if channel is None:
            return
        self.last[channel] = pulse
        if not self.dry_run:
            self.pca.channels[channel].duty_cycle = pulse

    def release(self):
        """Stop pulsing every channel we used -- the servos go limp."""
        for channel in list(self.last):
            if not self.dry_run:
                self.pca.channels[channel].duty_cycle = 0
        self.last.clear()


def write_wrist(bus, dev_deg, flex_deg):
    pulses, _ = wrist_pulses(dev_deg, flex_deg)
    for name, pulse in pulses.items():
        bus.set(WRIST_SERVOS[name]["channel"], pulse)
    return pulses


def ease_wrist(bus, start, end, seconds=EASE_SECONDS):
    """Move the wrist between two (dev, flex) poses along a smoothstep, going
    through the kinematics every step so both strings of a pair stay matched."""
    steps = max(1, int(seconds * CONTROL_HZ))
    for k in range(1, steps + 1):
        s = smooth(k / steps)
        write_wrist(bus, start[0] + (end[0] - start[0]) * s, start[1] + (end[1] - start[1]) * s)
        if not bus.dry_run:
            time.sleep(1.0 / CONTROL_HZ)


# --- checks ------------------------------------------------------------------

def missing_calibration(wrist_only):
    problems = []
    for name, cfg in WRIST_SERVOS.items():
        if cfg["channel"] is None:
            problems.append(f"{name}: no channel")
        if cfg["neutral_pulse"] is None:
            problems.append(f"{name}: no neutral_pulse")
    channels = [c["channel"] for c in WRIST_SERVOS.values() if c["channel"] is not None]
    if not wrist_only:
        if OTHER_CHANNELS["grip"]["channel"] is None:
            problems.append("grip: no channel (or run with --wrist-only)")
        channels += [c["channel"] for c in OTHER_CHANNELS.values() if c["channel"] is not None]
    dupes = sorted({c for c in channels if channels.count(c) > 1})
    if dupes:
        problems.append(f"channel(s) {dupes} assigned to more than one servo")
    return problems


def demand_report(schedule, speed):
    """What the strum asks of each wrist servo, printed before anything moves."""
    label = f" at --speed {speed}" if speed != 1.0 else ""
    print(f"[servo] wrist servo demand for this schedule{label}:")
    worst = 0.0
    for name, (lo, hi, peak) in wk.profile(schedule, wrist_at).items():
        peak *= speed
        worst = max(worst, peak)
        print(f"          {name:8} spool travel {lo:+6.1f}..{hi:+6.1f} deg   peak {peak:5.0f} deg/s")
    if worst > wk.MG90S_RATED_DEG_PER_S:
        print(f"[servo]   peak {worst:.0f} deg/s is over the MG90S's ~{wk.MG90S_RATED_DEG_PER_S:.0f} deg/s "
              "no-load rating -- expect the wrist to lag the strike.")
    if is_calibrated():
        hits = set()
        for onset, _, strike, recover in schedule:
            for k in range(21):
                dev, flex, _ = wrist_at(schedule, onset + (strike + recover) * k / 20)
                hits.update(wrist_pulses(dev, flex)[1])
        if hits:
            print(f"[servo]   WARNING: {sorted(hits)} hit min/max_pulse during the strum. That pair's "
                  "strings stop matching there -- widen the limits or shrink the sweep.")


# --- main --------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true",
                        help="print targets instead of writing to the PCA9685; needs no I2C/adafruit libs")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="playback speed; 0.25 = quarter speed for the first hardware runs")
    parser.add_argument("--wrist-only", action="store_true",
                        help="drive only the four wrist servos (bench test before the fingers are wired)")
    parser.add_argument("--hold", action="store_true",
                        help="on stop, park the wrist at neutral and keep holding it instead of going limp")
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed must be > 0")

    problems = missing_calibration(args.wrist_only)
    if problems and not args.dry_run:
        print("[servo] not calibrated yet -- refusing to move anything:")
        for p in problems:
            print(f"          {p}")
        print("[servo]   see Hardware/README.md, or pass --dry-run to check timing only.")
        return
    calibrated = is_calibrated()
    if not calibrated:
        print("[servo] (dry run) wrist calibration not filled in -- printing spool angles instead of pulses.")

    schedule, source = build_schedule()
    end_time = schedule[-1][0] + schedule[-1][2] + schedule[-1][3] + 0.5
    print(f"[servo] {len(schedule)} strokes from {source}{' [DRY RUN]' if args.dry_run else ''}")
    demand_report(schedule, args.speed)

    bus = ServoBus(dry_run=args.dry_run)
    dev, flex = 0.0, 0.0

    try:
        if not args.wrist_only:
            grip = OTHER_CHANNELS["grip"]
            bus.set(grip["channel"], lerp_pulse(GRIP_CURL, grip["curl_lo"], grip["curl_hi"],
                                                grip["pulse_lo"], grip["pulse_hi"]))
            pro = OTHER_CHANNELS["wrist_PRO"]
            bus.set(pro["channel"], lerp_pulse(WRIST_PRO_BASE_DEG, pro["deg_lo"], pro["deg_hi"],
                                               pro["pulse_lo"], pro["pulse_hi"]))

        # Start from straight and ease into the first stroke's ready pose,
        # so the first real command is never a jump.
        ready = wrist_at(schedule, -1.0)[:2]
        if calibrated:
            ease_wrist(bus, (0.0, 0.0), ready)
        dev, flex = ready

        period = 1.0 / CONTROL_HZ
        start = time.perf_counter()
        next_tick = 0.0
        reported = -1
        while True:
            song_t = (time.perf_counter() - start) * args.speed
            if song_t > end_time:
                break

            dev, flex, index = wrist_at(schedule, song_t)
            if calibrated:
                pulses = write_wrist(bus, dev, flex)

            if args.dry_run and index > reported:
                direction = schedule[index][1] if index >= 0 else "-"
                if calibrated:
                    detail = " ".join(f"{n}={p}" for n, p in pulses.items())
                else:
                    detail = " ".join(f"{n}={a:+5.1f}d" for n, a in wk.servo_angles(dev, flex).items())
                print(f"t={song_t:6.2f}s stroke#{index:<3} {direction} dev={dev:6.1f} flex={flex:5.1f} | {detail}")
                reported = index

            next_tick += period
            sleep_for = next_tick - (time.perf_counter() - start)
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter() - start  # fell behind: resync, no catch-up burst
    except KeyboardInterrupt:
        print("\n[servo] interrupted.")
    finally:
        # CDR "Stopping and Fault Handling": on ANY exit -- end of song, Ctrl-C,
        # or an exception -- ease back to neutral, then release the outputs.
        # The hardware e-stop is still the real safety; this is a courtesy.
        if calibrated:
            try:
                ease_wrist(bus, (dev, flex), (0.0, 0.0))
            except Exception as e:  # never let parking hide the original error
                print(f"[servo] could not park the wrist ({e})")
        if not args.hold:
            bus.release()
        print(f"[servo] stopped -- wrist {'parked and holding' if args.hold else 'parked and released'}.")


if __name__ == "__main__":
    main()
