"""Drive the real right-hand servos from the same choreography as the sim.

Sim/mvmnt_demo.py and this file both import Sim/strum_motion.py and call the
exact same wrist_at(schedule, now) every tick. The sim feeds that (yaw_deg,
flex_deg) into PyBullet's setJointMotorControlArray; this file feeds it into
a PCA9685 over I2C, the same driver + bus + address last year's left hand
used (see CapstoneBioGuitar-main/Imitation/mimic.py, servozero.py). Nothing
about the schedule, timing, or wrist_at() math changes between the two --
only where the numbers end up.

*** Everything in CHANNEL_MAP below is a placeholder. *** Fill in the real
PCA9685 channel and the measured pulse bounds for each servo once the right
hand's wrist is wired up -- see calibrate_servo.py in this folder for a way
to find those numbers by jogging one channel at a time.

Run from the repo root, same convention as the sim:
    python Hardware/servo_output.py            # writes to real servos
    python Hardware/servo_output.py --dry-run  # prints targets, touches no hardware

--dry-run needs nothing beyond strum_motion.py's own dependencies (no
adafruit libs, no I2C bus) -- exactly the "sim-only" gotcha called out in
SIMULATION_SETUP.md, mirrored here for the hardware side: you can develop
and sanity-check the timing on a laptop with no Jetson attached.
"""

import argparse
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "Sim"))

from strum_motion import (  # noqa: E402  (path insert must come first)
    GRIP_CURL,
    WRIST_PRO_BASE_DEG,
    build_schedule,
    wrist_at,
)

# --- hardware map (TODO: fill in from the real build) -----------------------
# channel: PCA9685 channel number (0-15).
# deg_lo/deg_hi: the joint angle, in the SAME convention as strum_motion.py's
#   yaw_deg/flex_deg, at two known servo positions.
# pulse_lo/pulse_hi: the duty_cycle values (adafruit_pca9685 16-bit units,
#   valid range ~1638-8191 for a standard 50 Hz hobby servo) measured at
#   those same two positions with calibrate_servo.py. They do NOT need to be
#   pulse_lo < pulse_hi -- if the servo is mounted backwards relative to the
#   joint's positive direction, pulse_lo can be the larger number and the
#   interpolation below still works.
# Set channel to None to leave a DOF unactuated for now (e.g. if only yaw is
# wired up first).
CHANNEL_MAP = {
    "wrist_UDEV": {  # strum yaw -- moves every tick
        "channel": None,       # TODO
        "deg_lo": -20.0, "pulse_lo": 1638,   # TODO: measure with calibrate_servo.py
        "deg_hi": 25.0, "pulse_hi": 8191,    # TODO: measure with calibrate_servo.py
    },
    "wrist_FLEX": {  # strike clearance lift -- moves every tick
        "channel": None,       # TODO
        "deg_lo": 0.0, "pulse_lo": 1638,     # TODO: measure with calibrate_servo.py
        "deg_hi": 20.0, "pulse_hi": 8191,    # TODO: measure with calibrate_servo.py
    },
    "wrist_PRO": {  # held at WRIST_PRO_BASE_DEG, not yet driven by the strum -- optional for now
        "channel": None,       # TODO, or leave None to skip entirely
        "deg_lo": -140.0, "pulse_lo": 1638,  # TODO: measure with calibrate_servo.py
        "deg_hi": 60.0, "pulse_hi": 8191,    # TODO: measure with calibrate_servo.py
    },
    "grip": {  # single tendon/actuator closing the fist around the pick, set once at start
        "channel": None,       # TODO
        "curl_lo": 0.0, "pulse_lo": 1638,    # TODO: measure with calibrate_servo.py (open hand)
        "curl_hi": 1.0, "pulse_hi": 8191,    # TODO: measure with calibrate_servo.py (full fist)
    },
}

# Hard software ceiling regardless of the calibration numbers above -- last
# year's team found real servos should not be driven all the way to the
# PCA9685's raw 1638-8191 range in practice (see Testing/servotest.py's "DO
# NOT EXCEED 7000" note on one of their channels). Re-confirm a safe ceiling
# per servo during calibration and tighten this if needed; this is only a
# last-resort clamp, not a substitute for real per-channel bounds.
PULSE_ABS_MIN = 1638
PULSE_ABS_MAX = 8191

I2C_BUS = 7          # matches last year's left hand -- confirm this is still
I2C_ADDRESS = 0x40   # correct once the second Jetson's wiring is settled
PWM_FREQUENCY = 50

CONTROL_HZ = 50      # how often to push new targets; servos/PCA9685 hold
                     # position between updates, so this doesn't need to
                     # match the sim's 240 Hz physics step at all


def clamp(value, lo, hi):
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, value))


def lerp_pulse(value, in_lo, in_hi, pulse_lo, pulse_hi):
    span = in_hi - in_lo
    t = 0.0 if span == 0 else (value - in_lo) / span
    t = clamp(t, 0.0, 1.0)
    pulse = pulse_lo + (pulse_hi - pulse_lo) * t
    return int(clamp(pulse, PULSE_ABS_MIN, PULSE_ABS_MAX))


def deg_to_pulse(joint, deg):
    cfg = CHANNEL_MAP[joint]
    return lerp_pulse(deg, cfg["deg_lo"], cfg["deg_hi"], cfg["pulse_lo"], cfg["pulse_hi"])


def curl_to_pulse(cfg, curl):
    return lerp_pulse(curl, cfg["curl_lo"], cfg["curl_hi"], cfg["pulse_lo"], cfg["pulse_hi"])


class ServoBus:
    """Thin wrapper so the run loop below doesn't care whether it's actually
    talking to a PCA9685 or just printing (--dry-run)."""

    def __init__(self, dry_run):
        self.dry_run = dry_run
        self.pca = None
        if not dry_run:
            from adafruit_extended_bus import ExtendedI2C as I2C
            from adafruit_pca9685 import PCA9685

            i2c = I2C(I2C_BUS)
            self.pca = PCA9685(i2c, address=I2C_ADDRESS)
            self.pca.frequency = PWM_FREQUENCY

    def set_channel(self, channel, pulse):
        if channel is None:
            return
        if self.dry_run:
            return
        self.pca.channels[channel].duty_cycle = pulse


def missing_channels():
    return [name for name, cfg in CHANNEL_MAP.items() if cfg["channel"] is None]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true",
                        help="print targets instead of writing to the PCA9685; no I2C/adafruit libs required")
    args = parser.parse_args()

    missing = missing_channels()
    if missing and not args.dry_run:
        print(f"[servo] CHANNEL_MAP is missing channel numbers for: {', '.join(missing)}")
        print("[servo]   fill these in from the real build (see calibrate_servo.py), or pass --dry-run to test timing only.")
        return
    if missing:
        print(f"[servo] (dry run) CHANNEL_MAP still has placeholders for: {', '.join(missing)} -- fine for now.")

    bus = ServoBus(dry_run=args.dry_run)

    schedule, source = build_schedule()
    end_time = schedule[-1][0] + schedule[-1][2] + schedule[-1][3] + 0.5
    print(f"[servo] {len(schedule)} strokes from {source}{' [DRY RUN]' if args.dry_run else ''}")

    # Grip and wrist_PRO aren't part of the per-tick strum -- set them once.
    grip_cfg = CHANNEL_MAP["grip"]
    bus.set_channel(grip_cfg["channel"], curl_to_pulse(grip_cfg, GRIP_CURL))
    pro_cfg = CHANNEL_MAP["wrist_PRO"]
    if pro_cfg["channel"] is not None:
        bus.set_channel(pro_cfg["channel"], deg_to_pulse("wrist_PRO", WRIST_PRO_BASE_DEG))

    period = 1.0 / CONTROL_HZ
    start = time.perf_counter()
    reported = -1
    next_tick = start

    try:
        while True:
            now = time.perf_counter() - start
            if now > end_time:
                break

            yaw, flex, index = wrist_at(schedule, now)
            bus.set_channel(CHANNEL_MAP["wrist_UDEV"]["channel"], deg_to_pulse("wrist_UDEV", yaw))
            bus.set_channel(CHANNEL_MAP["wrist_FLEX"]["channel"], deg_to_pulse("wrist_FLEX", flex))

            if args.dry_run and index > reported:
                onset, direction, _, _ = schedule[index] if 0 <= index < len(schedule) else (now, "?", 0, 0)
                print(f"t={now:6.2f}s stroke#{index:<3} dir={direction} yaw={yaw:6.1f} flex={flex:5.1f}")
                reported = index

            next_tick += period
            sleep_for = next_tick - (time.perf_counter() - start)
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter() - start  # fell behind; resync instead of spiraling
    except KeyboardInterrupt:
        print("\n[servo] stopped (servos hold their last commanded position).")

    print(f"[servo] done. {len(schedule)} strokes.")


if __name__ == "__main__":
    main()
