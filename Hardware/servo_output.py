"""Drive the right hand's real servos from the shared strum choreography.

    python Hardware/servo_output.py --dry-run          # no hardware: prints targets + servo demand
    python Hardware/servo_output.py --wrist-only --speed 0.25   # first wrist bench run: quarter speed
    python Hardware/servo_output.py                    # full hand, real speed
    python Hardware/servo_output.py --wrist-only --sim-ip 192.168.1.20   # also stream to mvmnt_demo.py --follow

Servo power goes through the electrical box (see power_box.py): the script
starts with both relays OFF, asks you to press Enter to arm, then you press
the box's RESET button. The E-stop drops everything to SAFE at any time, and
the relays turn OFF on every exit. Run on the Jetson with sudo (GPIO access).

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
from motion_link import DEFAULT_PORT, PoseSender  # noqa: E402
import wrist_kinematics as wk  # noqa: E402
from power_box import EStop, PowerBox  # noqa: E402

# --- wrist servos (TODO: fill in on the bench with calibrate_servo.py) -------
# The marionette wrist has four Hiwonder HPS-0618SG servos (6-8.4 V, 2.5 A
# stall) on the 8.4 V board, two per axis. Names say which
# way that servo PULLS the wrist when it reels in (see wrist_kinematics.py).
#   board          which PCA9685: "hv" = 8.4 V board (0x42), "lv" = 5 V board (0x41).
#   channel        PCA9685 channel 0-15 on that board.
#   neutral_pulse  pulse with the wrist sitting straight (0 deg / 0 deg) and
#                  this servo's tendon just snug -- not slack, not straining.
#   reel_in_sign   +1 if a LARGER pulse winds the string in, -1 if a smaller
#                  one does. Depends on which way the servo is mounted.
#   min/max_pulse  hard limits for this servo: jog until the wrist reaches its
#                  mechanical stop, then back off a little. The defaults are
#                  deliberately narrow.
WRIST_SERVOS = {
    "dev_pos":  {"board": "hv", "channel": None, "neutral_pulse": None, "reel_in_sign": +1, "min_pulse": 2500, "max_pulse": 7300},
    "dev_neg":  {"board": "hv", "channel": None, "neutral_pulse": None, "reel_in_sign": +1, "min_pulse": 2500, "max_pulse": 7300},
    "flex_pos": {"board": "hv", "channel": None, "neutral_pulse": None, "reel_in_sign": +1, "min_pulse": 2500, "max_pulse": 7300},
    "flex_neg": {"board": "hv", "channel": None, "neutral_pulse": None, "reel_in_sign": +1, "min_pulse": 2500, "max_pulse": 7300},
}

# 500-2500 us over ~180 deg (the CDR servo test bench range; check the HPS-0618SG
# datasheet's pulse range -- if it differs, change this).
# At 50 Hz one duty-cycle count is 20000/65535 us, so 180 deg = 1638..8191.
# Check it once: command neutral +/- 1638 counts and confirm the horn turns
# about 45 deg each way. Adjust this if it doesn't.
PULSE_PER_SERVO_DEG = (8191 - 1638) / 180.0

# --- non-wrist channels ------------------------------------------------------
OTHER_CHANNELS = {
    "grip": {  # pick-grip pose, set once at start; becomes 10 finger servos later
        "board": "lv", "channel": None,   # TODO: confirm which board the fingers end up on
        "curl_lo": 0.0, "pulse_lo": 1638,    # TODO: open hand
        "curl_hi": 1.0, "pulse_hi": 8191,    # TODO: full fist
    },
    "wrist_PRO": {  # forearm rotation -- not actuated in the CDR design; leave None
        "board": "lv", "channel": None,
        "deg_lo": -140.0, "pulse_lo": 1638,
        "deg_hi": 60.0, "pulse_hi": 8191,
    },
}

PULSE_ABS_MIN = 1638   # last-resort clamp on every channel, on top of per-servo limits
PULSE_ABS_MAX = 8191

I2C_BUS = 7            # pins 3/5 on the Orin Nano -- confirm with: sudo i2cdetect -y -r 7
BOARDS = {
    "lv": 0x41,        # PCA9685 #1, 5 V servo group (relay K1)
    # TEMPORARY: the address pads aren't bridged yet, so both boards are still at
    # the default 0x40 and BOTH get every command. Change back to 0x42 once the
    # 8.4 V board's A1 pad is bridged (and the 5 V board's A0 for 0x41).
    "hv": 0x40,        # PCA9685 #2, 8.4 V servo group (relay K2) -- the wrist
}
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
    """Both PCA9685 boards, or a stand-in that only remembers what it was told
    (--dry-run). Channels are addressed as (board, channel), board = "lv"/"hv"."""

    def __init__(self, dry_run, boards=("lv", "hv")):
        self.dry_run = dry_run
        self.pca = {}
        self.last = {}
        self.sim = None    # optional PoseSender; set from --sim-ip
        if not dry_run:
            from adafruit_pca9685 import PCA9685
            from i2c_bus import open_bus

            i2c = open_bus(I2C_BUS)
            for name in boards:
                try:
                    pca = PCA9685(i2c, address=BOARDS[name])
                except ValueError as e:
                    raise SystemExit(f"[servo] no PCA9685 '{name}' at {hex(BOARDS[name])} on bus {I2C_BUS} ({e}). "
                                     f"Check: sudo i2cdetect -y -r {I2C_BUS}")
                pca.frequency = PWM_FREQUENCY
                self.pca[name] = pca

    def set(self, board, channel, pulse):
        if channel is None:
            return
        self.last[(board, channel)] = pulse
        if not self.dry_run:
            self.pca[board].channels[channel].duty_cycle = pulse

    def publish(self, dev_deg, flex_deg):
        """Tell the sim, if one is listening, what the wrist was just commanded to do."""
        if self.sim:
            self.sim.send(dev_deg, flex_deg)

    def release(self):
        """Stop pulsing every channel we used -- the servos go limp."""
        for board, channel in list(self.last):
            if not self.dry_run:
                self.pca[board].channels[channel].duty_cycle = 0
        self.last.clear()


def write_wrist(bus, dev_deg, flex_deg):
    pulses, _ = wrist_pulses(dev_deg, flex_deg)
    for name, pulse in pulses.items():
        cfg = WRIST_SERVOS[name]
        bus.set(cfg["board"], cfg["channel"], pulse)
    return pulses


def ease_wrist(bus, start, end, seconds=EASE_SECONDS, power=None):
    """Move the wrist between two (dev, flex) poses along a smoothstep, going
    through the kinematics every step so both strings of a pair stay matched."""
    steps = max(1, int(seconds * CONTROL_HZ))
    for k in range(1, steps + 1):
        s = smooth(k / steps)
        dev, flex = start[0] + (end[0] - start[0]) * s, start[1] + (end[1] - start[1]) * s
        if power is not None:
            power.check()
        write_wrist(bus, dev, flex)
        bus.publish(dev, flex)
        if not bus.dry_run:
            time.sleep(1.0 / CONTROL_HZ)


# --- checks ------------------------------------------------------------------

def missing_calibration(wrist_only):
    problems = []
    for name, cfg in WRIST_SERVOS.items():
        if cfg.get("board") not in BOARDS:
            problems.append(f"{name}: board must be one of {sorted(BOARDS)}")
        if cfg["channel"] is None:
            problems.append(f"{name}: no channel")
        if cfg["neutral_pulse"] is None:
            problems.append(f"{name}: no neutral_pulse")
    channels = [(c["board"], c["channel"]) for c in WRIST_SERVOS.values() if c["channel"] is not None]
    if not wrist_only:
        if OTHER_CHANNELS["grip"]["channel"] is None:
            problems.append("grip: no channel (or run with --wrist-only)")
        channels += [(c["board"], c["channel"]) for c in OTHER_CHANNELS.values() if c["channel"] is not None]
    dupes = sorted({c for c in channels if channels.count(c) > 1})
    if dupes:
        problems.append(f"(board, channel) {dupes} assigned to more than one servo")
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
    if worst > wk.SERVO_RATED_DEG_PER_S:
        print(f"[servo]   peak {worst:.0f} deg/s is over the {wk.SERVO_NAME}'s ~{wk.SERVO_RATED_DEG_PER_S:.0f} deg/s "
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
    parser.add_argument("--sim-ip",
                        help="stream wrist commands to the computer running mvmnt_demo.py --follow "
                             "(127.0.0.1 if it's this computer)")
    parser.add_argument("--sim-port", type=int, default=DEFAULT_PORT, help="UDP port for --sim-ip")
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

    wrist_boards = {c["board"] for c in WRIST_SERVOS.values()}
    boards = tuple(sorted(wrist_boards if args.wrist_only else set(BOARDS)))
    bus = ServoBus(dry_run=args.dry_run, boards=boards)
    power = PowerBox(dry_run=args.dry_run, on_safe=bus.release)
    if args.sim_ip:
        try:
            bus.sim = PoseSender(args.sim_ip, args.sim_port)
            print(f"[servo] streaming wrist commands to {bus.sim.addr[0]}:{args.sim_port}")
        except OSError as e:
            print(f"[servo] can't reach sim address {args.sim_ip} ({e}); continuing without the sim.")
    dev, flex = 0.0, 0.0

    def write_home():
        """Safe HOME pulses on every channel we drive, written before power is allowed."""
        if calibrated:
            write_wrist(bus, 0.0, 0.0)
        if not args.wrist_only:
            grip = OTHER_CHANNELS["grip"]
            bus.set(grip["board"], grip["channel"],
                    lerp_pulse(GRIP_CURL, grip["curl_lo"], grip["curl_hi"], grip["pulse_lo"], grip["pulse_hi"]))
            pro = OTHER_CHANNELS["wrist_PRO"]
            bus.set(pro["board"], pro["channel"],
                    lerp_pulse(WRIST_PRO_BASE_DEG, pro["deg_lo"], pro["deg_hi"], pro["pulse_lo"], pro["pulse_hi"]))

    try:
        power.arm(write_home)

        # Start from straight and ease into the first stroke's ready pose,
        # so the first real command is never a jump.
        ready = wrist_at(schedule, -1.0)[:2]
        if calibrated:
            ease_wrist(bus, (0.0, 0.0), ready, power=power)
        dev, flex = ready
        bus.publish(dev, flex)

        period = 1.0 / CONTROL_HZ
        start = time.perf_counter()
        next_tick = 0.0
        reported = -1
        while True:
            song_t = (time.perf_counter() - start) * args.speed
            if song_t > end_time:
                break

            power.check()
            dev, flex, index = wrist_at(schedule, song_t)
            if calibrated:
                pulses = write_wrist(bus, dev, flex)
            bus.publish(dev, flex)

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
    except EStop:
        print("[servo] stopped by the E-stop.")
    except KeyboardInterrupt:
        print("\n[servo] interrupted.")
    finally:
        # On a normal finish, ease back to neutral while power is still on.
        # On an E-stop or Ctrl-C the relays are already off, so skip straight to
        # the shutdown. Every path ends with relays OFF and PWM off.
        if calibrated and power.state == "ARMED" and not power.tripped.is_set() and not power.closed:
            try:
                ease_wrist(bus, (dev, flex), (0.0, 0.0), power=power)
            except Exception as e:  # never let parking hide the original error
                print(f"[servo] could not park the wrist ({e})")
        power.close()
        bus.release()
        print("[servo] stopped -- relays off, servos released.")


if __name__ == "__main__":
    main()
