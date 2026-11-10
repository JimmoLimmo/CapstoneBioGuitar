"""Finger servos: curl (0..1) -> PCA9685 pulse for the ten MG90S finger servos.

open_pulse = finger straight, tendon just snug. closed_pulse = curled, just touching its stop.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "Sim"))

from finger_motion import FINGERS, SERVO_NAMES, clamp_pose  # noqa: E402

PULSE_ABS_MIN = 1638   # same last-resort clamp as servo_output.py (500-2500 us)
PULSE_ABS_MAX = 8191

# TODO: fill in on the bench (calibrate_servo.py <channel> lv)
FINGER_SERVOS = {
    "thumb_base":  {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "thumb_tip":   {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "index_base":  {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "index_tip":   {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "middle_base": {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "middle_tip":  {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "ring_base":   {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "ring_tip":    {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "pinky_base":  {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
    "pinky_tip":   {"board": "lv", "channel": None, "open_pulse": None, "closed_pulse": None},
}
assert set(FINGER_SERVOS) == set(SERVO_NAMES)


def servos_for(fingers):
    """Servo names belonging to the given fingers, e.g. ("index",) -> index_base, index_tip."""
    unknown = set(fingers) - set(FINGERS)
    if unknown:
        raise ValueError(f"unknown finger(s) {sorted(unknown)}; use {FINGERS}")
    return [n for n in SERVO_NAMES if n.split("_")[0] in fingers]


def is_calibrated(name):
    c = FINGER_SERVOS[name]
    return None not in (c["channel"], c["open_pulse"], c["closed_pulse"])


def missing_calibration(fingers=FINGERS):
    problems = []
    for name in servos_for(fingers):
        c = FINGER_SERVOS[name]
        for key in ("channel", "open_pulse", "closed_pulse"):
            if c[key] is None:
                problems.append(f"{name}: no {key}")
        for key in ("open_pulse", "closed_pulse"):
            if c[key] is not None and not (PULSE_ABS_MIN <= c[key] <= PULSE_ABS_MAX):
                problems.append(f"{name}: {key} {c[key]} outside {PULSE_ABS_MIN}-{PULSE_ABS_MAX}")
        if c["open_pulse"] is not None and c["open_pulse"] == c["closed_pulse"]:
            problems.append(f"{name}: open_pulse == closed_pulse")
    return problems


def channels(fingers=FINGERS):
    """[(board, channel)] used by these fingers (calibrated channels only)."""
    return [(FINGER_SERVOS[n]["board"], FINGER_SERVOS[n]["channel"])
            for n in servos_for(fingers) if FINGER_SERVOS[n]["channel"] is not None]


def finger_pulse(name, curl):
    c = FINGER_SERVOS[name]
    curl = max(0.0, min(1.0, float(curl)))
    pulse = c["open_pulse"] + (c["closed_pulse"] - c["open_pulse"]) * curl
    return int(round(max(PULSE_ABS_MIN, min(PULSE_ABS_MAX, pulse))))


def finger_pulses(pose, fingers=FINGERS):
    """{servo name: pulse} for the calibrated servos of `fingers` in `pose`."""
    pose = clamp_pose(pose)
    return {n: finger_pulse(n, pose.get(n, 0.0)) for n in servos_for(fingers) if is_calibrated(n)}


def write_fingers(bus, pose, fingers=FINGERS):
    pulses = finger_pulses(pose, fingers)
    for name, pulse in pulses.items():
        c = FINGER_SERVOS[name]
        bus.set(c["board"], c["channel"], pulse)
    return pulses
