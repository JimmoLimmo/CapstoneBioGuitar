"""Bench values kept out of the source, in Hardware/calibration.json:

    {"wrist":   {"dev_pos":   {"channel": 10, "neutral_pulse": 4900, "reel_in_sign": 1}},
     "fingers": {"index_tip": {"channel": 3, "open_pulse": 3000, "closed_pulse": 6200}},
     "poses":   {"pick_grip": {"thumb_base": 0.7, "thumb_tip": 0.45}}}

The file is optional and git-ignored. Whatever it doesn't set keeps the default
in servo_output.py / fingers.py / finger_motion.py. calibrate_servo.py and
hand_console.py write it; `calibrate_servo.py --show` prints what is in use.
Set HAND_CALIBRATION to point at a different file.
"""

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.join(HERE, "calibration.json")
PULSE_MIN, PULSE_MAX = 1638, 8191
BOARD_NAMES = ("lv", "hv")


def path():
    return os.environ.get("HAND_CALIBRATION") or DEFAULT_PATH


def load():
    try:
        with open(path()) as f:
            data = json.load(f)
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as e:
        raise SystemExit(f"[calibration] can't read {path()} ({e}). Fix the file or move it away.")
    if not isinstance(data, dict) or not all(isinstance(v, dict) for v in data.values()):
        raise SystemExit(f"[calibration] {path()} isn't laid out as sections of servos. Fix the file or move it away.")
    return data


def check(key, value):
    """ValueError if `value` can't be a servo's `key`."""
    whole = type(value) is int
    if key == "board":
        ok = value in BOARD_NAMES
    elif key == "channel":
        ok = value is None or (whole and 0 <= value <= 15)
    elif key == "reel_in_sign":
        ok = whole and value in (1, -1)
    elif key.endswith("_pulse"):
        ok = value is None or (whole and PULSE_MIN <= value <= PULSE_MAX)
    else:
        ok = False
    if not ok:
        raise ValueError(f"{key} = {value!r} is not valid")


def apply(section, table):
    """Overwrite the defaults in `table` ({servo: {key: value}}) with what the file holds."""
    for name, values in load().get(section, {}).items():
        if name not in table or not isinstance(values, dict):
            print(f"[calibration] {section}.{name} in {path()} is not a servo here; ignored.")
            continue
        for key, value in values.items():
            if key not in table[name]:
                print(f"[calibration] {section}.{name}.{key} in {path()} is not a setting here; ignored.")
                continue
            try:
                check(key, value)
            except ValueError as e:
                raise SystemExit(f"[calibration] {path()}: {section}.{name}: {e}")
            table[name][key] = value


def apply_poses(finger_motion):
    for name, pose in load().get("poses", {}).items():
        try:
            finger_motion.set_pose(name, pose)
        except (ValueError, TypeError, AttributeError) as e:
            raise SystemExit(f"[calibration] {path()}: pose {name!r}: {e}")


def save(section, name, values):
    """Merge `values` into section.name and write the file. Returns the entry as saved."""
    if section != "poses":
        for key, value in values.items():
            check(key, value)
    data = load()
    entry = data.setdefault(section, {}).setdefault(name, {})
    entry.update(values)
    text = json.dumps(data, indent=2, sort_keys=True) + "\n"
    with open(path(), "w") as f:
        f.write(text)
    hand_back(path())
    return dict(entry)


def hand_back(file_path):
    """Under sudo, give a file we made back to the user who ran sudo."""
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid and gid and hasattr(os, "chown"):
        try:
            os.chown(file_path, int(uid), int(gid))
        except OSError:
            pass
