"""Finger poses shared by the sim and the real servos.

A pose is {servo name: curl}: 0.0 = straight, 1.0 = fully curled.
"""

import math

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
SEGMENTS = ("base", "tip")
SERVO_NAMES = tuple(f"{f}_{s}" for f in FINGERS for s in SEGMENTS)


def make_pose(default=0.0, **fingers):
    """make_pose(0.0, index=0.5, thumb=(0.7, 0.4)): a number sets both segments, a pair sets (base, tip)."""
    pose = {name: float(default) for name in SERVO_NAMES}
    for finger, value in fingers.items():
        if finger not in FINGERS:
            raise ValueError(f"unknown finger {finger!r}; use one of {FINGERS}")
        base, tip = (value, value) if isinstance(value, (int, float)) else value
        pose[f"{finger}_base"] = float(base)
        pose[f"{finger}_tip"] = float(tip)
    return clamp_pose(pose)


def clamp_pose(pose):
    return {name: max(0.0, min(1.0, float(curl))) for name, curl in pose.items()}


def blend(a, b, s):
    """Pose part way from a to b (s = 0..1). Servos missing from either pose keep the other's value."""
    out = {}
    for n in set(a) | set(b):
        x = a.get(n, b.get(n))
        y = b.get(n, a.get(n))
        out[n] = x + (y - x) * s
    return out


# --- named poses (starting guesses; poses saved from hand_console.py replace them) ---
OPEN = make_pose(0.0)
RELAXED = make_pose(0.2)
FIST = make_pose(1.0)
PICK_READY = make_pose(0.85, thumb=(0.15, 0.0), index=(0.55, 0.65))   # platform made, thumb still open
PICK_GRIP = make_pose(0.85, thumb=(0.70, 0.45), index=(0.55, 0.65))   # thumb pressed onto the pick

NAMED_POSES = {
    "open": OPEN,
    "relaxed": RELAXED,
    "fist": FIST,
    "pick_ready": PICK_READY,
    "pick_grip": PICK_GRIP,
}


def set_pose(name, pose):
    """Replace or add a named pose, e.g. one tuned and saved on the bench. "open" is home and stays fixed."""
    if not name.isidentifier() or name != name.lower():
        raise ValueError(f"pose names are lower-case words like pick_grip, not {name!r}")
    if name == "open":
        raise ValueError('"open" is the home pose and can\'t be changed')
    unknown = set(pose) - set(SERVO_NAMES)
    if unknown:
        raise ValueError(f"unknown servo(s) {sorted(unknown)}")
    new = clamp_pose({n: pose.get(n, 0.0) for n in SERVO_NAMES})
    if name in NAMED_POSES:
        NAMED_POSES[name].clear()
        NAMED_POSES[name].update(new)    # in place, so PICK_READY / PICK_GRIP follow
    else:
        NAMED_POSES[name] = new


# --- sim mapping: MPL joint angle (deg) at curl 1.0 ---
SIM_FULL_CURL_DEG = {
    "base": {"MCP": 85.0},
    "tip": {"PIP": 98.0, "DIP": 70.0},
}
SIM_THUMB_FULL_CURL_DEG = {
    "base": {"ABD": 30.0, "MCP": 50.0},
    "tip": {"PIP": 58.0, "DIP": 40.0},
}


def sim_joints(pose):
    """Finger pose -> {MPL joint name: radians} for the PyBullet hand."""
    joints = {}
    for name, curl in clamp_pose(pose).items():
        finger, segment = name.split("_")
        table = SIM_THUMB_FULL_CURL_DEG if finger == "thumb" else SIM_FULL_CURL_DEG
        for joint, full_deg in table[segment].items():
            joints[f"{finger}_{joint}"] = math.radians(full_deg * curl)
    return joints


if __name__ == "__main__":
    for label, p in NAMED_POSES.items():
        print(f"{label:10} " + "  ".join(f"{n}={c:.2f}" for n, c in p.items()))
