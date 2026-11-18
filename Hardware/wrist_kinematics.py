"""Wrist angles -> tendon lengths -> servo angles for the marionette wrist.

The CDR's final wrist (parts 301-304) is a ball-and-socket joint on a center
post, with four arms hanging down from the upper wrist piece. Each arm is tied
to a tendon that runs down to a spool horn on one of four MG995 servos. Two
opposite servos make one axis: one reels in while the other pays out.

So a commanded wrist pose is NOT one servo angle per joint. This module turns
(deviation_deg, flexion_deg) -- the numbers strum_motion.wrist_at() produces --
into four servo rotations by asking, for each tendon, "how long does this
string need to be for the wrist to sit at this pose?" and then "how far does
the spool have to turn to make it that long?".

Why geometry and not "mirror the opposite servo's pulse": on a ball joint the
pulling side and the paying-out side do not change length by exactly the same
amount, and moving one axis also changes the other pair's lengths slightly.
A straight mirror leaves one string slack (the wrist rattles at the end of a
stroke) or over-tight (the servos fight each other and stall). Computing each
string length directly avoids both. Run this file on its own to see how big
that difference is for the current geometry numbers.

Pure Python, no numpy, no hardware imports -- safe to import anywhere.

Coordinate frame (forearm frame, millimetres):
    origin = ball center, +z = up the forearm toward the palm.
    Deviation (wrist_UDEV, the strum sweep) rotates about +x.
    Flexion   (wrist_FLEX, the lift)        rotates about +y.
"""

import math

# geometry (TODO: measure the rest from the CAD / the printed parts)
# All measured on the built hand (2026-11-17).
# All measured with the wrist straight. The two pairs can differ:
#   "dev"  = thumb-side and pinky-side strings (dev_neg, dev_pos), seen facing the palm
#   "flex" = back-of-hand and palm-side strings (flex_pos, flex_neg), seen side on
ARM_RADIUS_MM = {"dev": 30.0, "flex": 30.0}      # sideways from the ball center to the knots; measured 1.18 in (30 mm), same all round
ARM_DROP_MM = {"dev": 0.0, "flex": 0.0}          # how far BELOW the ball center that tie point sits (negative if above); measured 0
ANCHOR_RADIUS_MM = {"dev": 16.51, "flex": 16.51}  # sideways from the post's center to where the string leaves the post; measured 0.65 in to the hole center
ANCHOR_DROP_MM = {"dev": 98.425, "flex": 98.425}  # how far below the ball center that exit point is; measured 3.875 in
SPOOL_RADIUS_MM = 11.1125  # radius the tendon wraps on; measured 0.4375 in

# MG995 run at 6 V: 0.16 s/60 deg, no load (~375 deg/s; ~300 deg/s at 4.8 V).
# Rated 4.8-7.2 V. Was the HPS-0618SG (~1000 deg/s).
SERVO_NAME = "MG995"
SERVO_RATED_DEG_PER_S = 375.0

# Which side each servo's tendon is on. A servo is named for the direction it
# PULLS the wrist: "dev_pos" reeling in moves the wrist toward +deviation.
# Tendons pull DOWN, so the puller for a direction is on the side that drops:
#   +deviation about +x lifts +y and drops -y, so its puller sits at -y.
#   +flexion   about +y lifts -x and drops +x, so its puller sits at +x.
# If the real hand moves the wrong way on the bench, swap that pair with
# calibrate_servo.py --swap dev (or flex), don't edit these.
SERVO_SIDES = {
    "dev_pos": (0.0, -1.0),
    "dev_neg": (0.0, 1.0),
    "flex_pos": (1.0, 0.0),
    "flex_neg": (-1.0, 0.0),
}
SERVO_NAMES = tuple(SERVO_SIDES)


# Rotate a forearm-frame point by flexion about y, then deviation about x.
def _rotate(point, dev_deg, flex_deg):
    x, y, z = point
    f = math.radians(flex_deg)
    x, z = x * math.cos(f) + z * math.sin(f), -x * math.sin(f) + z * math.cos(f)
    d = math.radians(dev_deg)
    y, z = y * math.cos(d) - z * math.sin(d), y * math.sin(d) + z * math.cos(d)
    return x, y, z


def _axis(name):
    return name.split("_")[0]


def _tie_point(name):
    sx, sy = SERVO_SIDES[name]
    r, drop = ARM_RADIUS_MM[_axis(name)], ARM_DROP_MM[_axis(name)]
    return (sx * r, sy * r, -drop)


def _anchor_point(name):
    sx, sy = SERVO_SIDES[name]
    r, drop = ANCHOR_RADIUS_MM[_axis(name)], ANCHOR_DROP_MM[_axis(name)]
    return (sx * r, sy * r, -drop)


# {servo_name: tendon length in mm} with the wrist at this pose.
def tendon_lengths(dev_deg, flex_deg):
    lengths = {}
    for name in SERVO_NAMES:
        tx, ty, tz = _rotate(_tie_point(name), dev_deg, flex_deg)
        ax, ay, az = _anchor_point(name)
        lengths[name] = math.sqrt((tx - ax) ** 2 + (ty - ay) ** 2 + (tz - az) ** 2)
    return lengths


# {servo_name: spool rotation in degrees from neutral}.
#
# Positive = reel IN (string gets shorter). Neutral (0, 0) is all zeros,
# which is the pose at which each servo's neutral_pulse is measured.
def servo_angles(dev_deg, flex_deg):
    now = tendon_lengths(dev_deg, flex_deg)
    rest = tendon_lengths(0.0, 0.0)
    return {name: math.degrees((rest[name] - now[name]) / SPOOL_RADIUS_MM) for name in SERVO_NAMES}


# How much string a 'just mirror the opposite servo' scheme would get wrong,
# per pair, in mm. Positive = that pair would go slack, negative = over-tight.
def mirrored_error_mm(dev_deg, flex_deg):
    change = {n: tendon_lengths(dev_deg, flex_deg)[n] - tendon_lengths(0.0, 0.0)[n] for n in SERVO_NAMES}
    return {
        "dev": change["dev_pos"] + change["dev_neg"],
        "flex": change["flex_pos"] + change["flex_neg"],
    }


# Sweep a whole strum schedule and report what the servos actually have to do.
#
# Returns {servo_name: (min_deg, max_deg, peak_deg_per_s)}, the travel each
# servo needs and the fastest it ever has to turn. Independent of the
# control loop rate, so it shows the true demand on the servos.
def profile(schedule, wrist_at, step=0.002):
    if not schedule:
        return {}
    end = schedule[-1][0] + schedule[-1][2] + schedule[-1][3]
    t = schedule[0][0] - 0.1
    prev = None
    stats = {n: [float("inf"), float("-inf"), 0.0] for n in SERVO_NAMES}
    while t <= end + 0.1:
        dev, flex, _ = wrist_at(schedule, t)
        angles = servo_angles(dev, flex)
        for n, a in angles.items():
            s = stats[n]
            s[0], s[1] = min(s[0], a), max(s[1], a)
            if prev is not None:
                s[2] = max(s[2], abs(a - prev[n]) / step)
        prev = angles
        t += step
    return {n: tuple(v) for n, v in stats.items()}


def _report():
    print("Marionette wrist -- tendon kinematics with the CURRENT geometry constants")
    for axis in ("dev", "flex"):
        print(f"  {axis:4}  arm r={ARM_RADIUS_MM[axis]} drop={ARM_DROP_MM[axis]} | "
              f"anchor r={ANCHOR_RADIUS_MM[axis]} drop={ANCHOR_DROP_MM[axis]}")
    print(f"  spool r={SPOOL_RADIUS_MM}  (mm)\n")
    print(f"{'dev':>5} {'flex':>5} | " + " ".join(f"{n:>9}" for n in SERVO_NAMES) + " | mirror error dev/flex (mm)")
    for dev, flex in [(-20, 0), (-10, 0), (0, 0), (10, 0), (25, 0), (0, 20), (25, 20), (-20, 20)]:
        a = servo_angles(dev, flex)
        e = mirrored_error_mm(dev, flex)
        print(f"{dev:5.0f} {flex:5.0f} | " + " ".join(f"{a[n]:8.1f}d" for n in SERVO_NAMES)
              + f" | {e['dev']:+.2f} / {e['flex']:+.2f}")

    ratio = abs(servo_angles(1.0, 0.0)["dev_pos"])
    flex_ratio = abs(servo_angles(0.0, 1.0)["flex_pos"])
    print(f"\nNear neutral a servo turns ~{ratio:.2f} deg per wrist deg on the strum axis (dev) and "
          f"~{flex_ratio:.2f} on the lift axis (flex), about arm radius / spool radius.")
    print(f"CDR strike needs ~563 deg/s at the wrist -> ~{563 * ratio:.0f} deg/s at the servo "
          f"({SERVO_NAME} rated ~{SERVO_RATED_DEG_PER_S:.0f} deg/s, no load).")
    if 563 * ratio > SERVO_RATED_DEG_PER_S:
        print("  -> over the rating. A larger spool radius (or tie points closer to the ball) trades "
              "torque for speed; otherwise lengthen STRIKE_DURATION.")


if __name__ == "__main__":
    _report()
