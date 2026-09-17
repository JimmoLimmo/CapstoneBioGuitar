"""Wrist-driven strum demo for the Prostheticasters right (strumming) hand.

Loads right_hand.xml over the guitar model and strums in time with either a
Guitar Pro file or a constant-tempo fallback. Set CALIBRATE = True for live
sliders to re-tune the hand/guitar placement.

Run from the repo root:  python Sim/mvmnt_demo.py
"""

import bisect
import math
import os
import sys
from time import perf_counter, sleep

import pybullet as pb
import pybullet_data

# --- placement (tuned 2026-09-10 in CALIBRATE mode) -------------------------
HAND_MODEL_PATH = "Sim/MPL/right_hand.xml"
CALIBRATE = False
HAND_ANCHOR_POS = [0.411, 0.000, 0.537]
HAND_ANCHOR_EULER_DEG = [-13.263, -18.947, -123.158]
GUITAR_POS = [-0.375, -0.268, 0.468]

# --- song -------------------------------------------------------------------
SONG_FILE = None                # path to a .gp3/.gp4/.gp5, or None for the BPM fallback
SONG_TRACK = 0
BPM = 90.0
STRUM_INTERVAL_BEATS = 1.0      # 1.0 = one strum per beat, 0.5 = eighths
FALLBACK_STRUMS = 32
STRUM_DIRECTIONS = ["D"]        # cycled per stroke; ["D", "U"] to alternate

# --- motion -----------------------------------------------------------------
STRIKE_DURATION = 0.12          # sweep across the strings
RECOVER_DURATION = 0.18         # lifted return, only when the next stroke needs it
WRIST_YAW_DOWN_DEG = -20.0      # wrist_UDEV range is about -14.9..45, deliberately asymmetric
WRIST_YAW_UP_DEG = 25.0
LIFT_FLEX_DEG = 20.0            # wrist_FLEX clearance lift; flip the sign if it dips into the guitar
WRIST_PRO_BASE_DEG = 0.0
GRIP_CURL = 1.0                 # 0.0 = open hand, 1.0 = full fist (full fist currently sits in strings 1-4)

DT = 1 / 240
MAX_CATCHUP_STEPS = 8

STRING_LINK_NAMES = {f"string{i}" for i in range(1, 7)}
FINGER_HINTS = ("thumb", "index", "middle", "ring", "pinky")

FIST_POSE = {
    "thumb_ABD": 15, "thumb_MCP": 45, "thumb_PIP": 55, "thumb_DIP": 35,
    "index_MCP": 70, "index_PIP": 95, "index_DIP": 60,
    "middle_MCP": 75, "middle_PIP": 100, "middle_DIP": 65,
    "ring_MCP": 75, "ring_PIP": 100, "ring_DIP": 65,
    "pinky_MCP": 75, "pinky_PIP": 100, "pinky_DIP": 65,
}


def d(deg):
    return deg * math.pi / 180.0


def smooth(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


# --- schedule ---------------------------------------------------------------
# Each stroke is (onset, direction, strike_duration, recover_duration) with
# onset an absolute time in seconds from the start of the run.

def song_onsets(path, track):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Parsing"))
    from GuitarProReader import GuitarProSong

    song = GuitarProSong(path)
    times = []
    for measure in song.yield_timed_measures(track):
        for beat in measure.yield_timed_beats():
            if beat.beat.notes:                      # rests just widen the gap
                times.append(beat.time)
    return [t - times[0] for t in times] if times else []


def fallback_onsets():
    step = 60.0 / BPM * STRUM_INTERVAL_BEATS
    return [i * step for i in range(FALLBACK_STRUMS)]


def build_schedule():
    onsets, source = [], f"{BPM} BPM fallback"
    if SONG_FILE and os.path.exists(SONG_FILE):
        try:
            onsets = song_onsets(SONG_FILE, SONG_TRACK)
            source = SONG_FILE
        except Exception as e:
            print(f"[strum] {SONG_FILE} failed to parse ({e})")
    if not onsets:
        onsets = fallback_onsets()

    schedule = []
    for i, onset in enumerate(onsets):
        budget = (onsets[i + 1] - onset) if i + 1 < len(onsets) else STRIKE_DURATION + RECOVER_DURATION
        strike = min(STRIKE_DURATION, budget * 0.5)
        schedule.append((
            onset,
            STRUM_DIRECTIONS[i % len(STRUM_DIRECTIONS)],
            strike,
            min(RECOVER_DURATION, budget - strike),
        ))
    return schedule, source


# --- pose -------------------------------------------------------------------
# A down-stroke starts at the UP angle and sweeps to the DOWN angle; an
# up-stroke mirrors it. So the follow-through of one stroke is already the
# ready position of the opposite stroke, and recovery only has distance to
# travel when two strokes repeat the same direction -- which is exactly when
# the return trip would otherwise drag back through the strings. The lift
# scales with that distance, so alternating strumming lifts nothing at all.

def ready_yaw(direction):
    return WRIST_YAW_UP_DEG if direction == "D" else WRIST_YAW_DOWN_DEG


def follow_yaw(direction):
    return WRIST_YAW_DOWN_DEG if direction == "D" else WRIST_YAW_UP_DEG


def wrist_at(schedule, t):
    """(yaw_deg, flex_deg) for time t, plus the index of the stroke in progress."""
    if not schedule or t < schedule[0][0]:
        return ready_yaw(schedule[0][1] if schedule else "D"), 0.0, -1

    i = bisect.bisect_right([s[0] for s in schedule], t) - 1
    onset, direction, strike, recover = schedule[i]
    elapsed = t - onset

    if elapsed <= strike:
        start, end = ready_yaw(direction), follow_yaw(direction)
        return start + (end - start) * smooth(elapsed / strike), 0.0, i

    next_direction = schedule[i + 1][1] if i + 1 < len(schedule) else direction
    start, end = follow_yaw(direction), ready_yaw(next_direction)
    span = abs(WRIST_YAW_UP_DEG - WRIST_YAW_DOWN_DEG)
    lift = LIFT_FLEX_DEG * (abs(end - start) / span if span else 0.0)

    if elapsed >= strike + recover or recover <= 0:
        return end, 0.0, i

    u = (elapsed - strike) / recover
    return start + (end - start) * smooth(u), lift * math.sin(math.pi * u), i


def pose(yaw_deg, flex_deg):
    p = {name: d(angle * GRIP_CURL) for name, angle in FIST_POSE.items()}
    p["wrist_PRO"] = d(WRIST_PRO_BASE_DEG)
    p["wrist_UDEV"] = d(yaw_deg)
    p["wrist_FLEX"] = d(flex_deg)
    return p


# --- world ------------------------------------------------------------------
pb.connect(pb.GUI)
pb.setGravity(0, 0, -9.81)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
pb.setTimeStep(DT)

hand = pb.loadMJCF(HAND_MODEL_PATH)[0]
guitar = pb.loadURDF(
    "Sim/guitar_model/guitar.urdf",
    basePosition=GUITAR_POS,
    baseOrientation=pb.getQuaternionFromEuler([0, 0, math.pi / 2]),
    useFixedBase=True,
)

anchor = pb.createConstraint(
    parentBodyUniqueId=hand, parentLinkIndex=-1,
    childBodyUniqueId=-1, childLinkIndex=-1,
    jointType=pb.JOINT_FIXED, jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=HAND_ANCHOR_POS,
    childFrameOrientation=pb.getQuaternionFromEuler([d(a) for a in HAND_ANCHOR_EULER_DEG]),
)

pb.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.55]
)

joint_index = {}
for i in range(pb.getNumJoints(hand)):
    info = pb.getJointInfo(hand, i)
    if info[2] != pb.JOINT_FIXED:
        joint_index[info[1].decode()] = i

missing = {"wrist_PRO", "wrist_UDEV", "wrist_FLEX", "thumb_ABD"} - set(joint_index)
if missing:
    raise RuntimeError(f"{HAND_MODEL_PATH} is missing joints {sorted(missing)} -- needs a full wrist and thumb.")

# getJointInfo()[12] is the link name; [1] is the joint name.
hand_links = {-1: "base"}
for i in range(pb.getNumJoints(hand)):
    hand_links[i] = pb.getJointInfo(hand, i)[12].decode()

guitar_links = {-1: "base"}
for i in range(pb.getNumJoints(guitar)):
    guitar_links[i] = pb.getJointInfo(guitar, i)[12].decode()
string_links = {i for i, n in guitar_links.items() if n in STRING_LINK_NAMES}


def apply(p):
    indices = [joint_index[n] for n in p if n in joint_index]
    targets = [p[n] for n in p if n in joint_index]
    pb.setJointMotorControlArray(hand, indices, pb.POSITION_CONTROL, targetPositions=targets, forces=[10] * len(indices))


def strings_touching():
    hit = set()
    for c in pb.getContactPoints(hand, guitar):
        if c[4] in string_links and any(h in hand_links.get(c[3], "") for h in FINGER_HINTS):
            hit.add(guitar_links[c[4]])
    return hit


# --- calibrate --------------------------------------------------------------
if CALIBRATE:
    names = ["anchor_x", "anchor_y", "anchor_z", "anchor_roll_deg", "anchor_pitch_deg",
             "anchor_yaw_deg", "guitar_x", "guitar_y", "guitar_z"]
    defaults = HAND_ANCHOR_POS + HAND_ANCHOR_EULER_DEG + GUITAR_POS
    ranges = [(-1, 1)] * 3 + [(-180, 180)] * 3 + [(-1.5, 1.5)] * 3
    sliders = [pb.addUserDebugParameter(n, lo, hi, v) for n, (lo, hi), v in zip(names, ranges, defaults)]
    guitar_orn = pb.getQuaternionFromEuler([0, 0, math.pi / 2])
    print("[calibrate] Drag the sliders, then paste the printed values above and set CALIBRATE = False.")

    last_print = perf_counter()
    while True:
        v = [pb.readUserDebugParameter(s) for s in sliders]
        pb.changeConstraint(anchor, jointChildPivot=v[:3],
                            jointChildFrameOrientation=pb.getQuaternionFromEuler([d(a) for a in v[3:6]]),
                            maxForce=500)
        pb.resetBasePositionAndOrientation(guitar, v[6:9], guitar_orn)
        apply(pose(0.0, 0.0))
        pb.stepSimulation()
        if perf_counter() - last_print >= 1.0:
            last_print = perf_counter()
            print(f"HAND_ANCHOR_POS = [{v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f}]   "
                  f"HAND_ANCHOR_EULER_DEG = [{v[3]:.3f}, {v[4]:.3f}, {v[5]:.3f}]   "
                  f"GUITAR_POS = [{v[6]:.3f}, {v[7]:.3f}, {v[8]:.3f}]")
        sleep(DT)

# --- run --------------------------------------------------------------------
# Pose comes from the wall clock, so strokes land on time whatever the physics
# manages; physics steps catch up to real time and resync if they can't.

schedule, source = build_schedule()
end_time = schedule[-1][0] + schedule[-1][2] + schedule[-1][3] + 0.5
print(f"[strum] {len(schedule)} strokes from {source}, directions {STRUM_DIRECTIONS}")

def report(i, hits):
    if i < 0:
        return
    onset, direction, _, _ = schedule[i]
    arrow = "v" if direction == "D" else "^"
    print(f"{arrow} #{i:<3} t={onset:6.2f}s  strings:{','.join(sorted(s[-1] for s in hits)) or '-'}")


start = perf_counter()
sim_time = 0.0
reported = -1
hit_this_stroke = set()
worst_lag = 0.0

while True:
    now = perf_counter() - start
    if now > end_time:
        break

    yaw, flex, index = wrist_at(schedule, now)
    apply(pose(yaw, flex))

    steps = 0
    while sim_time < now and steps < MAX_CATCHUP_STEPS:
        pb.stepSimulation()
        sim_time += DT
        steps += 1
    if sim_time < now:
        worst_lag = max(worst_lag, now - sim_time)
        sim_time = now

    hit_this_stroke |= strings_touching()

    if index > reported:
        report(reported, hit_this_stroke)
        reported = index
        hit_this_stroke = set()

    if sim_time > now:
        sleep(sim_time - now)

report(reported, hit_this_stroke)   # the last stroke has no successor to trigger its line
print(f"[strum] done. {len(schedule)} strokes, worst physics lag {worst_lag * 1000:.0f} ms")
pb.disconnect()
