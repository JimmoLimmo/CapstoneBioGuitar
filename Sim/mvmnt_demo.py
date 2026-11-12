"""Wrist-driven strum demo

Loads right_hand.xml over the guitar model and strums in time with either a
Guitar Pro file or a constant-tempo fallback. Set CALIBRATE = True for live
sliders to re-tune the hand/guitar placement.

"""

import argparse
import math
import sys
from time import perf_counter, sleep

import pybullet as pb
import pybullet_data

from motion_link import DEFAULT_PORT, PoseFollower, PoseReceiver
from strum_motion import (
    FINGER_HINTS,
    STRING_LINK_NAMES,
    add_song_args,
    d,
    pose,
    schedule_from_args,
    wrist_at,
)

#placement 
HAND_MODEL_PATH = "Sim/MPL/right_hand.xml"
CALIBRATE = True
HAND_ANCHOR_POS = [0.411, 0.000, 0.537]
HAND_ANCHOR_EULER_DEG = [-13.263, -18.947, -123.158]
GUITAR_POS = [-0.375, -0.268, 0.468]

# Song, timing and motion settings live in strum_motion.py, shared with Hardware/servo_output.py.
DT = 1 / 240
MAX_CATCHUP_STEPS = 8

parser = argparse.ArgumentParser(description="Strum demo, or a live view of the real hand's commands.")
parser.add_argument("--follow", action="store_true",
                    help="show live wrist commands from Hardware/servo_output.py --sim-ip instead of playing on its own")
parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port for --follow")
parser.add_argument("--delay", type=float, default=0.0,
                    help="seconds to hold each pose back in --follow, to line up with servo lag")
add_song_args(parser)
ARGS = parser.parse_args()
if ARGS.delay < 0:
    parser.error("--delay must be >= 0")

# world
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


# live follow
# Poses are set directly because physics motors would add lag of their own, and
# hand-guitar collisions are off so contact can't shake the view. That means no
# string-contact report here; the standalone run is still the clearance check.

def set_pose_directly(p):
    for name, value in p.items():
        if name in joint_index:
            info = pb.getJointInfo(hand, joint_index[name])
            lo, hi = info[8], info[9]
            if lo < hi:     # resetJointState ignores limits, and physics would fight it
                value = max(lo, min(hi, value))
            pb.resetJointState(hand, joint_index[name], value)
    apply(p)


def run_follow(port, delay):
    for hand_link in hand_links:
        for guitar_link in guitar_links:
            pb.setCollisionFilterPair(hand, guitar, hand_link, guitar_link, 0)

    set_pose_directly(pose(0.0, 0.0))
    for _ in range(240):    # let the anchor pull the hand into place
        pb.stepSimulation()

    try:
        receiver = PoseReceiver(port)
    except OSError as e:
        print(f"[follow] can't listen on UDP port {port} ({e}). Is another copy of the sim running?")
        return
    follower = PoseFollower(delay)
    print(f"[follow] listening on UDP port {port}, delay {delay * 1000:.0f} ms. Close the window or Ctrl-C to quit.")

    streaming = False
    try:
        while True:
            now = perf_counter()
            receiver.drain(follower, now)
            due = follower.poll(now)
            if due:
                set_pose_directly(pose(*due))
            live = follower.last_received is not None and now - follower.last_received < 1.0
            if live != streaming:
                streaming = live
                print("[follow] receiving commands" if live else "[follow] stream stopped, holding last pose")
            pb.stepSimulation()
            sleep(0.001)
    except KeyboardInterrupt:
        pass
    except pb.error:
        pass    # window closed


if ARGS.follow:
    run_follow(ARGS.port, ARGS.delay)
    try:
        pb.disconnect()
    except pb.error:
        pass
    sys.exit(0)

# calibration
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

# run 9 j
# Pose comes from the wall clock, so strokes land on time whatever the physics
# manages; physics steps catch up to real time and resync if they can't.

try:
    schedule, source = schedule_from_args(ARGS)
except ValueError as e:
    sys.exit(f"[strum] {e}")
end_time = schedule[-1][0] + schedule[-1][2] + schedule[-1][3] + 0.5
print(f"[strum] {len(schedule)} strokes from {source}, starting {''.join(s[1] for s in schedule[:8])}")

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
