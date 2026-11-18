"""Wrist-driven strum demo

Loads right_hand.xml over the guitar model and strums in time with either a
Guitar Pro file or a constant-tempo fallback. Set CALIBRATE = True for live
sliders to re-tune the hand/guitar placement.

A mirrored left hand (left_hand.xml) sits on the neck. --follow shows the real
hands' commands; --mimic follows hands on the camera, no hardware needed:

    python Sim/mvmnt_demo.py --mimic --show

"""

import argparse
import math
import sys
from time import perf_counter, sleep

import pybullet as pb
import pybullet_data

from finger_motion import FINGERS, LEFT_SERVO_NAMES, OPEN, left_sim_joints
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
LEFT_HAND_MODEL_PATH = "Sim/MPL/left_hand.xml"
LEFT_ANCHOR_POS = [-0.150, 0.272, 0.440]     # first guess, fingers over frets 2-4: tune with CALIBRATE
LEFT_ANCHOR_EULER_DEG = [-30.0, 0.0, 0.0]
LEFT_REST = {name: 0.35 for name in LEFT_SERVO_NAMES}
BOTH_HANDS_VIEW = {"cameraDistance": 0.9, "cameraYaw": 45, "cameraPitch": -35, "cameraTargetPosition": [0.08, 0.06, 0.5]}
LOST_HAND_S = 1.0    # --mimic: a hand out of view this long eases back to rest

# Song, timing and motion settings live in strum_motion.py, shared with Hardware/servo_output.py.
DT = 1 / 240
MAX_CATCHUP_STEPS = 8

parser = argparse.ArgumentParser(description="Strum demo, or a live view of the real hand's commands.")
parser.add_argument("--follow", action="store_true",
                    help="show live wrist commands from Hardware/servo_output.py --sim-ip instead of playing on its own")
parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port for --follow")
parser.add_argument("--delay", type=float, default=0.0,
                    help="seconds to hold each pose back in --follow, to line up with servo lag")
parser.add_argument("--mimic", action="store_true",
                    help="both sim hands follow hands on the camera (needs opencv-python and mediapipe)")
parser.add_argument("--camera", type=int, default=0, help="camera number for --mimic (default 0)")
parser.add_argument("--show", action="store_true", help="--mimic: preview window with the tracking; q quits")
parser.add_argument("--swap-hands", action="store_true",
                    help="--mimic: flip the camera's left/right label (if your right hand shows as left)")
add_song_args(parser)
ARGS = parser.parse_args()
if ARGS.delay < 0:
    parser.error("--delay must be >= 0")
if ARGS.mimic and ARGS.follow:
    parser.error("use --follow or --mimic, not both")

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

# left hand: a mirror image of the right one, so it takes the same joint names and poses
left_orn = pb.getQuaternionFromEuler([d(a) for a in LEFT_ANCHOR_EULER_DEG])
left_hand = pb.loadMJCF(LEFT_HAND_MODEL_PATH)[0]
pb.resetBasePositionAndOrientation(left_hand, LEFT_ANCHOR_POS, left_orn)
left_anchor = pb.createConstraint(
    parentBodyUniqueId=left_hand, parentLinkIndex=-1,
    childBodyUniqueId=-1, childLinkIndex=-1,
    jointType=pb.JOINT_FIXED, jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=LEFT_ANCHOR_POS,
    childFrameOrientation=left_orn,
)
left_joint_index = {}
for i in range(pb.getNumJoints(left_hand)):
    info = pb.getJointInfo(left_hand, i)
    if info[2] != pb.JOINT_FIXED:
        left_joint_index[info[1].decode()] = i


def no_collisions(a, b):
    for link_a in range(-1, pb.getNumJoints(a)):
        for link_b in range(-1, pb.getNumJoints(b)):
            pb.setCollisionFilterPair(a, b, link_a, link_b, 0)


# The left hand never touches anything, so it can't disturb the strum or the string report.
no_collisions(left_hand, guitar)
no_collisions(left_hand, hand)


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

def set_joints(body, index, p):
    for name, value in p.items():
        if name in index:
            info = pb.getJointInfo(body, index[name])
            lo, hi = info[8], info[9]
            if lo < hi:     # resetJointState ignores limits, and physics would fight it
                value = max(lo, min(hi, value))
            pb.resetJointState(body, index[name], value)


def set_pose_directly(p):
    set_joints(hand, joint_index, p)
    apply(p)


# Left hand curls ({left_index: curl, ...}) to joint angles; its wrist stays straight.
def left_pose(curls):
    p = left_sim_joints(curls)
    p.update(wrist_PRO=0.0, wrist_UDEV=0.0, wrist_FLEX=0.0)
    return p


def apply_left(curls):
    p = left_pose(curls)
    names = [n for n in p if n in left_joint_index]
    pb.setJointMotorControlArray(left_hand, [left_joint_index[n] for n in names], pb.POSITION_CONTROL,
                                 targetPositions=[p[n] for n in names], forces=[10] * len(names))


def set_left_directly(curls):
    set_joints(left_hand, left_joint_index, left_pose(curls))
    apply_left(curls)


set_left_directly(LEFT_REST)


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
    left_seen = False
    try:
        while True:
            now = perf_counter()
            receiver.drain(follower, now)
            due = follower.poll(now)
            if due:
                dev, flex, fingers, left = due
                set_pose_directly(pose(dev, flex, fingers))
                if left is not None:
                    if not left_seen:   # the mimic drives both hands: widen the view once
                        pb.resetDebugVisualizerCamera(**BOTH_HANDS_VIEW)
                        left_seen = True
                    set_left_directly(left)
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


# camera mimic
# Both sim hands follow the camera at once: a left hand drives the left sim hand,
# a right hand the right one. Same tracking and smoothing as Hardware/hand_mimic.py.

def mimic_line(side, curls, bends):
    if side == "left":
        parts = [f"{n[5:]} {curls[n]:.2f}" + (f" ({bends[n]:.0f})" if n in bends else "") for n in LEFT_SERVO_NAMES]
    else:
        parts = [f"{f} {curls[f + '_base']:.2f}/{curls[f + '_tip']:.2f}" for f in FINGERS]
    return f"{side:5} " + "  ".join(parts)


def run_mimic():
    import hand_tracking as ht

    no_collisions(hand, guitar)
    rest = {"left": dict(LEFT_REST), "right": dict(OPEN)}
    curls = {side: dict(p) for side, p in rest.items()}
    set_pose_directly(pose(0.0, 0.0, curls["right"]))
    set_left_directly(curls["left"])
    pb.resetDebugVisualizerCamera(**BOTH_HANDS_VIEW)
    for _ in range(240):    # let the anchors pull the hands into place
        pb.stepSimulation()

    camera = ht.Camera(ARGS.camera, hands=2, swap=ARGS.swap_hands)
    show = ARGS.show and camera.preview_works()
    if ARGS.show and not show:
        print("[mimic] can't open a preview window here; carrying on without it.")
    filters = {side: ht.CurlFilter() for side in curls}
    last_seen = {side: 0.0 for side in curls}
    print("[mimic] hold one or both hands up to the camera. Close the sim window or Ctrl-C to quit"
          + (", or q in the preview." if show else "."))
    last = perf_counter()
    try:
        while True:
            frame, seen = camera.read()
            if frame is None:
                print("[mimic] the camera stopped sending frames.")
                break
            now = perf_counter()
            dt, last = min(now - last, 0.2), now
            by_side = {}
            for h in seen:
                by_side.setdefault(h.label, h)
            lines = []
            for side in curls:
                bends = {}
                if side in by_side:
                    raw, bends = ht.curls(by_side[side].world, side)
                    target = filters[side](raw, now)
                    last_seen[side] = now
                elif now - last_seen[side] > LOST_HAND_S:
                    target = rest[side]
                else:
                    target = {}    # just lost it: hold
                curls[side] = ht.step(curls[side], target, ht.MAX_CURL_PER_S * dt)
                lines.append(mimic_line(side, curls[side], bends) if side in by_side else f"{side:5} not in view")
            set_pose_directly(pose(0.0, 0.0, curls["right"]))
            set_left_directly(curls["left"])
            pb.stepSimulation()
            if show and not camera.show(frame, seen, lines):
                break
    except KeyboardInterrupt:
        pass
    except pb.error:
        pass    # window closed
    finally:
        camera.close()


if ARGS.follow or ARGS.mimic:
    if ARGS.follow:
        run_follow(ARGS.port, ARGS.delay)
    else:
        run_mimic()
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
    left_names = ["left_x", "left_y", "left_z", "left_roll_deg", "left_pitch_deg", "left_yaw_deg"]
    left_sliders = [pb.addUserDebugParameter(n, lo, hi, v) for n, (lo, hi), v in
                    zip(left_names, [(-1, 1)] * 3 + [(-180, 180)] * 3, LEFT_ANCHOR_POS + LEFT_ANCHOR_EULER_DEG)]
    guitar_orn = pb.getQuaternionFromEuler([0, 0, math.pi / 2])
    print("[calibrate] Drag the sliders, then paste the printed values above and set CALIBRATE = False.")

    last_print = perf_counter()
    while True:
        v = [pb.readUserDebugParameter(s) for s in sliders]
        pb.changeConstraint(anchor, jointChildPivot=v[:3],
                            jointChildFrameOrientation=pb.getQuaternionFromEuler([d(a) for a in v[3:6]]),
                            maxForce=500)
        pb.resetBasePositionAndOrientation(guitar, v[6:9], guitar_orn)
        lv = [pb.readUserDebugParameter(s) for s in left_sliders]
        pb.changeConstraint(left_anchor, jointChildPivot=lv[:3],
                            jointChildFrameOrientation=pb.getQuaternionFromEuler([d(a) for a in lv[3:]]),
                            maxForce=500)
        apply(pose(0.0, 0.0))
        pb.stepSimulation()
        if perf_counter() - last_print >= 1.0:
            last_print = perf_counter()
            print(f"HAND_ANCHOR_POS = [{v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f}]   "
                  f"HAND_ANCHOR_EULER_DEG = [{v[3]:.3f}, {v[4]:.3f}, {v[5]:.3f}]   "
                  f"GUITAR_POS = [{v[6]:.3f}, {v[7]:.3f}, {v[8]:.3f}]")
            print(f"LEFT_ANCHOR_POS = [{lv[0]:.3f}, {lv[1]:.3f}, {lv[2]:.3f}]   "
                  f"LEFT_ANCHOR_EULER_DEG = [{lv[3]:.3f}, {lv[4]:.3f}, {lv[5]:.3f}]")
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
