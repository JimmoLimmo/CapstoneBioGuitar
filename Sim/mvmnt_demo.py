import pybullet as pb
import pybullet_data
from time import sleep
import math

physicsClient = pb.connect(pb.GUI)
pb.setGravity(0, 0, -9.81)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")

models = pb.loadMJCF("Sim/MPL/MPL.xml")
hand = models[0]

for i in range(-1, pb.getNumJoints(hand)):
    pb.changeDynamics(hand, i, collisionMargin=0.00001)

guitar_parts = pb.loadURDF(
    "Sim/guitar_model/guitar.urdf",
    basePosition=[-0.4, -0.4, 0.58],
    baseOrientation=pb.getQuaternionFromEuler([0, 0, math.pi / 2]),
    useFixedBase=True
)

pb.setPhysicsEngineParameter(
    numSolverIterations=200,
    numSubSteps=4,
    contactBreakingThreshold=0.00001,
    enableConeFriction=1
)
pb.setTimeStep(1/240)

constraint = pb.createConstraint(
    parentBodyUniqueId=hand,
    parentLinkIndex=-1,
    childBodyUniqueId=-1,
    childLinkIndex=-1,
    jointType=pb.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0.45],
    childFrameOrientation=pb.getQuaternionFromEuler([-math.pi / 2, 0, math.pi])
)

pb.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45,
    cameraPitch=-20, cameraTargetPosition=[0, 0, 0.55]
)

def d(deg):
    return deg * math.pi / 180.0

POSES = {
    "open": {
        "wrist_FLEX": d(10),
    },
    "C_chord": {
        "wrist_FLEX":  d(20),
        "index_MCP":   d(45), "index_PIP":  d(60), "index_DIP":  d(30),
        "middle_MCP":  d(55), "middle_PIP": d(70), "middle_DIP": d(35),
        "ring_MCP":    d(70), "ring_PIP":   d(80), "ring_DIP":   d(40),
        "pinky_MCP":   d(30), "pinky_PIP":  d(40), "pinky_DIP":  d(20),
        "thumb_ABD":   d(-5), "thumb_MCP":  d(30), "thumb_PIP":  d(40),
    },
    "G_chord": {
        "wrist_FLEX":  d(15),
        "index_MCP":   d(20), "index_PIP":  d(25),
        "middle_MCP":  d(60), "middle_PIP": d(75), "middle_DIP": d(40),
        "ring_MCP":    d(65), "ring_PIP":   d(80), "ring_DIP":   d(40),
        "pinky_MCP":   d(70), "pinky_PIP":  d(85), "pinky_DIP":  d(45),
        "thumb_ABD":   d( 5), "thumb_MCP":  d(20),
    },
    "power_strum": {
        "wrist_FLEX":  d(-10), "wrist_PRO": d(20),
        "index_MCP":   d(10),  "index_PIP": d(15),
        "middle_MCP":  d(12),  "middle_PIP": d(18),
        "ring_MCP":    d(10),  "ring_PIP":  d(12),
        "pinky_MCP":   d(8),   "pinky_PIP": d(10),
        "thumb_ABD":   d(-4),  "thumb_MCP": d(10),
    },
    "pinch": {
        "wrist_FLEX":  d(5),
        "index_MCP":   d(70), "index_PIP":  d(90), "index_DIP":  d(50),
        "middle_MCP":  d(75), "middle_PIP": d(95), "middle_DIP": d(50),
        "ring_MCP":    d(70), "ring_PIP":   d(85), "ring_DIP":   d(45),
        "pinky_MCP":   d(65), "pinky_PIP":  d(80), "pinky_DIP":  d(40),
        "thumb_ABD":   d(-6), "thumb_MCP":  d(55), "thumb_PIP":  d(70),
    },
}

POSE_ORDER   = ["open", "C_chord", "open", "G_chord", "open", "power_strum", "pinch", "open"]
HOLD_FRAMES  = 80   
BLEND_FRAMES = 60   

def smooth_step(t):
    """Ease-in-out curve: starts/ends slow, fast in the middle."""
    return t * t * (3 - 2 * t)

name_index = {}
for i in range(pb.getNumJoints(hand)):
    info = pb.getJointInfo(hand, i)
    if info[2] != pb.JOINT_FIXED:
        name_index[info[1].decode()] = i

def apply_pose(pose_a, pose_b, alpha):
    """Drive every joint to a blend of pose_a and pose_b (alpha 0→1)."""
    t = smooth_step(alpha)
    for name, idx in name_index.items():
        a = pose_a.get(name, 0.0)
        b = pose_b.get(name, 0.0)
        target = a + (b - a) * t
        pb.setJointMotorControl2(
            bodyUniqueId=hand,
            jointIndex=idx,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=target,
            force=10
        )

pose_idx   = 0
frame      = 0
phase      = "hold"   # "hold" | "blend"

while True:
    cur_pose  = POSES[POSE_ORDER[pose_idx]]
    next_pose = POSES[POSE_ORDER[(pose_idx + 1) % len(POSE_ORDER)]]

    if phase == "hold":
        apply_pose(cur_pose, cur_pose, 0.0)
        frame += 1
        if frame >= HOLD_FRAMES:
            frame = 0
            phase = "blend"

    elif phase == "blend":
        alpha = frame / BLEND_FRAMES
        apply_pose(cur_pose, next_pose, alpha)
        frame += 1
        if frame > BLEND_FRAMES:
            frame = 0
            phase = "hold"
            pose_idx = (pose_idx + 1) % len(POSE_ORDER)
            pose_name = POSE_ORDER[pose_idx]
            print(f"► Pose: {pose_name}")

    pb.stepSimulation()

    contacts = pb.getContactPoints(hand, guitar_parts)
    if contacts:
        for c in contacts:
            h_link = c[3]; g_link = c[4]
            h_name = pb.getJointInfo(hand, h_link)[1].decode() if h_link >= 0 else "base"
            g_name = pb.getJointInfo(guitar_parts, g_link)[1].decode() if g_link >= 0 else "base"
            print(f"Contact: [{h_name}] → [{g_name}]  "
                  f"dist={c[8]:.5f}  force={c[9]:.3f}")

    sleep(1/240)