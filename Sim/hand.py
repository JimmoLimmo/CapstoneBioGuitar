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
    basePosition=[-0.4, -0.42, 0.57],
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

pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=45, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.55])

def degToRad(degrees):
    return (degrees * math.pi) / 180.0

sliders = {}
nameIndex = {}

ROM_LIMITS = {
    "wrist_PRO":    (degToRad(-140.0), degToRad(60.0)),
    "wrist_UDEV":   (degToRad(-10.0), degToRad(17.0)),
    "wrist_FLEX":   (degToRad(-40.0), degToRad(70.0)),
    "thumb_ABD":    (degToRad(-7.5), degToRad(7.5)),
    "thumb_MCP":    (degToRad(0.0), degToRad(90.0)),
    "thumb_PIP":    (degToRad(0.0), degToRad(120.0)),
    "thumb_DIP":    (degToRad(0.0), degToRad(80.0)),
    "index_ABD":    (degToRad(-7.5), degToRad(7.5)),
    "index_MCP":    (degToRad(0.0), degToRad(90.0)),
    "index_PIP":    (degToRad(0.0), degToRad(120.0)),
    "index_DIP":    (degToRad(0.0), degToRad(80.0)),
    "middle_ABD":   (degToRad(-7.5), degToRad(7.5)),
    "middle_MCP":   (degToRad(0.0), degToRad(90.0)),
    "middle_PIP":   (degToRad(0.0), degToRad(120.0)),
    "middle_DIP":   (degToRad(0.0), degToRad(80.0)),
    "ring_ABD":     (degToRad(-7.5), degToRad(7.5)),
    "ring_MCP":     (degToRad(0.0), degToRad(90.0)),
    "ring_PIP":     (degToRad(0.0), degToRad(120.0)),
    "ring_DIP":     (degToRad(0.0), degToRad(80.0)),
    "pinky_ABD":    (degToRad(-7.5), degToRad(7.5)),
    "pinky_MCP":    (degToRad(0.0), degToRad(90.0)),
    "pinky_PIP":    (degToRad(0.0), degToRad(120.0)),
    "pinky_DIP":    (degToRad(0.0), degToRad(80.0)),
}

for i in range(pb.getNumJoints(hand)):
    info = pb.getJointInfo(hand, i)
    jointType = info[2]
    jointName = info[1].decode()
    nameIndex[jointName] = i

    if jointType == pb.JOINT_FIXED:
        continue
    if jointName in ROM_LIMITS:
        lower, upper = ROM_LIMITS[jointName]

    q0 = pb.getJointState(hand, i)[0]
    sliderID = pb.addUserDebugParameter(jointName, lower, upper, q0)
    sliders[jointName] = sliderID

while True:
    for name, sliderID in sliders.items():
        angle = pb.readUserDebugParameter(sliderID)
        pb.setJointMotorControl2(
            bodyUniqueId=hand,
            jointIndex=nameIndex[name],
            controlMode=pb.POSITION_CONTROL,
            targetPosition=angle,
            force=10
        )

    pb.stepSimulation()

    contacts = pb.getContactPoints(hand, guitar_parts)
    if contacts:
        for c in contacts:
            hand_link = c[3]
            guitar_link = c[4]
            pos_on_guitar = c[6]
            normal_force = c[9]
            distance = c[8]
            hand_link_name = pb.getJointInfo(hand, hand_link)[1].decode() if hand_link >= 0 else "base"
            guitar_link_name = pb.getJointInfo(guitar_parts, guitar_link)[1].decode() if guitar_link >= 0 else "base"
            print(f"Contact: hand [{hand_link_name}] | guitar [{guitar_link_name}] | distance: {distance:.5f} | force: {normal_force:.3f}")
    else:
        print("No contacts detected")

    sleep(0.01)