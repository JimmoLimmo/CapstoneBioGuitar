"""
Wrist-driven strum demo for right hand.

This replaces the old chord-pose demo (open/C_chord/G_chord/power_strum/pinch),
which was left over from last year's fretting-hand team and cycled static hand
SHAPES rather than showing a strum.
Semester's sim needs to:
  1. Be tied to a hand model that actually has a thumb and functional wrist
     joints We now load right_hand.xml instead, which has
     the full wrist_PRO / wrist_UDEV / wrist_FLEX chain plus a complete thumb.
  2. Demonstrate STRUMMING specifically: a wrist sweep across the strings
     over time, with a pick held in a fixed grip, timed to a tempo/song --
     not a sequence of static fretting-style chord shapes.
  3. Reuse the hand-agnostic timing engine from Parsing/GuitarProReader.py
     when a real song is available, since it already computes real-world
     beat timing independent of which hand executes it.

python3 Sim/mvmnt_demo.py

"""

import math
import os
import sys
from time import sleep

import pybullet as pb
import pybullet_data


HAND_MODEL_PATH = "Sim/MPL/right_hand.xml"   # full thumb + 3-DOF wrist model
HAND_ANCHOR_POS = [0, 0, 0.45]               # fixed world point the hand's forearm attaches to
GUITAR_POS = [-0.4, -0.4, 0.58]

BPM = 90.0                                    # fallback tempo if no song file is loaded
SONG_FILE = r"Parsing\inputs\Apollo-Live.gp3"   # e.g. "Parsing/inputs/queen-bohemian_rhapsody_complete.gp3"
SONG_TRACK = 0

DOWN_STRUM_DEG = -30.0   # wrist_FLEX target on a downstroke
UP_STRUM_DEG = 30.0      # wrist_FLEX target on an upstroke
LATERAL_SWEEP_DEG = 8.0  # small wrist_UDEV sweep so the stroke actually crosses strings
WRIST_PRO_BASE_DEG = 0.0  # constant forearm rotation aiming the hand over the strings

STRING_LINK_NAMES = {f"string{i}" for i in range(1, 7)}
CONTACT_LINK_HINTS = ("thumb", "index", "middle", "ring", "pinky")  # fingertip-ish links we care about for contact logging


def d(deg):
    return deg * math.pi / 180.0


def smooth_step(t):
    """Ease-in-out curve: starts/ends slow, fast in the middle."""
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)



def build_strum_events_from_song(song_path, track=0):
    """Returns a list of (direction, duration_seconds) using real beat timing.
    direction alternates D/U per beat, which is the standard alternate-
    strumming convention when the source notation doesn't carry explicit
    stroke direction. Returns None if the song can't be loaded (missing
    file, missing `guitarpro` package, etc.) so callers can fall back."""
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Parsing"))
        from GuitarProReader import GuitarProSong
    except Exception as e:
        print(f"[strum] Couldn't import GuitarProReader ({e}); using fallback metronome pattern.")
        return None

    if not song_path or not os.path.exists(song_path):
        print(f"[strum] SONG_FILE not found ({song_path}); using fallback metronome pattern.")
        return None

    try:
        song = GuitarProSong(song_path)
        events = []
        direction_cycle = ["D", "U"]
        i = 0
        for timed_measure in song.yield_timed_measures(track):
            for timed_beat in timed_measure.yield_timed_beats():
                duration = timed_beat.get_duration()
                if duration <= 0:
                    continue
                events.append((direction_cycle[i % 2], duration))
                i += 1
        if not events:
            print("[strum] Song loaded but produced no beats; using fallback metronome pattern.")
            return None
        print(f"[strum] Loaded {len(events)} strum events from {song_path}")
        return events
    except Exception as e:
        print(f"[strum] Failed to parse {song_path} ({e}); using fallback metronome pattern.")
        return None


def build_fallback_strum_events(bpm, n_events=64):
    """Alternating down/up eighth-note pattern at a constant tempo."""
    eighth_note_seconds = 60.0 / bpm / 2.0
    direction_cycle = ["D", "U"]
    return [(direction_cycle[i % 2], eighth_note_seconds) for i in range(n_events)]


STRUM_EVENTS = build_strum_events_from_song(SONG_FILE, SONG_TRACK) if SONG_FILE else None
if STRUM_EVENTS is None:
    STRUM_EVENTS = build_fallback_strum_events(BPM)


# ---------------------------------------------------------------------------
# Hand shape: a single fixed "pick grip" pose (thumb pinched against index
# holding a pick, remaining fingers curled loosely out of the way). Per the
# PDR: this semester's right hand still needs to "grab a pick and strum" --
# the wrist does the strumming motion; the fingers just need to hold on.
# ---------------------------------------------------------------------------

GRIP_POSE = {
    "wrist_PRO": d(WRIST_PRO_BASE_DEG),
    "thumb_ABD": d(20), "thumb_MCP": d(35), "thumb_PIP": d(25), "thumb_DIP": d(15),
    "index_MCP": d(40), "index_PIP": d(35), "index_DIP": d(20),
    "middle_MCP": d(55), "middle_PIP": d(60), "middle_DIP": d(30),
    "ring_MCP": d(60), "ring_PIP": d(65), "ring_DIP": d(30),
    "pinky_MCP": d(60), "pinky_PIP": d(65), "pinky_DIP": d(30),
}


def strum_pose(direction, alpha):
    """Grip pose plus a wrist sweep toward the down- or up-strum extreme.
    alpha 0->1 blends from neutral wrist angle to the stroke's target angle."""
    pose = dict(GRIP_POSE)
    flex_target = d(DOWN_STRUM_DEG) if direction == "D" else d(UP_STRUM_DEG)
    lateral_target = d(LATERAL_SWEEP_DEG) if direction == "D" else d(-LATERAL_SWEEP_DEG)
    t = smooth_step(alpha)
    pose["wrist_FLEX"] = flex_target * t
    pose["wrist_UDEV"] = lateral_target * t
    return pose


# ---------------------------------------------------------------------------
# PyBullet setup
# ---------------------------------------------------------------------------

physicsClient = pb.connect(pb.GUI)
pb.setGravity(0, 0, -9.81)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")

models = pb.loadMJCF(HAND_MODEL_PATH)
hand = models[0]

for i in range(-1, pb.getNumJoints(hand)):
    pb.changeDynamics(hand, i, collisionMargin=0.00001)

guitar_parts = pb.loadURDF(
    "Sim/guitar_model/guitar.urdf",
    basePosition=GUITAR_POS,
    baseOrientation=pb.getQuaternionFromEuler([0, 0, math.pi / 2]),
    useFixedBase=True
)

pb.setPhysicsEngineParameter(
    numSolverIterations=200,
    numSubSteps=4,
    contactBreakingThreshold=0.00001,
    enableConeFriction=1
)
pb.setTimeStep(1 / 240)

constraint = pb.createConstraint(
    parentBodyUniqueId=hand,
    parentLinkIndex=-1,
    childBodyUniqueId=-1,
    childLinkIndex=-1,
    jointType=pb.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=HAND_ANCHOR_POS,
    childFrameOrientation=pb.getQuaternionFromEuler([-math.pi / 2, 0, math.pi])
)

pb.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45,
    cameraPitch=-20, cameraTargetPosition=[0, 0, 0.55]
)

name_index = {}
for i in range(pb.getNumJoints(hand)):
    info = pb.getJointInfo(hand, i)
    if info[2] != pb.JOINT_FIXED:
        name_index[info[1].decode()] = i

# Guardrail: catch a future accidental revert to a thumb-less / wrist-less
# model early with a clear message instead of silently doing nothing, since
# that's exactly the bug this rewrite fixes.
_required = {"wrist_PRO", "wrist_UDEV", "wrist_FLEX", "thumb_ABD", "thumb_MCP"}
_missing = _required - set(name_index.keys())
if _missing:
    raise RuntimeError(
        f"Loaded hand model '{HAND_MODEL_PATH}' is missing joints {sorted(_missing)}. "
        "This demo needs a model with a full wrist (PRO/UDEV/FLEX) and a thumb -- "
        "MPL.xml does not have these; use right_hand.xml or mpl2.xml instead."
    )

# Precompute which link indices belong to the guitar's 6 strings, and which
# hand link indices look like fingertip/thumb links, so contact logging can
# report something readable (e.g. "down-strum hit string3, string4") instead
# of a raw per-timestep dump of every contact pair.
guitar_link_names = {-1: "base"}
for i in range(pb.getNumJoints(guitar_parts)):
    guitar_link_names[i] = pb.getJointInfo(guitar_parts, i)[1].decode()
string_link_indices = {i for i, n in guitar_link_names.items() if n in STRING_LINK_NAMES}

hand_link_names = {-1: "base"}
for i in range(pb.getNumJoints(hand)):
    hand_link_names[i] = pb.getJointInfo(hand, i)[1].decode()


def apply_pose(pose):
    for name, idx in name_index.items():
        target = pose.get(name, 0.0)
        pb.setJointMotorControl2(
            bodyUniqueId=hand,
            jointIndex=idx,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=target,
            force=10
        )


# ---------------------------------------------------------------------------
# Main loop: walk the strum timeline, driving the wrist through each stroke
# and logging which strings got touched during that stroke.
# ---------------------------------------------------------------------------

event_idx = 0
event_elapsed = 0.0
strings_hit_this_stroke = set()
dt = 1 / 240

print(f"[strum] {len(STRUM_EVENTS)} events queued, tempo source: "
      f"{'song file' if SONG_FILE else f'{BPM} BPM fallback'}")

while True:
    direction, duration = STRUM_EVENTS[event_idx % len(STRUM_EVENTS)]
    alpha = min(event_elapsed / duration, 1.0) if duration > 0 else 1.0
    apply_pose(strum_pose(direction, alpha))

    pb.stepSimulation()

    for c in pb.getContactPoints(hand, guitar_parts):
        h_link, g_link = c[3], c[4]
        if g_link in string_link_indices and any(hint in hand_link_names.get(h_link, "") for hint in CONTACT_LINK_HINTS):
            strings_hit_this_stroke.add(guitar_link_names[g_link])

    event_elapsed += dt
    if event_elapsed >= duration:
        arrow = "↓" if direction == "D" else "↑"
        hit_str = ", ".join(sorted(strings_hit_this_stroke)) if strings_hit_this_stroke else "(no string contact)"
        print(f"{arrow} strum #{event_idx}: {hit_str}")
        strings_hit_this_stroke = set()
        event_elapsed = 0.0
        event_idx += 1

    sleep(dt)
