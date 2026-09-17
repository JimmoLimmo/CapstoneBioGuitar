"""Song-driven strum choreography for the Prostheticasters right hand.

Pure math + I/O for turning a Guitar Pro file (or a constant-tempo fallback)
into a wrist yaw/flex trajectory over wall-clock time. No pybullet import and
no hardware import here on purpose: both Sim/mvmnt_demo.py (PyBullet) and
Hardware/servo_output.py (real PCA9685 servos) import this module and drive
whatever "actuator" they have from the same wrist_at()/pose() output, so the
choreography only ever lives in one place.

Run either consumer from the repo root, same as before:
    python Sim/mvmnt_demo.py
    python Hardware/servo_output.py
"""

import bisect
import math
import os
import sys

# --- song -------------------------------------------------------------------
SONG_FILE = "Parsing/inputs/Apollo-Live.gp3"   # .gp3/.gp4/.gp5 relative to the repo root, a bare filename, or None
SONG_TRACK = 1
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

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SONG_DIR = os.path.join(REPO_ROOT, "Parsing", "inputs")


def find_song(name):
    """Accepts a repo-relative path, a bare filename, or a stray leading slash."""
    for candidate in (name,
                      os.path.join(REPO_ROOT, name.lstrip("/\\")),
                      os.path.join(SONG_DIR, os.path.basename(name))):
        if os.path.isfile(candidate):
            return candidate
    return None


def song_onsets(path, track):
    """(onsets_from_zero, track_names). Rests are skipped; they just widen the gap."""
    sys.path.insert(0, os.path.join(REPO_ROOT, "Parsing"))
    from GuitarProReader import GuitarProSong

    song = GuitarProSong(path)
    names = [t.name for t in song.song.tracks]
    times = []
    for measure in song.yield_timed_measures(track):
        for beat in measure.yield_timed_beats():
            if beat.beat.notes:
                times.append(beat.time)
    return ([t - times[0] for t in times] if times else []), names


def fallback_onsets():
    step = 60.0 / BPM * STRUM_INTERVAL_BEATS
    return [i * step for i in range(FALLBACK_STRUMS)]


def build_schedule():
    onsets, source = [], f"{BPM} BPM fallback"
    if SONG_FILE:
        found = find_song(SONG_FILE)
        if not found:
            available = sorted(f for f in os.listdir(SONG_DIR) if f.lower().endswith((".gp3", ".gp4", ".gp5")))
            print(f"[strum] SONG_FILE not found: {SONG_FILE}")
            print(f"[strum]   songs available in Parsing/inputs: {', '.join(available) or 'none'}")
        else:
            try:
                onsets, tracks = song_onsets(found, SONG_TRACK)
                if onsets:
                    source = f"{os.path.basename(found)} track {SONG_TRACK} ({tracks[SONG_TRACK]})"
                else:
                    print(f"[strum] {os.path.basename(found)} track {SONG_TRACK} has no played notes.")
                    print("[strum]   tracks: " + ", ".join(f"{i}={n}" for i, n in enumerate(tracks)))
            except ImportError:
                print("[strum] PyGuitarPro is not installed in this environment.")
                print("[strum]   fix with:  pip install PyGuitarPro")
            except IndexError:
                print(f"[strum] SONG_TRACK {SONG_TRACK} is out of range for this song.")
            except Exception as e:
                print(f"[strum] {os.path.basename(found)} failed to parse ({e})")
    if not onsets:
        print("[strum] using the constant-tempo fallback instead.")
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
    """(yaw_deg, flex_deg, stroke_index) for time t. yaw drives wrist_UDEV,
    flex drives wrist_FLEX -- these are the two DOFs a real strum needs to
    track continuously. stroke_index is -1 before the first stroke."""
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
    """Full 16-joint target dict in RADIANS, keyed by MJCF joint name --
    what the PyBullet sim consumes directly. Kept here so the sim and any
    future higher-DOF hand still share one source of truth for the grip."""
    p = {name: d(angle * GRIP_CURL) for name, angle in FIST_POSE.items()}
    p["wrist_PRO"] = d(WRIST_PRO_BASE_DEG)
    p["wrist_UDEV"] = d(yaw_deg)
    p["wrist_FLEX"] = d(flex_deg)
    return p
