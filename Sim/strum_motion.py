"""Song-driven strum choreography.

Pure math + I/O for turning a Guitar Pro file (or a constant-tempo fallback)
into a wrist yaw/flex trajectory over wall-clock time.
"""

import bisect
import math
import os
import sys

from finger_motion import sim_joints

# song 
SONG_FILE = "Parsing/inputs/Apollo-Live.gp3"   # .gp3/.gp4/.gp5 relative to the repo root, a bare filename, or None
# SONG_FILE = "Parsing/inputs/DragonForce - Through the Fire and Flames.gp5"
# SONG_FILE = None
SONG_TRACK = 1
BPM = 90.0
STRUM_INTERVAL_BEATS = 1.0      # 1.0 = one strum per beat, 0.5 = eighths
FALLBACK_STRUMS = 32
STRUM_DIRECTIONS = ["D"]        # cycled per stroke; ["D", "U"] to alternate; "auto" = down on the beat, up off it
PATTERN_SLOT_BEATS = 0.5        # --pattern: one character per eighth note
PATTERN_REPEATS = 4

# motion
STRIKE_DURATION = 0.12          # sweep across the strings
CONTACT_POINT = 0.5             # how far into the strike (0..1) the pick meets the strings; that moment lands on the note
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


# schedule
# Each stroke is (start, direction, strike_duration, recover_duration); start is when the strike begins, in seconds from the start of the run.

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
    """(onsets_from_zero, beats_into_measure, track_names). Rests are skipped; they just widen the gap."""
    sys.path.insert(0, os.path.join(REPO_ROOT, "Parsing"))
    from GuitarProReader import GuitarProSong

    song = GuitarProSong(path)
    names = [t.name for t in song.song.tracks]
    times, positions = [], []
    for measure in song.yield_timed_measures(track):
        for beat in measure.yield_timed_beats():
            if beat.beat.notes:
                times.append(beat.time)
                positions.append((beat.time - measure.time) * song.beat_frequency)
    return ([t - times[0] for t in times] if times else []), positions, names


def fallback_onsets(bpm=None):
    step = 60.0 / (BPM if bpm is None else bpm) * STRUM_INTERVAL_BEATS
    return [i * step for i in range(FALLBACK_STRUMS)]


def tempo_onsets(bpm, strokes, beats_per_stroke=0.5):
    """Evenly spaced onsets: 0.5 beats per stroke = eighth notes (two strums per beat)."""
    step = 60.0 / bpm * beats_per_stroke
    return [i * step for i in range(strokes)]


def parse_directions(value):
    """["D", "U"], "DU" or "auto" -> a list of "D"/"U", or the string "auto"."""
    if isinstance(value, str):
        if value.strip().lower() == "auto":
            return "auto"
        value = list(value.strip())
    value = [str(v).upper() for v in value]
    if not value or set(value) - {"D", "U"}:
        raise ValueError('directions are made of D and U (D, DU, DDU...), or "auto"')
    return value


def directions_from_positions(positions):
    """Pendulum strumming: down on the even slots of each measure, up on the odd ones.
    Slots are eighth notes, or sixteenths if any note falls between eighths."""
    def off_grid(grid):
        return any(abs(p / grid - round(p / grid)) > 0.05 for p in positions)

    grid = 0.25 if off_grid(0.5) else 0.5
    return ["D" if int(round(p / grid)) % 2 == 0 else "U" for p in positions]


def parse_pattern(text):
    """"D-DU-UDU" -> (slots in the pattern, [(slot, direction)]). "-" or "." is a slot with no stroke."""
    slots = [c for c in str(text).upper() if not c.isspace()]
    if not slots or set(slots) - set("DU-."):
        raise ValueError('a pattern is made of D, U and - (no stroke), e.g. "D-DU-UDU"')
    strokes = [(i, c) for i, c in enumerate(slots) if c in "DU"]
    if not strokes:
        raise ValueError("the pattern has no strokes in it")
    return len(slots), strokes


def pattern_onsets(pattern, bpm, repeats=None):
    """(onsets, directions) for a pattern played `repeats` times, PATTERN_SLOT_BEATS per character."""
    length, strokes = parse_pattern(pattern)
    repeats = PATTERN_REPEATS if repeats is None else repeats
    step = 60.0 / bpm * PATTERN_SLOT_BEATS
    first = strokes[0][0]
    onsets = [(r * length + slot - first) * step for r in range(repeats) for slot, _ in strokes]
    return onsets, [direction for _ in range(repeats) for _, direction in strokes]


def schedule_from_onsets(onsets, directions=None):
    """Note times (s) -> schedule; directions cycle per stroke (None = STRUM_DIRECTIONS).

    Each strike is placed so the pick meets the strings on the note
    (CONTACT_POINT of the way through the strike), and is shortened where notes
    are close together so strikes never overlap. The first strike still starts
    at onsets[0].
    """
    directions = parse_directions(STRUM_DIRECTIONS if directions is None else directions)
    if directions == "auto":
        directions = ["D", "U"]
    contact = max(0.0, min(1.0, CONTACT_POINT))
    tail = STRIKE_DURATION + RECOVER_DURATION
    strikes = []
    for i, onset in enumerate(onsets):
        room = (onsets[i + 1] - onset) if i + 1 < len(onsets) else tail
        if contact > 0 and i > 0:
            room = min(room, onset - onsets[i - 1])
        strikes.append(min(STRIKE_DURATION, room * 0.5))
    lead = strikes[0] * contact if strikes else 0.0
    starts = [onset - strike * contact + lead for onset, strike in zip(onsets, strikes)]
    schedule = []
    for i, (start, strike) in enumerate(zip(starts, strikes)):
        free = (starts[i + 1] - start if i + 1 < len(starts) else tail) - strike
        schedule.append((start, directions[i % len(directions)], strike, max(0.0, min(RECOVER_DURATION, free))))
    return schedule


def build_schedule(song=None, track=None, bpm=None, pattern=None, repeats=None, directions=None):
    """(schedule, where it came from). Anything left as None uses the constants at the top of this file.

    A pattern, or a bpm with no song, strums at constant tempo; so does song="none".
    Raises ValueError for settings that make no sense.
    """
    directions = parse_directions(STRUM_DIRECTIONS if directions is None else directions)
    if bpm is not None and not (20 <= bpm <= 300):
        raise ValueError("bpm must be between 20 and 300")
    if repeats is not None and not (1 <= repeats <= 200):
        raise ValueError("repeats must be between 1 and 200")
    if track is not None and track < 0:
        raise ValueError("track can't be negative")
    tempo = BPM if bpm is None else bpm

    if pattern:
        repeats = PATTERN_REPEATS if repeats is None else repeats
        onsets, pattern_directions = pattern_onsets(pattern, tempo, repeats)
        return schedule_from_onsets(onsets, pattern_directions), f'pattern "{pattern}" x{repeats} at {tempo:g} BPM'

    if song is None:
        song = None if bpm is not None else SONG_FILE
    elif song.strip().lower() == "none":
        song = None
    track = SONG_TRACK if track is None else track

    onsets, positions, source = [], [], f"{tempo:g} BPM constant tempo"
    if song:
        found = find_song(song)
        if not found:
            available = sorted(f for f in os.listdir(SONG_DIR) if f.lower().endswith((".gp3", ".gp4", ".gp5")))
            print(f"[strum] song not found: {song}")
            print(f"[strum]   songs available in Parsing/inputs: {', '.join(available) or 'none'}")
        else:
            try:
                onsets, positions, tracks = song_onsets(found, track)
                if onsets:
                    source = f"{os.path.basename(found)} track {track} ({tracks[track]})"
                    if bpm is not None:
                        print("[strum] --bpm is ignored for a song; the tab sets the tempo (use --speed to slow it).")
                else:
                    print(f"[strum] {os.path.basename(found)} track {track} has no played notes.")
                    print("[strum]   tracks: " + ", ".join(f"{i}={n}" for i, n in enumerate(tracks)))
            except ImportError:
                print("[strum] PyGuitarPro is not installed in this environment.")
                print("[strum]   fix with:  pip install PyGuitarPro")
            except IndexError:
                print(f"[strum] track {track} is out of range for this song.")
            except Exception as e:
                print(f"[strum] {os.path.basename(found)} failed to parse ({e})")
        if not onsets:
            print("[strum] using the constant-tempo fallback instead.")
    if not onsets:
        onsets = fallback_onsets(tempo)
        positions = [i * STRUM_INTERVAL_BEATS for i in range(len(onsets))]

    if directions == "auto":
        directions = directions_from_positions(positions)
    return schedule_from_onsets(onsets, directions), source


def add_song_args(parser):
    """--song / --track / --bpm / --pattern / --repeats / --directions, for any script that plays a schedule."""
    group = parser.add_argument_group("what to play (defaults are the constants in Sim/strum_motion.py)")
    group.add_argument("--song", help="Guitar Pro file: a name in Parsing/inputs or a path; 'none' = constant tempo")
    group.add_argument("--track", type=int, help="track number in the song")
    group.add_argument("--bpm", type=float,
                       help="constant-tempo strumming at this tempo (also the tempo of --pattern)")
    group.add_argument("--pattern",
                       help='strum pattern, one character per eighth note: D, U, - = no stroke, e.g. "D-DU-UDU"')
    group.add_argument("--repeats", type=int, help=f"times to play --pattern (default {PATTERN_REPEATS})")
    group.add_argument("--directions",
                       help="D, DU, DDU... cycled per stroke, or auto (down on the beat, up off it)")


def schedule_from_args(args):
    return build_schedule(song=args.song, track=args.track, bpm=args.bpm, pattern=args.pattern,
                          repeats=args.repeats, directions=args.directions)


# pose
# A down-stroke starts at the UP angle and sweeps to the DOWN angle; an up-stroke mirrors it. 
# So the follow-through of one stroke is already the ready position of the opposite stroke, and recovery only has distance to travel when two strokes repeat the same direction  
# which is exactly when the return trip would otherwise drag back through the strings. 
# The lift scales with that distance, so alternating strumming lifts nothing at all.

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

    if strike > 0 and elapsed <= strike:
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


def pose(yaw_deg, flex_deg, fingers=None):
    """Full 16-joint target dict in RADIANS, keyed by MJCF joint name.
    fingers = a finger_motion pose; None = the default fist."""
    if fingers is None:
        p = {name: d(angle * GRIP_CURL) for name, angle in FIST_POSE.items()}
    else:
        p = sim_joints(fingers)
    p["wrist_PRO"] = d(WRIST_PRO_BASE_DEG)
    p["wrist_UDEV"] = d(yaw_deg)
    p["wrist_FLEX"] = d(flex_deg)
    return p
