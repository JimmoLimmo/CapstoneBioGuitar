"""Play a learned MediaPipe strumming trajectory in the existing PyBullet guitar scene.

Run from any directory, pointing --project-root at the unchanged
CapstoneBioGuitar repository:

    python Sim/play_learned_strum.py motions/strum.npz \
        --project-root C:/Users/cerwo/Downloads/CapstoneBioGuitar/CapstoneBioGuitar

The player deliberately loads the project's right_hand.xml and guitar.urdf but
does not import, edit, or overwrite the project's Python modules.

Use --animate-fingers to apply MediaPipe's estimated finger flexion angles.
It is opt-in because a wrist-first strum is safer to calibrate before allowing
the fingers to approach the strings.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from time import perf_counter, sleep

import numpy as np
import pybullet as pb
import pybullet_data


# These are the validated placement values from the supplied Sim/mvmnt_demo.py.
HAND_ANCHOR_POS = [0.411, 0.000, 0.557]
HAND_ANCHOR_EULER_DEG = [-13.263, -18.947, -123.158]
GUITAR_POS = [-0.375, -0.268, 0.468]
DT = 1.0 / 240.0
MAX_CATCHUP_STEPS = 8
STRING_LINK_NAMES = {f"string{i}" for i in range(1, 7)}
FINGER_HINTS = ("thumb", "index", "middle", "ring", "pinky")

# Keep the existing demo's pick grip so visual changes come from the learned
# wrist motion rather than a different initial hand posture.
FIST_POSE_DEG = {
    "thumb_ABD": 15, "thumb_MCP": 45, "thumb_PIP": 55, "thumb_DIP": 35,
    "index_MCP": 70, "index_PIP": 95, "index_DIP": 60,
    "middle_MCP": 75, "middle_PIP": 100, "middle_DIP": 65,
    "ring_MCP": 75, "ring_PIP": 100, "ring_DIP": 65,
    "pinky_MCP": 75, "pinky_PIP": 100, "pinky_DIP": 65,
}

FINGER_LIMITS_DEG = {
    "thumb_MCP": (0.0, 90.0), "thumb_PIP": (0.0, 120.0), "thumb_DIP": (0.0, 80.0),
    "index_MCP": (0.0, 90.0), "index_PIP": (0.0, 120.0), "index_DIP": (0.0, 80.0),
    "middle_MCP": (0.0, 90.0), "middle_PIP": (0.0, 120.0), "middle_DIP": (0.0, 80.0),
    "ring_MCP": (0.0, 90.0), "ring_PIP": (0.0, 120.0), "ring_DIP": (0.0, 80.0),
    "pinky_MCP": (0.0, 90.0), "pinky_PIP": (0.0, 120.0), "pinky_DIP": (0.0, 80.0),
}

WRIST_JOINTS = {"wrist_UDEV", "wrist_FLEX", "wrist_PRO"}


def smoothstep(value: float) -> float:
    """Cubic blend with zero velocity at both endpoints."""
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def degrees(values):
    return {name: math.radians(value) for name, value in values.items()}


class JointMotionLimiter:
    """Rate and acceleration limiter in real time, preventing target snaps."""

    def __init__(self, wrist_speed_deg_s, wrist_accel_deg_s2, finger_speed_deg_s, finger_accel_deg_s2):
        self.wrist_speed = math.radians(wrist_speed_deg_s)
        self.wrist_accel = math.radians(wrist_accel_deg_s2)
        self.finger_speed = math.radians(finger_speed_deg_s)
        self.finger_accel = math.radians(finger_accel_deg_s2)
        self.position = {}
        self.velocity = {}

    def reset(self, positions):
        self.position = dict(positions)
        self.velocity = {name: 0.0 for name in positions}

    def limit(self, wanted, dt):
        # Long application stalls must not create one large catch-up command.
        dt = max(1e-4, min(float(dt), 0.05))
        limited = {}
        for name, desired in wanted.items():
            position = self.position.get(name, desired)
            velocity = self.velocity.get(name, 0.0)
            if name in WRIST_JOINTS:
                max_speed, max_accel = self.wrist_speed, self.wrist_accel
            else:
                max_speed, max_accel = self.finger_speed, self.finger_accel

            desired_velocity = float(np.clip((desired - position) / dt, -max_speed, max_speed))
            velocity += float(np.clip(desired_velocity - velocity, -max_accel * dt, max_accel * dt))
            next_position = position + velocity * dt
            # Do not overshoot a target when braking from a previous stroke.
            if (desired - position) * (desired - next_position) <= 0.0:
                next_position, velocity = desired, 0.0
            self.position[name] = next_position
            self.velocity[name] = velocity
            limited[name] = next_position
        return limited


class StrumContactReporter:
    """Groups string contacts into individual strum passes."""

    def __init__(self, release_seconds):
        self.release_seconds = release_seconds
        self.active = None
        self.records = []

    def observe(self, time_s, desired_udev_rad, strings):
        if strings:
            if self.active is None:
                self.active = {
                    "start_s": time_s,
                    "last_s": time_s,
                    "start_udev_deg": math.degrees(desired_udev_rad),
                    "last_udev_deg": math.degrees(desired_udev_rad),
                    "strings": set(strings),
                }
            else:
                self.active["last_s"] = time_s
                self.active["last_udev_deg"] = math.degrees(desired_udev_rad)
                self.active["strings"].update(strings)
        elif self.active is not None and time_s - self.active["last_s"] >= self.release_seconds:
            self._finish()

    def close(self):
        if self.active is not None:
            self._finish()

    def _finish(self):
        record = self.active
        self.active = None
        delta = record["last_udev_deg"] - record["start_udev_deg"]
        if delta < -1.0:
            direction = "toward -UDEV"
        elif delta > 1.0:
            direction = "toward +UDEV"
        else:
            direction = "stationary"
        record["direction"] = direction
        record["string_names"] = tuple(sorted(record.pop("strings")))
        self.records.append(record)
        print(
            f"[strum-pass] #{len(self.records):<3} {record['start_s']:6.2f}-{record['last_s']:6.2f}s "
            f"{direction:13} UDEV {record['start_udev_deg']:+5.1f}->{record['last_udev_deg']:+5.1f} "
            f"strings: {','.join(record['string_names'])}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("motion", type=Path, help=".npz created by train_strum_from_video.py")
    parser.add_argument("--project-root", type=Path, default=Path.cwd(),
                        help="CapstoneBioGuitar repository root (default: current directory)")
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed (default: 1)")
    parser.add_argument("--repeat", type=int, default=1, help="number of times to play the learned pattern")
    parser.add_argument("--headless", action="store_true", help="run PyBullet without its GUI")
    parser.add_argument("--animate-fingers", action="store_true",
                        help="also replay estimated MediaPipe finger flexion (first use only after wrist calibration)")
    parser.add_argument("--finger-gain", type=float, default=1.0,
                        help="scale recorded finger flexion when --animate-fingers is enabled (default: 1)")
    parser.add_argument("--loop-blend-seconds", type=float, default=0.25,
                        help="lifted, smooth return between repetitions; zero disables it (default: 0.25)")
    parser.add_argument("--loop-lift-deg", type=float, default=15.0,
                        help="extra wrist_FLEX during each repeat-boundary blend (default: 15)")
    parser.add_argument("--max-wrist-speed", type=float, default=360.0,
                        help="maximum wrist target speed in degrees/real second (default: 360)")
    parser.add_argument("--max-wrist-accel", type=float, default=3600.0,
                        help="maximum wrist target acceleration in degrees/real second^2 (default: 3600)")
    parser.add_argument("--max-finger-speed", type=float, default=540.0,
                        help="maximum finger target speed in degrees/real second (default: 540)")
    parser.add_argument("--max-finger-accel", type=float, default=5400.0,
                        help="maximum finger target acceleration in degrees/real second^2 (default: 5400)")
    parser.add_argument("--contact-release-seconds", type=float, default=0.06,
                        help="no-contact gap that ends one reported strum pass (default: 0.06)")
    parser.add_argument("--contact-log", type=Path,
                        help="optional CSV path for individual detected strum passes")
    return parser.parse_args()


def load_motion(path: Path):
    if not path.is_file():
        raise SystemExit(f"Motion file not found: {path}")
    with np.load(path, allow_pickle=False) as data:
        required = ("times", "yaw_deg", "flex_deg", "pro_deg")
        missing = [name for name in required if name not in data]
        if missing:
            raise SystemExit(f"{path} is not a learned strum file; missing {missing}")
        arrays = [np.asarray(data[name], dtype=float) for name in required]
        finger_joint_deg = None
        finger_joint_names = ()
        if "finger_joint_deg" in data and "finger_joint_names" in data:
            finger_joint_deg = np.asarray(data["finger_joint_deg"], dtype=float)
            finger_joint_names = tuple(str(name) for name in data["finger_joint_names"].tolist())
        metadata = json.loads(str(data["metadata_json"].item())) if "metadata_json" in data else {}
    times, yaw, flex, pro = arrays
    if len(times) < 2 or any(len(a) != len(times) for a in arrays[1:]):
        raise SystemExit("Motion samples must contain matching arrays with at least two points.")
    if not np.all(np.diff(times) > 0):
        raise SystemExit("Motion time samples must be strictly increasing.")
    if finger_joint_deg is not None and finger_joint_deg.shape != (len(times), len(finger_joint_names)):
        raise SystemExit("Finger motion data has an unexpected shape.")
    return times, yaw, flex, pro, finger_joint_deg, finger_joint_names, metadata


def main() -> None:
    args = parse_args()
    if args.speed <= 0:
        raise SystemExit("--speed must be positive")
    if args.repeat <= 0:
        raise SystemExit("--repeat must be positive")
    nonnegative = (
        "loop_blend_seconds", "loop_lift_deg", "max_wrist_speed", "max_wrist_accel",
        "max_finger_speed", "max_finger_accel",
    )
    if any(getattr(args, name) < 0 for name in nonnegative):
        raise SystemExit("Loop, lift, speed, and acceleration values cannot be negative")
    if args.contact_release_seconds <= 0:
        raise SystemExit("--contact-release-seconds must be positive")
    root = args.project_root.resolve()
    hand_model = root / "Sim" / "MPL" / "right_hand.xml"
    guitar_model = root / "Sim" / "guitar_model" / "guitar.urdf"
    for asset in (hand_model, guitar_model):
        if not asset.is_file():
            raise SystemExit(f"Expected project asset not found: {asset}")

    times, yaw, flex, pro, finger_joint_deg, finger_joint_names, metadata = load_motion(args.motion)
    if args.finger_gain < 0:
        raise SystemExit("--finger-gain cannot be negative")
    if args.animate_fingers and finger_joint_deg is None:
        raise SystemExit("This motion file has no finger data. Train it again with the current train_strum_from_video.py.")
    trajectory_start = float(times[0])
    duration = float(times[-1] - times[0])
    if duration <= 0:
        raise SystemExit("The learned motion has no playable duration.")
    total_duration = duration * args.repeat + args.loop_blend_seconds * (args.repeat - 1)
    print(
        f"[learned-strum] {len(times)} samples, {duration:.2f} s each, {args.repeat} playthrough(s) "
        f"at {args.speed:g}x; total {total_duration:.2f} s"
    )
    if metadata:
        print(f"[learned-strum] source: {metadata.get('source_video', 'unknown')}")

    connection = pb.connect(pb.DIRECT if args.headless else pb.GUI)
    if connection < 0:
        raise SystemExit("PyBullet could not start.")
    try:
        pb.setGravity(0, 0, -9.81)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.loadURDF("plane.urdf")
        pb.setTimeStep(DT)
        pb.setPhysicsEngineParameter(
            numSolverIterations=200,
            numSubSteps=4,
            contactBreakingThreshold=0.00001,
            enableConeFriction=1,
        )

        hand = pb.loadMJCF(str(hand_model))[0]
        guitar = pb.loadURDF(
            str(guitar_model),
            basePosition=GUITAR_POS,
            baseOrientation=pb.getQuaternionFromEuler([0, 0, math.pi / 2]),
            useFixedBase=True,
        )
        pb.createConstraint(
            parentBodyUniqueId=hand, parentLinkIndex=-1,
            childBodyUniqueId=-1, childLinkIndex=-1,
            jointType=pb.JOINT_FIXED, jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=HAND_ANCHOR_POS,
            childFrameOrientation=pb.getQuaternionFromEuler([math.radians(v) for v in HAND_ANCHOR_EULER_DEG]),
        )
        if not args.headless:
            pb.resetDebugVisualizerCamera(
                cameraDistance=0.5, cameraYaw=45, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.55]
            )

        joint_index = {}
        hand_links = {-1: "base"}
        for i in range(pb.getNumJoints(hand)):
            info = pb.getJointInfo(hand, i)
            if info[2] != pb.JOINT_FIXED:
                joint_index[info[1].decode()] = i
            hand_links[i] = info[12].decode()
        needed = {"wrist_PRO", "wrist_UDEV", "wrist_FLEX"}
        if missing := needed - set(joint_index):
            raise RuntimeError(f"{hand_model} is missing joints {sorted(missing)}")

        guitar_links = {-1: "base"}
        for i in range(pb.getNumJoints(guitar)):
            guitar_links[i] = pb.getJointInfo(guitar, i)[12].decode()
        string_links = {i for i, name in guitar_links.items() if name in STRING_LINK_NAMES}

        fixed_pose = degrees(FIST_POSE_DEG)

        def trajectory_target_at(relative_time: float):
            """Unrestricted learned target at a time within one recorded take."""
            play_time = trajectory_start + max(0.0, min(relative_time, duration))
            target = dict(fixed_pose)
            target["wrist_UDEV"] = math.radians(float(np.interp(play_time, times, yaw)))
            target["wrist_FLEX"] = math.radians(float(np.interp(play_time, times, flex)))
            target["wrist_PRO"] = math.radians(float(np.interp(play_time, times, pro)))
            if args.animate_fingers:
                for column, name in enumerate(finger_joint_names):
                    if name in FINGER_LIMITS_DEG:
                        low, high = FINGER_LIMITS_DEG[name]
                        value = np.interp(play_time, times, finger_joint_deg[:, column]) * args.finger_gain
                        target[name] = math.radians(float(np.clip(value, low, high)))
            return target

        first_target = trajectory_target_at(0.0)
        final_target = trajectory_target_at(duration)

        def target_for_playback(playback_time: float):
            """Trajectory target, including a lifted smooth transition between repeats."""
            cycle_duration = duration + args.loop_blend_seconds
            cycle = min(int(playback_time / cycle_duration), args.repeat - 1)
            cycle_time = playback_time - cycle * cycle_duration
            if cycle < args.repeat - 1 and cycle_time > duration:
                blend = smoothstep((cycle_time - duration) / args.loop_blend_seconds)
                target = {
                    name: final_target[name] + (first_target[name] - final_target[name]) * blend
                    for name in final_target
                }
                # Lift while travelling back to the ready pose so the return
                # is less likely to drag across the strings.
                target["wrist_FLEX"] += math.radians(args.loop_lift_deg) * math.sin(math.pi * blend)
                return target
            return trajectory_target_at(cycle_time)

        limiter = JointMotionLimiter(
            args.max_wrist_speed,
            args.max_wrist_accel,
            args.max_finger_speed,
            args.max_finger_accel,
        )
        limiter.reset({name: pb.getJointState(hand, index)[0] for name, index in joint_index.items()})

        def apply_target(desired_target, control_dt: float) -> None:
            target = limiter.limit(desired_target, control_dt)
            names = [name for name in target if name in joint_index]
            pb.setJointMotorControlArray(
                hand,
                [joint_index[name] for name in names],
                pb.POSITION_CONTROL,
                targetPositions=[target[name] for name in names],
                forces=[10.0] * len(names),
            )

        def touched_strings() -> set[str]:
            hit = set()
            for contact in pb.getContactPoints(hand, guitar):
                hand_link, guitar_link = contact[3], contact[4]
                if guitar_link in string_links and any(hint in hand_links.get(hand_link, "") for hint in FINGER_HINTS):
                    hit.add(guitar_links[guitar_link])
            return hit

        start = perf_counter()
        last_control_time = start
        sim_time = 0.0
        reporter = StrumContactReporter(args.contact_release_seconds)
        while True:
            now_clock = perf_counter()
            playback_elapsed = (now_clock - start) * args.speed
            if playback_elapsed > total_duration:
                break
            desired_target = target_for_playback(playback_elapsed)
            apply_target(desired_target, now_clock - last_control_time)
            last_control_time = now_clock
            steps = 0
            target_sim_time = now_clock - start
            while sim_time < target_sim_time and steps < MAX_CATCHUP_STEPS:
                pb.stepSimulation()
                sim_time += DT
                steps += 1
            if sim_time < target_sim_time:
                sim_time = target_sim_time
            reporter.observe(playback_elapsed, desired_target["wrist_UDEV"], touched_strings())
            if sim_time > target_sim_time:
                sleep(sim_time - target_sim_time)

        reporter.close()
        if args.contact_log:
            args.contact_log.parent.mkdir(parents=True, exist_ok=True)
            with args.contact_log.open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=("pass_number", "start_s", "end_s", "direction", "start_udev_deg", "end_udev_deg", "strings"),
                )
                writer.writeheader()
                for number, record in enumerate(reporter.records, start=1):
                    writer.writerow({
                        "pass_number": number,
                        "start_s": f"{record['start_s']:.4f}",
                        "end_s": f"{record['last_s']:.4f}",
                        "direction": record["direction"],
                        "start_udev_deg": f"{record['start_udev_deg']:.3f}",
                        "end_udev_deg": f"{record['last_udev_deg']:.3f}",
                        "strings": ",".join(record["string_names"]),
                    })
            print(f"[learned-strum] wrote {args.contact_log}")
        print(f"[learned-strum] done; detected {len(reporter.records)} string-contact pass(es).")
    finally:
        pb.disconnect()


if __name__ == "__main__":
    main()
