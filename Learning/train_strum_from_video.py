"""Turn a strumming video into a reusable robot-hand motion template.

This is intentionally a demonstration-learning program, not a neural-network
trainer: one video supplies one motion example, so fitting a neural network
would only overfit it.  Instead, MediaPipe tracks the strumming hand and this
program learns a smooth, re-timed wrist trajectory for the existing PyBullet
guitar simulation and saves all 21 MediaPipe landmarks for later finger
retargeting.  It also estimates every finger's flexion angles; the player can
use those estimates with its --animate-fingers option.

Example (run from the CapstoneBioGuitar repository root):

    python path/to/train_strum_from_video.py videos/strum.mp4 motions/strum.npz \
        --model MediaPipe/hand_landmarker.task --display

The output .npz is consumed by play_learned_strum.py.  A companion CSV is also
written for inspection in a spreadsheet.  No project file is changed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode


LANDMARK_NAMES = (
    "wrist", "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
    "index_mcp", "index_pip", "index_dip", "index_tip",
    "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
    "ring_mcp", "ring_pip", "ring_dip", "ring_tip",
    "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip",
)

# MediaPipe sees three flexion bends per finger.  The robot's abduction joints
# are deliberately not inferred from one camera view: their sign depends on
# camera orientation, whereas flexion is robust and useful for a first pass.
FINGER_JOINT_NAMES = (
    "thumb_MCP", "thumb_PIP", "thumb_DIP",
    "index_MCP", "index_PIP", "index_DIP",
    "middle_MCP", "middle_PIP", "middle_DIP",
    "ring_MCP", "ring_PIP", "ring_DIP",
    "pinky_MCP", "pinky_PIP", "pinky_DIP",
)

HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
)
FINGERTIP_INDICES = {4, 8, 12, 16, 20}


def unit_vector(values: Iterable[float], name: str) -> np.ndarray:
    vector = np.asarray(tuple(values), dtype=float)
    length = float(np.linalg.norm(vector))
    if length < 1e-9:
        raise ValueError(f"{name} must not be [0, 0]")
    return vector / length


def unwrap_degrees(values: np.ndarray) -> np.ndarray:
    """Unwrap a sequence of angles measured in degrees."""
    return np.rad2deg(np.unwrap(np.deg2rad(values)))


def moving_average(values: np.ndarray, sample_hz: float, seconds: float) -> np.ndarray:
    """Centered moving average with edge padding; zero disables smoothing."""
    radius = int(round(seconds * sample_hz))
    if radius <= 0 or len(values) < 3:
        return values.copy()
    kernel = np.ones(2 * radius + 1, dtype=float) / (2 * radius + 1)
    padded = np.pad(values, (radius, radius), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def interpolate_samples(source_times: np.ndarray, source_values: np.ndarray, target_times: np.ndarray) -> np.ndarray:
    """Linearly interpolate an N-by-anything sequence along its time axis."""
    trailing_shape = source_values.shape[1:]
    flat = source_values.reshape(len(source_values), -1)
    interpolated = np.column_stack([
        np.interp(target_times, source_times, flat[:, column])
        for column in range(flat.shape[1])
    ])
    return interpolated.reshape((len(target_times),) + trailing_shape)


def bend_degrees(points: np.ndarray, first: int, joint: int, third: int) -> float:
    """Return the 0-to-180 degree bend at one landmark joint."""
    incoming = points[joint] - points[first]
    outgoing = points[third] - points[joint]
    denominator = float(np.linalg.norm(incoming) * np.linalg.norm(outgoing))
    if denominator < 1e-9:
        return 0.0
    cosine = float(np.clip(np.dot(incoming, outgoing) / denominator, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def wrist_relative_landmarks(points: np.ndarray) -> np.ndarray:
    """Remove camera translation and hand size while retaining finger shape."""
    palm_width = float(np.linalg.norm(points[5] - points[17]))
    if palm_width < 1e-9:
        palm_width = 1.0
    return (points - points[0]) / palm_width


def finger_flexion_degrees(points: np.ndarray) -> np.ndarray:
    """Estimate the three flexion angles for thumb and four fingers."""
    chains = (
        (0, 1, 2, 3, 4),       # thumb starts from wrist
        (0, 5, 6, 7, 8),
        (0, 9, 10, 11, 12),
        (0, 13, 14, 15, 16),
        (0, 17, 18, 19, 20),
    )
    bends = []
    for root, mcp, pip, dip, tip in chains:
        bends.extend((
            bend_degrees(points, root, mcp, pip),
            bend_degrees(points, mcp, pip, dip),
            bend_degrees(points, pip, dip, tip),
        ))
    return np.asarray(bends, dtype=float)


def draw_hand_overlay(frame: np.ndarray, landmarks) -> None:
    """Draw every tracked joint and its bone connections for --display."""
    height, width = frame.shape[:2]
    pixels = [(int(point.x * width), int(point.y * height)) for point in landmarks]
    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, pixels[start], pixels[end], (255, 180, 0), 2)
    for index, pixel in enumerate(pixels):
        color = (0, 255, 0) if index == 0 else ((0, 255, 255) if index in FINGERTIP_INDICES else (255, 80, 80))
        cv2.circle(frame, pixel, 5, color, -1)


def scale_to_range(values: np.ndarray, low: float, high: float, invert: bool = False) -> np.ndarray:
    """Map robust 5th/95th percentile limits to a joint-angle range."""
    p5, p95 = np.percentile(values, [5, 95])
    if abs(p95 - p5) < 1e-8 or abs(high - low) < 1e-8:
        return np.full_like(values, (low + high) / 2.0, dtype=float)
    fraction = np.clip((values - p5) / (p95 - p5), 0.0, 1.0)
    if invert:
        fraction = 1.0 - fraction
    return low + fraction * (high - low)


def select_hand(result, wanted_hand: str):
    """Return the matching detected hand index, or None.

    MediaPipe's handedness label is the subject's hand in a non-mirrored image.
    Select ``any`` if the video has been mirrored or this label is unreliable.
    """
    if not result.hand_landmarks:
        return None
    if wanted_hand == "any":
        return 0
    for i, categories in enumerate(result.handedness):
        if categories and categories[0].category_name.lower() == wanted_hand:
            return i
    return None


def landmarks_to_features(landmarks, world_landmarks, strum_axis: np.ndarray):
    """Extract camera-space sweep, palm-normal flexion, and palm-roll signals."""
    image = np.array([(p.x, p.y, p.z) for p in landmarks], dtype=float)
    wrist_xy = image[0, :2]
    sweep = float(np.dot(wrist_xy, strum_axis))

    # Palm roll in image space. It is useful when the physical hand rotates
    # about the forearm during a stroke. unwrap_degrees() handles the -180/180
    # boundary after all video frames have been collected.
    across_palm = image[17, :2] - image[5, :2]
    pro = math.degrees(math.atan2(across_palm[1], across_palm[0]))

    # The normal is invariant to camera translation. World landmarks give the
    # most stable result; image-space 3-D landmarks are a fallback for older
    # MediaPipe task files that do not return world landmarks.
    points = np.array([(p.x, p.y, p.z) for p in (world_landmarks or landmarks)], dtype=float)
    to_index = points[5] - points[0]
    to_pinky = points[17] - points[0]
    normal = np.cross(to_index, to_pinky)
    normal_length = float(np.linalg.norm(normal))
    flex = float(normal[2] / normal_length) if normal_length > 1e-9 else 0.0
    return sweep, flex, pro, wrist_xy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", type=Path, help="input strumming video")
    parser.add_argument("output", type=Path, help="learned motion output (.npz)")
    parser.add_argument("--model", type=Path, required=True, help="MediaPipe hand_landmarker.task model")
    parser.add_argument("--hand", choices=("right", "left", "any"), default="right",
                        help="strumming hand to follow (default: right; use any for a mirrored video)")
    parser.add_argument("--strum-axis", nargs=2, type=float, metavar=("X", "Y"), default=(0.0, 1.0),
                        help="screen direction of a down-strum; default is down the image")
    parser.add_argument("--sample-hz", type=float, default=60.0, help="output trajectory rate (default: 60)")
    parser.add_argument("--smooth-seconds", type=float, default=0.06,
                        help="centered smoothing width applied after resampling (default: 0.06)")
    parser.add_argument("--yaw-min", type=float, default=-20.0, help="robot wrist_UDEV minimum, degrees")
    parser.add_argument("--yaw-max", type=float, default=25.0, help="robot wrist_UDEV maximum, degrees")
    parser.add_argument("--invert-yaw", action="store_true",
                        help="reverse the video-to-wrist_UDEV mapping")
    parser.add_argument("--flex-min", type=float, default=0.0, help="robot wrist_FLEX minimum, degrees")
    parser.add_argument("--flex-max", type=float, default=15.0, help="robot wrist_FLEX maximum, degrees")
    parser.add_argument("--invert-flex", action="store_true", help="reverse the palm-normal-to-FLEX mapping")
    parser.add_argument("--pro-min", type=float, default=0.0, help="robot wrist_PRO minimum, degrees")
    parser.add_argument("--pro-max", type=float, default=0.0, help="robot wrist_PRO maximum, degrees")
    parser.add_argument("--invert-pro", action="store_true", help="reverse the palm-roll-to-PRO mapping")
    parser.add_argument("--display", action="store_true", help="show tracking while processing; press q to stop")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.sample_hz <= 0:
        raise SystemExit("--sample-hz must be positive")
    if args.smooth_seconds < 0:
        raise SystemExit("--smooth-seconds cannot be negative")
    if not args.video.is_file():
        raise SystemExit(f"Video not found: {args.video}")
    if not args.model.is_file():
        raise SystemExit(f"MediaPipe model not found: {args.model}")

    strum_axis = unit_vector(args.strum_axis, "--strum-axis")
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise SystemExit(f"OpenCV could not open: {args.video}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(fps) or fps <= 0:
        fps = 30.0

    observations = []
    image_landmark_observations = []
    relative_landmark_observations = []
    finger_joint_observations = []
    frames_read = 0
    last_timestamp_ms = -1
    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(args.model)),
        running_mode=RunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    try:
        with HandLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                frames_read += 1
                timestamp_s = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                if timestamp_s <= 0:
                    timestamp_s = (frames_read - 1) / fps
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                timestamp_ms = int(round(timestamp_s * 1000))
                # Some video codecs expose repeated POS_MSEC values. The Tasks
                # VIDEO API requires strictly increasing timestamps.
                timestamp_ms = max(timestamp_ms, last_timestamp_ms + 1)
                last_timestamp_ms = timestamp_ms
                timestamp_s = timestamp_ms / 1000.0
                result = landmarker.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                    timestamp_ms,
                )
                hand_index = select_hand(result, args.hand)
                if hand_index is not None:
                    world = result.hand_world_landmarks[hand_index] if result.hand_world_landmarks else None
                    sweep, flex, pro, wrist_xy = landmarks_to_features(
                        result.hand_landmarks[hand_index], world, strum_axis
                    )
                    observations.append((timestamp_s, sweep, flex, pro, wrist_xy[0], wrist_xy[1]))
                    image_points = np.array(
                        [(point.x, point.y, point.z) for point in result.hand_landmarks[hand_index]], dtype=float
                    )
                    # World landmarks retain the most useful 3-D finger shape.
                    # The image landmarks remain available in the output too,
                    # so a later camera calibration can use pixel coordinates.
                    shape_points = np.array(
                        [(point.x, point.y, point.z) for point in (world or result.hand_landmarks[hand_index])],
                        dtype=float,
                    )
                    image_landmark_observations.append(image_points)
                    relative_landmark_observations.append(wrist_relative_landmarks(shape_points))
                    finger_joint_observations.append(finger_flexion_degrees(shape_points))
                    if args.display:
                        draw_hand_overlay(frame, result.hand_landmarks[hand_index])
                        h, w = frame.shape[:2]
                        x, y = int(wrist_xy[0] * w), int(wrist_xy[1] * h)
                        cv2.circle(frame, (x, y), 8, (0, 255, 0), 2)
                        cv2.putText(frame, "tracked", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                elif args.display:
                    cv2.putText(frame, "hand not found", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                if args.display:
                    cv2.imshow("Strum motion capture (q = stop)", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    finally:
        capture.release()
        if args.display:
            cv2.destroyAllWindows()

    if len(observations) < 10:
        raise SystemExit(
            f"Only {len(observations)} usable hand frames were found. "
            "Use --hand any for mirrored video, improve lighting, or show the whole strumming hand."
        )

    observed = np.asarray(observations, dtype=float)
    # Start the learned motion at the first confident observation. Short gaps
    # are linearly filled during resampling; this preserves the song timing.
    observed[:, 0] -= observed[0, 0]
    duration = float(observed[-1, 0])
    if duration <= 0:
        raise SystemExit("The video contains no usable time span.")
    times = np.arange(0.0, duration + 0.5 / args.sample_hz, 1.0 / args.sample_hz)
    raw_sweep = np.interp(times, observed[:, 0], observed[:, 1])
    raw_flex = np.interp(times, observed[:, 0], observed[:, 2])
    raw_pro = np.interp(times, observed[:, 0], unwrap_degrees(observed[:, 3]))
    image_landmarks_xyz = interpolate_samples(
        observed[:, 0], np.asarray(image_landmark_observations, dtype=float), times
    )
    finger_landmarks_relative_xyz = interpolate_samples(
        observed[:, 0], np.asarray(relative_landmark_observations, dtype=float), times
    )
    finger_joint_deg = interpolate_samples(
        observed[:, 0], np.asarray(finger_joint_observations, dtype=float), times
    )
    for column in range(finger_joint_deg.shape[1]):
        finger_joint_deg[:, column] = moving_average(
            finger_joint_deg[:, column], args.sample_hz, args.smooth_seconds
        )

    yaw_deg = moving_average(
        scale_to_range(raw_sweep, args.yaw_min, args.yaw_max, args.invert_yaw), args.sample_hz, args.smooth_seconds
    )
    flex_deg = moving_average(
        scale_to_range(raw_flex, args.flex_min, args.flex_max, args.invert_flex), args.sample_hz, args.smooth_seconds
    )
    pro_deg = moving_average(
        scale_to_range(raw_pro, args.pro_min, args.pro_max, args.invert_pro), args.sample_hz, args.smooth_seconds
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    metadata = {
        "format": "capstone_bioguitar_learned_strum_v1",
        "source_video": str(args.video.resolve()),
        "model": str(args.model.resolve()),
        "hand": args.hand,
        "input_fps": fps,
        "frames_read": frames_read,
        "frames_tracked": len(observations),
        "tracking_coverage": len(observations) / max(frames_read, 1),
        "sample_hz": args.sample_hz,
        "strum_axis": strum_axis.tolist(),
        "yaw_range_deg": [args.yaw_min, args.yaw_max],
        "flex_range_deg": [args.flex_min, args.flex_max],
        "pro_range_deg": [args.pro_min, args.pro_max],
        "landmarks": "21 MediaPipe landmarks; image_landmarks_xyz is camera-normalized, finger_landmarks_relative_xyz is wrist-relative and palm-width normalized",
        "finger_joints": "15 estimated flexion angles, ordered by finger_joint_names; abduction is not inferred",
    }
    np.savez_compressed(
        args.output,
        times=times,
        yaw_deg=yaw_deg,
        flex_deg=flex_deg,
        pro_deg=pro_deg,
        tracked_times=observed[:, 0],
        tracked_wrist_xy=observed[:, 4:6],
        landmark_names=np.asarray(LANDMARK_NAMES),
        finger_joint_names=np.asarray(FINGER_JOINT_NAMES),
        image_landmarks_xyz=image_landmarks_xyz,
        tracked_image_landmarks_xyz=np.asarray(image_landmark_observations, dtype=float),
        finger_landmarks_relative_xyz=finger_landmarks_relative_xyz,
        finger_joint_deg=finger_joint_deg,
        metadata_json=json.dumps(metadata, indent=2),
    )

    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        landmark_columns = [
            f"finger_{name}_{axis}" for name in LANDMARK_NAMES for axis in ("x", "y", "z")
        ]
        writer.writerow(("time_s", "wrist_UDEV_deg", "wrist_FLEX_deg", "wrist_PRO_deg", *landmark_columns))
        for time_s, udev, wrist_flex, wrist_pro, finger_landmarks in zip(
            times, yaw_deg, flex_deg, pro_deg, finger_landmarks_relative_xyz
        ):
            writer.writerow((time_s, udev, wrist_flex, wrist_pro, *finger_landmarks.reshape(-1)))

    print(f"Learned {len(times)} motion samples across {duration:.2f} s.")
    print(f"Tracked {len(observations)}/{frames_read} frames ({metadata['tracking_coverage']:.0%} coverage).")
    print("Saved all 21 landmarks and 15 estimated finger-flexion angles in the NPZ output.")
    print(f"Wrote {args.output}")
    print(f"Wrote {csv_path}")
    if metadata["tracking_coverage"] < 0.7:
        print("WARNING: tracking coverage is below 70%; review the video or use --display.")


if __name__ == "__main__":
    main()
