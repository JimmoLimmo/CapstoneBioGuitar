"""Mirror four MediaPipe-tracked fingers to the prosthetic hand's servos.

Each finger has three tracked human joints:
    MCP / knuckle  -> landmark angle wrist-MCP-PIP
    PIP / middle   -> landmark angle MCP-PIP-DIP
    DIP / top      -> landmark angle PIP-DIP-tip

The hardware uses TWO servos per finger:
    bottom servo: mechanically couples MCP + PIP (one weighted command)
    top servo:    drives DIP alone

The existing bottom-servo channels (12--15) are retained. Fill in the four
``tip.channel`` values below after confirming the PCA9685 wiring. A None
channel is intentionally skipped, never guessed, so this script cannot move an
unknown servo channel.

Run from the repository root:
    python Imitation/mimic.py

Software-only test from the webcam:
    python Imitation/mimic.py --dry-run

Software-only test from a saved video:
    python Imitation/mimic.py --dry-run --video C:/path/to/hand_video.mov
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode


HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE.parent / "MediaPipe" / "hand_landmarker.task"

# MediaPipe landmark order for the four non-thumb fingers.
FINGER_INDICES = {
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}

HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
)

# Servo calibration ---------------------------------------------------------
# Angle 170 is a straight finger; angle 0 is fully bent. These pulse values
# preserve the direction used by the original mimic.py bottom-servo mapping.
# If a particular servo moves backward, swap its pwm_straight and pwm_bent.
#
# IMPORTANT: Set each tip.channel only after its wire has been identified. The
# known bottom channels are kept exactly as they were in the original file.
FINGER_SERVOS = {
    "index": {
        "bottom": {"channel": 15, "mcp_weight": 0.5, "pip_weight": 0.5,
                   "pwm_straight": 1638, "pwm_bent": 7000},
        "tip": {"channel": None, "pwm_straight": 1638, "pwm_bent": 7000},  # TODO: PCA channel
    },
    "middle": {
        "bottom": {"channel": 14, "mcp_weight": 0.5, "pip_weight": 0.5,
                   "pwm_straight": 1638, "pwm_bent": 7000},
        "tip": {"channel": None, "pwm_straight": 1638, "pwm_bent": 7000},  # TODO: PCA channel
    },
    "ring": {
        "bottom": {"channel": 13, "mcp_weight": 0.5, "pip_weight": 0.5,
                   "pwm_straight": 1638, "pwm_bent": 7000},
        "tip": {"channel": None, "pwm_straight": 1638, "pwm_bent": 7000},  # TODO: PCA channel
    },
    "pinky": {
        "bottom": {"channel": 12, "mcp_weight": 0.5, "pip_weight": 0.5,
                   "pwm_straight": 1638, "pwm_bent": 7000},
        "tip": {"channel": None, "pwm_straight": 1638, "pwm_bent": 7000},  # TODO: PCA channel
    },
}

STRAIGHT_ANGLE_DEG = 170.0
SMOOTHING_ALPHA = 0.35  # lower = smoother/laggier, higher = more responsive


def interior_angle(a, b, c) -> float:
    """Return the 0--180 degree angle at landmark b; 180 is straight."""
    ba = np.array((a.x - b.x, a.y - b.y, a.z - b.z), dtype=float)
    bc = np.array((c.x - b.x, c.y - b.y, c.z - b.z), dtype=float)
    denominator = float(np.linalg.norm(ba) * np.linalg.norm(bc))
    if denominator < 1e-9:
        return STRAIGHT_ANGLE_DEG
    cosine = float(np.clip(np.dot(ba, bc) / denominator, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def get_finger_angles(hand_landmarks):
    """Return MCP, PIP, DIP, and their mechanically coupled bottom command."""
    angles = {}
    wrist = hand_landmarks[0]
    for name, (mcp_i, pip_i, dip_i, tip_i) in FINGER_INDICES.items():
        mcp = interior_angle(wrist, hand_landmarks[mcp_i], hand_landmarks[pip_i])
        pip = interior_angle(hand_landmarks[mcp_i], hand_landmarks[pip_i], hand_landmarks[dip_i])
        dip = interior_angle(hand_landmarks[pip_i], hand_landmarks[dip_i], hand_landmarks[tip_i])
        bottom = FINGER_SERVOS[name]["bottom"]
        coupled = bottom["mcp_weight"] * mcp + bottom["pip_weight"] * pip
        angles[name] = {"mcp": mcp, "pip": pip, "dip": dip, "coupled": coupled}
    return angles


def angle_to_pwm(angle_deg: float, servo_config: dict) -> int:
    """Finger angle (straight=170, bent=0) to a calibrated PCA9685 pulse."""
    angle = float(np.clip(angle_deg, 0.0, STRAIGHT_ANGLE_DEG))
    straight_fraction = angle / STRAIGHT_ANGLE_DEG
    pulse = servo_config["pwm_bent"] + straight_fraction * (
        servo_config["pwm_straight"] - servo_config["pwm_bent"]
    )
    return int(round(np.clip(pulse, 1638, 8191)))


def move(pca, channel, pulse) -> None:
    if channel is not None:
        pca.channels[channel].duty_cycle = pulse


def smooth(current: dict, previous: dict | None) -> dict:
    if previous is None:
        return current
    return {
        finger: {
            joint: SMOOTHING_ALPHA * value + (1.0 - SMOOTHING_ALPHA) * previous[finger][joint]
            for joint, value in joints.items()
        }
        for finger, joints in current.items()
    }


def validate_configuration() -> None:
    used_channels = []
    missing_tips = []
    for finger, servos in FINGER_SERVOS.items():
        for location, config in servos.items():
            channel = config["channel"]
            if channel is None:
                if location == "tip":
                    missing_tips.append(finger)
                continue
            if not isinstance(channel, int) or not 0 <= channel <= 15:
                raise ValueError(f"{finger} {location} channel must be an integer from 0 through 15")
            used_channels.append(channel)
    duplicates = sorted({channel for channel in used_channels if used_channels.count(channel) > 1})
    if duplicates:
        raise ValueError(f"PCA9685 channel(s) assigned twice: {duplicates}")
    if missing_tips:
        print("[mimic] DIP/top angles are being tracked but not sent for: " + ", ".join(missing_tips))
        print("[mimic] Set FINGER_SERVOS[<finger>]['tip']['channel'] after verifying each PCA9685 wire.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true",
                        help="do not initialize I2C or command servos; show the calculated values instead")
    parser.add_argument("--video", type=Path,
                        help="optional video file instead of webcam 0; useful for repeatable tests")
    return parser.parse_args()


def draw_software_view(frame: np.ndarray, hand_landmarks, angles: dict) -> None:
    """Overlay landmark bones, all three joint angles, and both virtual PWM outputs."""
    height, width = frame.shape[:2]
    pixels = [(int(point.x * width), int(point.y * height)) for point in hand_landmarks]
    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, pixels[start], pixels[end], (255, 180, 0), 2)
    for index, pixel in enumerate(pixels):
        cv2.circle(frame, pixel, 5 if index in (4, 8, 12, 16, 20) else 3, (0, 255, 0), -1)

    y = 28
    for finger, joints in angles.items():
        servos = FINGER_SERVOS[finger]
        bottom_pwm = angle_to_pwm(joints["coupled"], servos["bottom"])
        tip_pwm = angle_to_pwm(joints["dip"], servos["tip"])
        top_channel = servos["tip"]["channel"]
        text = (
            f"{finger:<6} MCP {joints['mcp']:5.1f}  PIP {joints['pip']:5.1f}  DIP {joints['dip']:5.1f} "
            f"| bottom ch{servos['bottom']['channel']}: {bottom_pwm}  top ch{top_channel}: {tip_pwm}"
        )
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 255, 255), 2)
        y += 26
    cv2.putText(frame, "DRY RUN -- no servo output (q = quit)", (10, height - 16),
                cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 255), 2)


def main() -> None:
    args = parse_args()
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(f"MediaPipe model not found: {MODEL_PATH}")
    validate_configuration()

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(MODEL_PATH)),
        running_mode=RunningMode.IMAGE,
        num_hands=1,
    )
    pca = None
    if args.dry_run:
        print("[mimic] DRY RUN: displaying MediaPipe angles and virtual PWM; no I2C hardware is opened.")
    else:
        # Import hardware packages only when physical output is requested, so
        # --dry-run works on a laptop without Adafruit/I2C installed.
        from adafruit_extended_bus import ExtendedI2C as I2C
        from adafruit_pca9685 import PCA9685
        i2c = I2C(7)
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 50

    capture = cv2.VideoCapture(str(args.video) if args.video else 0)
    if not capture.isOpened():
        source = args.video if args.video else "webcam 0"
        raise RuntimeError(f"Could not open {source}")
    previous_angles = None

    try:
        with HandLandmarker.create_from_options(options) as landmarker:
            while capture.isOpened():
                ok, frame = capture.read()
                if not ok:
                    break
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = landmarker.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
                if not results.hand_landmarks:
                    continue

                angles = smooth(get_finger_angles(results.hand_landmarks[0]), previous_angles)
                previous_angles = angles
                for finger, joints in angles.items():
                    servos = FINGER_SERVOS[finger]
                    # The bottom servo follows both knuckle (MCP) and middle
                    # (PIP) joints; the top servo follows DIP alone.
                    if not args.dry_run:
                        move(pca, servos["bottom"]["channel"], angle_to_pwm(joints["coupled"], servos["bottom"]))
                        move(pca, servos["tip"]["channel"], angle_to_pwm(joints["dip"], servos["tip"]))

                if args.dry_run:
                    draw_software_view(frame, results.hand_landmarks[0], angles)
                    cv2.imshow("mimic -- software dry run", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    except KeyboardInterrupt:
        print("\n[mimic] stopped.")
    finally:
        capture.release()
        if args.dry_run:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
