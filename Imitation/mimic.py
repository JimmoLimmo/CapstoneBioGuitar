import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_pca9685 import PCA9685

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path="../MediaPipe/hand_landmarker.task"),
    running_mode=RunningMode.IMAGE,
    num_hands=1
)

i2c = I2C(7)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

FINGER_INDICES = {
    "index":  [5, 6, 7, 8],
    "middle": [9, 10, 11, 12],
    "ring":   [13, 14, 15, 16],
    "pinky":  [17, 18, 19, 20],
}

FINGER_CHANNELS = {
    "index":  15,
    "middle": 14,
    "ring":   13,
    "pinky":  12,
}

def angle_between(a, b, c):
    ba = np.array([a.x - b.x, a.y - b.y])
    bc = np.array([c.x - b.x, c.y - b.y])
    cos_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    return np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))

def get_finger_angles(hand_landmarks):
    angles = {}
    for name, idx in FINGER_INDICES.items():
        a = hand_landmarks[idx[0]]
        b = hand_landmarks[idx[1]]
        c = hand_landmarks[idx[3]]
        angles[name] = angle_between(a, b, c)
    return angles

def move(servo, pulse):
    pca.channels[servo].duty_cycle = pulse

def angle_to_pwm(deg, in_min=0, in_max=170, pwm_min=7000, pwm_max=1638):
    clamped = max(in_min, min(in_max, deg))
    return int((clamped - in_min) / (in_max - in_min) * (pwm_max - pwm_min) + pwm_min)

capture = cv2.VideoCapture(0)

try:
    with HandLandmarker.create_from_options(options) as landmarker:
        while capture.isOpened():
            ret, frame = capture.read()
            if not ret:
                break

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            results = landmarker.detect(mp_image)

            if results.hand_landmarks:
                hand = results.hand_landmarks[0]

                angles = get_finger_angles(hand)
                servo_positions = {name: angle_to_pwm(deg) for name, deg in angles.items()}

                for name, pwm in servo_positions.items():
                    move(FINGER_CHANNELS[name], pwm)

except KeyboardInterrupt:
    pass
finally:
    capture.release()
