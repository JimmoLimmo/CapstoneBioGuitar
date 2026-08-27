import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path="MediaPipe/hand_landmarker.task"),
    running_mode=RunningMode.IMAGE,
    num_hands=1
)

FINGER_INDICES = {
    "index":  [5, 6, 7, 8],
    "middle": [9, 10, 11, 12],
    "ring":   [13, 14, 15, 16],
    "pinky":  [17, 18, 19, 20],
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

capture = cv2.VideoCapture(0)

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

            for landmark in hand:
                height, width, _ = frame.shape
                x_pix, y_pix = int(landmark.x * width), int(landmark.y * height)
                cv2.circle(frame, (x_pix, y_pix), 5, (0, 0, 0), -1)

            angles = get_finger_angles(hand)
            print({name: f"{deg:.1f}" for name, deg in angles.items()})

            y_offset = 30
            for name, deg in angles.items():
                cv2.putText(frame, f"{name}: {deg:.1f}", (10, y_offset),
                            cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)
                y_offset += 25

        display_frame = cv2.resize(frame, (960, 540))
        cv2.imshow("debug", display_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

capture.release()
cv2.destroyAllWindows()