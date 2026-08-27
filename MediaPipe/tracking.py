import cv2
import mediapipe as mp
import time
import csv
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode

first_frame = True
times = []
next_index = 0
joint_count = 0

#puts all timestamps into list 
with open(r"Parsing/parsed.csv", newline='') as csvfile:   #change csv path as needed
    reader = csv.reader(csvfile)
    next(reader)
    for row in reader:
        times.append(float(row[0]))

header = ["Time", "Wx", "Wy", "Wz", "Tx", "Ty", "Tz", "Ix", "Iy", "Iz", "Mx", "My", "Mz", "Rx", "Ry", "Rz", "Px", "Py", "Pz"]
with open(r"MediaPipe/snow.csv", 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=r"MediaPipe/hand_landmarker.task"),
    running_mode=RunningMode.IMAGE,
    num_hands=1
)

FRETS = [
    (83, 333, 418), (103, 330, 418), (122, 331, 417),
    (142, 331, 416), (164, 329, 415), (187, 329, 414),
    (211, 329, 411), (236, 328, 411), (265, 328, 410),
    (293, 327, 407), (325, 327, 405), (358, 327, 404),
    (394, 324, 403), (432, 323, 399), (472, 322, 398),
    (514, 322, 395), (561, 320, 392), (609, 319, 391),
    (660, 317, 387), (716, 315, 383), (776, 316, 383),
]

def draw_fretboard(frame):
    for x, y_top, y_bot in FRETS:
        cv2.line(frame, (x, y_top), (x, y_bot), (0, 200, 255), 1)

capture = cv2.VideoCapture(0)

countdown_start = None
countdown_duration = 3
countdown_active = True
start_time = None
show_frets = True  # toggle state for fret lines

with HandLandmarker.create_from_options(options) as landmarker:
    while capture.isOpened():

        ret, frame = capture.read()
        if not ret:
            break

        if countdown_active:
            if countdown_start is None:
                countdown_start = time.perf_counter()

            elapsed_countdown = time.perf_counter() - countdown_start
            remaining = int(countdown_duration - elapsed_countdown) + 1

            if remaining > 0:
                cv2.putText(frame, str(remaining), (400, 400), cv2.FONT_HERSHEY_PLAIN, 10.0, (255, 255, 255), 2)
            else:
                if start_time is None:
                    start_time = time.perf_counter()
                countdown_active = False

            display_frame = cv2.resize(frame, (960, 540))
            if show_frets:
                draw_fretboard(display_frame)
            cv2.imshow("landmarks", display_frame) 

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('f'):
                show_frets = not show_frets
            continue 
        
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        results = landmarker.detect(mp_image)

        landmarks = []
        if results.hand_landmarks:
            hand = results.hand_landmarks[0]
            joint_count = 0
            for landmark in hand:     #loops over each of the 21 hand landmarks
                height, width, _ = frame.shape  #extracts height and width of camera frame
                x_pix, y_pix = int(landmark.x * width), int(landmark.y * height)    #converts landmark position (0-1) into pixel
                cv2.circle(frame, (x_pix, y_pix), 5, (0, 0, 0), -1)   #draws circle at given pixels
                if joint_count % 4 == 0:
                    landmarks += [landmark.x, landmark.y, landmark.z]
                joint_count += 1

        if next_index < len(times) and (time.perf_counter() - start_time) >= times[next_index]: 
            with open(r"MediaPipe/snow.csv", 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([times[next_index]] + landmarks)
            next_index += 1
                
        if start_time is not None:
            elapsed = time.perf_counter() - start_time
        else:
            elapsed = 0.0

        cv2.putText(frame, f"t = {elapsed:.3f}s", (50, 100), cv2.FONT_HERSHEY_PLAIN, 5.0, (255, 255, 255), 2)

        display_frame = cv2.resize(frame, (960, 540))
        if show_frets:
            draw_fretboard(display_frame)
        cv2.imshow("landmarks", display_frame)  

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):
            show_frets = not show_frets

capture.release()
cv2.destroyAllWindows()
