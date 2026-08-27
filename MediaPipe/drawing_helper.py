import cv2

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cap.release()

display_frame = cv2.resize(frame, (960, 540))

def click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"({x}, {y})")
        cv2.circle(display_frame, (x, y), 5, (0, 255, 0), -1)
        cv2.imshow("click", display_frame)

cv2.imshow("click", display_frame)
cv2.setMouseCallback("click", click)
cv2.waitKey(0)
cv2.destroyAllWindows()