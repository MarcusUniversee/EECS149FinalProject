import cv2

CAMERA_ID = 2  # try 0, 1, 2 etc if needed

cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_ANY)
print("Opened?", cap.isOpened())

if not cap.isOpened():
    raise SystemExit("Could not open camera")

while True:
    ret, frame = cap.read()
    print("ret:", ret)  # see if it ever becomes False

    if not ret:
        break

    cv2.imshow("Camera Test", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
