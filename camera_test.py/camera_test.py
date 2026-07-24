import cv2

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

print("Opened:", cap.isOpened())

# Try MJPG
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

ret, frame = cap.read()

print("Frame received:", ret)

if ret:
    cv2.imshow("Test", frame)
    cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()