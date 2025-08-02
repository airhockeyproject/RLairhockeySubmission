import cv2

cap = cv2.VideoCapture("/dev/video0")
if cap.isOpened():
    ret, frame = cap.read()
    print("Success:", ret)
    if ret:
        cv2.imshow("Frame", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
cap.release()
