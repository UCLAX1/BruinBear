import cv2

cap = cv2.VideoCapture(1)  # Try 0 or 1 for the camera device index
if not cap.isOpened():
    print("Error: Unable to open camera.")
else:
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow('Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cap.release()
    cv2.destroyAllWindows()
