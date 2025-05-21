import cv2

cap = cv2.VideoCapture(3)  # Try index 0 first

if not cap.isOpened():
    print("Error: Could not open camera 0")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Camera 0 Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
