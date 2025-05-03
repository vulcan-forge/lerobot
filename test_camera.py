import cv2
import os

# Try all /dev/video* devices from 0 to 16
for i in range(0, 17):
    device = f"/dev/video{i}"
    if not os.path.exists(device):
        continue
    print(f"Trying {device}...")
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"  {device} could not be opened.")
        continue
    ret, frame = cap.read()
    if not ret:
        print(f"  {device} opened but no frame could be read.")
        cap.release()
        continue
    print(f"  {device} is working! (Frame size: {frame.shape})")
    cap.release()

print("Done testing all cameras.")
