# Check what video devices exist
ls -la /dev/video*

# Check if you have any USB cameras connected
lsusb | grep -i camera

# Check if you have any video devices that work
python3 -c "
import cv2
import glob

print('Available video devices:')
for device in glob.glob('/dev/video*'):
    print(f'  {device}')

print('\nTesting each device:')
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f'  /dev/video{i}: ✓ Working (frame shape: {frame.shape})')
        else:
            print(f'  /dev/video{i}: ✗ No frame')
        cap.release()
    else:
        print(f'  /dev/video{i}: ✗ Cannot open')
"