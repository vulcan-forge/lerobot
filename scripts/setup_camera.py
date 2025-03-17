import sys
import os

# Force Python to use your current directory
sys.path.insert(0, os.path.abspath(".."))

from lerobot.common.robot_devices.cameras.configs import OpenCVCameraConfig
from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera

# Now, your script should use the correct version of LeRobot
camera_config = OpenCVCameraConfig(camera_index=0)
camera = OpenCVCamera(camera_config)

camera.connect()
color_image = camera.read()

print(f"Camera Image Shape: {color_image.shape}")
print(f"Camera Image Type: {color_image.dtype}")

camera.disconnect()
