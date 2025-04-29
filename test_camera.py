from lerobot.common.robot_devices.cameras.configs import OpenCVCameraConfig
from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera


camera_indices = [
    '/dev/video0',
    '/dev/video4',
    '/dev/video8',
    '/dev/video12',
]

for camera_index in camera_indices:
    config = OpenCVCameraConfig(camera_index=camera_index)
    camera = OpenCVCamera(config)
    camera.connect()
    color_image = camera.read()

    print(color_image.shape)
    print(color_image.dtype)


