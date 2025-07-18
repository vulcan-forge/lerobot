#!/usr/bin/env python3
"""
Comprehensive Camera Detection Test Script for Linux

This script provides multiple methods to detect and test cameras on Linux systems:
1. System-level detection using /dev/video* devices
2. USB device enumeration
3. LeRobot's built-in camera detection
4. OpenCV-based camera testing
5. RealSense camera detection (if available)
6. Advanced Linux tools (v4l2-ctl, etc.)

Usage:
    python camera_test.py [--opencv] [--realsense] [--all] [--save-images]
"""

import argparse
import cv2
import glob
import logging
import os
import platform
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Try to import LeRobot components
try:
    from lerobot.common.cameras.opencv.camera_opencv import OpenCVCamera
    from lerobot.common.cameras.realsense.camera_realsense import RealSenseCamera
    from lerobot.common.cameras.configs import ColorMode
    from lerobot.common.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.common.cameras.realsense.configuration_realsense import RealSenseCameraConfig
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False
    print("Warning: LeRobot not available. Some features will be limited.")

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


def run_shell_command(cmd: str, timeout: int = 10) -> str:
    """Run a shell command and return the output"""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return f"Command timed out after {timeout} seconds"
    except Exception as e:
        return f"Error running command: {e}"


def check_linux_system() -> bool:
    """Check if we're running on a Linux system"""
    if platform.system() != "Linux":
        print("Warning: This script is designed for Linux systems.")
        print(f"Current system: {platform.system()}")
        return False
    return True


def detect_video_devices() -> List[str]:
    """Detect all video devices in /dev/video*"""
    print("=== 1. Video Device Detection ===")
    video_devices = sorted(glob.glob('/dev/video*'), key=lambda x: int(x.split('video')[1]))

    if video_devices:
        print(f"Found {len(video_devices)} video device(s):")
        for device in video_devices:
            print(f"  {device}")

        # Get detailed info using v4l2-ctl if available
        print("\nDetailed device information:")
        for device in video_devices:
            print(f"\n{device}:")
            info = run_shell_command(f"v4l2-ctl --device={device} --info 2>/dev/null")
            if info and not info.startswith("Error") and not info.startswith("Command timed out"):
                # Extract device name
                lines = info.split('\n')
                for line in lines:
                    if 'Card type' in line or 'Driver name' in line:
                        print(f"  {line.strip()}")
            else:
                print(f"  Could not get detailed info (v4l2-ctl not available or device not accessible)")
    else:
        print("No video devices found in /dev/video*")

    return video_devices


def detect_usb_cameras() -> List[str]:
    """Detect USB cameras using lsusb"""
    print("\n=== 2. USB Camera Detection ===")
    usb_output = run_shell_command("lsusb")
    camera_lines = []

    if usb_output and not usb_output.startswith("Error"):
        lines = usb_output.split('\n')
        for line in lines:
            if any(keyword in line.lower() for keyword in ['camera', 'webcam', 'video', 'uvc']):
                camera_lines.append(line.strip())

    if camera_lines:
        print(f"Found {len(camera_lines)} camera-like USB device(s):")
        for line in camera_lines:
            print(f"  {line}")
    else:
        print("No camera-like USB devices found")

    return camera_lines


def test_opencv_cameras(max_index: int = 30) -> List[Dict]:
    """Test OpenCV cameras using indices and device paths"""
    print(f"\n=== 3. OpenCV Camera Testing (0-{max_index-1}) ===")
    working_cameras = []

    # Test numeric indices
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                backend = cap.getBackendName()

                camera_info = {
                    'id': i,
                    'type': 'index',
                    'working': True,
                    'shape': frame.shape,
                    'width': width,
                    'height': height,
                    'fps': fps,
                    'backend': backend
                }
                working_cameras.append(camera_info)

                print(f"  Camera {i}: ✓ Working")
                print(f"    Frame shape: {frame.shape}")
                print(f"    Properties: {width}x{height} @ {fps:.1f}fps")
                print(f"    Backend: {backend}")
            else:
                print(f"  Camera {i}: ✗ Opened but no frame")
            cap.release()
        else:
            print(f"  Camera {i}: ✗ Cannot open")

    # Test device paths
    video_devices = glob.glob('/dev/video*')
    for device in video_devices:
        cap = cv2.VideoCapture(device)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                backend = cap.getBackendName()

                camera_info = {
                    'id': device,
                    'type': 'device_path',
                    'working': True,
                    'shape': frame.shape,
                    'width': width,
                    'height': height,
                    'fps': fps,
                    'backend': backend
                }
                working_cameras.append(camera_info)

                print(f"  {device}: ✓ Working")
                print(f"    Frame shape: {frame.shape}")
                print(f"    Properties: {width}x{height} @ {fps:.1f}fps")
                print(f"    Backend: {backend}")
            else:
                print(f"  {device}: ✗ Opened but no frame")
            cap.release()
        else:
            print(f"  {device}: ✗ Cannot open")

    return working_cameras


def test_lerobot_cameras() -> List[Dict]:
    """Test cameras using LeRobot's built-in detection"""
    if not LEROBOT_AVAILABLE:
        print("\n=== 4. LeRobot Camera Detection ===")
        print("LeRobot not available. Skipping.")
        return []

    print("\n=== 4. LeRobot Camera Detection ===")
    all_cameras = []

    # Test OpenCV cameras through LeRobot
    try:
        print("Testing OpenCV cameras through LeRobot...")
        opencv_cameras = OpenCVCamera.find_cameras()
        for cam_info in opencv_cameras:
            all_cameras.append(cam_info)
            print(f"  Found OpenCV camera: {cam_info['name']}")
            print(f"    ID: {cam_info['id']}")
            print(f"    Backend: {cam_info.get('backend_api', 'Unknown')}")
            if 'default_stream_profile' in cam_info:
                profile = cam_info['default_stream_profile']
                print(f"    Profile: {profile['width']}x{profile['height']} @ {profile['fps']}fps")
    except Exception as e:
        print(f"  Error detecting OpenCV cameras: {e}")

    # Test RealSense cameras through LeRobot
    try:
        print("\nTesting RealSense cameras through LeRobot...")
        realsense_cameras = RealSenseCamera.find_cameras()
        for cam_info in realsense_cameras:
            all_cameras.append(cam_info)
            print(f"  Found RealSense camera: {cam_info['name']}")
            print(f"    Serial: {cam_info['id']}")
            print(f"    Firmware: {cam_info.get('firmware_version', 'Unknown')}")
            print(f"    USB Type: {cam_info.get('usb_type_descriptor', 'Unknown')}")
            if 'default_stream_profile' in cam_info:
                profile = cam_info['default_stream_profile']
                print(f"    Profile: {profile['width']}x{profile['height']} @ {profile['fps']}fps")
    except ImportError:
        print("  pyrealsense2 not available. Skipping RealSense detection.")
    except Exception as e:
        print(f"  Error detecting RealSense cameras: {e}")

    return all_cameras


def test_camera_connection(camera_info: Dict, save_image: bool = False) -> bool:
    """Test connecting to a specific camera and optionally save an image"""
    if not LEROBOT_AVAILABLE:
        return False

    try:
        cam_type = camera_info.get('type')
        cam_id = camera_info.get('id')

        if cam_type == 'OpenCV':
            config = OpenCVCameraConfig(
                index_or_path=cam_id,
                color_mode=ColorMode.RGB,
            )
            camera = OpenCVCamera(config)
        elif cam_type == 'RealSense':
            config = RealSenseCameraConfig(
                serial_number_or_name=cam_id,
                color_mode=ColorMode.RGB,
            )
            camera = RealSenseCamera(config)
        else:
            return False

        camera.connect(warmup=False)

        if save_image:
            frame = camera.read()
            if frame is not None:
                timestamp = int(time.time())
                filename = f"camera_test_{cam_type}_{cam_id}_{timestamp}.png"
                cv2.imwrite(filename, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                print(f"    Saved test image: {filename}")

        camera.disconnect()
        return True

    except Exception as e:
        print(f"    Connection test failed: {e}")
        return False


def check_linux_tools() -> Dict[str, bool]:
    """Check availability of Linux camera tools"""
    print("\n=== 5. Linux Camera Tools Check ===")
    tools = {}

    # Check v4l2-ctl
    v4l2_output = run_shell_command("v4l2-ctl --version 2>/dev/null")
    tools['v4l2-ctl'] = not v4l2_output.startswith("Error") and not v4l2_output.startswith("Command timed out")
    print(f"  v4l2-ctl: {'✓ Available' if tools['v4l2-ctl'] else '✗ Not available'}")

    # Check v4l2loopback
    v4l2loopback_output = run_shell_command("modinfo v4l2loopback 2>/dev/null")
    tools['v4l2loopback'] = not v4l2loopback_output.startswith("Error") and not v4l2loopback_output.startswith("Command timed out")
    print(f"  v4l2loopback: {'✓ Available' if tools['v4l2loopback'] else '✗ Not available'}")

    # Check ffmpeg
    ffmpeg_output = run_shell_command("ffmpeg -version 2>/dev/null")
    tools['ffmpeg'] = not ffmpeg_output.startswith("Error") and not ffmpeg_output.startswith("Command timed out")
    print(f"  ffmpeg: {'✓ Available' if tools['ffmpeg'] else '✗ Not available'}")

    return tools


def generate_summary(video_devices: List[str], usb_cameras: List[str],
                    opencv_cameras: List[Dict], lerobot_cameras: List[Dict],
                    linux_tools: Dict[str, bool]) -> None:
    """Generate a summary of all findings"""
    print("\n" + "="*50)
    print("CAMERA DETECTION SUMMARY")
    print("="*50)

    print(f"System: {platform.system()} {platform.release()}")
    print(f"Python: {sys.version}")
    print(f"OpenCV: {cv2.__version__}")
    print(f"LeRobot: {'Available' if LEROBOT_AVAILABLE else 'Not available'}")

    print(f"\nVideo devices (/dev/video*): {len(video_devices)}")
    print(f"USB camera devices: {len(usb_cameras)}")
    print(f"Working OpenCV cameras: {len(opencv_cameras)}")
    print(f"LeRobot detected cameras: {len(lerobot_cameras)}")

    if opencv_cameras:
        print("\nWorking camera details:")
        for cam in opencv_cameras:
            print(f"  {cam['id']} ({cam['type']}): {cam['width']}x{cam['height']} @ {cam['fps']:.1f}fps")

    print(f"\nLinux tools available: {sum(linux_tools.values())}/{len(linux_tools)}")
    for tool, available in linux_tools.items():
        print(f"  {tool}: {'✓' if available else '✗'}")


def main():
    parser = argparse.ArgumentParser(
        description="Comprehensive camera detection test for Linux systems",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python camera_test.py                    # Run all tests
  python camera_test.py --opencv           # Test only OpenCV cameras
  python camera_test.py --realsense        # Test only RealSense cameras
  python camera_test.py --save-images      # Save test images from working cameras
        """
    )

    parser.add_argument('--opencv', action='store_true',
                       help='Test only OpenCV cameras')
    parser.add_argument('--realsense', action='store_true',
                       help='Test only RealSense cameras')
    parser.add_argument('--all', action='store_true',
                       help='Run all tests (default)')
    parser.add_argument('--save-images', action='store_true',
                       help='Save test images from working cameras')
    parser.add_argument('--max-index', type=int, default=30,
                       help='Maximum OpenCV camera index to test (default: 30)')

    args = parser.parse_args()

    # Check if we're on Linux
    if not check_linux_system():
        print("Continuing anyway...")

    # Determine what to test
    test_opencv = args.opencv or args.all or not (args.opencv or args.realsense)
    test_realsense = args.realsense or args.all or not (args.opencv or args.realsense)

    print("Comprehensive Camera Detection Test for Linux")
    print("=" * 50)

    # Run tests
    video_devices = detect_video_devices()
    usb_cameras = detect_usb_cameras()

    opencv_cameras = []
    if test_opencv:
        opencv_cameras = test_opencv_cameras(args.max_index)

    lerobot_cameras = []
    if LEROBOT_AVAILABLE and (test_opencv or test_realsense):
        lerobot_cameras = test_lerobot_cameras()

    linux_tools = check_linux_tools()

    # Test camera connections if requested
    if args.save_images and LEROBOT_AVAILABLE:
        print("\n=== 6. Camera Connection Testing ===")
        for camera_info in lerobot_cameras:
            print(f"Testing connection to {camera_info['name']}...")
            test_camera_connection(camera_info, save_image=True)

    # Generate summary
    generate_summary(video_devices, usb_cameras, opencv_cameras,
                    lerobot_cameras, linux_tools)

    print("\nTest completed!")


if __name__ == "__main__":
    main()
