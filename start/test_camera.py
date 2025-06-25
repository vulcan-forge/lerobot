#!/usr/bin/env python3
"""
Camera Detection Script for LeRobot

This script helps you find and test cameras connected to your system.
It supports OpenCV cameras (webcams, built-in cameras), Intel RealSense cameras,
and Raspberry Pi cameras (using picamera2).

Usage:
    python start/test_camera.py                    # Find all cameras
    python start/test_camera.py --type opencv      # Find only OpenCV cameras
    python start/test_camera.py --type realsense   # Find only RealSense cameras
    python start/test_camera.py --type raspberry   # Find only Raspberry Pi cameras
    python start/test_camera.py --test             # Test camera connections
    python start/test_camera.py --capture          # Capture test images
"""

import argparse
import logging
import sys
import time
import subprocess
import platform
from pathlib import Path
from typing import Any, Dict, List, Optional

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

try:
    import cv2
    import numpy as np
    from PIL import Image
except ImportError as e:
    logger.error(f"Required dependencies not found: {e}")
    logger.error("Please install: pip install opencv-python pillow numpy")
    sys.exit(1)

# Try to import LeRobot camera modules
try:
    from lerobot.common.cameras.configs import ColorMode
    from lerobot.common.cameras.opencv.camera_opencv import OpenCVCamera
    from lerobot.common.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    LEROBOT_AVAILABLE = True
except ImportError:
    logger.warning("LeRobot not available, using basic OpenCV camera detection")
    LEROBOT_AVAILABLE = False

# Try to import RealSense
try:
    from lerobot.common.cameras.realsense.camera_realsense import RealSenseCamera
    from lerobot.common.cameras.realsense.configuration_realsense import RealSenseCameraConfig
    REALSENSE_AVAILABLE = True
except ImportError:
    logger.warning("RealSense support not available (pyrealsense2 not installed)")
    REALSENSE_AVAILABLE = False

# Try to import PiCamera2 (modern Raspberry Pi camera library)
PICAMERA2_AVAILABLE = False
try:
    from picamera2 import Picamera2
    from picamera2.encoders import JpegEncoder
    from picamera2.outputs import FileOutput
    PICAMERA2_AVAILABLE = True
except ImportError as e:
    logger.warning(f"PiCamera2 support not available: {e}")
    logger.info("To install PiCamera2: sudo apt-get install python3-picamera2")
    logger.info("Or: pip install picamera2")


def is_raspberry_pi() -> bool:
    """
    Check if we're running on a Raspberry Pi.
    """
    try:
        # Check CPU info for Raspberry Pi
        with open('/proc/cpuinfo', 'r') as f:
            cpu_info = f.read()
            if 'Raspberry Pi' in cpu_info or 'BCM2708' in cpu_info or 'BCM2835' in cpu_info or 'BCM2711' in cpu_info:
                return True

        # Check for Raspberry Pi model in /proc/device-tree
        if Path('/proc/device-tree/model').exists():
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip()
                if 'Raspberry Pi' in model:
                    return True

        # Check for Raspberry Pi specific files
        if Path('/boot/config.txt').exists():
            return True

    except Exception:
        pass

    return False


def check_raspberry_pi_camera() -> bool:
    """
    Check if Raspberry Pi camera is available using system commands.
    """
    if not is_raspberry_pi():
        logger.debug("Not running on Raspberry Pi")
        return False

    try:
        # Check if camera is enabled in config
        result = subprocess.run(['vcgencmd', 'get_camera'],
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            output = result.stdout.strip()
            logger.debug(f"vcgencmd output: {output}")
            if 'detected=1' in output:
                return True

        # Alternative check using v4l2-ctl
        result = subprocess.run(['v4l2-ctl', '--list-devices'],
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            output = result.stdout
            logger.debug(f"v4l2-ctl output: {output}")
            if 'mmal' in output or 'bcm2835-v4l2' in output or 'unicam' in output:
                return True

        # Check for camera modules in /dev
        camera_devices = list(Path('/dev').glob('video*'))
        if camera_devices:
            logger.debug(f"Found video devices: {[str(d) for d in camera_devices]}")
            return True

    except (subprocess.TimeoutExpired, FileNotFoundError, subprocess.SubprocessError) as e:
        logger.debug(f"Error checking camera: {e}")

    return False


def find_raspberry_pi_cameras() -> List[Dict[str, Any]]:
    """
    Find Raspberry Pi cameras using PiCamera2.
    """
    cameras = []

    if not PICAMERA2_AVAILABLE:
        logger.warning("PiCamera2 library not available")
        return cameras

    logger.info("Searching for Raspberry Pi cameras...")

    try:
        # Check if we're on a Raspberry Pi
        if not check_raspberry_pi_camera():
            logger.debug("Raspberry Pi camera not detected in system")
            return cameras

        # Try to initialize PiCamera2
        camera = Picamera2()

        # Get camera info
        camera_info = camera.camera_properties
        logger.debug(f"Camera properties: {camera_info}")

        # Get available camera configurations
        configs = camera.sensor_modes
        logger.debug(f"Available sensor modes: {len(configs)}")

        # Use the first available configuration for default settings
        if configs:
            default_config = configs[0]
            width = default_config.get('size', [1920, 1080])[0]
            height = default_config.get('size', [1920, 1080])[1]
            fps = default_config.get('fps', 30)
        else:
            width, height, fps = 1920, 1080, 30

        camera_info = {
            "name": "Raspberry Pi Camera Module",
            "type": "RaspberryPi",
            "id": "picamera2",
            "sensor_modes": len(configs),
            "default_stream_profile": {
                "width": width,
                "height": height,
                "fps": fps,
                "format": "RGB"
            }
        }

        cameras.append(camera_info)
        camera.close()

    except Exception as e:
        logger.debug(f"Error detecting Raspberry Pi camera: {e}")

    return cameras


def find_opencv_cameras_basic() -> List[Dict[str, Any]]:
    """
    Basic OpenCV camera detection without LeRobot dependencies.
    """
    cameras = []
    logger.info("Searching for OpenCV cameras (basic method)...")

    # On Linux, check /dev/video* devices
    if Path("/dev").exists():
        video_devices = sorted(Path("/dev").glob("video*"), key=lambda p: p.name)
        for device in video_devices:
            try:
                cap = cv2.VideoCapture(str(device))
                if cap.isOpened():
                    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    fps = cap.get(cv2.CAP_PROP_FPS)

                    camera_info = {
                        "name": f"OpenCV Camera @ {device}",
                        "type": "OpenCV",
                        "id": str(device),
                        "backend_api": cap.getBackendName(),
                        "default_stream_profile": {
                            "width": width,
                            "height": height,
                            "fps": fps,
                            "format": cap.get(cv2.CAP_PROP_FORMAT)
                        }
                    }
                    cameras.append(camera_info)
                    cap.release()
            except Exception as e:
                logger.debug(f"Error checking {device}: {e}")

    # Also check camera indices (0-10)
    for i in range(10):
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)

                camera_info = {
                    "name": f"OpenCV Camera @ index {i}",
                    "type": "OpenCV",
                    "id": i,
                    "backend_api": cap.getBackendName(),
                    "default_stream_profile": {
                        "width": width,
                        "height": height,
                        "fps": fps,
                        "format": cap.get(cv2.CAP_PROP_FORMAT)
                    }
                }
                cameras.append(camera_info)
                cap.release()
        except Exception as e:
            logger.debug(f"Error checking camera index {i}: {e}")

    return cameras


def find_opencv_cameras_lerobot() -> List[Dict[str, Any]]:
    """
    Find OpenCV cameras using LeRobot's implementation.
    """
    if not LEROBOT_AVAILABLE:
        return []

    logger.info("Searching for OpenCV cameras (LeRobot method)...")
    try:
        return OpenCVCamera.find_cameras()
    except Exception as e:
        logger.error(f"Error finding OpenCV cameras with LeRobot: {e}")
        return []


def find_realsense_cameras() -> List[Dict[str, Any]]:
    """
    Find RealSense cameras using LeRobot's implementation.
    """
    if not REALSENSE_AVAILABLE:
        return []

    logger.info("Searching for RealSense cameras...")
    try:
        return RealSenseCamera.find_cameras()
    except Exception as e:
        logger.error(f"Error finding RealSense cameras: {e}")
        return []


def find_all_cameras(camera_type: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Find all available cameras, optionally filtered by type.

    Args:
        camera_type: Optional filter ("opencv", "realsense", or "raspberry")

    Returns:
        List of camera information dictionaries
    """
    all_cameras = []

    if camera_type is None or camera_type.lower() == "opencv":
        if LEROBOT_AVAILABLE:
            opencv_cameras = find_opencv_cameras_lerobot()
        else:
            opencv_cameras = find_opencv_cameras_basic()
        all_cameras.extend(opencv_cameras)
        logger.info(f"Found {len(opencv_cameras)} OpenCV cameras")

    if camera_type is None or camera_type.lower() == "realsense":
        realsense_cameras = find_realsense_cameras()
        all_cameras.extend(realsense_cameras)
        logger.info(f"Found {len(realsense_cameras)} RealSense cameras")

    if camera_type is None or camera_type.lower() == "raspberry":
        raspberry_cameras = find_raspberry_pi_cameras()
        all_cameras.extend(raspberry_cameras)
        logger.info(f"Found {len(raspberry_cameras)} Raspberry Pi cameras")

    return all_cameras


def print_camera_info(cameras: List[Dict[str, Any]]) -> None:
    """
    Print formatted camera information.
    """
    if not cameras:
        print("\n❌ No cameras detected!")
        return

    print(f"\n Found {len(cameras)} camera(s):")
    print("=" * 60)

    for i, cam_info in enumerate(cameras):
        print(f"\n🔹 Camera #{i + 1}:")
        print(f"   Name: {cam_info.get('name', 'Unknown')}")
        print(f"   Type: {cam_info.get('type', 'Unknown')}")
        print(f"   ID: {cam_info.get('id', 'Unknown')}")

        # Print additional info based on camera type
        if cam_info.get('type') == 'OpenCV':
            backend = cam_info.get('backend_api', 'Unknown')
            print(f"   Backend: {backend}")

        elif cam_info.get('type') == 'RealSense':
            firmware = cam_info.get('firmware_version', 'Unknown')
            usb_type = cam_info.get('usb_type_descriptor', 'Unknown')
            print(f"   Firmware: {firmware}")
            print(f"   USB Type: {usb_type}")

        elif cam_info.get('type') == 'RaspberryPi':
            sensor_modes = cam_info.get('sensor_modes', 'Unknown')
            print(f"   Sensor Modes: {sensor_modes}")

        # Print stream profile
        profile = cam_info.get('default_stream_profile', {})
        if profile:
            print(f"   Resolution: {profile.get('width', 'Unknown')}x{profile.get('height', 'Unknown')}")
            print(f"   FPS: {profile.get('fps', 'Unknown')}")
            if 'format' in profile:
                print(f"   Format: {profile.get('format', 'Unknown')}")

        print("-" * 40)


def test_camera_connection(camera_info: Dict[str, Any]) -> bool:
    """
    Test if a camera can be connected and read from.
    """
    cam_type = camera_info.get('type')
    cam_id = camera_info.get('id')

    try:
        if cam_type == 'OpenCV':
            if not LEROBOT_AVAILABLE:
                # Basic OpenCV test
                cap = cv2.VideoCapture(cam_id)
                if cap.isOpened():
                    ret, frame = cap.read()
                    cap.release()
                    return ret and frame is not None
                return False

            # LeRobot OpenCV test
            config = OpenCVCameraConfig(index_or_path=cam_id, color_mode=ColorMode.RGB)
            camera = OpenCVCamera(config)
            camera.connect(warmup=False)
            frame = camera.read()
            camera.disconnect()
            return frame is not None

        elif cam_type == 'RealSense':
            if not REALSENSE_AVAILABLE:
                return False

            config = RealSenseCameraConfig(serial_number_or_name=cam_id, color_mode=ColorMode.RGB)
            camera = RealSenseCamera(config)
            camera.connect(warmup=False)
            frame = camera.read()
            camera.disconnect()
            return frame is not None

        elif cam_type == 'RaspberryPi':
            if not PICAMERA2_AVAILABLE:
                return False

            camera = Picamera2()
            camera.configure(camera.create_preview_configuration())
            camera.start()
            frame = camera.capture_array()
            camera.stop()
            camera.close()
            return frame is not None

        return False

    except Exception as e:
        logger.debug(f"Connection test failed for {cam_type} camera {cam_id}: {e}")
        return False


def capture_test_image(camera_info: Dict[str, Any], output_dir: Path) -> bool:
    """
    Capture a test image from a camera.
    """
    cam_type = camera_info.get('type')
    cam_id = camera_info.get('id')

    try:
        if cam_type == 'OpenCV':
            if not LEROBOT_AVAILABLE:
                # Basic OpenCV capture
                cap = cv2.VideoCapture(cam_id)
                if cap.isOpened():
                    ret, frame = cap.read()
                    cap.release()
                    if ret and frame is not None:
                        # Convert BGR to RGB
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        img = Image.fromarray(frame_rgb)
                        filename = f"test_opencv_{cam_id}.png"
                        img.save(output_dir / filename)
                        logger.info(f"Saved test image: {filename}")
                        return True
                return False

            # LeRobot OpenCV capture
            config = OpenCVCameraConfig(index_or_path=cam_id, color_mode=ColorMode.RGB)
            camera = OpenCVCamera(config)
            camera.connect(warmup=False)
            frame = camera.read()
            camera.disconnect()

            if frame is not None:
                img = Image.fromarray(frame)
                filename = f"test_opencv_{cam_id}.png"
                img.save(output_dir / filename)
                logger.info(f"Saved test image: {filename}")
                return True

        elif cam_type == 'RealSense':
            if not REALSENSE_AVAILABLE:
                return False

            config = RealSenseCameraConfig(serial_number_or_name=cam_id, color_mode=ColorMode.RGB)
            camera = RealSenseCamera(config)
            camera.connect(warmup=False)
            frame = camera.read()
            camera.disconnect()

            if frame is not None:
                img = Image.fromarray(frame)
                filename = f"test_realsense_{cam_id}.png"
                img.save(output_dir / filename)
                logger.info(f"Saved test image: {filename}")
                return True

        elif cam_type == 'RaspberryPi':
            if not PICAMERA2_AVAILABLE:
                return False

            camera = Picamera2()
            camera.configure(camera.create_preview_configuration())
            camera.start()
            frame = camera.capture_array()
            camera.stop()
            camera.close()

            if frame is not None:
                img = Image.fromarray(frame)
                filename = f"test_raspberry_pi.png"
                img.save(output_dir / filename)
                logger.info(f"Saved test image: {filename}")
                return True

        return False

    except Exception as e:
        logger.error(f"Failed to capture test image from {cam_type} camera {cam_id}: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Find and test cameras connected to your system",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python start/test_camera.py                    # Find all cameras
  python start/test_camera.py --type opencv      # Find only OpenCV cameras
  python start/test_camera.py --type realsense   # Find only RealSense cameras
  python start/test_camera.py --type raspberry   # Find only Raspberry Pi cameras
  python start/test_camera.py --test             # Test camera connections
  python start/test_camera.py --capture          # Capture test images
        """
    )

    parser.add_argument(
        "--type",
        choices=["opencv", "realsense", "raspberry"],
        help="Filter cameras by type"
    )

    parser.add_argument(
        "--test",
        action="store_true",
        help="Test camera connections"
    )

    parser.add_argument(
        "--capture",
        action="store_true",
        help="Capture test images from all cameras"
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("test_images"),
        help="Output directory for test images (default: test_images)"
    )

    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose logging"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Find cameras
    print("🔍 Searching for cameras...")
    cameras = find_all_cameras(args.type)

    # Print camera information
    print_camera_info(cameras)

    if not cameras:
        print("\n💡 Troubleshooting tips:")
        print("   • For Raspberry Pi cameras: Make sure camera is enabled in raspi-config")
        print("   • Install PiCamera2: sudo apt-get install python3-picamera2")
        print("   • For USB cameras: Check if they're properly connected")
        print("   • Run with --verbose for more detailed information")
        return

    # Test connections if requested
    if args.test:
        print("\n🧪 Testing camera connections...")
        for i, camera_info in enumerate(cameras):
            cam_name = camera_info.get('name', f'Camera #{i+1}')
            success = test_camera_connection(camera_info)
            status = "✅ Connected" if success else "❌ Failed"
            print(f"   {cam_name}: {status}")

    # Capture test images if requested
    if args.capture:
        print(f"\n📸 Capturing test images to {args.output_dir}...")
        args.output_dir.mkdir(parents=True, exist_ok=True)

        for i, camera_info in enumerate(cameras):
            cam_name = camera_info.get('name', f'Camera #{i+1}')
            success = capture_test_image(camera_info, args.output_dir)
            status = "✅ Captured" if success else "❌ Failed"
            print(f"   {cam_name}: {status}")

        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            print(f"\n💾 Test images saved to: {args.output_dir.absolute()}")


if __name__ == "__main__":
    main()
