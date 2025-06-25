#!/usr/bin/env python3
"""
Camera Detection Script for LeRobot

This script helps you find and test cameras connected to your system.
It supports both OpenCV cameras (webcams, built-in cameras) and Intel RealSense cameras.

Usage:
    python start/test_camera.py                    # Find all cameras
    python start/test_camera.py --type opencv      # Find only OpenCV cameras
    python start/test_camera.py --type realsense   # Find only RealSense cameras
    python start/test_camera.py --test             # Test camera connections
    python start/test_camera.py --capture          # Capture test images
"""

import argparse
import logging
import sys
import time
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
        camera_type: Optional filter ("opencv" or "realsense")

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

    return all_cameras


def print_camera_info(cameras: List[Dict[str, Any]]) -> None:
    """
    Print formatted camera information.
    """
    if not cameras:
        print("\n❌ No cameras detected!")
        return

    print(f"\n�� Found {len(cameras)} camera(s):")
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
  python start/test_camera.py --test             # Test camera connections
  python start/test_camera.py --capture          # Capture test images
        """
    )

    parser.add_argument(
        "--type",
        choices=["opencv", "realsense"],
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
