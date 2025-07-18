#!/usr/bin/env python3
"""
Comprehensive Camera Detection Test Script for Linux

This script provides multiple methods to detect and test cameras on Linux systems:
1. System-level detection using /dev/video* devices
2. USB device enumeration with port information
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
import re
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


def get_usb_port_info() -> Dict[str, Dict]:
    """Get detailed USB port information for all devices"""
    port_info = {}

    # Get USB device tree
    usb_tree = run_shell_command("lsusb -t 2>/dev/null")
    if usb_tree and not usb_tree.startswith("Error"):
        current_bus = None
        current_port = None

        for line in usb_tree.split('\n'):
            if line.strip():
                # Parse bus and port information
                if 'Bus' in line and 'Port' in line:
                    # Extract bus and port numbers
                    bus_match = re.search(r'Bus (\d+)', line)
                    port_match = re.search(r'Port (\d+)(?:\.(\d+))?', line)

                    if bus_match and port_match:
                        current_bus = int(bus_match.group(1))
                        port_parts = port_match.groups()
                        if port_parts[1]:  # Has sub-port
                            current_port = f"{port_parts[0]}.{port_parts[1]}"
                        else:
                            current_port = port_parts[0]

                # Extract device info
                device_match = re.search(r'(\w+)\s+(\w+)\s+(\w+)\s+(.+)', line)
                if device_match and current_bus is not None and current_port is not None:
                    speed = device_match.group(1)
                    device_class = device_match.group(2)
                    device_subclass = device_match.group(3)
                    device_name = device_match.group(4)

                    device_key = f"{current_bus}:{current_port}"
                    port_info[device_key] = {
                        'bus': current_bus,
                        'port': current_port,
                        'speed': speed,
                        'class': device_class,
                        'subclass': device_subclass,
                        'name': device_name
                    }

    return port_info


def get_device_usb_info(device_path: str) -> Optional[Dict]:
    """Get USB information for a specific video device"""
    try:
        # Get device info using udevadm
        udev_info = run_shell_command(f"udevadm info --query=property --name={device_path}")
        if udev_info and not udev_info.startswith("Error"):
            usb_info = {}

            # Extract USB bus and device numbers
            for line in udev_info.split('\n'):
                if line.startswith('ID_USB_INTERFACE_NUM='):
                    usb_info['interface'] = line.split('=')[1]
                elif line.startswith('ID_USB_DRIVER='):
                    usb_info['driver'] = line.split('=')[1]
                elif line.startswith('ID_VENDOR_ID='):
                    usb_info['vendor_id'] = line.split('=')[1]
                elif line.startswith('ID_MODEL_ID='):
                    usb_info['model_id'] = line.split('=')[1]
                elif line.startswith('ID_SERIAL_SHORT='):
                    usb_info['serial'] = line.split('=')[1]
                elif line.startswith('ID_USB_INTERFACES='):
                    usb_info['interfaces'] = line.split('=')[1]

            # Get USB path using sysfs
            sysfs_path = run_shell_command(f"readlink -f /sys/class/video4linux/{os.path.basename(device_path)}/device")
            if sysfs_path and not sysfs_path.startswith("Error"):
                # Navigate up to find USB device path
                usb_path = run_shell_command(f"find {sysfs_path} -name 'busnum' -o -name 'devnum' | head -1")
                if usb_path and not usb_path.startswith("Error"):
                    usb_dir = os.path.dirname(usb_path)
                    busnum = run_shell_command(f"cat {usb_dir}/busnum 2>/dev/null")
                    devnum = run_shell_command(f"cat {usb_dir}/devnum 2>/dev/null")
                    if busnum and devnum and not busnum.startswith("Error") and not devnum.startswith("Error"):
                        usb_info['bus_num'] = busnum.strip()
                        usb_info['dev_num'] = devnum.strip()

            return usb_info if usb_info else None
    except Exception as e:
        logger.debug(f"Error getting USB info for {device_path}: {e}")

    return None


def map_usb_to_video_devices() -> Dict[str, List[str]]:
    """Map USB bus:device numbers to video device paths"""
    mapping = {}

    # Get all video devices
    video_devices = glob.glob('/dev/video*')

    for device in video_devices:
        usb_info = get_device_usb_info(device)
        if usb_info and 'bus_num' in usb_info and 'dev_num' in usb_info:
            usb_key = f"{usb_info['bus_num']}:{usb_info['dev_num']}"
            if usb_key not in mapping:
                mapping[usb_key] = []
            mapping[usb_key].append(device)

    return mapping


def detect_video_devices() -> List[Dict]:
    """Detect all video devices in /dev/video* with port information"""
    print("=== 1. Video Device Detection ===")
    video_devices = sorted(glob.glob('/dev/video*'), key=lambda x: int(x.split('video')[1]))
    device_info_list = []

    if video_devices:
        print(f"Found {len(video_devices)} video device(s):")

        # Get USB port information
        usb_port_info = get_usb_port_info()

        for device in video_devices:
            device_info = {'path': device, 'usb_info': None}

            # Get USB information for this device
            usb_info = get_device_usb_info(device)
            if usb_info:
                device_info['usb_info'] = usb_info

                # Try to match with port info
                if 'bus_num' in usb_info and 'dev_num' in usb_info:
                    device_key = f"{usb_info['bus_num']}:{usb_info['dev_num']}"
                    if device_key in usb_port_info:
                        device_info['port_info'] = usb_port_info[device_key]

            device_info_list.append(device_info)

            print(f"  {device}")
            if usb_info:
                print(f"    USB Bus: {usb_info.get('bus_num', 'Unknown')}")
                print(f"    USB Device: {usb_info.get('dev_num', 'Unknown')}")
                if 'port_info' in device_info:
                    port_info = device_info['port_info']
                    print(f"    USB Port: {port_info['port']}")
                    print(f"    USB Speed: {port_info['speed']}")
                if usb_info.get('vendor_id') and usb_info.get('model_id'):
                    print(f"    Vendor/Model: {usb_info['vendor_id']}:{usb_info['model_id']}")
                if usb_info.get('driver'):
                    print(f"    Driver: {usb_info['driver']}")

        # Get detailed info using v4l2-ctl if available
        print("\nDetailed device information:")
        for device_info in device_info_list:
            device = device_info['path']
            print(f"\n{device}:")
            info = run_shell_command(f"v4l2-ctl --device={device} --info 2>/dev/null")
            if info and not info.startswith("Error") and not info.startswith("Command timed out"):
                lines = info.split('\n')
                for line in lines:
                    if 'Card type' in line or 'Driver name' in line:
                        print(f"  {line.strip()}")
            else:
                print(f"  Could not get detailed info (v4l2-ctl not available or device not accessible)")
    else:
        print("No video devices found in /dev/video*")

    return device_info_list


def detect_usb_cameras() -> List[Dict]:
    """Detect USB cameras using lsusb with port information and video device mapping"""
    print("\n=== 2. USB Camera Detection ===")
    usb_output = run_shell_command("lsusb")
    camera_devices = []

    if usb_output and not usb_output.startswith("Error"):
        lines = usb_output.split('\n')
        for line in lines:
            if any(keyword in line.lower() for keyword in ['camera', 'webcam', 'video', 'uvc']):
                # Parse lsusb output: Bus 001 Device 003: ID 046d:0825 Logitech, Inc. Webcam C270
                match = re.match(r'Bus (\d+) Device (\d+): ID ([a-f0-9]{4}):([a-f0-9]{4}) (.+)', line)
                if match:
                    bus = match.group(1)
                    device = match.group(2)
                    vendor_id = match.group(3)
                    product_id = match.group(4)
                    name = match.group(5)

                    camera_info = {
                        'bus': bus,
                        'device': device,
                        'vendor_id': vendor_id,
                        'product_id': product_id,
                        'name': name,
                        'full_line': line.strip(),
                        'video_devices': []
                    }
                    camera_devices.append(camera_info)
                else:
                    # Fallback for non-standard format
                    camera_devices.append({
                        'full_line': line.strip(),
                        'bus': 'Unknown',
                        'device': 'Unknown',
                        'vendor_id': 'Unknown',
                        'product_id': 'Unknown',
                        'name': 'Unknown',
                        'video_devices': []
                    })

    # Map USB devices to video devices
    usb_to_video_mapping = map_usb_to_video_devices()

    # Add video device information to each camera
    for cam in camera_devices:
        usb_key = f"{cam['bus']}:{cam['device']}"
        if usb_key in usb_to_video_mapping:
            cam['video_devices'] = usb_to_video_mapping[usb_key]

    if camera_devices:
        print(f"Found {len(camera_devices)} camera-like USB device(s):")
        for cam in camera_devices:
            print(f"  Bus {cam['bus']} Device {cam['device']}: {cam['name']}")
            print(f"    Vendor/Product: {cam['vendor_id']}:{cam['product_id']}")
            if cam['video_devices']:
                print(f"    Video devices: {', '.join(cam['video_devices'])}")
            else:
                print(f"    Video devices: None found (may not be accessible)")
    else:
        print("No camera-like USB devices found")

    return camera_devices


def test_opencv_cameras(max_index: int = 30) -> List[Dict]:
    """Test OpenCV cameras using indices and device paths with port information"""
    print(f"\n=== 3. OpenCV Camera Testing (0-{max_index-1}) ===")
    working_cameras = []

    # Get USB port information
    usb_port_info = get_usb_port_info()

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
                    'backend': backend,
                    'usb_info': None,
                    'port_info': None
                }

                # Try to get USB information for this camera index
                # This is tricky for indices, but we can try to match with device paths
                for device_path in glob.glob('/dev/video*'):
                    cap_test = cv2.VideoCapture(device_path)
                    if cap_test.isOpened():
                        # Check if this device path corresponds to the same camera
                        # by comparing properties
                        test_width = int(cap_test.get(cv2.CAP_PROP_FRAME_WIDTH))
                        test_height = int(cap_test.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        if test_width == width and test_height == height:
                            usb_info = get_device_usb_info(device_path)
                            if usb_info:
                                camera_info['usb_info'] = usb_info
                                if 'bus_num' in usb_info and 'dev_num' in usb_info:
                                    device_key = f"{usb_info['bus_num']}:{usb_info['dev_num']}"
                                    if device_key in usb_port_info:
                                        camera_info['port_info'] = usb_port_info[device_key]
                            break
                        cap_test.release()

                working_cameras.append(camera_info)

                print(f"  Camera {i}: ✓ Working")
                print(f"    Frame shape: {frame.shape}")
                print(f"    Properties: {width}x{height} @ {fps:.1f}fps")
                print(f"    Backend: {backend}")
                if camera_info['usb_info']:
                    usb_info = camera_info['usb_info']
                    print(f"    USB Bus: {usb_info.get('bus_num', 'Unknown')}")
                    print(f"    USB Device: {usb_info.get('dev_num', 'Unknown')}")
                    if camera_info['port_info']:
                        port_info = camera_info['port_info']
                        print(f"    USB Port: {port_info['port']}")
                        print(f"    USB Speed: {port_info['speed']}")
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
                    'backend': backend,
                    'usb_info': None,
                    'port_info': None
                }

                # Get USB information for this device
                usb_info = get_device_usb_info(device)
                if usb_info:
                    camera_info['usb_info'] = usb_info
                    if 'bus_num' in usb_info and 'dev_num' in usb_info:
                        device_key = f"{usb_info['bus_num']}:{usb_info['dev_num']}"
                        if device_key in usb_port_info:
                            camera_info['port_info'] = usb_port_info[device_key]

                working_cameras.append(camera_info)

                print(f"  {device}: ✓ Working")
                print(f"    Frame shape: {frame.shape}")
                print(f"    Properties: {width}x{height} @ {fps:.1f}fps")
                print(f"    Backend: {backend}")
                if camera_info['usb_info']:
                    usb_info = camera_info['usb_info']
                    print(f"    USB Bus: {usb_info.get('bus_num', 'Unknown')}")
                    print(f"    USB Device: {usb_info.get('dev_num', 'Unknown')}")
                    if camera_info['port_info']:
                        port_info = camera_info['port_info']
                        print(f"    USB Port: {port_info['port']}")
                        print(f"    USB Speed: {port_info['speed']}")
            else:
                print(f"  {device}: ✗ Opened but no frame")
            cap.release()
        else:
            print(f"  {device}: ✗ Cannot open")

    return working_cameras


def test_lerobot_cameras() -> List[Dict]:
    """Test cameras using LeRobot's built-in detection with port information"""
    if not LEROBOT_AVAILABLE:
        print("\n=== 4. LeRobot Camera Detection ===")
        print("LeRobot not available. Skipping.")
        return []

    print("\n=== 4. LeRobot Camera Detection ===")
    all_cameras = []

    # Get USB port information
    usb_port_info = get_usb_port_info()

    # Test OpenCV cameras through LeRobot
    try:
        print("Testing OpenCV cameras through LeRobot...")
        opencv_cameras = OpenCVCamera.find_cameras()
        for cam_info in opencv_cameras:
            # Try to get USB information for this camera
            cam_id = cam_info.get('id')
            if isinstance(cam_id, str) and cam_id.startswith('/dev/video'):
                usb_info = get_device_usb_info(cam_id)
                if usb_info:
                    cam_info['usb_info'] = usb_info
                    if 'bus_num' in usb_info and 'dev_num' in usb_info:
                        device_key = f"{usb_info['bus_num']}:{usb_info['dev_num']}"
                        if device_key in usb_port_info:
                            cam_info['port_info'] = usb_port_info[device_key]

            all_cameras.append(cam_info)
            print(f"  Found OpenCV camera: {cam_info['name']}")
            print(f"    ID: {cam_info['id']}")
            print(f"    Backend: {cam_info.get('backend_api', 'Unknown')}")
            if 'usb_info' in cam_info:
                usb_info = cam_info['usb_info']
                print(f"    USB Bus: {usb_info.get('bus_num', 'Unknown')}")
                print(f"    USB Device: {usb_info.get('dev_num', 'Unknown')}")
                if 'port_info' in cam_info:
                    port_info = cam_info['port_info']
                    print(f"    USB Port: {port_info['port']}")
                    print(f"    USB Speed: {port_info['speed']}")
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
            # RealSense cameras already have physical port info
            all_cameras.append(cam_info)
            print(f"  Found RealSense camera: {cam_info['name']}")
            print(f"    Serial: {cam_info['id']}")
            print(f"    Physical Port: {cam_info.get('physical_port', 'Unknown')}")
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

    # Check udevadm
    udevadm_output = run_shell_command("udevadm --version 2>/dev/null")
    tools['udevadm'] = not udevadm_output.startswith("Error") and not udevadm_output.startswith("Command timed out")
    print(f"  udevadm: {'✓ Available' if tools['udevadm'] else '✗ Not available'}")

    return tools


def generate_summary(video_devices: List[Dict], usb_cameras: List[Dict],
                    opencv_cameras: List[Dict], lerobot_cameras: List[Dict],
                    linux_tools: Dict[str, bool]) -> None:
    """Generate a summary of all findings with port information"""
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
            port_info = ""
            if cam.get('port_info'):
                port_info = f" (USB Port: {cam['port_info']['port']})"
            elif cam.get('usb_info'):
                usb_info = cam['usb_info']
                port_info = f" (USB Bus: {usb_info.get('bus_num', 'Unknown')}, Device: {usb_info.get('dev_num', 'Unknown')})"
            print(f"  {cam['id']} ({cam['type']}): {cam['width']}x{cam['height']} @ {cam['fps']:.1f}fps{port_info}")

    if usb_cameras:
        print("\nUSB camera devices:")
        for cam in usb_cameras:
            video_devices_str = ", ".join(cam['video_devices']) if cam['video_devices'] else "None found"
            print(f"  Bus {cam['bus']} Device {cam['device']}: {cam['name']} ({cam['vendor_id']}:{cam['product_id']})")
            print(f"    Video devices: {video_devices_str}")

    print(f"\nLinux tools available: {sum(linux_tools.values())}/{len(linux_tools)}")
    for tool, available in linux_tools.items():
        print(f"  {tool}: {'✓' if available else '✗'}")


def main():
    parser = argparse.ArgumentParser(
        description="Comprehensive camera detection test for Linux systems with port information",
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

    print("Comprehensive Camera Detection Test for Linux (with Port Information)")
    print("=" * 70)

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
