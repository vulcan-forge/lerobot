#!/usr/bin/env python3
"""
Simple Camera Port Detection for Linux

Shows which USB ports your cameras are connected to.
"""

import glob
import re
import subprocess
from typing import Dict, List
import os


def run_command(cmd: str) -> str:
    """Run a shell command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.stdout.strip()
    except:
        return ""


def get_usb_port_info() -> Dict[str, str]:
    """Get USB port information for all devices"""
    port_info = {}
    usb_tree = run_command("lsusb -t 2>/dev/null")

    if usb_tree:
        current_bus = None
        current_port = None

        for line in usb_tree.split('\n'):
            if line.strip():
                # Parse bus and port
                bus_match = re.search(r'Bus (\d+)', line)
                port_match = re.search(r'Port (\d+)(?:\.(\d+))?', line)

                if bus_match and port_match:
                    current_bus = bus_match.group(1)
                    port_parts = port_match.groups()
                    current_port = f"{port_parts[0]}.{port_parts[1]}" if port_parts[1] else port_parts[0]

                # Extract device info
                device_match = re.search(r'(\w+)\s+(\w+)\s+(\w+)\s+(.+)', line)
                if device_match and current_bus and current_port:
                    device_name = device_match.group(4)
                    device_key = f"{current_bus}:{current_port}"
                    port_info[device_key] = device_name

    return port_info


def get_video_device_usb_info(device_path: str) -> Dict:
    """Get USB info for a video device using udevadm"""
    usb_info = {}

    # Use udevadm to get device properties
    udev_output = run_command(f"udevadm info --query=property --name={device_path}")

    if udev_output:
        for line in udev_output.split('\n'):
            if line.startswith('ID_VENDOR_ID='):
                usb_info['vendor_id'] = line.split('=')[1]
            elif line.startswith('ID_MODEL_ID='):
                usb_info['model_id'] = line.split('=')[1]
            elif line.startswith('ID_USB_INTERFACE_NUM='):
                usb_info['interface'] = line.split('=')[1]

    # Try to get bus/device numbers from sysfs
    try:
        # Get the device path in sysfs
        device_name = device_path.split('/')[-1]
        sysfs_path = run_command(f"readlink -f /sys/class/video4linux/{device_name}/device")

        if sysfs_path:
            # Navigate up to find the USB device directory
            current_path = sysfs_path
            for _ in range(10):  # Limit depth to avoid infinite loops
                if os.path.exists(f"{current_path}/busnum") and os.path.exists(f"{current_path}/devnum"):
                    busnum = run_command(f"cat {current_path}/busnum 2>/dev/null")
                    devnum = run_command(f"cat {current_path}/devnum 2>/dev/null")
                    if busnum and devnum:
                        usb_info['bus_num'] = busnum.strip()
                        usb_info['dev_num'] = devnum.strip()
                        break
                current_path = os.path.dirname(current_path)
                if current_path == '/':
                    break
    except:
        pass

    return usb_info


def find_camera_ports():
    """Find all cameras and their USB ports"""
    print("Camera USB Port Detection")
    print("=" * 30)

    # Get USB port information
    usb_ports = get_usb_port_info()

    # Find all video devices
    video_devices = glob.glob('/dev/video*')

    if not video_devices:
        print("No video devices found.")
        return

    print(f"Found {len(video_devices)} video device(s):\n")

    for device in sorted(video_devices):
        usb_info = get_video_device_usb_info(device)

        if usb_info and 'bus_num' in usb_info and 'dev_num' in usb_info:
            bus = usb_info['bus_num']
            dev = usb_info['dev_num']

            # Find the USB port for this device
            usb_key = f"{bus}:{dev}"
            port_name = usb_ports.get(usb_key, "Unknown")

            vendor_info = ""
            if 'vendor_id' in usb_info and 'model_id' in usb_info:
                vendor_info = f" ({usb_info['vendor_id']}:{usb_info['model_id']})"

            print(f"  {device} → USB Bus {bus}, Device {dev}{vendor_info}")
            print(f"    Port: {port_name}\n")
        else:
            print(f"  {device} → No USB info available\n")


if __name__ == "__main__":
    find_camera_ports()
