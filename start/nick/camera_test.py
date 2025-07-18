#!/usr/bin/env python3
"""
Simple Camera Port Detection for Linux

Shows which USB ports your cameras are connected to.
"""

import glob
import re
import subprocess
from typing import Dict, List


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
    """Get USB info for a video device"""
    usb_info = {}

    # Get USB bus/device numbers
    sysfs_path = run_command(f"readlink -f /sys/class/video4linux/{device_path.split('/')[-1]}/device")
    if sysfs_path:
        usb_path = run_command(f"find {sysfs_path} -name 'busnum' -o -name 'devnum' | head -1")
        if usb_path:
            usb_dir = usb_path.rsplit('/', 1)[0]
            busnum = run_command(f"cat {usb_dir}/busnum 2>/dev/null")
            devnum = run_command(f"cat {usb_dir}/devnum 2>/dev/null")
            if busnum and devnum:
                usb_info['bus_num'] = busnum.strip()
                usb_info['dev_num'] = devnum.strip()

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

            print(f"  {device} → USB Bus {bus}, Device {dev}")
            print(f"    Port: {port_name}\n")
        else:
            print(f"  {device} → No USB info available\n")


if __name__ == "__main__":
    find_camera_ports()
