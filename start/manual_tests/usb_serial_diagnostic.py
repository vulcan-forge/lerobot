#!/usr/bin/env python3
"""
USB Serial Chip Diagnostic Script
Compare working vs non-working USB-to-serial chips
"""

import subprocess
import sys
import time
import json

def run_command(cmd):
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        return f"Error: {e}"

def get_usb_device_info():
    """Get detailed USB device information"""
    devices = []
    lsusb_output = run_command("lsusb")

    for line in lsusb_output.split('\n'):
        if line.strip():
            parts = line.split()
            if len(parts) >= 6:
                bus = parts[1]
                device = parts[3].rstrip(':')
                vendor_product = parts[5]
                description = ' '.join(parts[6:]) if len(parts) > 6 else ''

                devices.append({
                    'bus': bus,
                    'device': device,
                    'vendor_product': vendor_product,
                    'description': description,
                    'full_path': f"{bus}-{device}"
                })

    return devices

def get_serial_port_info():
    """Get serial port information"""
    ports = []
    tty_devices = run_command("ls -la /dev/ttyUSB* 2>/dev/null")

    if tty_devices and not tty_devices.startswith("Error"):
        for line in tty_devices.split('\n'):
            if line.strip():
                parts = line.split()
                if len(parts) >= 10:
                    device = parts[-1]
                    major_minor = parts[4]
                    permissions = parts[0]

                    # Get USB device info for this port
                    usb_info = run_command(f"udevadm info -a -n {device} | grep -E 'ATTRS{{idVendor}}|ATTRS{{idProduct}}|ATTRS{{product}}' | head -3")

                    ports.append({
                        'device': device,
                        'major_minor': major_minor,
                        'permissions': permissions,
                        'usb_info': usb_info
                    })

    return ports

def get_driver_info():
    """Get USB serial driver information"""
    drivers = []

    # Check loaded USB serial modules
    usb_modules = run_command("lsmod | grep -i usb")

    # Check specific common USB-to-serial drivers
    common_drivers = ['ch341', 'ftdi', 'cp210x', 'pl2303', 'usbserial']
    for driver in common_drivers:
        driver_info = run_command(f"lsmod | grep -i {driver}")
        if driver_info:
            drivers.append(driver_info)

    return drivers

def test_serial_communication(port="/dev/ttyUSB0", baudrate=1000000):
    """Test basic serial communication"""
    print(f"\nTesting serial communication on {port} at {baudrate} baud...")

    # Test if port exists
    if not run_command(f"ls {port}"):
        print(f"  ✗ Port {port} does not exist")
        return False

    # Test permissions
    permissions = run_command(f"ls -la {port}")
    print(f"  Permissions: {permissions}")

    # Test if we can open the port
    try:
        import serial
        ser = serial.Serial(port, baudrate, timeout=1)
        ser.close()
        print(f"  ✓ Successfully opened {port} at {baudrate} baud")
        return True
    except Exception as e:
        print(f"  ✗ Failed to open {port}: {e}")
        return False

def diagnose_usb_serial():
    print("=== USB Serial Chip Diagnostic ===")
    print("Comparing working vs non-working USB-to-serial chips")

    print("\n1. All USB Devices:")
    usb_devices = get_usb_device_info()
    for device in usb_devices:
        print(f"  Bus {device['bus']} Device {device['device']}: {device['vendor_product']} - {device['description']}")

    print("\n2. Serial Ports:")
    serial_ports = get_serial_port_info()
    for port in serial_ports:
        print(f"  {port['device']}: {port['permissions']}")
        print(f"    USB Info: {port['usb_info']}")

    print("\n3. USB Serial Drivers:")
    drivers = get_driver_info()
    for driver in drivers:
        print(f"  {driver}")

    print("\n4. Recent USB Messages:")
    print(run_command("dmesg | grep -i usb | tail -10"))

    print("\n5. Testing Serial Communication:")
    # Test the current port
    test_serial_communication("/dev/ttyUSB0", 1000000)

    print("\n6. USB Buffer Settings:")
    print(f"  USB FS Memory: {run_command('cat /sys/module/usbcore/parameters/usbfs_memory_mb')} MB")

    print("\n7. Recommendations:")
    print("  - Compare the vendor/product IDs between working and non-working chips")
    print("  - Check if different drivers are being used")
    print("  - Verify USB buffer settings are sufficient")
    print("  - Test with different baud rates if needed")

if __name__ == "__main__":
    diagnose_usb_serial()
