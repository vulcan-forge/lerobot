#!/usr/bin/env python3

import sys
sys.path.append('.')

from lerobot.common.motors.feetech import FeetechMotorsBus

def test_port(port_name):
    print(f"\nTesting port: {port_name}")
    try:
        # Try to connect
        bus = FeetechMotorsBus(port=port_name, motors={})
        bus.connect()
        print(f"✓ Successfully connected to {port_name}")
        
        # Try to read from motor ID 1
        try:
            result = bus.sync_read("Present_Position", ["motor1"])
            print(f"✓ Found motor ID 1 on {port_name}")
        except Exception as e:
            print(f"✗ No motor ID 1 found on {port_name}: {e}")
        
        # Try to read from motor ID 7
        try:
            result = bus.sync_read("Present_Position", ["motor7"])
            print(f"✓ Found motor ID 7 on {port_name}")
        except Exception as e:
            print(f"✗ No motor ID 7 found on {port_name}: {e}")
        
        bus.disconnect()
        return True
    except Exception as e:
        print(f"✗ Error on {port_name}: {e}")
        return False

# Test all your ports
ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']

print("Testing all USB ports for motor connections...")
for port in ports:
    test_port(port) 