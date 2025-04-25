import time
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

def test_motor_connection(port_path, baudrate=1000000):
    """Test motor connection on a specific port."""
    print(f"\n=== Testing motor connection on {port_path} at {baudrate} baud ===")

    # Create proper config object
    config = FeetechMotorsBusConfig(
        port=port_path,
        motors={
            "back_left_wheel": [8, "sts3215"],
            "back_right_wheel": [10, "sts3215"],
            "front_left_wheel": [9, "sts3215"],
            "front_right_wheel": [11, "sts3215"]
        }
    )

    # Create motor bus with config
    bus = FeetechMotorsBus(config)

    try:
        print("1. Attempting to connect...")
        bus.connect()
        print("   ✓ Connected successfully")

        # Test basic read operations
        print("\n2. Testing individual motor reads...")
        for motor_name, (motor_id, _) in bus.motors.items():
            try:
                print(f"   Reading from {motor_name} (ID: {motor_id})...")
                pos = bus.read("Present_Position", motor_name)
                print(f"   ✓ {motor_name}: Position = {pos}")
            except Exception as e:
                print(f"   ✗ {motor_name}: Error - {str(e)}")

        # Test write operations
        print("\n3. Testing basic write operation...")
        try:
            print("   Setting Lock to 0...")
            bus.write("Lock", 0)
            print("   ✓ Lock write successful")
        except Exception as e:
            print(f"   ✗ Lock write failed: {str(e)}")

        # Test velocity mode
        print("\n4. Testing velocity mode setup...")
        try:
            motor_ids = ["back_left_wheel", "back_right_wheel", "front_left_wheel", "front_right_wheel"]
            print("   Setting velocity mode...")
            bus.write("Mode", [1, 1, 1, 1], motor_ids)
            print("   ✓ Mode set successfully")
        except Exception as e:
            print(f"   ✗ Mode set failed: {str(e)}")

        # Test velocity reading
        print("\n5. Testing velocity reading...")
        try:
            speeds = bus.read("Present_Speed", motor_ids)
            print(f"   ✓ Current speeds: {speeds}")
        except Exception as e:
            print(f"   ✗ Velocity read failed: {str(e)}")

    except Exception as e:
        print(f"   ✗ Connection failed: {str(e)}")
    finally:
        try:
            print("\n6. Disconnecting...")
            bus.disconnect()
            print("   ✓ Disconnected successfully")
        except Exception as e:
            print(f"   ✗ Disconnect failed: {str(e)}")

def main():
    # List of ports to test
    ports = [
        '/dev/ttyAMA0',
        '/dev/ttyTHS2',
        '/dev/ttyGS0',
        '/dev/ttyTHS1',
        '/dev/ttyTCU0'
    ]

    # Test each port
    for port in ports:
        try:
            test_motor_connection(port)
            time.sleep(1)  # Wait between tests
        except Exception as e:
            print(f"\nFatal error testing {port}: {str(e)}")
        print("\n" + "="*50)

if __name__ == "__main__":
    main()
