import time
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

def test_single_motor(port_path):
    """Test just one motor with basic operations"""
    print(f"\n=== Testing single motor on {port_path} ===")

    config = FeetechMotorsBusConfig(
        port=port_path,
        motors={
            "back_left_wheel": [8, "sts3215"],
        }
    )

    bus = FeetechMotorsBus(config)

    try:
        print("1. Connecting...")
        bus.connect()
        print("   ✓ Connected")

        print("\n2. Write test with verification...")
        try:
            print("   Writing Lock=0...")
            bus.write("Lock", 0, "back_left_wheel")
            time.sleep(0.1)

            print("   Verifying write...")
            value = bus.read("Lock", "back_left_wheel")
            print(f"   ✓ Write verified! Read back value: {value}")
        except Exception as e:
            print(f"   ✗ Write/verify failed: {e}")

    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        try:
            bus.disconnect()
            print("\nDisconnected cleanly")
        except:
            pass

def main():
    # Test each port
    ports = ['/dev/ttyTHS2', '/dev/ttyGS0', '/dev/ttyTHS1', '/dev/ttyTCU0', '/dev/ttyAMA0']

    for port in ports:
        test_single_motor(port)
        print("\n" + "="*50)

if __name__ == "__main__":
    main()
