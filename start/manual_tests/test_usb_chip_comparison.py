#!/usr/bin/env python3
"""
USB-to-Serial Chip Comparison Test
Test different USB-to-serial chips for motor communication reliability
"""

import time
import sys
import subprocess
from pathlib import Path

# Add the project root to the path so we can import lerobot
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.feetech import FeetechMotorsBus


def get_usb_chip_info(port="/dev/ttyUSB0"):
    """Get USB chip information for a specific port"""
    try:
        # Get USB device info
        cmd = f"udevadm info -a -n {port} | grep -E 'ATTRS{{idVendor}}|ATTRS{{idProduct}}|ATTRS{{product}}|ATTRS{{manufacturer}}'"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        return f"Error getting USB info: {e}"


def test_chip_performance(port="/dev/ttyUSB0", baudrate=1000000, num_motors=12):
    """Test the performance of a specific USB-to-serial chip"""

    print(f"\n=== Testing USB Chip on {port} ===")
    print(f"Baudrate: {baudrate}, Motors: {num_motors}")

    # Get chip info
    chip_info = get_usb_chip_info(port)
    print(f"Chip Info:\n{chip_info}")

    # Define motors (using first num_motors)
    motor_names = [
        "left_arm_shoulder_pan", "left_arm_shoulder_lift", "left_arm_elbow_flex",
        "left_arm_wrist_flex", "left_arm_wrist_roll", "left_arm_gripper",
        "right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
        "right_arm_wrist_flex", "right_arm_wrist_roll", "right_arm_gripper"
    ]

    motors = {}
    for i, name in enumerate(motor_names[:num_motors], 1):
        if i <= 6:
            norm_mode = MotorNormMode.RANGE_M100_100
        else:
            norm_mode = MotorNormMode.RANGE_0_100 if i == 6 or i == 12 else MotorNormMode.RANGE_M100_100
        motors[name] = Motor(i, "sts3215", norm_mode)

    try:
        # Create bus
        bus = FeetechMotorsBus(port=port, motors=motors)

        # Connect
        print("Connecting...")
        bus.connect(handshake=False)
        print("✓ Connected")

        # Set baudrate
        bus.set_baudrate(baudrate)
        print(f"✓ Baudrate set to {baudrate}")

        # Set longer timeout
        bus.set_timeout(2000)  # 2 seconds
        print("✓ Timeout set to 2000ms")

        # Test ping on all motors
        print("\nTesting ping...")
        ping_success = 0
        for motor_name, motor in motors.items():
            try:
                model_number = bus.ping(motor.id)
                print(f"  ✓ {motor_name} (ID {motor.id}): Model {model_number}")
                ping_success += 1
            except Exception as e:
                print(f"  ✗ {motor_name} (ID {motor.id}): Failed - {e}")

        print(f"Ping success rate: {ping_success}/{len(motors)} ({ping_success/len(motors)*100:.1f}%)")

        # Test sync read performance
        print(f"\nTesting sync read performance ({num_motors} motors)...")
        success_count = 0
        total_attempts = 20
        read_times = []

        for i in range(total_attempts):
            try:
                start_time = time.perf_counter()
                positions = bus.sync_read("Present_Position", normalize=False)
                end_time = time.perf_counter()

                read_time_ms = (end_time - start_time) * 1000
                read_times.append(read_time_ms)
                success_count += 1

                if i < 3:  # Show first few results
                    print(f"  Read {i+1}: Success ({read_time_ms:.1f}ms) - {len(positions)} motors")

                time.sleep(0.05)  # Small delay between reads

            except Exception as e:
                print(f"  Read {i+1}: Failed - {e}")
                time.sleep(0.1)  # Longer delay after failure

        # Calculate statistics
        success_rate = success_count / total_attempts * 100
        avg_read_time = sum(read_times) / len(read_times) if read_times else 0
        min_read_time = min(read_times) if read_times else 0
        max_read_time = max(read_times) if read_times else 0

        print(f"\nResults:")
        print(f"  Success rate: {success_count}/{total_attempts} ({success_rate:.1f}%)")
        print(f"  Average read time: {avg_read_time:.1f}ms")
        print(f"  Min read time: {min_read_time:.1f}ms")
        print(f"  Max read time: {max_read_time:.1f}ms")

        # Test different strategies
        print(f"\nTesting different strategies...")

        # Strategy 1: Read in smaller groups
        if num_motors > 6:
            print("Strategy 1: Reading in groups of 6...")
            try:
                group1 = list(motors.keys())[:6]
                group2 = list(motors.keys())[6:num_motors]

                start_time = time.perf_counter()
                pos1 = bus.sync_read("Present_Position", group1, normalize=False)
                time.sleep(0.01)
                pos2 = bus.sync_read("Present_Position", group2, normalize=False)
                end_time = time.perf_counter()

                total_time_ms = (end_time - start_time) * 1000
                print(f"  ✓ Success: {len(pos1) + len(pos2)} motors in {total_time_ms:.1f}ms")
            except Exception as e:
                print(f"  ✗ Failed: {e}")

        # Strategy 2: Individual reads
        print("Strategy 2: Reading motors individually...")
        try:
            start_time = time.perf_counter()
            all_positions = {}
            for motor_name in motors.keys():
                pos = bus.sync_read("Present_Position", [motor_name], normalize=False)
                all_positions.update(pos)
                time.sleep(0.001)  # Very small delay
            end_time = time.perf_counter()

            total_time_ms = (end_time - start_time) * 1000
            print(f"  ✓ Success: {len(all_positions)} motors in {total_time_ms:.1f}ms")
        except Exception as e:
            print(f"  ✗ Failed: {e}")

        return {
            'port': port,
            'baudrate': baudrate,
            'num_motors': num_motors,
            'ping_success_rate': ping_success / len(motors) * 100,
            'sync_read_success_rate': success_rate,
            'avg_read_time_ms': avg_read_time,
            'chip_info': chip_info
        }

    except Exception as e:
        print(f"✗ Test failed: {e}")
        return {
            'port': port,
            'baudrate': baudrate,
            'num_motors': num_motors,
            'error': str(e),
            'chip_info': chip_info
        }

    finally:
        try:
            bus.close()
        except:
            pass


def compare_chips():
    """Compare different USB-to-serial chips"""
    print("=== USB-to-Serial Chip Comparison Test ===")

    # Test different ports
    ports_to_test = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]

    results = []

    for port in ports_to_test:
        try:
            # Check if port exists
            if subprocess.run(f"ls {port}", shell=True, capture_output=True).returncode == 0:
                result = test_chip_performance(port, baudrate=1000000, num_motors=12)
                results.append(result)
            else:
                print(f"\nPort {port} does not exist, skipping...")
        except Exception as e:
            print(f"\nError testing {port}: {e}")

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)

    for result in results:
        print(f"\nPort: {result['port']}")
        print(f"Chip: {result.get('chip_info', 'Unknown')}")
        if 'error' in result:
            print(f"Status: FAILED - {result['error']}")
        else:
            print(f"Status: SUCCESS")
            print(f"  Ping success rate: {result['ping_success_rate']:.1f}%")
            print(f"  Sync read success rate: {result['sync_read_success_rate']:.1f}%")
            print(f"  Average read time: {result['avg_read_time_ms']:.1f}ms")

    # Recommendations
    print("\n" + "="*60)
    print("RECOMMENDATIONS")
    print("="*60)

    working_chips = [r for r in results if 'error' not in r and r['sync_read_success_rate'] > 90]

    if working_chips:
        best_chip = max(working_chips, key=lambda x: x['sync_read_success_rate'])
        print(f"✓ Best performing chip: {best_chip['port']}")
        print(f"  Use this chip for reliable motor communication")
    else:
        print("✗ No chips achieved >90% success rate")
        print("  Consider:")
        print("    - Using a different USB-to-serial adapter")
        print("    - Reducing the number of motors per sync read")
        print("    - Using individual motor reads instead of sync reads")


if __name__ == "__main__":
    compare_chips()
