#!/usr/bin/env python3
"""
Simple test script to move a single motor between positions 1000, 1500, and back to 1000.
"""

import time
import sys
from pathlib import Path

from lerobot.common.motors.motors_bus import MotorCalibration

# Add the project root to the path so we can import lerobot
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.feetech import FeetechMotorsBus, OperatingMode

def test_single_motor_movement():
    """Test moving a single motor between different positions."""

    # Motor configuration - adjust these based on your setup
    motor_id = 1  # Change this to your motor's ID
    motor_model = "sts3215"  # Change this to your motor's model
    port = "COM13"  # Change this to your port (e.g., "/dev/ttyACM0", "COM13", etc.)

    # Create a single motor configuration
    motors = {
        "test_motor": Motor(motor_id, motor_model, MotorNormMode.RANGE_M100_100),
    }

    print(f"Testing single motor movement")
    print(f"Motor ID: {motor_id}")
    print(f"Motor Model: {motor_model}")
    print(f"Port: {port}")
    print("-" * 50)

    # Create the motors bus
    bus = FeetechMotorsBus(port=port, motors=motors, calibration={"test_motor": MotorCalibration(id=motor_id, drive_mode=0, homing_offset=0, range_min=0, range_max=4095)})

    try:
        # Connect to the motor
        print("Connecting to motor...")
        bus.connect(handshake=False)
        print("✓ Connected successfully")

        # Test ping to verify motor is responding
        print(f"Testing ping on motor ID {motor_id}...")
        try:
            model_number = bus.ping(motor_id)
            print(f"✓ Motor responded with model number: {model_number}")
        except Exception as e:
            print(f"✗ Ping failed: {e}")
            return False

        # Read current position
        print("\nReading current position...")
        current_pos = bus.read("Present_Position", "test_motor", normalize=False)
        print(f"Current position: {current_pos}")

        # Define target positions
        positions = [1000, 3000, 1000]
        delays = [5, 5, 5]  # Increased delays

        print("\n=== STARTING MOVEMENT TEST ===")

        for i, (target_pos, delay) in enumerate(zip(positions, delays)):
            print(f"\n--- Movement {i+1}: Position {target_pos} ---")

            # Read position before movement
            before_pos = bus.read("Present_Position", "test_motor", normalize=False)
            print(f"Position before movement: {before_pos}")

            # Move to target position - IMPORTANT: Use normalize=False
            print(f"Moving to position {target_pos}...")
            bus.write("Goal_Position", "test_motor", target_pos, normalize=False)

            # Verify goal position was set correctly
            goal_pos = bus.read("Goal_Position", "test_motor", normalize=False)
            print(f"Goal position set to: {goal_pos}")

            # Monitor movement progress
            print("Monitoring movement...")
            for t in range(delay):
                time.sleep(1)
                current_pos = bus.read("Present_Position", "test_motor", normalize=False)
                moving = bus.read("Moving", "test_motor", normalize=False)
                print(f"  t={t+1}s - Position: {current_pos}, Moving: {moving}")

                # Check if movement completed
                if abs(current_pos - target_pos) <= 10:
                    print(f"  ✓ Reached target position")
                    break

            # Read actual position
            actual_pos = bus.read("Present_Position", "test_motor", normalize=False)
            print(f"Target: {target_pos}, Actual: {actual_pos}")

            # Check if movement was successful
            if abs(actual_pos - target_pos) <= 10:  # Allow 10 units tolerance
                print("✓ Movement successful")
            else:
                print("⚠ Movement may not have completed (position difference > 10)")

        print("\n=== MOVEMENT TEST COMPLETED ===")

        # Final position read
        final_pos = bus.read("Present_Position", "test_motor", normalize=False)
        print(f"Final position: {final_pos}")

        return True

    except Exception as e:
        print(f"✗ Error during test: {e}")
        return False

    finally:
        # Clean up
        try:
            print("\nDisconnecting...")
            bus.disconnect()
            print("✓ Disconnected successfully")
        except:
            pass

def test_current_monitoring():
    """Test current monitoring and safety stopping."""

    # Motor configuration - same as your working function
    motor_id = 1
    motor_model = "sts3215"
    port = "COM13"

    # Current safety thresholds
    MAX_CURRENT_THRESHOLD = 800  # 800mA - immediate stop
    WARNING_CURRENT_THRESHOLD = 600  # 600mA - warning level
    SUSTAINED_CURRENT_THRESHOLD = 500  # 500mA - sustained monitoring
    SUSTAINED_TIME_WINDOW = 3  # seconds to monitor sustained current

    motors = {
        "test_motor": Motor(motor_id, motor_model, MotorNormMode.RANGE_M100_100),
    }

    print(f"Testing current monitoring and safety")
    print(f"Motor ID: {motor_id}")
    print(f"Motor Model: {motor_model}")
    print(f"Port: {port}")
    print(f"Max Current Threshold: {MAX_CURRENT_THRESHOLD}mA")
    print(f"Warning Threshold: {WARNING_CURRENT_THRESHOLD}mA")
    print(f"Sustained Threshold: {SUSTAINED_CURRENT_THRESHOLD}mA")
    print("-" * 50)

    bus = FeetechMotorsBus(port=port, motors=motors, calibration={"test_motor": MotorCalibration(id=motor_id, drive_mode=0, homing_offset=0, range_min=0, range_max=4095)})

    try:
        # Connect to the motor
        print("Connecting to motor...")
        bus.connect(handshake=False)
        print("✓ Connected successfully")

        # Test ping
        print(f"Testing ping on motor ID {motor_id}...")
        try:
            model_number = bus.ping(motor_id)
            print(f"✓ Motor responded with model number: {model_number}")
        except Exception as e:
            print(f"✗ Ping failed: {e}")
            return False

        # Read baseline current for 5 seconds
        print("\n=== BASELINE CURRENT READING ===")
        print("Reading current for 5 seconds (don't touch the motor)...")
        baseline_readings = []
        for t in range(5):
            time.sleep(1)
            current = bus.read("Present_Current", "test_motor", normalize=False)
            load = bus.read("Present_Load", "test_motor", normalize=False)
            baseline_readings.append(current)
            print(f"  t={t+1}s - Current: {current}mA, Load: {load}")

        avg_baseline = sum(baseline_readings) / len(baseline_readings)
        print(f"Average baseline current: {avg_baseline:.1f}mA")

        # Test movement with current monitoring
        print("\n=== MOVEMENT WITH CURRENT MONITORING ===")
        print("Moving to position 3000 (monitoring current)...")
        print("Try to gently resist the movement to test current monitoring")

        # Current monitoring variables
        high_current_start_time = None
        current_readings = []

        # Move to target
        bus.write("Goal_Position", "test_motor", 3000, normalize=False)

        # Monitor for 10 seconds
        for t in range(10):
            time.sleep(1)
            current_pos = bus.read("Present_Position", "test_motor", normalize=False)
            moving = bus.read("Moving", "test_motor", normalize=False)
            current = bus.read("Present_Current", "test_motor", normalize=False)
            load = bus.read("Present_Load", "test_motor", normalize=False)

            print(f"  t={t+1}s - Position: {current_pos}, Moving: {moving}, Current: {current}mA, Load: {load}")

            # Track current readings for sustained monitoring
            current_readings.append(current)
            if len(current_readings) > SUSTAINED_TIME_WINDOW:
                current_readings.pop(0)

            # Check for immediate overcurrent
            if current > MAX_CURRENT_THRESHOLD:
                print(f"  🚨 CRITICAL CURRENT: {current}mA > {MAX_CURRENT_THRESHOLD}mA")
                print(f"  Emergency stop - disabling torque")
                bus.disable_torque(["test_motor"])
                return True  # Success - safety worked

            # Check for warning level current
            elif current > WARNING_CURRENT_THRESHOLD:
                print(f"  ⚠ HIGH CURRENT WARNING: {current}mA > {WARNING_CURRENT_THRESHOLD}mA")
                if high_current_start_time is None:
                    high_current_start_time = time.time()

            # Check for sustained high current
            elif current > SUSTAINED_CURRENT_THRESHOLD and len(current_readings) >= SUSTAINED_TIME_WINDOW:
                avg_current = sum(current_readings) / len(current_readings)
                if avg_current > SUSTAINED_CURRENT_THRESHOLD:
                    print(f"  ⚠ SUSTAINED HIGH CURRENT: {avg_current:.1f}mA average over {SUSTAINED_TIME_WINDOW}s")
                    if high_current_start_time is None:
                        high_current_start_time = time.time()
                    elif time.time() - high_current_start_time > 2:  # 2 seconds of sustained high current
                        print(f"  🚨 SUSTAINED OVERCURRENT - stopping motor")
                        bus.disable_torque(["test_motor"])
                        return True  # Success - safety worked
            else:
                high_current_start_time = None

            # Check if movement completed
            if abs(current_pos - 3000) <= 10:
                print(f"  ✓ Reached target position without triggering safety")
                break

        print("\n=== CURRENT MONITORING TEST COMPLETED ===")
        print("If no safety was triggered, try resisting the motor more next time")

        return True

    except Exception as e:
        print(f"✗ Error during current monitoring test: {e}")
        return False

    finally:
        # Clean up
        try:
            print("\nDisconnecting...")
            bus.disconnect()
            print("✓ Disconnected successfully")
        except:
            pass

if __name__ == "__main__":
    print("Single Motor Movement Test")
    print("=" * 50)

    success = False
    success = test_current_monitoring() #test_single_motor_movement()

    if success:
        print("\n🎉 Test completed successfully!")
        sys.exit(0)
    else:
        print("\n❌ Test failed!")
        sys.exit(1)
