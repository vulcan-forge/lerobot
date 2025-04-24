import os
import sys
import numpy as np
from pathlib import Path

from lerobot.common.robot_devices.robots.configs import SourcceyV1BetaRobotConfig
from lerobot.common.robot_devices.robots.mobile_manipulator import MobileManipulator


PYNPUT_AVAILABLE = True
try:
    # Only import if there's a valid X server or if we're not on a Pi
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        print("No DISPLAY set. Skipping pynput import.")
        raise ImportError("pynput blocked intentionally due to no display.")

    from pynput import keyboard
except ImportError:
    keyboard = None
    PYNPUT_AVAILABLE = False
except Exception as e:
    keyboard = None
    PYNPUT_AVAILABLE = False
    print(f"Could not import pynput: {e}")


class SourcceyVBetaManipulator(MobileManipulator):
    def __init__(self, config: SourcceyV1BetaRobotConfig):
        """
        Initializes the SourcceyVBetaManipulator with Feetech motors bus.
        """
        # Initialize parent class with the config
        super().__init__(config)
        self.speed_index = 2

    def body_to_wheel_raw(
        self,
        x_cmd: float,
        y_cmd: float,
        theta_cmd: float,
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
        max_raw: int = 3000,
    ) -> dict:
        """
        Convert desired body-frame velocities into wheel raw commands for a 4-wheeled mecanum robot.
        The wheels are mounted at 45 degrees.

        Parameters:
          x_cmd      : Linear velocity in x (m/s).
          y_cmd      : Linear velocity in y (m/s).
          theta_cmd  : Rotational velocity (deg/s).
          wheel_radius: Radius of each wheel (meters).
          base_radius : Distance from the center of rotation to each wheel (meters).
          max_raw    : Maximum allowed raw command (ticks) per wheel.

        Returns:
          A dictionary with wheel raw commands:
             {"back_left_wheel": value, "back_right_wheel": value,
              "front_left_wheel": value, "front_right_wheel": value}.
        """
        # Convert rotational velocity from deg/s to rad/s
        theta_rad = theta_cmd * (np.pi / 180.0)

        # For mecanum wheels:
        # x: strafing (left/right)
        # y: forward/back (already working)
        # theta: rotation
        scale = 1.0
        m = np.array([
            [scale, -scale, -base_radius],    # back_left
            [scale,  scale, -base_radius],    # back_right
            [scale, -scale,  base_radius],    # front_left
            [scale,  scale,  base_radius]     # front_right
        ])

        # Compute wheel velocities
        wheel_velocities = m.dot(np.array([x_cmd, y_cmd, theta_rad]))

        # Convert to angular velocities (rad/s)
        wheel_angular_velocities = wheel_velocities / wheel_radius

        # Convert to deg/s
        wheel_degps = wheel_angular_velocities * (180.0 / np.pi)

        # Scaling
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > max_raw:
            scale = max_raw / max_raw_computed
            wheel_degps = wheel_degps * scale

        # Convert each wheel's angular speed (deg/s) to a raw integer
        wheel_raw = [MobileManipulator.degps_to_raw(deg) for deg in wheel_degps]

        return {
            "back_left_wheel": wheel_raw[0],
            "back_right_wheel": wheel_raw[1],
            "front_left_wheel": wheel_raw[2],
            "front_right_wheel": wheel_raw[3]
        }

    def wheel_raw_to_body(
        self, wheel_raw: dict, wheel_radius: float = 0.05, base_radius: float = 0.125
    ) -> tuple:
        """
        Convert wheel raw command feedback back into body-frame velocities for a 4-wheeled mecanum robot.
        The wheels are mounted at 45 degrees.

        Parameters:
          wheel_raw   : Dictionary with raw wheel commands
                       (keys: "back_left_wheel", "back_right_wheel",
                             "front_left_wheel", "front_right_wheel").
          wheel_radius: Radius of each wheel (meters).
          base_radius : Distance from the robot center to each wheel (meters).

        Returns:
          A tuple (x_cmd, y_cmd, theta_cmd) where:
             x_cmd      : Linear velocity in x (m/s).
             y_cmd      : Linear velocity in y (m/s).
             theta_cmd  : Rotational velocity in deg/s.
        """
        # Extract the raw values in order
        raw_list = [
            int(wheel_raw.get("back_left_wheel", 0)),
            int(wheel_raw.get("back_right_wheel", 0)),
            int(wheel_raw.get("front_left_wheel", 0)),
            int(wheel_raw.get("front_right_wheel", 0)),
        ]

        # Convert each raw command back to an angular speed in deg/s
        wheel_degps = np.array([MobileManipulator.raw_to_degps(r) for r in raw_list])
        # Convert from deg/s to rad/s
        wheel_radps = wheel_degps * (np.pi / 180.0)
        # Compute each wheel's linear speed (m/s) from its angular speed
        wheel_linear_speeds = wheel_radps * wheel_radius

        # For mecanum wheels, the inverse kinematic matrix
        scale = 1
        m_inv = np.array([
            [scale,  scale,  scale, scale],    # x: [back_left, back_right, front_left, front_right]
            [-scale, scale, -scale, scale],    # y: [back_left, back_right, front_left, front_right]
            [-1/base_radius, -1/base_radius, 1/base_radius, 1/base_radius]  # theta: [back_left, back_right, front_left, front_right]
        ])

        # Calculate body velocities
        velocity_vector = m_inv.dot(wheel_linear_speeds)
        x_cmd, y_cmd, theta_rad = velocity_vector
        theta_cmd = theta_rad * (180.0 / np.pi)
        return (x_cmd, y_cmd, theta_cmd)


class SourcceyVBeta:
    def __init__(self, motor_bus):
        """
        Initializes the SourcceyVBeta with Feetech motors bus.
        """
        self.motor_bus = motor_bus
        self.motor_ids = ["back_left_wheel", "back_right_wheel", "front_left_wheel", "front_right_wheel"]

        # Initialize motors in velocity mode.
        self.motor_bus.write("Lock", 0)
        self.motor_bus.write("Mode", [1, 1, 1, 1], self.motor_ids)
        self.motor_bus.write("Lock", 1)
        print("Motors set to velocity mode.")

    def read_velocity(self):
        """
        Reads the raw speeds for all wheels. Returns a dictionary with motor names:
        """
        raw_speeds = self.motor_bus.read("Present_Speed", self.motor_ids)
        return {
            "back_left_wheel": int(raw_speeds[0]),
            "back_right_wheel": int(raw_speeds[1]),
            "front_left_wheel": int(raw_speeds[2]),
            "front_right_wheel": int(raw_speeds[3]),
        }

    def set_velocity(self, command_speeds):
        """
        Sends raw velocity commands (16-bit encoded values) directly to the motor bus.
        The order of speeds must correspond to self.motor_ids.
        """
        self.motor_bus.write("Goal_Speed", command_speeds, self.motor_ids)

    def stop(self):
        """Stops the robot by setting all motor speeds to zero."""
        self.motor_bus.write("Goal_Speed", [0, 0, 0, 0], self.motor_ids)
        print("Motors stopped.")
