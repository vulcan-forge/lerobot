import os
import sys
import numpy as np
from pathlib import Path
import json
import torch
import zmq
import base64
import cv2

from lerobot.common.robot_devices.robots.configs import SourcceyV1BetaRobotConfig
from lerobot.common.robot_devices.robots.mobile_manipulator import MobileManipulator
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError


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


class SourcceyV1BetaManipulator(MobileManipulator):
    def __init__(self, config: SourcceyV1BetaRobotConfig):
        """
        Initializes the SourcceyVBetaManipulator with Feetech motors bus.
        """
        # Initialize parent class with the config
        super().__init__(config)
        self.speed_index = 2

    @property
    def motor_features(self) -> dict:
        # Define the motor names for each arm
        arm_motor_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]

        # Combine all motor names for both arms
        combined_names = (
            [f"left_{motor}" for motor in arm_motor_names] +  # Left arm motors
            [f"right_{motor}" for motor in arm_motor_names]  # Right arm motors
        )

        return {
            "action": {
                "dtype": "float32",
                "shape": (len(combined_names),),
                "names": combined_names,
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(combined_names),),
                "names": combined_names,
            },
        }

    def _get_data(self):
        """
        Polls the video socket for up to 15 ms. If data arrives, decode only
        the *latest* message, returning frames, speed, and arm state. If
        nothing arrives for any field, use the last known values.
        """
        frames = {}
        present_speed = {}
        remote_arm_state_tensor = torch.zeros(12, dtype=torch.float32)  # Changed from 6 to 12

        # Poll up to 15 ms
        poller = zmq.Poller()
        poller.register(self.video_socket, zmq.POLLIN)
        socks = dict(poller.poll(15))
        if self.video_socket not in socks or socks[self.video_socket] != zmq.POLLIN:
            # No new data arrived → reuse ALL old data
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

        # Drain all messages, keep only the last
        last_msg = None
        while True:
            try:
                obs_string = self.video_socket.recv_string(zmq.NOBLOCK)
                last_msg = obs_string
            except zmq.Again:
                break

        if not last_msg:
            # No new message → also reuse old
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

        # Decode only the final message
        try:
            observation = json.loads(last_msg)

            images_dict = observation.get("images", {})
            new_speed = observation.get("present_speed", {})
            new_arm_state = observation.get("follower_arm_state", None)

            # Convert images
            for cam_name, image_b64 in images_dict.items():
                if image_b64:
                    jpg_data = base64.b64decode(image_b64)
                    np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
                    frame_candidate = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if frame_candidate is not None:
                        frames[cam_name] = frame_candidate

            # If remote_arm_state is None and frames is None there is no message then use the previous message
            if new_arm_state is not None and frames is not None:
                self.last_frames = frames

                # Ensure we have 12 values for the arm state
                if len(new_arm_state) < 12:
                    # Pad with zeros if we don't have enough values
                    padded_state = new_arm_state + [0] * (12 - len(new_arm_state))
                    remote_arm_state_tensor = torch.tensor(padded_state, dtype=torch.float32)
                else:
                    remote_arm_state_tensor = torch.tensor(new_arm_state, dtype=torch.float32)
                self.last_remote_arm_state = remote_arm_state_tensor

                present_speed = new_speed
                self.last_present_speed = new_speed
            else:
                frames = self.last_frames
                remote_arm_state_tensor = self.last_remote_arm_state
                present_speed = self.last_present_speed

        except Exception as e:
            print(f"[DEBUG] Error decoding video message: {e}")
            # If decode fails, fall back to old data
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

        return frames, present_speed, remote_arm_state_tensor

    def teleop_step(
        self, record_data: bool = False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:

        if not self.is_connected:
            raise RobotDeviceNotConnectedError("MobileManipulator is not connected. Run `connect()` first.")

        speed_setting = self.speed_levels[self.speed_index]
        xy_speed = speed_setting["xy"]  # e.g. 0.1, 0.25, or 0.4
        theta_speed = speed_setting["theta"]  # e.g. 30, 60, or 90

        # Prepare to assign the position of the leader to the follower
        left_arm_positions = []
        left_pos = self.leader_arms["left"].read("Present_Position")
        left_pos_tensor = torch.from_numpy(left_pos).float()
        left_arm_positions = left_pos_tensor.tolist()

        right_arm_positions = []
        right_pos = self.leader_arms["right"].read("Present_Position")
        right_pos_tensor = torch.from_numpy(right_pos).float()
        right_arm_positions = right_pos_tensor.tolist()

        y_cmd = 0.0  # m/s forward/backward
        x_cmd = 0.0  # m/s lateral
        theta_cmd = 0.0  # deg/s rotation
        if self.pressed_keys["forward"]:
            y_cmd += xy_speed
        if self.pressed_keys["backward"]:
            y_cmd -= xy_speed
        if self.pressed_keys["left"]:
            x_cmd += xy_speed
        if self.pressed_keys["right"]:
            x_cmd -= xy_speed
        if self.pressed_keys["rotate_left"]:
            theta_cmd += theta_speed
        if self.pressed_keys["rotate_right"]:
            theta_cmd -= theta_speed

        wheel_commands = self.body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)

        message = {"raw_velocity": wheel_commands, "left_arm_positions": left_arm_positions, "right_arm_positions": right_arm_positions}
        self.cmd_socket.send_string(json.dumps(message))

        if not record_data:
            return {}, {}

        obs_dict = self.capture_observation()

        left_arm_state_tensor = torch.tensor(left_arm_positions, dtype=torch.float32)
        right_arm_state_tensor = torch.tensor(right_arm_positions, dtype=torch.float32)

        wheel_velocity_tuple = self.wheel_raw_to_body(wheel_commands)
        wheel_velocity_mm = (
            wheel_velocity_tuple[0] * 1000.0,
            wheel_velocity_tuple[1] * 1000.0,
            wheel_velocity_tuple[2],
        )
        wheel_tensor = torch.tensor(wheel_velocity_mm, dtype=torch.float32)
        action_tensor = torch.cat([left_arm_state_tensor, right_arm_state_tensor, wheel_tensor])
        action_dict = {"action": action_tensor}

        return obs_dict, action_dict

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Not connected. Run `connect()` first.")

        # For all other control types, handle all motors
        # Ensure the action tensor has at least 17 elements:
        #   - First 6: left arm positions
        #   - Next 6: right arm positions
        #   - Next 4: wheel commands
        #   - Last 1: turn table command
        if action.numel() < 17:
            padded = torch.zeros(17, dtype=action.dtype)
            padded[:action.numel()] = action
            action = padded

        # Extract arm and base actions
        left_arm_actions = action[:6].flatten()
        right_arm_actions = action[6:12].flatten()
        wheel_actions = action[12:16].flatten()
        turn_table_action = action[16].flatten()

        # Convert wheel actions to wheel commands
        wheel_commands = {
            "back_left_wheel": int(wheel_actions[0].item()),
            "back_right_wheel": int(wheel_actions[1].item()),
            "front_left_wheel": int(wheel_actions[2].item()),
            "front_right_wheel": int(wheel_actions[3].item()),
        }

        # Create and send the message with all commands
        message = {
            "raw_velocity": wheel_commands,
            "left_arm_positions": left_arm_actions.tolist(),
            "right_arm_positions": right_arm_actions.tolist(),
            "turn_table_positions": turn_table_action.tolist(),
        }

        self.cmd_socket.send_string(json.dumps(message))
        return action

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
            [-scale, -scale,  base_radius],    # back_left
            [-scale,  scale,  base_radius],    # back_right
            [ scale, -scale,  base_radius],    # front_left
            [ scale,  scale,  base_radius]     # front_right
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
            [ -scale, -scale,  scale,  scale],    # x: [back_left, back_right, front_left, front_right]
            [ -scale,  scale, -scale,  scale],    # y: [back_left, back_right, front_left, front_right]
            [ 1/base_radius,  1/base_radius, 1/base_radius,  1/base_radius]  # theta: [back_left, back_right, front_left, front_right]
        ])

        # Calculate body velocities
        velocity_vector = m_inv.dot(wheel_linear_speeds)
        x_cmd, y_cmd, theta_rad = velocity_vector
        theta_cmd = theta_rad * (180.0 / np.pi)
        return (x_cmd, y_cmd, theta_cmd)

    def capture_observation(self) -> dict:
        """
        Capture observations from the remote robot: current follower arm positions and camera frames.
        """
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Not connected. Run `connect()` first.")

        frames, present_speed, remote_arm_state_tensor = self._get_data()

        obs_dict = {"observation.state": remote_arm_state_tensor}

        # Loop over each configured camera
        for cam_name, cam in self.cameras.items():
            frame = frames.get(cam_name, None)
            if frame is None:
                # Create a black image using the camera's configured width, height, and channels
                frame = np.zeros((cam.height, cam.width, cam.channels), dtype=np.uint8)
            obs_dict[f"observation.images.{cam_name}"] = torch.from_numpy(frame)

        return obs_dict

class SourcceyV1Beta:
    def __init__(self, left_motor_bus, right_motor_bus):
        """
        Initializes the SourcceyVBeta with Feetech motors bus.
        """
        self.left_motor_bus = left_motor_bus
        self.right_motor_bus = right_motor_bus

        self.wheel_motor_ids = ["back_left_wheel", "back_right_wheel", "front_left_wheel", "front_right_wheel"]
        self.turn_table_motor_ids = ["turn_table"]

        # Initialize wheel motors in velocity mode.
        self.left_motor_bus.write("Lock", 0)
        self.left_motor_bus.write("Mode", [1, 1, 1, 1], self.wheel_motor_ids)
        self.left_motor_bus.write("Lock", 1)

    def read_velocity(self):
        """
        Reads the raw speeds for all wheels. Returns a dictionary with motor names:
        """
        raw_speeds = self.left_motor_bus.read("Present_Speed", self.wheel_motor_ids)
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
        self.left_motor_bus.write("Goal_Speed", command_speeds, self.wheel_motor_ids)
        # Set turn table speed to the last element of command_speeds
        if len(command_speeds) > 4:
            self.right_motor_bus.write("Goal_Speed", [command_speeds[4]], self.turn_table_motor_ids)

    def stop(self):
        """Stops the robot by setting all motor speeds to zero."""
        self.left_motor_bus.write("Goal_Speed", [0, 0, 0, 0], self.wheel_motor_ids)
        self.right_motor_bus.write("Goal_Speed", [0], self.turn_table_motor_ids)
        print("Motors stopped.")
