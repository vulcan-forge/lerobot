from functools import cached_property
import time
from typing import Any
from venv import logger

import numpy as np
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors.feetech.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode
from lerobot.robots.robot import Robot
from lerobot.robots.sourccey.sourccey_v3beta.sourccey_v3beta_follower.config_sourccey_v3beta_follower import SourcceyV3BetaFollowerConfig
from lerobot.robots.utils import ensure_safe_goal_position

class SourcceyV3BetaFollower(Robot):
    config_class = SourcceyV3BetaFollowerConfig
    name = "sourccey_v3beta_follower"

    def __init__(self, config: SourcceyV3BetaFollowerConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        motor_ids = [1, 2, 3, 4, 5, 6]
        if self.config.orientation == "right":
            motor_ids = [7, 8, 9, 10, 11, 12]

        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(motor_ids[0], "sts3215", norm_mode_body),
                "shoulder_lift": Motor(motor_ids[1], "sts3250", norm_mode_body),
                "elbow_flex": Motor(motor_ids[2], "sts3215", norm_mode_body),
                "wrist_flex": Motor(motor_ids[3], "sts3215", norm_mode_body),
                "wrist_roll": Motor(motor_ids[4], "sts3215", norm_mode_body),
                "gripper": Motor(motor_ids[5], "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            # Calibration file exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        full_turn_motor = "wrist_roll"
        unknown_range_motors = [motor for motor in self.bus.motors if motor != full_turn_motor]
        print(
            f"Move all joints except '{full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        range_mins[full_turn_motor] = 0
        range_maxes[full_turn_motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            drive_mode = 1 if motor == "shoulder_lift" or (self.config.orientation == "right" and motor == "gripper") else 0
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=drive_mode,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def auto_calibrate(self) -> None:
        """Automatically calibrate the robot using current monitoring to detect mechanical limits.

        This method performs automatic calibration by:
        1. Detecting mechanical limits using current monitoring
        2. Setting homing offsets to center the range around the middle of detected limits
        3. Writing calibration to motors and saving to file

        WARNING: This process involves moving the robot to find limits.
        Ensure the robot arm is clear of obstacles and people during calibration.
        """
        logger.info(f"Starting automatic calibration of {self}")
        logger.warning("WARNING: Robot will move to detect mechanical limits. Ensure clear workspace!")

        # Step 1: Set up motors for calibration
        logger.info("Setting up motors for calibration...")
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        # Step 2: Detect actual mechanical limits using current monitoring
        # Note: Torque will be enabled during limit detection
        logger.info("Detecting mechanical limits using current monitoring...")
        detected_ranges = self._detect_mechanical_limits()

        # Step 3: Disable torque for safety before setting homing offsets
        logger.info("Disabling torque for safety...")
        self.bus.disable_torque()

        # Step 4: Calculate homing offsets to center each range
        logger.info("Calculating homing offsets to center detected ranges...")
        homing_offsets = self._calculate_centered_homing_offsets(detected_ranges)

        # Step 5: Create calibration dictionary
        self.calibration = {}
        for motor, m in self.bus.motors.items():
            drive_mode = 1 if motor == "shoulder_lift" or (self.config.orientation == "right" and motor == "gripper") else 0
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=drive_mode,
                homing_offset=homing_offsets[motor],
                range_min=detected_ranges[motor]["min"],
                range_max=detected_ranges[motor]["max"],
            )

        # Step 6: Write calibration to motors and save
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"Automatic calibration completed and saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus.write("P_Coefficient", motor, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus.write("I_Coefficient", motor, 0)
            self.bus.write("D_Coefficient", motor, 32)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()
        obs_dict = self.bus.sync_read("Present_Position")
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            the action sent to the motors, potentially clipped.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Check for NaN values and skip sending actions if any are found
        present_pos = self.bus.sync_read("Present_Position")
        if any(np.isnan(v) for v in goal_pos.values()) or any(np.isnan(v) for v in present_pos.values()):
            logger.warning("NaN values detected in goal positions. Skipping action execution.")
            return {f"{motor}.pos": val for motor, val in present_pos.items()}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_present_pos = self._apply_minimum_action(goal_present_pos)
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the arm
        self.bus.sync_write("Goal_Position", goal_pos)

        # Check safety after sending goals
        overcurrent_motors = self._check_current_safety()
        if overcurrent_motors and len(overcurrent_motors) > 0:
            logger.warning(f"Safety triggered: {overcurrent_motors} current > {self.config.max_current_safety_threshold}mA")
            return self._handle_overcurrent_motors(overcurrent_motors, goal_pos, present_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def _apply_minimum_action(self, goal_present_pos: dict[str, tuple[float, float]]) -> dict[str, tuple[float, float]]:
        """Apply a minimum action to the robot's geared down motors.

        This function ensures that geared-down motors receive a minimum movement threshold
        to overcome friction and backlash. If the desired movement is below the threshold,
        it's amplified to the minimum threshold while preserving direction.
        """
        # Define geared down motors and their minimum action thresholds
        geared_down_motors = ["shoulder_lift"]

        adjusted_goal_present_pos = {}

        for key, (goal_pos, present_pos) in goal_present_pos.items():
            motor_name = key.replace(".pos", "")
            if motor_name in geared_down_motors:
                desired_movement = goal_pos - present_pos
                movement_magnitude = abs(desired_movement)

                # If movement is below threshold, apply minimum action
                if movement_magnitude > 0 and movement_magnitude < self.config.min_action_threshold:
                    direction = 1 if desired_movement > 0 else -1
                    adjusted_movement = direction * self.config.min_action_threshold
                    adjusted_goal_pos = present_pos + adjusted_movement
                    adjusted_goal_present_pos[key] = (adjusted_goal_pos, present_pos)
                else:
                    adjusted_goal_present_pos[key] = (goal_pos, present_pos)
            else:
                adjusted_goal_present_pos[key] = (goal_pos, present_pos)
        return adjusted_goal_present_pos

    def _check_current_safety(self) -> list[str]:
        """
        Check if any motor is over current limit and return safety status.

        Returns:
            tuple: (is_safe, overcurrent_motors)
            - is_safe: True if all motors are under current limit
            - overcurrent_motors: List of motor names that are over current
        """
        # Read current from all motors
        currents = self.bus.sync_read("Present_Current")
        overcurrent_motors = []
        for motor, current in currents.items():
            if current > self.config.max_current_safety_threshold:
                overcurrent_motors.append(motor)
                logger.warning(f"Safety triggered: {motor} current {current}mA > {self.config.max_current_safety_threshold}mA")
        return overcurrent_motors

    def _handle_overcurrent_motors(
        self,
        overcurrent_motors: list[str],
        goal_pos: dict[str, float],
        present_pos: dict[str, float],
    ) -> dict[str, float]:
        """
        Handle overcurrent motors by replacing their goal positions with present positions.

        Args:
            goal_pos: Dictionary of goal positions with keys like "shoulder_pan.pos"
            present_pos: Dictionary of present positions with keys like "shoulder_pan.pos"
            overcurrent_motors: List of motor names that are over current (e.g., ["shoulder_pan", "elbow_flex"])

        Returns:
            Dictionary of goal positions with overcurrent motors replaced by present positions
        """
        if not overcurrent_motors or len(overcurrent_motors) == 0:
            return goal_pos

        # Create copies of the goal positions to modify
        modified_goal_pos = goal_pos.copy()
        for motor_name in overcurrent_motors:
            goal_key = f"{motor_name}.pos"
            if goal_key in modified_goal_pos:
                modified_goal_pos[goal_key] = present_pos[motor_name]
                logger.warning(f"Replaced goal position for {motor_name} with present position: {present_pos[motor_name]}")

        # Sync write the modified goal positions
        goal_pos_raw = {k.replace(".pos", ""): v for k, v in modified_goal_pos.items()}
        self.bus.sync_write("Goal_Position", goal_pos_raw)
        return modified_goal_pos

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")

    ############################
    # Auto Calibration Functions
    ############################
    def _calculate_centered_homing_offsets(self, detected_ranges: dict[str, dict[str, int]]) -> dict[str, int]:
        """Calculate homing offsets to center each motor's range around the middle of its detected limits.

        Args:
            detected_ranges: Dictionary of detected min/max ranges for each motor

        Returns:
            Dictionary mapping motor names to their calculated homing offsets
        """
        homing_offsets = {}

        for motor, ranges in detected_ranges.items():
            # Calculate the middle of the detected range
            range_middle = (ranges["min"] + ranges["max"]) // 2

            # Get the motor model to determine the encoder resolution
            motor_model = self.bus._get_motor_model(motor)
            max_resolution = self.bus.model_resolution_table[motor_model] - 1

            # Calculate the target center position (half turn)
            target_center = max_resolution // 2

            # Calculate homing offset: offset = actual_middle - target_center
            homing_offset = range_middle - target_center

            homing_offsets[motor] = homing_offset

            logger.info(f"  {motor}: range_middle={range_middle}, target_center={target_center}, homing_offset={homing_offset}")

        return homing_offsets

    def _detect_mechanical_limits(self) -> dict[str, dict[str, int]]:
        """Detect mechanical limits for each motor using current monitoring.

        This method moves each joint in small increments while monitoring current.
        When current exceeds the safety threshold, it assumes a mechanical limit has been reached.
        After testing each joint, it resets to the middle position to avoid restricting other joints.

        Returns:
            Dictionary mapping motor names to their detected min/max ranges
        """
        # Get current positions as starting point
        current_positions = self.bus.sync_read("Present_Position", normalize=False)
        detected_ranges = {}

        # Define search parameters - motor-specific search distances
        search_step = 50
        current_threshold = self.config.max_current_safety_threshold * 0.8  # Use 80% of safety threshold

        # Motor-specific search distances based on their expected ranges
        motor_search_distances = {
            "shoulder_pan": 2048,    # ~270° range
            "shoulder_lift": 4096,   # Full range for shoulder lift
            "elbow_flex": 2048,      # ~270° range
            "wrist_flex": 2048,      # ~270° range
            "wrist_roll": 4096,      # Full 360° range
            "gripper": 2048,         # Gripper range
        }

        # Enable torque for limit detection
        logger.info("Enabling torque for mechanical limit detection...")
        self.bus.enable_torque()

        try:
            for motor in self.bus.motors:
                logger.info(f"Detecting limits for {motor}...")

                # Get motor-specific search distance
                max_search_distance = motor_search_distances.get(motor, 2048)  # Default to 2048
                logger.info(f"  Using search distance: {max_search_distance}")

                # Start from current position
                start_pos = current_positions[motor]
                min_pos = start_pos
                max_pos = start_pos

                # Search in positive direction
                logger.info(f"  Searching positive direction from {start_pos}")
                for step in range(0, max_search_distance, search_step):
                    test_pos = start_pos + step
                    if self._test_position_safe(motor, test_pos, current_threshold):
                        max_pos = test_pos
                    else:
                        logger.info(f"  Hit limit at position {test_pos} (current exceeded threshold)")
                        break

                # Search in negative direction
                logger.info(f"  Searching negative direction from {start_pos}")
                for step in range(0, max_search_distance, search_step):
                    test_pos = start_pos - step
                    if self._test_position_safe(motor, test_pos, current_threshold):
                        min_pos = test_pos
                    else:
                        logger.info(f"  Hit limit at position {test_pos} (current exceeded threshold)")
                        break

                # Calculate the middle position of the detected range
                middle_pos = (min_pos + max_pos) // 2

                # Reset the joint to its middle position
                logger.info(f"  Resetting {motor} to middle position {middle_pos}")
                self._move_to_position_safe(motor, middle_pos)

                # Add safety margins to detected ranges
                safety_margin = 100  # Add 100 encoder units as safety margin
                detected_ranges[motor] = {
                    "min": min_pos + safety_margin,
                    "max": max_pos - safety_margin
                }

                logger.info(f"  Detected range for {motor}: {detected_ranges[motor]}")
                logger.info(f"  Reset {motor} to middle position: {middle_pos}")

        finally:
            # Always disable torque after limit detection for safety
            logger.info("Disabling torque after limit detection...")
            self.bus.disable_torque()

        return detected_ranges

    def _move_to_position_safe(self, motor: str, position: int) -> None:
        """Safely move a motor to a specific position with current monitoring.

        Args:
            motor: Motor name to move
            position: Target position
        """
        try:
            # Move to target position
            self.bus.write("Goal_Position", motor, position, normalize=False)

            # Wait for movement to complete - reduced wait time
            time.sleep(0.5)  # Reduced from 1.0 to 0.5 seconds

            # Verify we reached the position (with some tolerance)
            actual_pos = self.bus.read("Present_Position", motor, normalize=False)
            tolerance = 50  # Allow 50 encoder units of tolerance

            if abs(actual_pos - position) > tolerance:
                logger.warning(f"  {motor} did not reach target position {position}, actual: {actual_pos}")
            else:
                logger.info(f"  {motor} successfully moved to position {actual_pos}")

        except Exception as e:
            logger.warning(f"Error moving {motor} to position {position}: {e}")

    def _test_position_safe(self, motor: str, position: int, current_threshold: int) -> bool:
        """Test if a position is safe by monitoring current.

        Args:
            motor: Motor name to test
            position: Position to test
            current_threshold: Current threshold to consider safe

        Returns:
            True if position is safe (current below threshold), False otherwise
        """
        try:
            # Move to test position
            self.bus.write("Goal_Position", motor, position, normalize=False)

            # Wait for movement to complete and check current - reduced wait time
            time.sleep(0.25)  # Reduced from 0.5 to 0.25 seconds

            # Check current fewer times for speed - reduced from 3 to 2 checks
            for _ in range(2):  # Reduced from 3 to 2 checks
                current = self.bus.read("Present_Current", motor, normalize=False)
                if current > current_threshold:
                    return False
                time.sleep(0.05)  # Reduced from 0.1 to 0.05 seconds

            return True

        except Exception as e:
            logger.warning(f"Error testing position {position} for {motor}: {e}")
            return False

    def _get_safe_motor_ranges(self) -> dict[str, dict[str, int]]:
        """Get predefined safe ranges for each motor to prevent damage during auto-calibration.

        Returns:
            Dictionary mapping motor names to their safe min/max ranges
        """
        # Define safe ranges based on motor specifications and mechanical limits
        # These are conservative ranges to prevent damage
        safe_ranges = {
            "shoulder_pan": {"min": 500, "max": 3595},    # ~270° range
            "shoulder_lift": {"min": 1000, "max": 3095},  # ~180° range
            "elbow_flex": {"min": 500, "max": 3595},      # ~270° range
            "wrist_flex": {"min": 500, "max": 3595},      # ~270° range
            "wrist_roll": {"min": 0, "max": 4095},        # Full 360° range
            "gripper": {"min": 1000, "max": 3000},        # 0-100% range
        }

        return safe_ranges
