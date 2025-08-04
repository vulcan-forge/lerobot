from functools import cached_property
import json
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

    def auto_calibrate(self, reversed: bool = False, full_reset: bool = False) -> None:
        """Automatically calibrate the robot using current monitoring to detect mechanical limits.

        This method performs automatic calibration by:
        1. Adjusting calibration so current physical positions become desired logical positions
        2. Detecting mechanical limits using current monitoring
        3. Setting homing offsets to center the range around the middle of detected limits
        4. Writing calibration to motors and saving to file

        WARNING: This process involves moving the robot to find limits.
        Ensure the robot arm is clear of obstacles and people during calibration.
        """
        logger.info(f"Starting automatic calibration of {self}")
        logger.warning("WARNING: Robot will move to detect mechanical limits. Ensure clear workspace!")

        # Step 1: Adjust calibration so current positions become desired logical positions
        logger.info("Adjusting calibration to align current positions with desired logical positions...")
        homing_offsets = self._initialize_calibration(reversed)

        # If hard_reset is False, we don't need to detect mechanical limits
        # Only detect mechanical limits if the customer is doing a hard reset
        detected_ranges = {}
        if full_reset:
            # Step 2: Detect actual mechanical limits using current monitoring
            # Note: Torque will be enabled during limit detection
            logger.info("Detecting mechanical limits using current monitoring...")
            detected_ranges = self._detect_mechanical_limits(reversed)

            # Step 3: Disable torque for safety before setting homing offsets
            logger.info("Disabling torque for safety...")
            self.bus.disable_torque()
        else:
            # If we are not doing a full reset, we should manually set the range of motions
            # the homing offsets are set in the _initialize_calibration function
            # Manually get range of motions from the default calibration file
            default_calibration = self._load_default_calibration(reversed)
            for motor, m in self.bus.motors.items():
                detected_ranges[motor] = {
                    "min": default_calibration[motor].range_min,
                    "max": default_calibration[motor].range_max,
                }

        # Step 4: Create calibration dictionary
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

        # Step 5: Write calibration to motors and save
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
    def _initialize_calibration(self, reversed: bool = False) -> None:
        """
        Initialize the calibration of the robot.
        """
        # Set all motors to half turn homings except shoulder_lift
        homing_offsets = self.bus.set_half_turn_homings()
        shoulder_lift_homing_offset = self.bus.set_position_homings({"shoulder_lift": 296 if reversed else 3800})
        homing_offsets["shoulder_lift"] = shoulder_lift_homing_offset["shoulder_lift"]
        return homing_offsets

    def _load_default_calibration(self, reversed: bool = False) -> dict[str, MotorCalibration]:
        """
        Load the default calibration from the calibration file.
        """
        if reversed:
            calibration_file = "sourccey_v3beta_follower_left_arm_default_calibration.json"
        else:
            calibration_file = "sourccey_v3beta_follower_right_arm_default_calibration.json"

        with open(calibration_file, "r") as f:
            return json.load(f)

    def _detect_mechanical_limits(self, reversed: bool = False) -> dict[str, dict[str, float]]:
        """
        Detect the mechanical limits of the robot using current monitoring.

        This function moves each motor incrementally while monitoring current draw.
        When a motor hits a mechanical limit, the current will spike, indicating
        the limit has been reached.

        Search ranges:
        - shoulder_lift: 4096 steps in negative direction only, double step size and current threshold
        - All other motors: 2048 steps in both positive and negative directions

        Returns:
            dict[str, dict[str, float]]: Dictionary mapping motor names to their
            detected min/max position limits.
        """
        logger.info("Starting mechanical limit detection...")

        # Enable torque for all motors to allow movement
        self.bus.enable_torque()

        # Get current positions as starting points
        start_positions = self.bus.sync_read("Present_Position", normalize=False)
        reset_positions = start_positions.copy()
        reset_positions['shoulder_lift'] = 1792 if reversed else 2304 # Manually set shoulder_lift to half way position

        # Initialize results dictionary
        detected_ranges = {}

        # Base parameters
        base_step_size = 50
        settle_time = 0.1

        # Motor-specific configuration
        motor_configs = {
            "shoulder_lift": {
                "search_range": 3800,
                "search_step": base_step_size * 2,
                "max_current": self.config.max_current_safety_threshold,
                "search_positive": reversed,
                "search_negative": not reversed
            },
            "gripper": {
                "search_range": 1664,
                "search_step": base_step_size,
                "max_current": self.config.max_current_safety_threshold,
                "search_positive": False,
                "search_negative": True
            }
        }

        # Default configuration for all other motors
        default_config = {
            "search_range": 2048,
            "search_step": base_step_size,
            "max_current": self.config.max_current_safety_threshold,
            "search_positive": True,
            "search_negative": True
        }

        for motor_name in self.bus.motors:
            logger.info(f"Detecting limits for motor: {motor_name}")

            # Get motor-specific configuration or use default
            config = motor_configs.get(motor_name, default_config)

            # Get current position
            start_pos = start_positions[motor_name]
            reset_pos = reset_positions[motor_name]
            min_pos = start_pos
            max_pos = start_pos

            # Test positive direction (increasing position)
            if config["search_positive"]:
                logger.info(f"  Testing positive direction for {motor_name} (range: {config['search_range']})")
                current_pos = start_pos
                steps_taken = 0
                max_steps = config["search_range"] // config["search_step"]

                while steps_taken < max_steps:
                    target_pos = current_pos + config["search_step"]
                    self.bus.write("Goal_Position", motor_name, target_pos, normalize=False)

                    # Wait for movement to settle
                    time.sleep(settle_time)

                    # Check current draw with retry logic
                    current = self.read_calibration_current(motor_name)
                    if current > config["max_current"]:
                        actual_pos = self.bus.read("Present_Position", motor_name, normalize=False)
                        logger.info(f"    Hit positive limit for {motor_name} at position {actual_pos} (current: {current}mA)")
                        max_pos = actual_pos
                        break

                    current_pos = target_pos
                    steps_taken += 1
                else:
                    logger.info(f"    Reached search range limit ({config['search_range']}) for {motor_name} positive direction")
                    actual_pos = self.bus.read("Present_Position", motor_name, normalize=False)
                    max_pos = actual_pos

                self._move_calibration_slow(motor_name, reset_pos, duration=3.0)
                time.sleep(settle_time * 10)
            else:
                logger.info(f"  Skipping positive direction for {motor_name}")
                max_pos = start_pos

            # Test negative direction (decreasing position)
            if config["search_negative"]:
                logger.info(f"  Testing negative direction for {motor_name} (range: {config['search_range']})")
                current_pos = start_pos
                steps_taken = 0
                max_steps = config["search_range"] // config["search_step"]

                while steps_taken < max_steps:
                    target_pos = current_pos - config["search_step"]
                    self.bus.write("Goal_Position", motor_name, target_pos, normalize=False)

                    # Wait for movement to settle
                    time.sleep(settle_time)

                    # Check current draw with retry logic
                    current = self.read_calibration_current(motor_name)
                    if current > config["max_current"]:
                        actual_pos = self.bus.read("Present_Position", motor_name, normalize=False)
                        logger.info(f"    Hit negative limit for {motor_name} at position {actual_pos} (current: {current}mA)")
                        min_pos = actual_pos
                        break

                    current_pos = target_pos
                    steps_taken += 1
                else:
                    logger.info(f"    Reached search range limit ({config['search_range']}) for {motor_name} negative direction")
                    actual_pos = self.bus.read("Present_Position", motor_name, normalize=False)
                    min_pos = actual_pos

                self._move_calibration_slow(motor_name, reset_pos, duration=3.0)
                time.sleep(settle_time * 10)
            else:
                logger.info(f"  Skipping negative direction for {motor_name}")
                min_pos = start_pos

            # Store detected range
            detected_ranges[motor_name] = {
                "min": int(min_pos),  # Convert to int
                "max": int(max_pos)   # Convert to int
            }

            logger.info(f"  Detected range for {motor_name}: {min_pos} to {max_pos}")

        # Reset all motors to their start positions (Just the shoulder lift is out of position)
        reset_motor = "shoulder_lift"
        self._move_calibration_slow(reset_motor, start_positions[reset_motor], duration=3.0)

        logger.info("Mechanical limit detection completed")
        return detected_ranges

    def read_calibration_current(self, motor_name: str, max_retries: int = 3, base_delay: float = 0.1) -> float:
        """
        Read the calibration current of the robot with exponential backoff retry.

        Args:
            motor_name: Name of the motor to read current from
            max_retries: Maximum number of retry attempts (default: 3)
            base_delay: Base delay in seconds for exponential backoff (default: 0.1s)

        Returns:
            Current reading in mA, or 1001 if all retries fail
        """
        for attempt in range(max_retries + 1):  # +1 to include initial attempt
            try:
                current = self.bus.read("Present_Current", motor_name, normalize=False)
                return current
            except Exception as e:
                if attempt == max_retries:
                    # Final attempt failed, log error and return default value
                    logger.error(f"Error reading calibration current for {motor_name} after {max_retries + 1} attempts: {e}")
                    return 1001
                else:
                    # Calculate exponential backoff delay
                    delay = base_delay * (2 ** attempt)
                    logger.warning(f"Attempt {attempt + 1} failed for {motor_name}: {e}. Retrying in {delay:.3f}s...")
                    time.sleep(delay)

        # This should never be reached, but just in case
        return 1001

    def _move_calibration_slow(self, motor_name: str, target_position: float, duration: float = 3.0,
                             steps_per_second: float = 10.0, max_retries: int = 3) -> bool:
        """
        Move a single motor slowly to target position during calibration.

        This function moves the motor in small steps over the specified duration
        to prevent high g-forces that could damage the robot during calibration.

        Args:
            motor_name: Name of the motor to move
            target_position: Target position to move to
            duration: Time in seconds to complete the movement (default: 3.0s)
            steps_per_second: Number of position updates per second (default: 10Hz)
            max_retries: Maximum retries for each position write (default: 3)

        Returns:
            True if movement completed successfully, False otherwise
        """
        try:
            # Get current position
            current_position = self.bus.read("Present_Position", motor_name, normalize=False)

            # Calculate movement parameters
            total_steps = int(duration * steps_per_second)
            step_time = duration / total_steps

            logger.info(f"Moving {motor_name} from {current_position:.1f} to {target_position:.1f} over {duration}s")

            # Move in small steps
            for step in range(total_steps + 1):  # +1 to include final position
                # Calculate interpolation factor (0 to 1)
                t = step / total_steps

                # Interpolate between current and target position
                interpolated_position = current_position + t * (target_position - current_position)

                # Convert to integer for motor bus
                interpolated_position_int = int(round(interpolated_position))

                # Write position with retry logic
                self.bus.write("Goal_Position", motor_name, interpolated_position_int, normalize=False)

                # Wait for next step (except on final step)
                if step < total_steps:
                    time.sleep(step_time)

            logger.info(f"Successfully moved {motor_name} to {target_position:.1f}")
            return True

        except Exception as e:
            logger.error(f"Error during slow movement of {motor_name}: {e}")
            return False


