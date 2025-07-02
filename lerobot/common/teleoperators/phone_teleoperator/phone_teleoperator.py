#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from typing import Any, Optional

import numpy as np
import torch

try:
    import pyroki as pk
    import viser
    import yourdfpy
    from scipy.spatial.transform import Rotation as R
    from viser.extras import ViserUrdf
except ImportError as e:
    raise ImportError(
        "Phone teleoperator requires additional dependencies. "
        "Please install with: pip install pyroki viser yourdfpy"
    ) from e

from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.teleoperators.teleoperator import Teleoperator

from .config_phone_teleoperator import PhoneTeleoperatorConfig

logger = logging.getLogger(__name__)


class PhoneTeleoperator(Teleoperator):
    """
    Phone-based teleoperator that uses the proven VirtualManipulator implementation.
    
    This teleoperator integrates with the VirtualManipulator system from the daxie package
    to provide phone-based robot control with proper movement patterns and button handling.
    """

    config_class = PhoneTeleoperatorConfig
    name = "phone_teleoperator"

    def __init__(self, config: PhoneTeleoperatorConfig, **kwargs):
        super().__init__(config, **kwargs)
        self.config = config
        
        # Debug logging can be enabled when needed
        logger.info("PhoneTeleoperator initialization started")
        
        # Initialize robot model for IK
        self.urdf = None
        self.robot = None
        self.urdf_vis = None
        self.server = None
        
        # Phone connection state
        self._is_connected = False
        self._phone_connected = False
        self.start_teleop = False  # ALWAYS start as False - wait for phone signal
        self.prev_is_resetting = False
        
        # Pose tracking
        self.current_t_R = np.array(self.config.initial_position)
        self.current_q_R = np.array(self.config.initial_wxyz)
        self.initial_phone_quat = None
        self.initial_phone_pos = None
        self.last_precision_mode = False
        
        # Mapping parameters
        self.quat_RP = None
        self.translation_RP = None
        
        # gRPC server and pose service (to be initialized in connect())
        self.grpc_server = None
        self.pose_service = None
        
        # Timer for reading motor positions after 5 seconds
        self.teleop_start_time = None
        self.motor_positions_read = False
        
        # Flag to show initial motor positions on first get_action call
        self.initial_positions_shown = False
        
        logger.info("PhoneTeleoperator initialization completed")

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Features for the actions produced by this teleoperator."""
        # Assuming 6 DOF arm + gripper (adjust based on your robot)
        motor_names = [
            "shoulder_pan.pos",
            "shoulder_lift.pos", 
            "elbow_flex.pos",
            "wrist_flex.pos",
            "wrist_roll.pos",
            "gripper.pos"
        ]
        return {name: float for name in motor_names}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        """Features for the feedback actions sent to this teleoperator."""
        return {}

    @property
    def is_connected(self) -> bool:
        """Whether the phone teleoperator is connected."""
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        """Phone teleoperator doesn't require calibration."""
        return True

    def connect(self, calibrate: bool = True) -> None:
        """Establish connection with phone via gRPC and initialize robot model."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            # Initialize robot model
            if not self.config.urdf_path or not self.config.mesh_path:
                raise ValueError("URDF path and mesh path must be provided in config")
                
            self.urdf = yourdfpy.URDF.load(self.config.urdf_path, mesh_dir=self.config.mesh_path)
            self.robot = pk.Robot.from_urdf(self.urdf)
            
            # Initialize visualization if enabled
            if self.config.enable_visualization:
                self._init_visualization()
            
            # Start gRPC server for phone communication
            self._start_grpc_server()
            
            self._is_connected = True
            logger.info(f"{self} connected successfully")
            
        except Exception as e:
            logger.error(f"Failed to connect {self}: {e}")
            raise

    def calibrate(self) -> None:
        """Phone teleoperator doesn't require calibration."""
        pass

    def configure(self) -> None:
        """Configure the phone teleoperator."""
        pass

    def _init_visualization(self) -> None:
        """Initialize the Viser visualization server and URDF model."""
        self.server = viser.ViserServer(port=self.config.viser_port)
        self.server.scene.add_grid("/ground", width=2.0, height=2.0)
        self.urdf_vis = ViserUrdf(self.server, self.urdf, root_node_name="/base")

    def _start_grpc_server(self) -> None:
        """Start the gRPC server for phone pose streaming."""
        try:
            # Import from daxie package
            from daxie.src.server.pos_grpc_server import start_grpc_server
            
            self.grpc_server, self.pose_service = start_grpc_server(port=self.config.grpc_port)
            self.hz_grpc = 0.0
            self.pose_service.get_latest_pose(block=False)
            logger.info("gRPC server started for phone communication")
        except ImportError as e:
            # Try to add daxie to sys.path and retry import
            import sys
            from pathlib import Path
            
            # Try to find daxie directory relative to this file
            current_file = Path(__file__)
            # Go up from lerobot/common/teleoperators/phone_teleoperator/ to find daxie
            possible_daxie_paths = [
                current_file.parent.parent.parent.parent.parent,  # lerobot-vulcan/../daxie
                current_file.parent.parent.parent.parent.parent / "daxie",  # lerobot-vulcan/../daxie/daxie
                Path.cwd().parent,  # Current working directory parent
                Path.cwd().parent / "daxie",  # Current working directory parent/daxie
            ]
            
            daxie_found = False
            for daxie_path in possible_daxie_paths:
                if (daxie_path / "daxie" / "__init__.py").exists():
                    daxie_str = str(daxie_path)
                    if daxie_str not in sys.path:
                        sys.path.insert(0, daxie_str)
                        logger.info(f"Added {daxie_str} to Python path for daxie import")
                    
                    try:
                        from daxie.src.server.pos_grpc_server import start_grpc_server
                        self.grpc_server, self.pose_service = start_grpc_server(port=self.config.grpc_port)
                        self.hz_grpc = 0.0
                        self.pose_service.get_latest_pose(block=False)
                        logger.info("gRPC server started for phone communication (after path fix)")
                        daxie_found = True
                        break
                    except ImportError:
                        continue
            
            if not daxie_found:
                logger.error(f"Could not import gRPC server from daxie package: {e}")
                logger.error("Make sure the daxie package is installed or accessible")
                raise ImportError(f"Failed to import daxie.src.server.pos_grpc_server: {e}")

    def _open_phone_connection(self, init_qpos: np.ndarray) -> tuple[np.ndarray, np.ndarray, bool]:
        """Wait for phone to connect and set initial mapping."""
        try:
            init_rot_robot = R.from_quat(self.current_q_R, scalar_first=True)
        except TypeError:
            init_rot_robot = R.from_quat(self.current_q_R)
            
        self.current_t_R = np.array(self.config.initial_position)
        self.current_q_R = np.array(self.config.initial_wxyz)

        # CRITICAL FIX: Reset teleop state to ensure we wait for phone connection
        self.start_teleop = False

        logger.info("📱 Phone connected! Please press the START button on your phone to begin teleoperation...")
        logger.info(f"gRPC server listening on port {self.config.grpc_port}")
        
        data = None
        attempt_count = 0
        last_log_time = time.time()
        
        while not self.start_teleop:
            attempt_count += 1
            try:
                data = self.pose_service.get_latest_pose(block=True, timeout=self.config.grpc_timeout)
                if data is not None and "switch" in data:
                    self.start_teleop = data["switch"]
                    
                    # Only log every 3 seconds to avoid spam
                    current_time = time.time()
                    if current_time - last_log_time > 3.0:
                        logger.info(f"📱 Waiting for START button press... (received {attempt_count} pose updates)")
                        last_log_time = current_time
                        
            except Exception as e:
                logger.debug(f"Exception during connection: {type(e).__name__}: {e}")
                continue

        # Ensure we have valid data before proceeding
        if data is None:
            logger.error("DEBUG: data is None after exiting connection loop")
            raise RuntimeError("Failed to get valid data from phone connection")
        
        logger.debug(f"DEBUG: Connection successful, processing data: {data}")
        pos, quat, gripper_value = data["position"], data["rotation"], data.get("gripper_value", 50.0)
        # Convert gripper_value (0-100) to boolean (True if > 50, False otherwise)
        gripper = gripper_value > 50.0
        logger.debug(f"DEBUG: Extracted initial pose - pos={pos}, quat={quat}, gripper={gripper}")
        try:
            initial_rot_phone = R.from_quat(quat, scalar_first=True)
        except TypeError:
            initial_rot_phone = R.from_quat(quat)
        initial_pos_phone = np.array(pos)

        self.initial_phone_quat = quat.copy()
        self.initial_phone_pos = initial_pos_phone.copy()

        quat_RP = init_rot_robot * initial_rot_phone.inv()
        translation_RP = self.current_t_R - quat_RP.apply(initial_pos_phone)
        
        logger.info("Phone connected and teleop started!")
        return quat_RP, translation_RP, gripper

    def _reset_mapping(self, phone_pos: np.ndarray, phone_quat: np.ndarray) -> None:
        """Reset mapping parameters when precision mode toggles."""
        self.initial_phone_pos = phone_pos.copy()
        self.initial_phone_quat = phone_quat.copy()

        try:
            rot_init = R.from_quat(self.initial_phone_quat, scalar_first=True)
            rot_curr = R.from_quat(self.current_q_R, scalar_first=True)
        except TypeError:
            rot_init = R.from_quat(self.initial_phone_quat)
            rot_curr = R.from_quat(self.current_q_R)
            
        self.quat_RP = rot_curr * rot_init.inv()
        self.translation_RP = self.current_t_R - self.quat_RP.apply(self.initial_phone_pos)

    def _map_phone_to_robot(
        self, phone_pos: np.ndarray, phone_quat: np.ndarray, precision_mode: bool
    ) -> tuple[np.ndarray, np.ndarray]:
        """Map phone translation and rotation to robot's coordinate frame."""
        phone_pos = np.array(phone_pos, float)
        phone_quat = np.array(phone_quat, float)

        if precision_mode != self.last_precision_mode:
            self._reset_mapping(phone_pos, phone_quat)

        self.last_precision_mode = precision_mode
        scale = (
            self.config.sensitivity_precision if precision_mode 
            else self.config.sensitivity_normal
        )

        # Translate
        delta = (phone_pos - self.initial_phone_pos) * scale
        scaled_pos = self.initial_phone_pos + delta

        # Rotate
        try:
            init_rot = R.from_quat(self.initial_phone_quat, scalar_first=True)
            curr_rot = R.from_quat(phone_quat, scalar_first=True)
        except TypeError:
            init_rot = R.from_quat(self.initial_phone_quat)
            curr_rot = R.from_quat(phone_quat)
            
        relative_rot = init_rot.inv() * curr_rot
        rotvec = relative_rot.as_rotvec() * self.config.rotation_sensitivity
        scaled_rot = R.from_rotvec(rotvec)
        quat_scaled = init_rot * scaled_rot

        # Apply mapping
        quat_robot = self.quat_RP * quat_scaled
        pos_robot = self.quat_RP.apply(scaled_pos) + self.translation_RP

        try:
            self.current_q_R = quat_robot.as_quat(scalar_first=True)
        except TypeError:
            self.current_q_R = quat_robot.as_quat()
        self.current_t_R = pos_robot
        return pos_robot, self.current_q_R

    def get_action(self, observation: dict[str, Any] | None = None) -> dict[str, Any]:
        """
        Get the current action from phone input.
        
        Args:
            observation: Current robot observation containing joint positions
        
        This method processes phone pose data, solves inverse kinematics,
        and returns the target joint positions using the proven VirtualManipulator approach.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # Extract current robot position from observation
        current_joint_pos_deg = None
        if observation is not None:
            try:
                # Extract joint positions from observation (assumes motor names follow pattern)
                motor_keys = ["shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos", 
                             "wrist_flex.pos", "wrist_roll.pos", "gripper.pos"]
                current_joint_pos_deg = [observation.get(key, 0.0) for key in motor_keys]
            except Exception as e:
                logger.warning(f"Could not extract joint positions from observation: {e}")
        
        # If no observation or extraction failed, use rest pose
        if current_joint_pos_deg is None:
            current_joint_pos_deg = np.rad2deg(self.config.rest_pose)

        # Show initial motor positions immediately on first call (before phone connection)
        if not self.initial_positions_shown:
            self._display_motor_positions_formatted(current_joint_pos_deg, "INITIAL ROBOT POSITION")
            self.initial_positions_shown = True

        # Initialize variables to ensure they're always defined
        data = None
        pos = quat = gripper = None
        
        try:
            # Handle phone connection
            if not self._phone_connected:
                curr_qpos_rad = np.deg2rad(current_joint_pos_deg)
                self.quat_RP, self.translation_RP, _ = self._open_phone_connection(curr_qpos_rad)
                self._phone_connected = True

            # Check if teleop is active
            if not self.start_teleop:
                self._phone_connected = False
                # Return current position when not teleoperating (keeps robot in place)
                return self._format_action_dict(current_joint_pos_deg)
            
            # Start timer when teleop becomes active
            if self.teleop_start_time is None:
                self.teleop_start_time = time.time()
                logger.info("✅ Teleoperation started!")
            
            # Check if 5 seconds have passed and we haven't read positions yet
            if not self.motor_positions_read and time.time() - self.teleop_start_time >= 5.0:
                self._read_and_display_motor_positions(current_joint_pos_deg)
                self.motor_positions_read = True

            # Get latest pose from gRPC
            try:
                data = self.pose_service.get_latest_pose(block=False)
            except Exception as e:
                # Phone connection lost - reset state
                logger.warning(f"Phone connection lost: {e}. Reconnecting...")
                self._phone_connected = False
                self.start_teleop = False
                self.teleop_start_time = None
                self.motor_positions_read = False
                return self._format_action_dict(current_joint_pos_deg)

            # Check if we have valid data before proceeding
            if data is None:
                # Phone disconnected - reset connection state
                logger.warning("Phone disconnected. Reconnecting...")
                self._phone_connected = False
                self.start_teleop = False
                self.teleop_start_time = None
                self.motor_positions_read = False
                return self._format_action_dict(current_joint_pos_deg)

            logger.debug(f"DEBUG: Processing valid data with keys: {list(data.keys()) if isinstance(data, dict) else 'NOT A DICT'}")

            # Now we know data is valid, proceed with processing
            # Update the previous state BEFORE checking for reset
            current_is_resetting = data.get("is_resetting", False)
            logger.debug(f"DEBUG: current_is_resetting={current_is_resetting}, prev_is_resetting={self.prev_is_resetting}")
            
            if current_is_resetting:
                # Update prev_is_resetting before returning
                self.prev_is_resetting = current_is_resetting
                logger.debug("DEBUG: In reset mode, returning current position")
                return self._format_action_dict(current_joint_pos_deg)
            
            # Check for reset transition (prev=True, current=False)
            if self.prev_is_resetting == True and current_is_resetting == False:
                # Call reset function with current phone pose
                pos, quat = data.get("position"), data.get("rotation")
                if pos is not None and quat is not None:
                    logger.debug("DEBUG: Reset transition detected, calling _reset_mapping")
                    self._reset_mapping(pos, quat)
            
            # Update the previous state
            self.prev_is_resetting = current_is_resetting

            pos, quat, gripper_value = data.get("position"), data.get("rotation"), data.get("gripper_value", 50.0)
            # Convert gripper_value (0-100) to boolean (True if > 50, False otherwise) 
            gripper = gripper_value > 50.0
            logger.debug(f"DEBUG: Extracted pose data - pos={pos}, quat={quat}, gripper_value={gripper_value}, gripper={gripper}")
            
            # Validate required data fields
            if pos is None or quat is None:
                logger.warning(f"DEBUG: Missing required data - pos={pos}, quat={quat}")
                return self._format_action_dict(current_joint_pos_deg)

            # Map phone pose to robot pose
            precision_mode = data.get("precision", False)
            logger.debug(f"DEBUG: Mapping phone to robot with precision_mode={precision_mode}")
            t_robot, q_robot = self._map_phone_to_robot(pos, quat, precision_mode)
            logger.debug(f"DEBUG: Mapped to robot pose - t_robot={t_robot}, q_robot={q_robot}")

            # Solve inverse kinematics
            logger.debug("DEBUG: Solving inverse kinematics")
            solution = self._solve_ik(t_robot, q_robot)
            logger.debug(f"DEBUG: IK solution (radians): {solution}")

            # Update visualization (expects radians)
            if self.config.enable_visualization and self.urdf_vis:
                self.urdf_vis.update_cfg(solution)
                logger.debug("DEBUG: Updated visualization")

            # Convert to degrees for robot (SO100 expects degrees)
            solution_deg = np.rad2deg(solution)
            logger.debug(f"DEBUG: Solution in degrees: {solution_deg}")

            # Apply backward compatibility transformations for old calibration system
            # Based on PR #777 backward compatibility documentation
            
            # For SO100/SO101 backward compatibility:
            # shoulder_lift (index 1): direction reversal + 90° offset
            if len(solution_deg) > 1:
                old_val = solution_deg[1]
                solution_deg[1] = -(solution_deg[1] - 90)
                logger.debug(f"DEBUG: shoulder_lift transform: {old_val} -> {solution_deg[1]}")
            
            # elbow_flex (index 2): 90° offset
            if len(solution_deg) > 2:
                old_val = solution_deg[2]
                solution_deg[2] -= 90
                logger.debug(f"DEBUG: elbow_flex transform: {old_val} -> {solution_deg[2]}")
            
            # wrist_roll (index 4): direction reversal + 90° offset
            if len(solution_deg) > 4:
                old_val = solution_deg[4]
                solution_deg[4] = -(solution_deg[4] + 90)
                logger.debug(f"DEBUG: wrist_roll transform: {old_val} -> {solution_deg[4]}")

            # Update gripper state - use the gripper_open boolean from phone
            gripper_position = self.config.gripper_max_pos if gripper else self.config.gripper_min_pos
            solution_deg[-1] = gripper_position
            logger.debug(f"DEBUG: Set gripper position: {gripper_position} (gripper_open={gripper})")
            
            # Update teleop state - ensure data is valid before accessing
            if "switch" in data:
                old_teleop = self.start_teleop
                self.start_teleop = data["switch"]
                logger.debug(f"DEBUG: Updated teleop state: {old_teleop} -> {self.start_teleop}")

            logger.debug(f"DEBUG: Final solution: {solution_deg}")
            return self._format_action_dict(solution_deg)

        except Exception as e:
            logger.error(f"DEBUG: Exception in get_action: {type(e).__name__}: {e}")
            import traceback
            logger.error(f"DEBUG: Traceback: {traceback.format_exc()}")
            # Return current position on error (safer than rest pose)
            return self._format_action_dict(current_joint_pos_deg)

    def _solve_ik(self, target_position: np.ndarray, target_wxyz: np.ndarray) -> list[float]:
        """Solve inverse kinematics for target pose. Returns solution in radians."""
        try:
            # Import IK solver from daxie package
            from daxie.src.teleop.solve_ik import solve_ik
            
            solution = solve_ik(
                robot=self.robot,
                target_link_name=self.config.target_link_name,
                target_position=target_position,
                target_wxyz=target_wxyz,
            )
            
            return solution  # Always return radians
        except ImportError as e:
            # Try to add daxie to sys.path and retry import
            import sys
            from pathlib import Path
            
            # Try to find daxie directory relative to this file
            current_file = Path(__file__)
            # Go up from lerobot/common/teleoperators/phone_teleoperator/ to find daxie
            possible_daxie_paths = [
                current_file.parent.parent.parent.parent.parent,  # lerobot-vulcan/../daxie
                current_file.parent.parent.parent.parent.parent / "daxie",  # lerobot-vulcan/../daxie/daxie
                Path.cwd().parent,  # Current working directory parent
                Path.cwd().parent / "daxie",  # Current working directory parent/daxie
            ]
            
            for daxie_path in possible_daxie_paths:
                if (daxie_path / "daxie" / "__init__.py").exists():
                    daxie_str = str(daxie_path)
                    if daxie_str not in sys.path:
                        sys.path.insert(0, daxie_str)
                        logger.info(f"Added {daxie_str} to Python path for IK solver import")
                    
                    try:
                        from daxie.src.teleop.solve_ik import solve_ik
                        solution = solve_ik(
                            robot=self.robot,
                            target_link_name=self.config.target_link_name,
                            target_position=target_position,
                            target_wxyz=target_wxyz,
                        )
                        
                        return solution  # Always return radians
                    except ImportError:
                        continue
            
            logger.error(f"Could not import IK solver from daxie package: {e}")
            # Return rest pose in radians
            return list(self.config.rest_pose)

    def _format_action_dict(self, joint_positions: list[float]) -> dict[str, Any]:
        """Format joint positions into action dictionary."""
        action_keys = list(self.action_features.keys())
        if len(joint_positions) != len(action_keys):
            logger.warning(
                f"Joint positions length ({len(joint_positions)}) doesn't match "
                f"action features length ({len(action_keys)})"
            )
            # Pad or truncate as needed
            joint_positions = joint_positions[:len(action_keys)]
            while len(joint_positions) < len(action_keys):
                joint_positions.append(0.0)
        
        return {key: pos for key, pos in zip(action_keys, joint_positions)}

    def _read_and_display_motor_positions(self, current_joint_pos_deg: list[float]) -> None:
        """Read and display current motor positions in rest_pose format (radians)."""
        self._display_motor_positions_formatted(current_joint_pos_deg, "5-SECOND TELEOP READING")
        
        # Also log to logger
        current_joint_pos_rad = np.deg2rad(current_joint_pos_deg)
        logger.info(f"Motor positions after 5 seconds - Degrees: {current_joint_pos_deg}")
        logger.info(f"Motor positions after 5 seconds - Radians: {current_joint_pos_rad}")
        logger.info(f"rest_pose format: {tuple(current_joint_pos_rad)}")

    def _display_motor_positions_formatted(self, current_joint_pos_deg: list[float], context: str) -> None:
        """Display motor positions in rest_pose format with given context."""
        # Convert degrees to radians for rest_pose format
        current_joint_pos_rad = np.deg2rad(current_joint_pos_deg)
        
        # Format as tuple like rest_pose in config
        position_tuple = tuple(current_joint_pos_rad)
        
        formatted_values = ", ".join([f"{pos:.6f}" for pos in current_joint_pos_rad])
        logger.info(f"{context}: {formatted_values}")

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Send feedback to phone teleoperator."""
        pass

    def disconnect(self) -> None:
        """Disconnect from phone and cleanup resources."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        try:
            if self.grpc_server:
                self.grpc_server.stop(0)
            
            if self.server:
                self.server.stop()
                
            self._is_connected = False
            self._phone_connected = False
            logger.info(f"{self} disconnected")
            
        except Exception as e:
            logger.error(f"Error disconnecting {self}: {e}")

 