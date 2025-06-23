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
from scipy.spatial.transform import Rotation as R

try:
    import pyroki as pk
    import viser
    import yourdfpy
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
    Phone-based teleoperator that receives pose data from mobile phone via gRPC
    and converts it to robot control commands using inverse kinematics.
    
    This teleoperator integrates with the VirtualManipulator system from the daxie package
    to provide phone-based robot control.
    """

    config_class = PhoneTeleoperatorConfig
    name = "phone_teleoperator"

    def __init__(self, config: PhoneTeleoperatorConfig, **kwargs):
        super().__init__(config, **kwargs)
        self.config = config
        
        # Initialize robot model for IK
        self.urdf = None
        self.robot = None
        self.urdf_vis = None
        self.server = None
        
        # Phone connection state
        self._is_connected = False
        self._phone_connected = False
        self.start_teleop = False
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
        # Phone teleoperator doesn't typically need feedback
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
        """Configure the phone teleoperator (no-op for phone teleoperator)."""
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
        except ImportError:
            logger.error("Could not import gRPC server from daxie package")
            raise

    def _open_phone_connection(self, curr_qpos_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray, bool]:
        """Wait for phone to connect and set initial mapping."""
        # Use the initial target pose, not the current robot joint positions
        # The current joint positions are used elsewhere, but the target pose is what we map to
        init_rot_robot = R.from_quat(self.current_q_R, scalar_first=True)
        self.current_t_R = np.array(self.config.initial_position)
        self.current_q_R = np.array(self.config.initial_wxyz)

        logger.info("Waiting for phone to send start signal...")
        logger.info(f"gRPC server listening on port {self.config.grpc_port}")
        logger.info("Please activate the start signal on your phone app")
        
        attempts = 0
        while not self.start_teleop:
            attempts += 1
            try:
                data = self.pose_service.get_latest_pose(block=True, timeout=self.config.grpc_timeout)
                if data is None:
                    logger.warning(f"gRPC timeout after {self.config.grpc_timeout}s (attempt {attempts})")
                    continue
                    
                self.start_teleop = data["switch"]
                
                if attempts % 10 == 1:  # Log every 10th attempt to avoid spam
                    logger.info(f"Received pose data (attempt {attempts}): switch={data['switch']}, "
                              f"position={data['position']}, gripper={data['gripper_open']}")
                              
                if not self.start_teleop:
                    logger.debug(f"Start signal is False, waiting... (attempt {attempts})")
                else:
                    logger.info(f"✓ Start signal received after {attempts} attempts!")
                    
            except Exception as e:
                logger.error(f"Error waiting for phone connection: {e}")
                raise

        pos, quat, gripper = data["position"], data["rotation"], data["gripper_open"]
        initial_rot_phone = R.from_quat(quat, scalar_first=True)
        initial_pos_phone = np.array(pos)

        self.initial_phone_quat = quat.copy()
        self.initial_phone_pos = initial_pos_phone.copy()

        quat_RP = init_rot_robot * initial_rot_phone.inv()
        translation_RP = self.current_t_R - quat_RP.apply(initial_pos_phone)
        
        logger.info("Phone connection established successfully!")
        return quat_RP, translation_RP, gripper

    def _reset_mapping(self, phone_pos: np.ndarray, phone_quat: np.ndarray) -> None:
        """Reset mapping parameters when precision mode toggles."""
        self.initial_phone_pos = phone_pos.copy()
        self.initial_phone_quat = phone_quat.copy()

        rot_init = R.from_quat(self.initial_phone_quat, scalar_first=True)
        rot_curr = R.from_quat(self.current_q_R, scalar_first=True)
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
        init_rot = R.from_quat(self.initial_phone_quat, scalar_first=True)
        curr_rot = R.from_quat(phone_quat, scalar_first=True)
        relative_rot = init_rot.inv() * curr_rot
        rotvec = relative_rot.as_rotvec() * self.config.rotation_sensitivity
        scaled_rot = R.from_rotvec(rotvec)
        quat_scaled = init_rot * scaled_rot

        # Apply mapping
        quat_robot = self.quat_RP * quat_scaled
        pos_robot = self.quat_RP.apply(scaled_pos) + self.translation_RP

        self.current_q_R = quat_robot.as_quat(scalar_first=True)
        self.current_t_R = pos_robot
        return pos_robot, self.current_q_R

    def get_action(self, observation: dict[str, Any] | None = None) -> dict[str, Any]:
        """
        Always return the initial position (for debugging comparison with old system).
        """
        # Debug: Track the complete flow
        if not hasattr(self, '_debug_timer'):
            self._debug_timer = 0
            self._debug_printed = False
        
        self._debug_timer += 1
        
        # Print debug info after ~5 seconds (assuming ~10Hz loop)
        if self._debug_timer == 50 and not self._debug_printed:
            print(f"\n=== NEW SYSTEM COMPLETE FLOW DEBUG ===")
            print(f"Step 1: get_action() called")
            print(f"  observation keys: {list(observation.keys()) if observation else 'None'}")
            print(f"  initial_position: {self.config.initial_position}")
            print(f"  initial_wxyz: {self.config.initial_wxyz}")
            self._debug_printed = True

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
            logger.debug("Using rest pose as current position")

        if self._debug_timer == 50 and self._debug_printed:
            print(f"\nStep 2: Current robot position")
            print(f"  current_joint_pos_deg: {current_joint_pos_deg}")

        try:
            # Auto-start without waiting for phone connection
            if not self._phone_connected:
                # Set up initial mapping using current position
                curr_qpos_rad = np.deg2rad(current_joint_pos_deg)
                self.quat_RP = np.array([1, 0, 0, 0])  # Identity quaternion
                self.translation_RP = np.array([0, 0, 0])  # No translation
                self._phone_connected = True
                self.start_teleop = True  # Auto-start teleop

            # ALWAYS use initial position for debugging (like old system)
            target_position = np.array(self.config.initial_position)
            target_wxyz = np.array(self.config.initial_wxyz)

            if self._debug_timer == 50 and self._debug_printed:
                print(f"\nStep 3: Target coordinates (always initial)")
                print(f"  target_position: {target_position}")
                print(f"  target_wxyz: {target_wxyz}")

            # Solve inverse kinematics (returns radians)
            solution_rad = self._solve_ik(target_position, target_wxyz)

            if self._debug_timer == 50 and self._debug_printed:
                print(f"\nStep 4: IK solver output")
                print(f"  solution_rad: {solution_rad}")
                print(f"  solution_deg: {np.rad2deg(solution_rad)}")

            # Update visualization (expects radians)
            if self.config.enable_visualization and self.urdf_vis:
                self.urdf_vis.update_cfg(solution_rad)

            # Convert to degrees for robot (SO100 expects degrees)
            solution_deg = np.rad2deg(solution_rad)

            # Update gripper state (keep gripper closed for debugging)
            solution_deg[-1] = 0.0  # Closed gripper

            if self._debug_timer == 50 and self._debug_printed:
                print(f"\nStep 5: Final output")
                print(f"  solution_deg (final): {solution_deg}")
                print(f"  action_dict: {self._format_action_dict(solution_deg)}")
                print(f"========================================\n")

            return self._format_action_dict(solution_deg)

        except Exception as e:
            logger.error(f"Error getting action from {self}: {e}")
            # Return current position on error (safer than rest pose)
            return self._format_action_dict(current_joint_pos_deg)

    def _solve_ik(self, target_position: np.ndarray, target_wxyz: np.ndarray) -> list[float]:
        """Solve inverse kinematics for target pose. Returns solution in radians."""
        try:
            # Import IK solver from daxie package
            from daxie.src.teleop.solve_ik import solve_ik
            
            # Debug IK calculation
            if hasattr(self, '_debug_timer') and self._debug_timer == 50 and hasattr(self, '_debug_printed') and self._debug_printed:
                print(f"\nStep 4a: IK solver details")
                print(f"  robot type: {type(self.robot)}")
                print(f"  target_link_name: {self.config.target_link_name}")
                print(f"  target_position: {target_position}")
                print(f"  target_wxyz: {target_wxyz}")
            
            solution = solve_ik(
                robot=self.robot,
                target_link_name=self.config.target_link_name,
                target_position=target_position,
                target_wxyz=target_wxyz,
            )
            
            return solution  # Always return radians
        except ImportError:
            logger.error("Could not import IK solver from daxie package")
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

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Send feedback to phone teleoperator (no-op for phone teleoperator)."""
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