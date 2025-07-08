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
import scipy

# Check scipy version for scalar_first compatibility
_scipy_version = tuple(map(int, scipy.__version__.split('.')[:2]))
_supports_scalar_first = _scipy_version >= (1, 7)


def _rotation_from_quat(quat: np.ndarray, scalar_first: bool = True) -> R:
    """
    Create a Rotation object from quaternion with backward compatibility.
    
    Args:
        quat: Quaternion array
        scalar_first: Whether the first element is the scalar component (w,x,y,z vs x,y,z,w)
    
    Returns:
        Rotation object
    """
    if _supports_scalar_first:
        return R.from_quat(quat, scalar_first=scalar_first)
    else:
        # For older scipy versions, convert quaternion format if needed
        if scalar_first:
            # Convert from (w,x,y,z) to (x,y,z,w) for older scipy
            quat_converted = np.array([quat[1], quat[2], quat[3], quat[0]])
        else:
            quat_converted = quat
        return R.from_quat(quat_converted)


def _quat_as_scalar_first(rotation: R) -> np.ndarray:
    """
    Get quaternion in scalar-first format (w,x,y,z) with backward compatibility.
    
    Args:
        rotation: Rotation object
        
    Returns:
        Quaternion as (w,x,y,z)
    """
    if _supports_scalar_first:
        return rotation.as_quat(scalar_first=True)
    else:
        # For older scipy versions, convert from (x,y,z,w) to (w,x,y,z)
        quat = rotation.as_quat()  # Returns (x,y,z,w)
        return np.array([quat[3], quat[0], quat[1], quat[2]])  # Convert to (w,x,y,z)


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

from .config_sourccey_v2beta_phone_teleop import SourcceyV2BetaPhoneTeleopConfig

logger = logging.getLogger(__name__)


class SourcceyV2BetaPhoneTeleop(Teleoperator):
    """
    Phone-based teleoperator for Sourccey V2 Beta that receives pose data from mobile phone via gRPC
    and converts it to robot control commands using inverse kinematics.
    
    This teleoperator can control either the left or right arm of the Sourccey robot.
    """

    config_class = SourcceyV2BetaPhoneTeleopConfig
    name = "sourccey_v2beta_phone_teleop"

    def __init__(self, config: SourcceyV2BetaPhoneTeleopConfig, **kwargs):
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
        
        # Reset position holding - store the position when reset starts
        self.reset_hold_position = None
        
        # Pose tracking for left arm
        self.left_arm_current_t_R = np.array(self.config.left_arm_initial_position)
        self.left_arm_current_q_R = np.array(self.config.left_arm_initial_wxyz)
        
        # Pose tracking for right arm
        self.right_arm_current_t_R = np.array(self.config.right_arm_initial_position)
        self.right_arm_current_q_R = np.array(self.config.right_arm_initial_wxyz)
        
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

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Features for the actions produced by this teleoperator."""
        # Sourccey has 12 motors total (6 per arm, including gripper)
        motor_names = [
            # Left arm (6 motors, including gripper)
            "left_arm_shoulder_pan.pos",
            "left_arm_shoulder_lift.pos", 
            "left_arm_elbow_flex.pos",
            "left_arm_wrist_flex.pos",
            "left_arm_wrist_roll.pos",
            "left_arm_gripper.pos",
            # Right arm (6 motors, including gripper)
            "right_arm_shoulder_pan.pos",
            "right_arm_shoulder_lift.pos", 
            "right_arm_elbow_flex.pos",
            "right_arm_wrist_flex.pos",
            "right_arm_wrist_roll.pos",
            "right_arm_gripper.pos",
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
            urdf_path = self.config.urdf_path
            mesh_path = self.config.mesh_path
            
            # Resolve relative paths to absolute paths
            if urdf_path and mesh_path:
                from pathlib import Path
                import lerobot
                
                # Get the lerobot package root directory
                lerobot_root = Path(lerobot.__file__).parent.parent
                
                # Resolve relative paths
                if not Path(urdf_path).is_absolute():
                    urdf_path = str(lerobot_root / urdf_path)
                if not Path(mesh_path).is_absolute():
                    mesh_path = str(lerobot_root / mesh_path)
                
                logger.info(f"Using Sourccey paths - URDF: {urdf_path}, Mesh: {mesh_path}")
            else:
                # Fallback auto-detection if paths are empty
                try:
                    from pathlib import Path
                    import lerobot
                    
                    # Get the lerobot package root directory
                    lerobot_root = Path(lerobot.__file__).parent.parent
                    sourccey_model_path = lerobot_root / "lerobot" / "common" / "robots" / "sourccey_v2beta" / "model"
                    
                    if sourccey_model_path.exists():
                        auto_urdf_path = str(sourccey_model_path / "sourccey_v2beta.urdf")
                        auto_mesh_path = str(sourccey_model_path / "meshes")
                        urdf_path = urdf_path or auto_urdf_path
                        mesh_path = mesh_path or auto_mesh_path
                        logger.info(f"Auto-detected Sourccey paths - URDF: {urdf_path}, Mesh: {mesh_path}")
                    else:
                        raise FileNotFoundError(f"Could not find Sourccey model directory at {sourccey_model_path}")
                except Exception as e:
                    logger.warning(f"Could not auto-detect Sourccey paths: {e}")
                    if not urdf_path or not mesh_path:
                        raise ValueError("URDF path and mesh path must be provided in config or Sourccey model must be available in lerobot/common/robots/sourccey_v2beta/model/")
                
            self.urdf = yourdfpy.URDF.load(urdf_path, mesh_dir=mesh_path)
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
        """Start the gRPC server for phone communication."""
        import grpc
        from concurrent.futures import ThreadPoolExecutor
        
        from lerobot.common.transport.phone_teleop_grpc.pose_telemetry_pb2_grpc import PoseTelemetryServicer, add_PoseTelemetryServicer_to_server
        from lerobot.common.transport.phone_teleop_grpc.pos_grpc_server import PoseTelemetryService
        
        self.pose_service = PoseTelemetryService()
        self.grpc_server = grpc.server(ThreadPoolExecutor(max_workers=10))
        add_PoseTelemetryServicer_to_server(self.pose_service, self.grpc_server)
        
        # Start server
        self.grpc_server.add_insecure_port(f"[::]:{self.config.grpc_port}")
        self.grpc_server.start()
        logger.info(f"gRPC server started on port {self.config.grpc_port}")

    def _open_phone_connection(self, curr_qpos_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Open connection with phone and establish initial mapping."""
        logger.info("Waiting for phone connection...")
        
        # Get phone data once to set up mapping - don't wait for start signal
        data = self.pose_service.get_latest_pose(block=True, timeout=self.config.grpc_timeout)
        if data is not None:
            self.start_teleop = data["switch"]
        else:
            # Use default data if no phone connected yet
            data = {
                "position": [0.0, 0.0, 0.0],
                "rotation": [1.0, 0.0, 0.0, 0.0],  # w,x,y,z
                "gripper_value": 0.0,
                "switch": False
            }
            self.start_teleop = False

        pos, quat, gripper_value = data["position"], data["rotation"], data["gripper_value"]
        
        # Set initial mapping
        self._reset_mapping(pos, quat)
        
        # Set initial robot pose based on current joint positions
        if self.config.control_left_arm:
            target_link = self.config.left_arm_target_link_name
            rest_pose = self.config.left_arm_rest_pose
        else:
            target_link = self.config.right_arm_target_link_name
            rest_pose = self.config.right_arm_rest_pose
        
        # Get current end-effector pose
        # PyRoKi expects the joint configuration length to match the number of actuated joints
        # present in the URDF for the *specific* kinematic chain we are querying. The Sourccey arm
        # URDF contains 7 actuated joints for **one** arm (5 revolute + 2 prismatic gripper sliders).
        # If we control only one arm we must therefore pass exactly those 7 joints and **not** the
        # concatenated left+right configuration (14 values). Slice the correct segment depending on
        # which arm is being controlled.

        if self.config.control_left_arm:
            arm_joints = np.array(curr_qpos_rad[:7])  # left arm joints
        else:
            arm_joints = np.array(curr_qpos_rad[7:])  # right arm joints

        # PyRoKi expects the link index, not the name
        if isinstance(target_link, str):
            link_index = list(self.robot.links.names).index(target_link)
        else:
            link_index = target_link

        curr_pose = self.robot.forward_kinematics(arm_joints, link_index)
        curr_pos = curr_pose[:3, 3]
        curr_rot = curr_pose[:3, :3]
        curr_quat = _quat_as_scalar_first(R.from_matrix(curr_rot))
        
        # Update current robot pose
        if self.config.control_left_arm:
            self.left_arm_current_t_R = curr_pos
            self.left_arm_current_q_R = curr_quat
        else:
            self.right_arm_current_t_R = curr_pos
            self.right_arm_current_q_R = curr_quat
        
        logger.info("Phone connection established successfully!")
        return pos, quat

    def _reset_mapping(self, phone_pos: np.ndarray, phone_quat: np.ndarray) -> None:
        """Reset the mapping between phone and robot poses."""
        import numpy as np
        phone_pos = np.array(phone_pos)
        phone_quat = np.array(phone_quat)
        self.initial_phone_pos = phone_pos.copy()
        self.initial_phone_quat = phone_quat.copy()
        
        # Calculate initial transformation
        phone_rot = _rotation_from_quat(phone_quat)
        robot_rot = _rotation_from_quat(self.left_arm_current_q_R if self.config.control_left_arm else self.right_arm_current_q_R)
        
        # Calculate relative rotation and translation
        self.quat_RP = (robot_rot * phone_rot.inv()).as_quat()
        self.translation_RP = (self.left_arm_current_t_R if self.config.control_left_arm else self.right_arm_current_t_R) - phone_pos

    def _map_phone_to_robot(
        self, phone_pos: np.ndarray, phone_quat: np.ndarray, precision_mode: bool
    ) -> tuple[np.ndarray, np.ndarray]:
        """Map phone pose to robot pose."""
        import numpy as np
        phone_pos = np.array(phone_pos)
        phone_quat = np.array(phone_quat)
        self.initial_phone_pos = np.array(self.initial_phone_pos)
        self.initial_phone_quat = np.array(self.initial_phone_quat)
        if self.quat_RP is None or self.translation_RP is None:
            raise RuntimeError("Mapping not initialized")
        
        # Apply sensitivity scaling
        sensitivity = self.config.sensitivity_precision if precision_mode else self.config.sensitivity_normal
        
        # Calculate relative phone movement
        phone_delta_pos = (phone_pos - self.initial_phone_pos) * sensitivity
        phone_rot = _rotation_from_quat(phone_quat)
        phone_delta_rot = (phone_rot * _rotation_from_quat(self.initial_phone_quat).inv())
        
        # Apply mapping transformation
        robot_pos = self.translation_RP + phone_delta_pos
        robot_rot = _rotation_from_quat(self.quat_RP) * phone_delta_rot
        robot_quat = _quat_as_scalar_first(robot_rot)
        
        return robot_pos, robot_quat

    def get_action(self, observation: dict[str, Any] | None = None) -> dict[str, Any]:
        """Get action from phone teleoperation."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        # Get current joint positions from observation
        if observation is not None:
            # For URDF IK, we need 7 joints per arm (5 arm + 2 gripper)
            # But we only control the 5 arm joints, gripper joints stay at current positions
            left_arm_joints = [
                observation.get("left_arm_shoulder_pan.pos", 0.0),
                observation.get("left_arm_shoulder_lift.pos", 0.0),
                observation.get("left_arm_elbow_flex.pos", 0.0),
                observation.get("left_arm_wrist_flex.pos", 0.0),
                observation.get("left_arm_wrist_roll.pos", 0.0),
                observation.get("left_arm_gripper.pos", 0.0),  # Gripper slider 1
                observation.get("left_arm_gripper.pos", 0.0),  # Gripper slider 2 (same value)
            ]
            right_arm_joints = [
                observation.get("right_arm_shoulder_pan.pos", 0.0),
                observation.get("right_arm_shoulder_lift.pos", 0.0),
                observation.get("right_arm_elbow_flex.pos", 0.0),
                observation.get("right_arm_wrist_flex.pos", 0.0),
                observation.get("right_arm_wrist_roll.pos", 0.0),
                observation.get("right_arm_gripper.pos", 0.0),  # Gripper slider 1
                observation.get("right_arm_gripper.pos", 0.0),  # Gripper slider 2 (same value)
            ]
            curr_qpos_rad = left_arm_joints + right_arm_joints
        else:
            # Use rest pose if no observation provided (7 joints per arm for URDF)
            curr_qpos_rad = list(self.config.left_arm_rest_pose) + list(self.config.right_arm_rest_pose)
        
        # Initialize teleop if not started
        if not self.start_teleop:
            phone_pos, phone_quat = self._open_phone_connection(np.array(curr_qpos_rad))
            self.start_teleop = True
            self.teleop_start_time = time.time()
            logger.info("Teleoperation started!")
        
        # Get latest pose from gRPC
        data = self.pose_service.get_latest_pose(block=False)
        if data is None:
            return self._format_action_dict(curr_qpos_rad)

        # Always convert phone data to numpy arrays
        data["position"] = np.array(data["position"])
        data["rotation"] = np.array(data["rotation"])
        if self.initial_phone_pos is not None and not isinstance(self.initial_phone_pos, np.ndarray):
            self.initial_phone_pos = np.array(self.initial_phone_pos)
        if self.initial_phone_quat is not None and not isinstance(self.initial_phone_quat, np.ndarray):
            self.initial_phone_quat = np.array(self.initial_phone_quat)

        # Check for reset gesture
        is_resetting = data.get("is_resetting", False)
        if is_resetting and not self.prev_is_resetting:
            logger.info("Reset gesture detected - recalibrating mapping")
            self._reset_mapping(data["position"], data["rotation"])
            if self.config.control_left_arm:
                self.left_arm_current_t_R = np.array(self.config.left_arm_initial_position)
                self.left_arm_current_q_R = np.array(self.config.left_arm_initial_wxyz)
            else:
                self.right_arm_current_t_R = np.array(self.config.right_arm_initial_position)
                self.right_arm_current_q_R = np.array(self.config.right_arm_initial_wxyz)
        
        self.prev_is_resetting = is_resetting
        
        # Get precision mode
        precision_mode = data.get("precision", False)
        if precision_mode != self.last_precision_mode:
            logger.info(f"Precision mode: {'ON' if precision_mode else 'OFF'}")
            self.last_precision_mode = precision_mode
        
        # Map phone pose to robot pose
        target_pos, target_quat = self._map_phone_to_robot(data["position"], data["rotation"], precision_mode)
        
        # Update current robot pose
        if self.config.control_left_arm:
            self.left_arm_current_t_R = target_pos
            self.left_arm_current_q_R = target_quat
        else:
            self.right_arm_current_t_R = target_pos
            self.right_arm_current_q_R = target_quat
        
        # Solve inverse kinematics
        if self.config.control_left_arm:
            target_link = self.config.left_arm_target_link_name
            # Use left arm joints for IK (7 joints including gripper)
            left_arm_joints = curr_qpos_rad[:7]
            joint_positions = self._solve_ik(target_pos, target_quat, target_link, left_arm_joints)
            # Combine with right arm (keep current positions)
            joint_positions = joint_positions + curr_qpos_rad[7:]
        else:
            target_link = self.config.right_arm_target_link_name
            # Use right arm joints for IK (7 joints including gripper)
            right_arm_joints = curr_qpos_rad[7:]
            joint_positions = self._solve_ik(target_pos, target_quat, target_link, right_arm_joints)
            # Combine with left arm (keep current positions)
            joint_positions = curr_qpos_rad[:7] + joint_positions
        
        # Display motor positions after 5 seconds
        if not self.motor_positions_read and time.time() - self.teleop_start_time > 5.0:
            self._read_and_display_motor_positions(joint_positions)
            self.motor_positions_read = True
        
        return self._format_action_dict(joint_positions)

    def _solve_ik(self, target_position: np.ndarray, target_wxyz: np.ndarray, target_link: str, current_joints: list[float]) -> list[float]:
        """Solve inverse kinematics for the target pose."""
        try:
            # Import the IK solver
            from lerobot.common.teleoperators.phone_teleoperator.solve_ik import solve_ik
            
            # Convert current joints to numpy array
            current_joints_array = np.array(current_joints)
            
            # Solve IK using the imported solver
            solution = solve_ik(
                self.robot,
                target_link,
                target_wxyz,
                target_position
            )
            
            if solution is None:
                logger.warning("IK solution not found, using current joint positions")
                return current_joints
            
            return solution.tolist()
            
        except Exception as e:
            logger.error(f"Error in IK solver: {e}")
            return current_joints

    def _format_action_dict(self, joint_positions: list[float]) -> dict[str, Any]:
        """Format joint positions as action dictionary."""
        motor_names = [
            "left_arm_shoulder_pan.pos", "left_arm_shoulder_lift.pos", "left_arm_elbow_flex.pos",
            "left_arm_wrist_flex.pos", "left_arm_wrist_roll.pos", "left_arm_gripper.pos",
            "right_arm_shoulder_pan.pos", "right_arm_shoulder_lift.pos", "right_arm_elbow_flex.pos",
            "right_arm_wrist_flex.pos", "right_arm_wrist_roll.pos", "right_arm_gripper.pos"
        ]
        
        return {name: pos for name, pos in zip(motor_names, joint_positions)}

    def _read_and_display_motor_positions(self, current_joint_pos: list[float]) -> None:
        """Read and display current motor positions."""
        logger.info("Current motor positions (radians):")
        motor_names = [
            "left_arm_shoulder_pan", "left_arm_shoulder_lift", "left_arm_elbow_flex",
            "left_arm_wrist_flex", "left_arm_wrist_roll", "left_arm_gripper",
            "right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
            "right_arm_wrist_flex", "right_arm_wrist_roll", "right_arm_gripper"
        ]
        
        for i, (name, pos) in enumerate(zip(motor_names, current_joint_pos)):
            logger.info(f"  {name}: {pos:.6f}")

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Send feedback to the phone (not implemented)."""
        pass

    def disconnect(self) -> None:
        """Disconnect the phone teleoperator."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        if self.grpc_server:
            self.grpc_server.stop(0)
        
        if self.server:
            self.server.close()
        
        self._is_connected = False
        logger.info(f"{self} disconnected") 