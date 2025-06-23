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
        print(f"🔗 OPENING PHONE CONNECTION with current position (rad): {curr_qpos_rad}")
        
        # Use the initial target pose, not the current robot joint positions
        # The current joint positions are used elsewhere, but the target pose is what we map to
        init_rot_robot = R.from_quat(self.current_q_R, scalar_first=True)
        self.current_t_R = np.array(self.config.initial_position)
        self.current_q_R = np.array(self.config.initial_wxyz)
        
        print(f"🎯 Initial robot target - Position: {self.current_t_R}, Rotation: {self.current_q_R}")

        logger.info("Waiting for phone to send start signal...")
        logger.info(f"gRPC server listening on port {self.config.grpc_port}")
        logger.info("Please activate the start signal on your phone app")
        
        attempts = 0
        while not self.start_teleop:
            attempts += 1
            try:
                print(f"📞 Waiting for phone data (attempt {attempts})...")
                data = self.pose_service.get_latest_pose(block=True, timeout=self.config.grpc_timeout)
                if data is None:
                    logger.warning(f"gRPC timeout after {self.config.grpc_timeout}s (attempt {attempts})")
                    print(f"⏰ TIMEOUT on attempt {attempts}")
                    continue
                    
                self.start_teleop = data["switch"]
                print(f"📱 Phone data received - Switch: {self.start_teleop}, Position: {data['position']}, Rotation: {data['rotation']}")
                
                if attempts % 10 == 1:  # Log every 10th attempt to avoid spam
                    logger.info(f"Received pose data (attempt {attempts}): switch={data['switch']}, "
                              f"position={data['position']}, gripper={data['gripper_open']}")
                              
                if not self.start_teleop:
                    print("🔄 Teleop not active yet, waiting...")
                    time.sleep(0.1)
                    continue
                else:
                    print("🎮 TELEOP ACTIVATED!")
                    break
                    
            except Exception as e:
                logger.error(f"Error waiting for phone connection: {e}")
                print(f"💥 ERROR waiting for phone: {e}")
                raise

        pos, quat, gripper = data["position"], data["rotation"], data["gripper_open"]
        print(f"📍 Initial phone pose - Position: {pos}, Rotation: {quat}, Gripper: {gripper}")
        
        initial_rot_phone = R.from_quat(quat, scalar_first=True)
        initial_pos_phone = np.array(pos)

        self.initial_phone_quat = quat.copy()
        self.initial_phone_pos = initial_pos_phone.copy()
        print(f"💾 Stored initial phone pose - Position: {self.initial_phone_pos}, Rotation: {self.initial_phone_quat}")

        quat_RP = init_rot_robot * initial_rot_phone.inv()
        translation_RP = self.current_t_R - quat_RP.apply(initial_pos_phone)
        print(f"🔧 Computed mapping - quat_RP: {quat_RP.as_quat()}, translation_RP: {translation_RP}")
        
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
        print(f"🗺️ MAPPING START - Phone pos: {phone_pos}, quat: {phone_quat}, precision: {precision_mode}")
        
        phone_pos = np.array(phone_pos, float)
        phone_quat = np.array(phone_quat, float)

        if precision_mode != self.last_precision_mode:
            print(f"🔄 PRECISION MODE CHANGED: {self.last_precision_mode} → {precision_mode} - Resetting mapping")
            self._reset_mapping(phone_pos, phone_quat)

        self.last_precision_mode = precision_mode
        scale = (
            self.config.sensitivity_precision if precision_mode 
            else self.config.sensitivity_normal
        )
        print(f"📏 Using sensitivity scale: {scale} ({'precision' if precision_mode else 'normal'} mode)")

        # Translate
        delta = (phone_pos - self.initial_phone_pos) * scale
        scaled_pos = self.initial_phone_pos + delta
        print(f"📍 Translation - Delta: {delta}, Scaled pos: {scaled_pos}")

        # Rotate
        init_rot = R.from_quat(self.initial_phone_quat, scalar_first=True)
        curr_rot = R.from_quat(phone_quat, scalar_first=True)
        relative_rot = init_rot.inv() * curr_rot
        rotvec = relative_rot.as_rotvec() * self.config.rotation_sensitivity
        scaled_rot = R.from_rotvec(rotvec)
        quat_scaled = init_rot * scaled_rot
        print(f"🔄 Rotation - Relative rotvec: {rotvec}, Scaled quat: {quat_scaled.as_quat()}")

        # Apply mapping
        quat_robot = self.quat_RP * quat_scaled
        pos_robot = self.quat_RP.apply(scaled_pos) + self.translation_RP
        print(f"🤖 Mapped robot pose - Position: {pos_robot}, Rotation: {quat_robot.as_quat()}")

        self.current_q_R = quat_robot.as_quat(scalar_first=True)
        self.current_t_R = pos_robot
        print(f"💾 Updated current robot state - Position: {self.current_t_R}, Rotation: {self.current_q_R}")
        
        return pos_robot, self.current_q_R

    def get_action(self, observation: dict[str, Any] | None = None) -> dict[str, Any]:
        """
        Get the current action from phone input.
        
        Args:
            observation: Current robot observation containing joint positions
        
        This method processes phone pose data, solves inverse kinematics,
        and returns the target joint positions.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        print(f"🔍 GET_ACTION START - Phone connected: {self._phone_connected}, Teleop active: {self.start_teleop}")

        # Extract current robot position from observation
        current_joint_pos_deg = None
        if observation is not None:
            try:
                # Extract joint positions from observation (assumes motor names follow pattern)
                motor_keys = ["shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos", 
                             "wrist_flex.pos", "wrist_roll.pos", "gripper.pos"]
                current_joint_pos_deg = [observation.get(key, 0.0) for key in motor_keys]
                print(f"📊 Current robot position from observation: {current_joint_pos_deg}")
            except Exception as e:
                logger.warning(f"Could not extract joint positions from observation: {e}")
                print(f"⚠️ Failed to extract joint positions: {e}")
        
        # If no observation or extraction failed, use rest pose
        if current_joint_pos_deg is None:
            current_joint_pos_deg = np.rad2deg(self.config.rest_pose)
            print(f"🏠 Using rest pose as current position: {current_joint_pos_deg}")
            logger.debug("Using rest pose as current position")

        try:
            # Handle phone connection
            if not self._phone_connected:
                print("📱 PHONE NOT CONNECTED - Opening connection...")
                # Pass current position to connection setup (converted to radians)
                curr_qpos_rad = np.deg2rad(current_joint_pos_deg)
                print(f"🔧 Converting current position to radians: {curr_qpos_rad}")
                self.quat_RP, self.translation_RP, _ = self._open_phone_connection(curr_qpos_rad)
                self._phone_connected = True
                print("✅ PHONE CONNECTION ESTABLISHED")

            if not self.start_teleop:
                print(f"🛑 TELEOP NOT ACTIVE - Returning current position: {current_joint_pos_deg}")
                self._phone_connected = False
                # Return current position when not teleoperating
                return self._format_action_dict(current_joint_pos_deg)

            print("🎮 TELEOP ACTIVE - Getting pose data...")
            # Get latest pose from gRPC
            data = self.pose_service.get_latest_pose(block=False)
            print(f"📡 Raw pose data: {data}")

            # Update reset state tracking
            current_is_resetting = data["is_resetting"]
            print(f"🔄 Reset state - Current: {current_is_resetting}, Previous: {self.prev_is_resetting}")
            
            if current_is_resetting:
                print(f"🛑 RESETTING - Returning current position: {current_joint_pos_deg}")
                self.prev_is_resetting = current_is_resetting
                # Return current position during reset
                return self._format_action_dict(current_joint_pos_deg)

            # Check for reset transition (prev=True, current=False) 
            if self.prev_is_resetting == True and current_is_resetting == False:
                print("🔄 RESET TRANSITION DETECTED - Resetting mapping")
                pos, quat = data["position"], data["rotation"]
                self._reset_mapping(pos, quat)

            self.prev_is_resetting = current_is_resetting

            pos, quat, gripper = data["position"], data["rotation"], data["gripper_open"]
            print(f"📍 Phone pose - Position: {pos}, Rotation: {quat}, Gripper: {gripper}, Precision: {data['precision']}")

            # Map phone pose to robot pose
            print("🗺️ MAPPING PHONE TO ROBOT...")
            t_robot, q_robot = self._map_phone_to_robot(pos, quat, data["precision"])
            print(f"🤖 Robot target pose - Position: {t_robot}, Rotation: {q_robot}")

            # Solve inverse kinematics (returns radians)
            print("🧮 SOLVING IK...")
            solution_rad = self._solve_ik(t_robot, q_robot)
            print(f"⚙️ IK solution (radians): {solution_rad}")

            # Update visualization (expects radians)
            if self.config.enable_visualization and self.urdf_vis:
                print("👁️ Updating visualization...")
                self.urdf_vis.update_cfg(solution_rad)

            # Convert to degrees for robot (SO100 expects degrees)
            solution_deg = np.rad2deg(solution_rad)
            print(f"📐 IK solution (degrees): {solution_deg}")

            # Apply backward compatibility transformations for old calibration system
            # Based on PR #777 backward compatibility documentation
            print("🔄 APPLYING BACKWARD COMPATIBILITY TRANSFORMATIONS...")
            print(f"📊 Before transformations: {solution_deg}")
            
            # For SO100/SO101 backward compatibility:
            # shoulder_lift (index 1): direction reversal + 90° offset
            if len(solution_deg) > 1:
                original_shoulder_lift = solution_deg[1]
                solution_deg[1] = -(solution_deg[1] - 90)
                print(f"🔧 shoulder_lift: {original_shoulder_lift} → {solution_deg[1]} (reversed + 90° offset)")
            
            # elbow_flex (index 2): 90° offset
            if len(solution_deg) > 2:
                original_elbow_flex = solution_deg[2]
                solution_deg[2] -= 90
                print(f"🔧 elbow_flex: {original_elbow_flex} → {solution_deg[2]} (90° offset)")
            
            # wrist_flex (index 3): 90° offset
            if len(solution_deg) > 3:
                original_wrist_flex = solution_deg[3]
                solution_deg[3] += 90
                print(f"🔧 wrist_flex: {original_wrist_flex} → {solution_deg[3]} (+90° offset)")
            
            # wrist_roll (index 4): direction reversal + 90° offset
            if len(solution_deg) > 4:
                original_wrist_roll = solution_deg[4]
                solution_deg[4] = -(solution_deg[4] - 90)
                print(f"🔧 wrist_roll: {original_wrist_roll} → {solution_deg[4]} (reversed + 90° offset)")
            
            print(f"📊 After transformations: {solution_deg}")

            # Update gripper state
            solution_deg[-1] = 0.875 if gripper else 0.0
            print(f"🦾 Final solution with gripper: {solution_deg}")
            
            # Update teleop state
            self.start_teleop = data["switch"]
            print(f"🎯 Teleop switch state: {self.start_teleop}")

            final_action = self._format_action_dict(solution_deg)
            print(f"🎬 FINAL ACTION TO SEND: {final_action}")
            return final_action

        except Exception as e:
            print(f"💥 ERROR in get_action: {e}")
            logger.error(f"Error getting action from {self}: {e}")
            # Return current position on error (safer than rest pose)
            fallback_action = self._format_action_dict(current_joint_pos_deg)
            print(f"🆘 FALLBACK ACTION: {fallback_action}")
            return fallback_action

    def _solve_ik(self, target_position: np.ndarray, target_wxyz: np.ndarray) -> list[float]:
        """Solve inverse kinematics for target pose. Returns solution in radians."""
        print(f"🧮 IK SOLVE - Target position: {target_position}, Target rotation: {target_wxyz}")
        
        try:
            # Import IK solver from daxie package
            from daxie.src.teleop.solve_ik import solve_ik
            
            print("📦 IK solver imported successfully")
            solution = solve_ik(
                robot=self.robot,
                target_link_name=self.config.target_link_name,
                target_position=target_position,
                target_wxyz=target_wxyz,
            )
            
            print(f"✅ IK solution computed: {solution}")
            return solution  # Always return radians
        except ImportError:
            print("💥 ERROR: Could not import IK solver from daxie package")
            logger.error("Could not import IK solver from daxie package")
            # Return rest pose in radians
            fallback = list(self.config.rest_pose)
            print(f"🆘 Using fallback rest pose: {fallback}")
            return fallback

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

 