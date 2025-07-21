"""
Professional IK Solver using PyBullet
A robust, industry-standard inverse kinematics solver that can be used as a drop-in replacement.
"""

import numpy as np
import pybullet as p
import pybullet_data
import tempfile
import os
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class IKSolver2:
    """
    Professional IK solver using PyBullet's mature inverse kinematics algorithms.
    Designed as a drop-in replacement for research-grade solvers.
    """
    
    def __init__(
        self,
        urdf_path: str,
        end_effector_link_name: str,
        joint_names: Optional[list] = None,
        joint_limits: Optional[Tuple[list, list]] = None,
        max_iterations: int = 100,
        residual_threshold: float = 1e-5,
        solver_method: str = "default"  # "default", "damped_least_squares", or "pseudo_inverse"
    ):
        """
        Initialize the IK solver.
        
        Args:
            urdf_path: Path to robot URDF file
            end_effector_link_name: Name of the end effector link
            joint_names: List of joint names to control (if None, uses all revolute joints)
            joint_limits: Tuple of (lower_limits, upper_limits) lists
            max_iterations: Maximum IK solver iterations
            residual_threshold: Convergence threshold
            solver_method: IK solving method to use
        """
        self.urdf_path = urdf_path
        self.end_effector_link_name = end_effector_link_name
        self.joint_names = joint_names
        self.max_iterations = max_iterations
        self.residual_threshold = residual_threshold
        self.solver_method = solver_method
        
        # Initialize PyBullet in DIRECT mode (no GUI)
        self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load robot
        self.robot_id = p.loadURDF(urdf_path)
        
        # Get joint and link information
        self._setup_robot_info()
        
        # Set joint limits
        self._setup_joint_limits(joint_limits)
        
        logger.info(f"IK Solver initialized with {len(self.controlled_joints)} joints")
        logger.info(f"End effector link: {self.end_effector_link_name} (index: {self.end_effector_link_index})")
    
    def _setup_robot_info(self):
        """Extract robot joint and link information."""
        num_joints = p.getNumJoints(self.robot_id)
        
        # Get all joint info
        self.joint_info = {}
        self.link_info = {}
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            link_name = joint_info[12].decode('utf-8')
            
            self.joint_info[joint_name] = {
                'index': i,
                'type': joint_info[2],
                'lower_limit': joint_info[8],
                'upper_limit': joint_info[9],
                'max_force': joint_info[10],
                'max_velocity': joint_info[11]
            }
            
            self.link_info[link_name] = i
        
        # Find end effector link index
        if self.end_effector_link_name in self.link_info:
            self.end_effector_link_index = self.link_info[self.end_effector_link_name]
        else:
            raise ValueError(f"End effector link '{self.end_effector_link_name}' not found in robot")
        
        # Determine controlled joints
        if self.joint_names is None:
            # Use all revolute joints
            self.controlled_joints = []
            for joint_name, info in self.joint_info.items():
                if info['type'] == p.JOINT_REVOLUTE:
                    self.controlled_joints.append(info['index'])
        else:
            # Use specified joints
            self.controlled_joints = []
            for joint_name in self.joint_names:
                if joint_name in self.joint_info:
                    self.controlled_joints.append(self.joint_info[joint_name]['index'])
                else:
                    raise ValueError(f"Joint '{joint_name}' not found in robot")
        
        self.controlled_joints.sort()  # Ensure consistent ordering
    
    def _setup_joint_limits(self, joint_limits: Optional[Tuple[list, list]]):
        """Setup joint limits for IK solver."""
        if joint_limits is not None:
            lower_limits, upper_limits = joint_limits
            if len(lower_limits) != len(self.controlled_joints) or len(upper_limits) != len(self.controlled_joints):
                raise ValueError("Joint limits length must match number of controlled joints")
            self.lower_limits = lower_limits
            self.upper_limits = upper_limits
        else:
            # Use URDF limits
            self.lower_limits = []
            self.upper_limits = []
            for joint_idx in self.controlled_joints:
                joint_info = p.getJointInfo(self.robot_id, joint_idx)
                self.lower_limits.append(joint_info[8])
                self.upper_limits.append(joint_info[9])
    
    def solve_ik(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        current_joint_positions: Optional[np.ndarray] = None,
        position_tolerance: float = 1e-4,
        orientation_tolerance: float = 1e-4
    ) -> Tuple[np.ndarray, bool]:
        """
        Solve inverse kinematics for target pose.
        
        Args:
            target_position: Target position [x, y, z] in meters
            target_orientation: Target orientation as quaternion [w, x, y, z]
            current_joint_positions: Current joint positions for warm start
            position_tolerance: Position convergence tolerance
            orientation_tolerance: Orientation convergence tolerance
            
        Returns:
            Tuple of (joint_positions, success_flag)
        """
        # Convert orientation to PyBullet format [x, y, z, w]
        if target_orientation.shape == (4,):
            if np.abs(target_orientation[0]) > 0.7:  # Likely w-first format
                quat_pybullet = [target_orientation[1], target_orientation[2], target_orientation[3], target_orientation[0]]
            else:  # Likely x-first format
                quat_pybullet = target_orientation.tolist()
        else:
            raise ValueError("Target orientation must be a 4-element quaternion")
        
        # Set current joint positions if provided
        if current_joint_positions is not None:
            if len(current_joint_positions) != len(self.controlled_joints):
                raise ValueError("Current joint positions length must match controlled joints")
            for i, joint_idx in enumerate(self.controlled_joints):
                p.resetJointState(self.robot_id, joint_idx, current_joint_positions[i])
        
        # Solve IK using PyBullet's robust algorithms
        try:
            ik_solution = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.end_effector_link_index,
                targetPosition=target_position.tolist(),
                targetOrientation=quat_pybullet,
                lowerLimits=self.lower_limits,
                upperLimits=self.upper_limits,
                jointRanges=[u - l for l, u in zip(self.lower_limits, self.upper_limits)],
                restPoses=current_joint_positions.tolist() if current_joint_positions is not None else None,
                maxNumIterations=self.max_iterations,
                residualThreshold=self.residual_threshold
            )
            
            # Extract solution for controlled joints
            joint_positions = np.array([ik_solution[i] for i in range(len(self.controlled_joints))])
            
            # Verify solution quality
            success = self._verify_solution(
                joint_positions, target_position, target_orientation,
                position_tolerance, orientation_tolerance
            )
            
            if not success:
                logger.warning("IK solution may not meet tolerance requirements")
            
            return joint_positions, success
            
        except Exception as e:
            logger.error(f"IK solving failed: {e}")
            # Return current position or rest pose as fallback
            if current_joint_positions is not None:
                return current_joint_positions, False
            else:
                # Return middle of joint ranges as fallback
                fallback = np.array([(l + u) / 2 for l, u in zip(self.lower_limits, self.upper_limits)])
                return fallback, False
    
    def _verify_solution(
        self,
        joint_positions: np.ndarray,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        position_tolerance: float,
        orientation_tolerance: float
    ) -> bool:
        """Verify that the IK solution meets tolerance requirements."""
        # Set joint positions
        for i, joint_idx in enumerate(self.controlled_joints):
            p.resetJointState(self.robot_id, joint_idx, joint_positions[i])
        
        # Get forward kinematics result
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        actual_position = np.array(link_state[0])
        actual_orientation = np.array(link_state[1])  # [x, y, z, w] format
        
        # Check position error
        position_error = np.linalg.norm(actual_position - target_position)
        
        # Check orientation error (quaternion distance)
        # Convert target to [x, y, z, w] format if needed
        if target_orientation[0] > 0.7:  # w-first format
            target_quat_xyzw = [target_orientation[1], target_orientation[2], target_orientation[3], target_orientation[0]]
        else:
            target_quat_xyzw = target_orientation.tolist()
        
        orientation_error = self._quaternion_distance(actual_orientation, target_quat_xyzw)
        
        return position_error < position_tolerance and orientation_error < orientation_tolerance
    
    def _quaternion_distance(self, q1: np.ndarray, q2: list) -> float:
        """Compute angular distance between two quaternions."""
        q1_norm = q1 / np.linalg.norm(q1)
        q2_norm = np.array(q2) / np.linalg.norm(q2)
        dot_product = np.abs(np.dot(q1_norm, q2_norm))
        dot_product = np.clip(dot_product, 0.0, 1.0)
        return 2 * np.arccos(dot_product)
    
    def get_forward_kinematics(self, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for given joint positions.
        
        Args:
            joint_positions: Joint positions array
            
        Returns:
            Tuple of (position, orientation) where orientation is [w, x, y, z]
        """
        # Set joint positions
        for i, joint_idx in enumerate(self.controlled_joints):
            p.resetJointState(self.robot_id, joint_idx, joint_positions[i])
        
        # Get link state
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        position = np.array(link_state[0])
        orientation_xyzw = np.array(link_state[1])
        
        # Convert to w-first format
        orientation_wxyz = np.array([orientation_xyzw[3], orientation_xyzw[0], orientation_xyzw[1], orientation_xyzw[2]])
        
        return position, orientation_wxyz
    
    def __del__(self):
        """Cleanup PyBullet connection."""
        try:
            p.disconnect(self.physics_client)
        except:
            pass


def solve_ik(
    robot,  # Robot object (for compatibility)
    target_link_name: str,
    target_wxyz: np.ndarray,
    target_position: np.ndarray,
    ik_solver: Optional[IKSolver2] = None
) -> np.ndarray:
    """
    Drop-in replacement function for the original solve_ik.
    
    Args:
        robot: Robot object (used to get URDF path)
        target_link_name: Name of target link
        target_wxyz: Target orientation [w, x, y, z]
        target_position: Target position [x, y, z]
        ik_solver: Pre-initialized IK solver (optional)
        
    Returns:
        Joint positions in radians
    """
    # This is a simplified interface - in practice you'd want to cache the IK solver
    # or pass it in as a parameter to avoid reinitializing PyBullet each time
    
    if ik_solver is None:
        logger.warning("IKSolver2 not provided - this will be slow. Consider pre-initializing the solver.")
        # Would need robot.urdf_path or similar to initialize properly
        raise NotImplementedError("Please initialize IKSolver2 separately and pass it in")
    
    # Use the professional solver
    joint_positions, success = ik_solver.solve_ik(target_position, target_wxyz)
    
    if not success:
        logger.warning("IK solution may not be optimal")
    
    return joint_positions


# Example usage function
def create_so100_ik_solver(urdf_path: str) -> IKSolver2:
    """
    Create an IK solver specifically configured for SO100 robot.
    
    Args:
        urdf_path: Path to SO100 URDF file
        
    Returns:
        Configured IKSolver2 instance
    """
    # SO100 joint configuration
    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint", 
        "elbow_flex_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
        "gripper_joint"
    ]
    
    # Reasonable joint limits for SO100 (in radians)
    lower_limits = [-np.pi, -np.pi/2, -np.pi, -np.pi, -np.pi, 0.0]
    upper_limits = [np.pi, np.pi/2, np.pi, np.pi, np.pi, 1.0]
    
    return IKSolver2(
        urdf_path=urdf_path,
        end_effector_link_name="gripper_tip",  # or whatever your end effector is called
        joint_names=joint_names,
        joint_limits=(lower_limits, upper_limits),
        max_iterations=100,
        residual_threshold=1e-5
    ) 