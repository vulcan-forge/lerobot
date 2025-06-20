# Phone Teleoperation Integration Guide

This guide explains how to integrate the phone teleoperation system from the old lerobot-daxie repository into the new lerobot-vulcan architecture.

## Overview

The phone teleoperation system has been successfully ported from the monolithic `ManipulatorRobot` class architecture to the new modular system that separates robots and teleoperators.

### Key Changes

**Old Architecture (lerobot-daxie):**
- Single `ManipulatorRobot` class handled both leader and follower arms
- `phone_teleop_step()` method integrated directly into the robot class
- Tight coupling between phone control and robot control

**New Architecture (lerobot-vulcan):**
- Separate `Robot` (follower) and `Teleoperator` (leader) classes
- `PhoneTeleoperator` as a dedicated teleoperator class
- Clean separation of concerns and better modularity

## Components

### 1. PhoneTeleoperator Class

Located at: `lerobot/common/teleoperators/phone_teleoperator/phone_teleoperator.py`

**Key Features:**
- Inherits from the abstract `Teleoperator` base class
- Integrates with existing gRPC server from daxie package
- Handles inverse kinematics solving
- Provides 3D visualization via Viser
- Supports precision/normal mode switching
- Handles phone pose mapping to robot coordinates

### 2. PhoneTeleoperatorConfig Class

Located at: `lerobot/common/teleoperators/phone_teleoperator/config_phone_teleoperator.py`

**Configuration Options:**
- Robot URDF and mesh paths
- Sensitivity settings (normal/precision modes)
- Initial robot pose and rest position
- gRPC server settings
- Visualization options
- Safety parameters

### 3. Integration Example

Located at: `examples/phone_teleop_example.py`

## Setup Instructions

### Prerequisites

1. **Install Additional Dependencies:**
   ```bash
   pip install pyroki viser yourdfpy
   ```

2. **Ensure Daxie Package Access:**
   - The `daxie` package must be installed and accessible
   - gRPC server components from `daxie.src.server.pos_grpc_server`
   - IK solver from `daxie.src.teleop.solve_ik`

3. **Robot Assets:**
   - SO100 URDF file (`so100.urdf`)
   - Robot mesh files directory
   - These can be obtained from the daxie package via `get_so100_path()`

### Basic Usage

```python
from lerobot.common.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.common.teleoperators.phone_teleoperator import PhoneTeleoperator, PhoneTeleoperatorConfig

# Configure robot
robot_config = SO100FollowerConfig(
    id="so100_follower_main",
    port="/dev/ttyUSB0",
    use_degrees=True,
    max_relative_target=30.0,
)

# Configure phone teleoperator
phone_config = PhoneTeleoperatorConfig(
    id="phone_teleop_main",
    urdf_path="/path/to/so100.urdf",
    mesh_path="/path/to/meshes/",
    target_link_name="Fixed_Jaw",
    normal_sensitivity=0.5,
    precision_sensitivity=0.2,
    enable_visualization=True,
)

# Initialize and connect
robot = SO100Follower(robot_config)
phone_teleop = PhoneTeleoperator(phone_config)

robot.connect()
phone_teleop.connect()

# Control loop
while True:
    action = phone_teleop.get_action()
    robot.send_action(action)
```

## Phone Teleoperation Features

### 1. Pose Mapping
- Converts phone pose (position + orientation) to robot end-effector pose
- Supports sensitivity scaling for normal and precision modes
- Handles coordinate frame transformations
- Provides reset functionality for re-calibration

### 2. Inverse Kinematics
- Uses pyroki for IK solving
- Targets specific end-effector link (configurable)
- Handles joint limits and constraints
- Returns valid joint configurations

### 3. Visualization
- Real-time 3D robot visualization via Viser
- Shows robot state during teleoperation
- Configurable server port
- Optional ground grid and coordinate frames

### 4. Safety Features
- Maximum relative target limits
- Error handling and graceful degradation
- Rest pose fallback on errors
- Connection state monitoring

## Migration Notes

### From Old `phone_teleop_step()` Method

The old `phone_teleop_step()` method has been replaced with:

1. **`PhoneTeleoperator.get_action()`** - Gets the target joint positions from phone input
2. **`Robot.send_action()`** - Sends the action to the physical robot

### Key Differences

| Old System | New System |
|------------|------------|
| `robot.phone_teleop_step(record_data=True)` | `action = phone_teleop.get_action()` + `robot.send_action(action)` |
| Monolithic robot class | Separate robot and teleoperator classes |
| Built-in phone handling | Dedicated phone teleoperator |
| Fixed configuration | Configurable via dataclass |

### Data Recording

For data recording in the new system:

```python
# Get action from phone
action = phone_teleop.get_action()

# Send to robot and get actual action sent
actual_action = robot.send_action(action)

# Get observation
observation = robot.get_observation()

# Record to dataset
frame = {**observation, **actual_action, "task": task_name}
dataset.add_frame(frame)
```

## Troubleshooting

### Common Issues

1. **Import Errors:**
   - Ensure `pyroki`, `viser`, and `yourdfpy` are installed
   - Check that `daxie` package is accessible

2. **Connection Issues:**
   - Verify gRPC server is running and accessible
   - Check robot USB connection and port settings
   - Ensure proper permissions for device access

3. **URDF/Mesh Issues:**
   - Verify paths to URDF and mesh files are correct
   - Use `get_so100_path()` from daxie package for default paths

4. **Performance Issues:**
   - Adjust control loop frequency (target 30 Hz)
   - Monitor network latency for gRPC communication
   - Consider disabling visualization for better performance

### Configuration Tips

1. **Sensitivity Tuning:**
   - Start with conservative values (0.2-0.5)
   - Adjust based on robot workspace and user preference
   - Use precision mode for fine manipulation

2. **Safety Settings:**
   - Set appropriate `max_relative_target` limits
   - Monitor for sudden movements or oscillations
   - Test in safe environment first

3. **Visualization:**
   - Enable for development and debugging
   - Disable for production or high-frequency control
   - Adjust server port if conflicts occur

## Future Enhancements

Potential improvements for the phone teleoperation system:

1. **Enhanced Feedback:**
   - Haptic feedback via phone vibration
   - Visual indicators for robot state
   - Audio cues for state changes

2. **Advanced Features:**
   - Multi-robot control
   - Collaborative teleoperation
   - Gesture recognition
   - Voice commands

3. **Performance Optimizations:**
   - Reduced latency communication
   - Predictive control algorithms
   - Adaptive sensitivity based on task

4. **Safety Improvements:**
   - Collision detection and avoidance
   - Emergency stop mechanisms
   - Workspace boundary enforcement

## Conclusion

The phone teleoperation system has been successfully integrated into the new lerobot architecture while maintaining all original functionality. The modular design provides better maintainability, extensibility, and separation of concerns compared to the original implementation. 