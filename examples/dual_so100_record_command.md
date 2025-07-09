# Dual SO100 Recording with Default Record Function

This document shows how to use the new `so100_dual_follower` and `so100_dual_leader` types with the default LeRobot record function.

## New Robot Types

- **`so100_dual_follower`**: Dual arm follower robot with left and right ports
- **`so100_dual_leader`**: Dual arm leader teleoperator with left and right ports

## Command Line Usage

### Basic Recording

```bash
python -m lerobot.record \
    --robot.type=so100_dual_follower \
    --robot.left_port=/dev/tty.usbmodem585A0076841 \
    --robot.right_port=/dev/tty.usbmodem585A0076842 \
    --robot.id=dual_so100_follower \
    --teleop.type=so100_dual_leader \
    --teleop.left_port=/dev/tty.usbmodem575E0031751 \
    --teleop.right_port=/dev/tty.usbmodem575E0031752 \
    --teleop.id=dual_so100_leader \
    --dataset.repo_id=username/dual_so100_dataset \
    --dataset.num_episodes=5 \
    --dataset.single_task="Dual arm pick and place task" \
    --display_data=true
```

### With Cameras

```bash
python -m lerobot.record \
    --robot.type=so100_dual_follower \
    --robot.left_port=/dev/tty.usbmodem585A0076841 \
    --robot.right_port=/dev/tty.usbmodem585A0076842 \
    --robot.id=dual_so100_follower \
    --robot.cameras="{front: {type: opencv, camera_index: 0, width: 640, height: 480}}" \
    --teleop.type=so100_dual_leader \
    --teleop.left_port=/dev/tty.usbmodem575E0031751 \
    --teleop.right_port=/dev/tty.usbmodem575E0031752 \
    --teleop.id=dual_so100_leader \
    --dataset.repo_id=username/dual_so100_dataset \
    --dataset.num_episodes=10 \
    --dataset.single_task="Dual arm manipulation with visual feedback" \
    --display_data=true
```

### With Safety Limits

```bash
python -m lerobot.record \
    --robot.type=so100_dual_follower \
    --robot.left_port=/dev/tty.usbmodem585A0076841 \
    --robot.right_port=/dev/tty.usbmodem585A0076842 \
    --robot.id=dual_so100_follower \
    --robot.max_relative_target=50 \
    --teleop.type=so100_dual_leader \
    --teleop.left_port=/dev/tty.usbmodem575E0031751 \
    --teleop.right_port=/dev/tty.usbmodem575E0031752 \
    --teleop.id=dual_so100_leader \
    --dataset.repo_id=username/dual_so100_dataset \
    --dataset.num_episodes=5 \
    --dataset.single_task="Safe dual arm manipulation" \
    --display_data=true
```

## Motor Configuration

The dual arm setup uses the following motor ID configuration:

### Left Arm (Motor IDs 1-6)
- Motor 1: shoulder_pan
- Motor 2: shoulder_lift  
- Motor 3: elbow_flex
- Motor 4: wrist_flex
- Motor 5: wrist_roll
- Motor 6: gripper

### Right Arm (Motor IDs 7-12)
- Motor 7: shoulder_pan
- Motor 8: shoulder_lift
- Motor 9: elbow_flex
- Motor 10: wrist_flex
- Motor 11: wrist_roll
- Motor 12: gripper

## State and Action Names

The dual arm robot provides the following state and action names:

### Observations
- `left_shoulder_pan.pos`
- `left_shoulder_lift.pos`
- `left_elbow_flex.pos`
- `left_wrist_flex.pos`
- `left_wrist_roll.pos`
- `left_gripper.pos`
- `right_shoulder_pan.pos`
- `right_shoulder_lift.pos`
- `right_elbow_flex.pos`
- `right_wrist_flex.pos`
- `right_wrist_roll.pos`
- `right_gripper.pos`
- `camera_name` (if cameras are configured)

### Actions
- `left_shoulder_pan.pos`
- `left_shoulder_lift.pos`
- `left_elbow_flex.pos`
- `left_wrist_flex.pos`
- `left_wrist_roll.pos`
- `left_gripper.pos`
- `right_shoulder_pan.pos`
- `right_shoulder_lift.pos`
- `right_elbow_flex.pos`
- `right_wrist_flex.pos`
- `right_wrist_roll.pos`
- `right_gripper.pos`

## Calibration

Both the follower and leader arms need to be calibrated separately. The calibration process is the same as the single arm version but performed for each arm individually.

### Calibrate Follower Arms

```bash
python -m lerobot.calibrate \
    --robot.type=so100_dual_follower \
    --robot.left_port=/dev/tty.usbmodem585A0076841 \
    --robot.right_port=/dev/tty.usbmodem585A0076842 \
    --robot.id=dual_so100_follower
```

### Calibrate Leader Arms

```bash
python -m lerobot.calibrate \
    --teleop.type=so100_dual_leader \
    --teleop.left_port=/dev/tty.usbmodem575E0031751 \
    --teleop.right_port=/dev/tty.usbmodem575E0031752 \
    --teleop.id=dual_so100_leader
```

## Benefits of the New Types

1. **Clear Separation**: The dual arm types are distinct from single arm types, making configuration clearer
2. **Default Compatibility**: Works seamlessly with the default record function
3. **Proper State Management**: Each arm's state is properly prefixed and managed
4. **Safety Features**: Supports safety limits and proper error handling
5. **Calibration Support**: Full calibration support for both arms
6. **Camera Integration**: Supports camera integration for visual feedback

## Migration from Old Dual Arm Setup

If you were previously using the `so100_follower` type with dual ports, you can now use the dedicated `so100_dual_follower` type for better clarity and functionality. 