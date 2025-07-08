# Sourccey V2 Beta Teleoperation Examples

This directory contains three different approaches for teleoperating the Sourccey V2 Beta robot:

## 1. Client-Server Architecture (`teleoperate.py`)

**What it does:**
- Uses `SourcceyV2BetaClient` to connect to a remote server
- Requires a server script running on the robot: `python -m lerobot.common.robots.sourccey_v2beta.sourccey_v2beta_host`
- Communicates via ZMQ over network
- Supports both arm and base movement

**When to use:**
- When the robot is on a different computer/network
- When you want to separate the control logic from the robot hardware
- For distributed systems

**Setup:**
1. Start the server on the robot:
   ```bash
   python -m lerobot.common.robots.sourccey_v2beta.sourccey_v2beta_host --robot.id=sourccey_v2beta
   ```
2. Run the teleoperation script:
   ```bash
   python examples/sourccey_v2beta/teleoperate.py
   ```

## 2. Direct Connection Architecture (`teleoperate_direct.py`)

**What it does:**
- Uses `SourcceyV2Beta` to connect directly to the robot motors
- No server required - direct serial communication
- Similar to how SO100 works
- Currently supports arm movement only (base movement commented out)

**When to use:**
- When the robot is connected directly to your computer
- When you want lower latency and simpler setup
- For local development and testing

**Setup:**
1. Connect the robot directly to your computer via USB
2. Update the port names in the script (COM28, COM29 for Windows)
3. Run the teleoperation script:
   ```bash
   python examples/sourccey_v2beta/teleoperate_direct.py
   ```

## 3. Phone Teleoperation (`teleoperate_phone.py`)

**What it does:**
- Uses phone motion to control the robot arms
- Connects to VirtualManipulator app via gRPC
- Uses inverse kinematics to convert phone pose to joint commands
- Supports both left and right arm control
- Includes visualization with Viser

**When to use:**
- When you want intuitive motion-based control
- For demonstrations and presentations
- When you need precise end-effector control

**Setup:**
1. Install additional dependencies:
   ```bash
   pip install pyroki viser yourdfpy
   ```
2. Connect the robot directly to your computer via USB
3. Update the port names in the script
4. Run the phone teleoperation script:
   ```bash
   python examples/sourccey_v2beta/teleoperate_phone.py
   ```
5. Open the VirtualManipulator app on your phone and connect to the gRPC server

## Key Differences

| Feature | Client-Server | Direct Connection | Phone Teleop |
|---------|---------------|-------------------|---------------|
| Network Required | Yes | No | No (local gRPC) |
| Latency | Higher (network) | Lower (direct) | Low (local) |
| Setup Complexity | Higher (server + client) | Lower (direct only) | Medium (dependencies) |
| Base Movement | Supported | Not implemented yet | Not supported |
| Arm Movement | Supported | Supported | Supported |
| Remote Operation | Yes | No | No |
| Control Method | Leader arm | Leader arm | Phone motion |
| Visualization | Basic | Basic | Advanced (Viser) |

## Port Configuration

Update the port names in all scripts according to your system:

**Windows:**
```python
robot_config = SourcceyV2BetaConfig(left_arm_port="COM28", right_arm_port="COM29", id="sourccey_v2beta")
```

**Linux/Mac:**
```python
robot_config = SourcceyV2BetaConfig(left_arm_port="/dev/ttyUSB0", right_arm_port="/dev/ttyUSB1", id="sourccey_v2beta")
```

## Phone Teleoperation Setup

### Prerequisites
1. Install the VirtualManipulator app on your phone
2. Install Python dependencies:
   ```bash
   pip install pyroki viser yourdfpy
   ```

### Configuration
You can control either the left or right arm by modifying the configuration:

```python
phone_teleop_config = SourcceyV2BetaPhoneTeleopConfig(
    control_left_arm=True,   # Control left arm
    control_right_arm=False, # Don't control right arm
    # ... other settings
)
```

### Phone App Connection
1. Run the teleoperation script
2. Open VirtualManipulator app on your phone
3. Connect to the gRPC server (default: localhost:8765)
4. Use phone motion to control the robot arm

### Gestures
- **Normal mode**: Move phone to control robot arm
- **Precision mode**: Toggle for fine control
- **Reset gesture**: Make a fist to recalibrate mapping

## Troubleshooting

### Finding Port Names
Use the port finder script:
```bash
python lerobot/find_port.py
```

### Connection Issues
- Make sure the robot is powered on
- Check that the USB cables are properly connected
- Verify the port names match your system
- For client-server: ensure the server is running and accessible on the network

### Phone Teleoperation Issues
- Ensure all dependencies are installed: `pip install pyroki viser yourdfpy`
- Check that the gRPC port (8765) is not blocked by firewall
- Verify the phone app can connect to the server
- Make sure the URDF file exists for the robot model

### Base Movement in Direct Mode
The base movement functionality is currently commented out in the direct `SourcceyV2Beta` class. To add it back, you would need to:

1. Uncomment the base motor configuration in `sourccey_v2beta.py`
2. Implement the `_wheel_raw_to_body` and `_body_to_wheel_raw` methods
3. Add the `_from_keyboard_to_base_action` method to the direct class

### URDF Model
For phone teleoperation, you'll need a URDF model of the Sourccey robot. If it doesn't exist yet, you can:

1. Create a URDF file for the Sourccey robot
2. Place it in `lerobot/common/robots/sourccey_v2beta/model/`
3. Update the configuration to point to the correct URDF file 