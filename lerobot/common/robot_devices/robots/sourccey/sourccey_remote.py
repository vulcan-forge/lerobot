import base64
import json
import threading
import time
from pathlib import Path

import cv2
import zmq

from lerobot.common.robot_devices.robots.sourccey.sourccey_manipulator import SourcceyV1Beta

def setup_zmq_sockets(config):
    print("Starting ZMQ socket setup...")
    context = zmq.Context()
    print("Created ZMQ context")

    print(f"Creating PULL socket for commands on port {config.port}...")
    cmd_socket = context.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    print(f"Attempting to bind command socket to tcp://*:{config.port}")
    cmd_socket.bind(f"tcp://*:{config.port}")
    print("Successfully bound command socket")

    print(f"Creating PUSH socket for video on port {config.video_port}...")
    video_socket = context.socket(zmq.PUSH)
    video_socket.setsockopt(zmq.CONFLATE, 1)
    print(f"Attempting to bind video socket to tcp://*:{config.video_port}")
    video_socket.bind(f"tcp://*:{config.video_port}")
    print("Successfully bound video socket")

    print("ZMQ socket setup complete")
    return context, cmd_socket, video_socket

def run_camera_capture(cameras, images_lock, latest_images_dict, stop_event):
    while not stop_event.is_set():
        local_dict = {}
        for name, cam in cameras.items():
            frame = cam.async_read()
            ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if ret:
                local_dict[name] = base64.b64encode(buffer).decode("utf-8")
            else:
                local_dict[name] = ""
        with images_lock:
            latest_images_dict.update(local_dict)
        time.sleep(0.01)

def calibrate_follower_arms(left_motors_bus, right_motors_bus, calib_dir_str):
    """
    Calibrates the follower arm. Attempts to load an existing calibration file;
    if not found, runs manual calibration and saves the result.
    """
    calib_dir = Path(calib_dir_str)
    calib_dir.mkdir(parents=True, exist_ok=True)
    calib_left_file = calib_dir / "left_follower.json"
    calib_right_file = calib_dir / "right_follower.json"
    print(f"[INFO] Calibration file: {calib_left_file}")
    print(f"[INFO] Calibration file: {calib_right_file}")
    try:
        from lerobot.common.robot_devices.robots.feetech_calibration import run_arm_manual_calibration
    except ImportError:
        print("[WARNING] Calibration function not available. Skipping calibration.")
        return

    if calib_left_file.exists():
        with open(calib_left_file) as f:
            calibration = json.load(f)
        print(f"[INFO] Loaded calibration from {calib_left_file}")
    else:
        print("[INFO] Calibration file not found. Running manual calibration...")
        calibration = run_arm_manual_calibration(left_motors_bus, "sourccey_v1beta", "left_follower", "follower")
        print(f"[INFO] Calibration complete. Saving to {calib_left_file}")
        with open(calib_left_file, "w") as f:
            json.dump(calibration, f)

    try:
        left_motors_bus.set_calibration(calibration)
        print("[INFO] Applied calibration for left follower arm.")
    except Exception as e:
        print(f"[WARNING] Could not apply calibration: {e}")

    if calib_right_file.exists():
        with open(calib_right_file) as f:
            calibration = json.load(f)
        print(f"[INFO] Loaded calibration from {calib_right_file}")
    else:
        print("[INFO] Calibration file not found. Running manual calibration...")
        calibration = run_arm_manual_calibration(right_motors_bus, "sourccey_v1beta", "right_follower", "follower")
        print(f"[INFO] Calibration complete. Saving to {calib_right_file}")
        with open(calib_right_file, "w") as f:
            json.dump(calibration, f)

    try:
        right_motors_bus.set_calibration(calibration)
        print("[INFO] Applied calibration for right follower arm.")
    except Exception as e:
        print(f"[WARNING] Could not apply calibration: {e}")


def run_sourccey_v1beta(robot_config, control_config):
    """
    Runs the SourcceyV1Beta robot:
      - Sets up cameras and connects them.
      - Initializes the follower arm motors.
      - Calibrates the follower arm if necessary.
      - Creates ZeroMQ sockets for receiving commands and streaming observations.
      - Processes incoming commands (arm positions) and sends back sensor and camera data.
    """

    print("here 5")

    # Import helper functions and classes
    from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
    from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, TorqueMode

    print("here 6")

    # Initialize cameras from the robot configuration.
    cameras = make_cameras_from_configs(robot_config.cameras, robot_config, control_config)
    for cam in cameras.values():
        cam.connect()

    print("here 7")

    # Initialize the motors bus using the follower arm configuration.
    left_motor_config = robot_config.follower_arms.get("left")
    right_motor_config = robot_config.follower_arms.get("right")
    if left_motor_config is None:
        print("[ERROR] Follower arm 'left' configuration not found.")
        return
    if right_motor_config is None:
        print("[ERROR] Follower arm 'right' configuration not found.")
        return

    left_motors_bus = FeetechMotorsBus(left_motor_config)
    right_motors_bus = FeetechMotorsBus(right_motor_config)
    left_motors_bus.connect()
    right_motors_bus.connect()

    print("here 8")

    # Calibrate the follower arm.
    calibrate_follower_arms(left_motors_bus, right_motors_bus, robot_config.calibration_dir)

    print("here 9")

    robot = SourcceyV1Beta(left_motors_bus, right_motors_bus)

    # Define the expected arm motor IDs.
    arm_motor_ids = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    # Disable torque for each arm motor.
    for motor in arm_motor_ids:
        left_motors_bus.write("Torque_Enable", TorqueMode.DISABLED.value, motor)
        right_motors_bus.write("Torque_Enable", TorqueMode.DISABLED.value, motor)

    print("here 10")

    # Set up ZeroMQ sockets.
    context, cmd_socket, video_socket = setup_zmq_sockets(robot_config)

    # Start the camera capture thread.
    latest_images_dict = {}
    images_lock = threading.Lock()
    stop_event = threading.Event()
    cam_thread = threading.Thread(
        target=run_camera_capture, args=(cameras, images_lock, latest_images_dict, stop_event), daemon=True
    )
    cam_thread.start()

    print("here 11")

    last_cmd_time = time.time()
    print("SourcceyV1Beta robot server started. Waiting for commands...")

    try:
        while True:
            loop_start_time = time.time()

            # Process incoming commands (non-blocking).
            while True:
                try:
                    msg = cmd_socket.recv_string(zmq.NOBLOCK)
                except zmq.Again:
                    break
                try:
                    data = json.loads(msg)
                    # Process arm position commands.
                    if "left_arm_positions" in data:
                        left_arm_positions = data["left_arm_positions"]
                        if not isinstance(left_arm_positions, list):
                            print(f"[ERROR] Invalid left_arm_positions: {left_arm_positions}")
                        elif len(left_arm_positions) < len(arm_motor_ids):
                            print(
                                f"[WARNING] Received {len(left_arm_positions)} left_arm_positions, expected {len(arm_motor_ids)}"
                            )
                        else:
                            left_arm_positions = left_arm_positions[:6]
                            for motor, pos in zip(arm_motor_ids, left_arm_positions, strict=False):
                                left_motors_bus.write("Goal_Position", pos, motor)

                    if "right_arm_positions" in data:
                        right_arm_positions = data["right_arm_positions"]
                        if not isinstance(right_arm_positions, list):
                            print(f"[ERROR] Invalid right_arm_positions: {right_arm_positions}")
                        elif len(right_arm_positions) < len(arm_motor_ids):
                            print(
                                f"[WARNING] Received {len(right_arm_positions)} right_arm_positions, expected {len(arm_motor_ids)}"
                            )
                        else:
                            right_arm_positions = right_arm_positions[:6]
                            for motor, pos in zip(arm_motor_ids, right_arm_positions, strict=False):
                                right_motors_bus.write("Goal_Position", pos, motor)

                    # Process wheel (base) commands.
                    if "raw_velocity" in data:
                        raw_command = data["raw_velocity"]
                        # Expect keys: "back_left_wheel", "back_right_wheel", "front_left_wheel", "front_right_wheel".
                        command_speeds = [
                            int(raw_command.get("back_left_wheel", 0)),
                            int(raw_command.get("back_right_wheel", 0)),
                            int(raw_command.get("front_left_wheel", 0)),
                            int(raw_command.get("front_right_wheel", 0)),
                        ]
                        robot.set_velocity(command_speeds)
                        last_cmd_time = time.time()
                except Exception as e:
                    print(f"[ERROR] Parsing message failed: {e}")

            # Watchdog: stop the robot if no command is received for over 0.5 seconds.
            now = time.time()
            if now - last_cmd_time > 0.5:
                robot.stop()
                last_cmd_time = now

            # Read current wheel speeds from the robot.
            current_velocity = robot.read_velocity()

            # Read the follower arm state from the motors bus.
            left_arm_state = []
            right_arm_state = []
            for motor in arm_motor_ids:
                try:
                    left_pos = left_motors_bus.read("Present_Position", motor)
                    right_pos = right_motors_bus.read("Present_Position", motor)
                    # Convert the position to a float (or use as is if already numeric).
                    left_arm_state.append(float(left_pos) if not isinstance(left_pos, (int, float)) else left_pos)
                    right_arm_state.append(float(right_pos) if not isinstance(right_pos, (int, float)) else right_pos)
                except Exception as e:
                    print(f"[ERROR] Reading motor {motor} failed: {e}")

            # Get the latest camera images.
            with images_lock:
                images_dict_copy = dict(latest_images_dict)

            # Build the observation dictionary.
            observation = {
                "images": images_dict_copy,
                "present_speed": current_velocity,
                "left_arm_state": left_arm_state,
                "right_arm_state": right_arm_state,
            }
            # Send the observation over the video socket.
            video_socket.send_string(json.dumps(observation))

            # Ensure a short sleep to avoid overloading the CPU.
            elapsed = time.time() - loop_start_time
            time.sleep(
                max(0.033 - elapsed, 0)
            )  # If robot jitters increase the sleep and monitor cpu load with `top` in cmd
    except KeyboardInterrupt:
        print("Shutting down SourcceyV1Beta server.")
    finally:
        stop_event.set()
        cam_thread.join()
        robot.stop()
        print("Motors stopped.")
        left_motors_bus.disconnect()
        right_motors_bus.disconnect()
        cmd_socket.close()
        video_socket.close()
        context.term()
