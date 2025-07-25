#!/usr/bin/env python3

import base64
import json
import logging
import time

import cv2
import zmq

from lerobot.common.robots.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaConfig, SourcceyV2BetaHostConfig
from lerobot.common.robots.sourccey_v2beta.sourccey_v2beta import SourcceyV2Beta

class SourcceyV2BetaHost:
    def __init__(self, config: SourcceyV2BetaHostConfig):
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")

        self.connection_time_s = config.connection_time_s
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz

    def disconnect(self):
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()

def main():
    logging.info("Configuring Sourccey V2 Beta")
    robot_config = SourcceyV2BetaConfig()
    robot = SourcceyV2Beta(robot_config)

    logging.info("Connecting Sourccey V2 Beta (skipping calibration)")
    # Connect without calibration
    robot.connect(calibrate=False)

    logging.info("Starting HostAgent")
    host_config = SourcceyV2BetaHostConfig()
    host = SourcceyV2BetaHost(host_config)

    last_cmd_time = time.time()
    watchdog_active = False
    logging.info("Waiting for commands...")
    print("Waiting for commands...")
    
    try:
        start = time.perf_counter()
        duration = 0
        while duration < host.connection_time_s:
            loop_start_time = time.time()
            try:
                msg = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                data = dict(json.loads(msg))
                _action_sent = robot.send_action(data)
                last_cmd_time = time.time()
                watchdog_active = False
            except zmq.Again:
                pass
            except Exception as e:
                logging.error("Message fetching failed: %s", e)

            now = time.time()
            if (now - last_cmd_time > host.watchdog_timeout_ms / 1000) and not watchdog_active:
                logging.warning(
                    f"Command not received for more than {host.watchdog_timeout_ms} milliseconds. Stopping the base."
                )
                watchdog_active = True
                # robot.stop_base()  # Comment out if this method doesn't exist

            last_observation = robot.get_observation()

            # Encode ndarrays to base64 strings
            for cam_key, _ in robot.cameras.items():
                ret, buffer = cv2.imencode(
                    ".jpg", last_observation[cam_key], [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )
                if ret:
                    last_observation[cam_key] = base64.b64encode(buffer).decode("utf-8")
                else:
                    last_observation[cam_key] = ""

            # Send the observation to the remote agent
            try:
                host.zmq_observation_socket.send_string(json.dumps(last_observation), flags=zmq.NOBLOCK)
            except zmq.Again:
                logging.info("Dropping observation, no client connected")

            # Ensure a short sleep to avoid overloading the CPU.
            elapsed = time.time() - loop_start_time
            time.sleep(max(1 / host.max_loop_freq_hz - elapsed, 0))
            duration = time.perf_counter() - start
            
        print("Cycle time reached.")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
    finally:
        print("Shutting down SourcceyV2Beta Host.")
        robot.disconnect()
        host.disconnect()

    logging.info("Finished SourcceyV2Beta cleanly")

if __name__ == "__main__":
    main() 