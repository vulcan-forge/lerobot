#!/bin/bash

# Permission has been automatically added from git:
# git update-index --chmod=+x --add start/start_teleop.sh
# git add start/start_teleop.sh
# git commit -m "Add teleop start script with executable permissions"

# Activate the virtual environment
source .venv/Scripts/activate

# Run the teleop script
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=teleoperate \
  --control.fps=30 \
  --control.display_data=true

