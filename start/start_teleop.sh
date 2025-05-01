# Activate the virtual environment
source .venv/Scripts/activate

# Run the teleop script
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=teleoperate \
  --control.fps=30 \
  --control.display_data=true

