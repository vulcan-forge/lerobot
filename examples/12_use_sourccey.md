To teleoperate SSH into your Raspberry Pi, and run `conda activate lerobot` and this script:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_vbeta \
  --control.type=remote_robot
```

Then on your laptop, also run `conda activate lerobot` and this script:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_vbeta \
  --control.type=teleoperate \
  --control.fps=30
```
