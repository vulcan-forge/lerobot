To teleoperate SSH into your Raspberry Pi, and run `conda activate lerobot` and this script:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=remote_robot
```

Then on your laptop, also run `conda activate lerobot` and this script:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=teleoperate \
  --control.fps=30 \
  --control.display_data=true
```

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.arms='["left_follower", "right_follower"]'
```

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.arms='["right_follower"]'
```

--/dev/ttyGS0

```
sudo iptables -I DOCKER-USER -p tcp --dport 5555 -j ACCEPT
sudo iptables -I DOCKER-USER -p tcp --dport 5556 -j ACCEPT
```
