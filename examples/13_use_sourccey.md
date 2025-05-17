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
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a towel with sourccey and fold it." \
  --control.repo_id=local/sourccey_v1beta_towel_subtask_002 \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=500 \
  --control.reset_time_s=10 \
  --control.num_episodes=1 \
  --control.push_to_hub=false \
  --control.resume=true
```

```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=local/sourccey_v1beta_e50_shirt_001 \
  --policy.type=act \
  --output_dir=outputs/train/act_sourccey_v1beta_e50_shirt_001 \
  --job_name=act_sourccey_v1beta_e50_shirt_001 \
  --policy.device=cuda \
  --wandb.enable=false \
  --steps=200000
```

```
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a towel with sourccey and attempt to fold it." \
  --control.repo_id=local/eval_act_sourccey_v1beta_towel_subtask_002 \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=500 \
  --control.reset_time_s=30 \
  --control.num_episodes=1 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/train/act_sourccey_v1beta_towel_subtask_002/checkpoints/100000/pretrained_model
```

---

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=teleoperate \
  --control.fps=30 \
  --control.display_data=true \
  --control.arm_keyboard_control=true
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
  --control.arms='["left_leader", "right_leader"]'
```

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.arms='["main_follower"]'
```

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.arms='["main_leader"]'
```

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=teleoperate \
  --control.fps=30
```

--/dev/ttyGS0

```
sudo iptables -I DOCKER-USER -p tcp --dport 5555 -j ACCEPT
sudo iptables -I DOCKER-USER -p tcp --dport 5556 -j ACCEPT
```
