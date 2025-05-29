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
  --control.repo_id=local/sourccey_v1beta_towel_014 \
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
  --dataset.repo_id=local/sourccey_v1beta_towel_014 \
  --policy.type=act \
  --output_dir=outputs/train/act_sourccey_v1beta_towel_014 \
  --job_name=act_sourccey_v1beta_towel_014 \
  --policy.device=cuda \
  --policy.use_amp=true \
  --wandb.enable=false \
  --steps=100000
```

```bash
torchrun --nproc_per_node=2 lerobot/scripts/train.py \
  --dataset.repo_id=local/sourccey_v1beta_towel_014 \
  --policy.type=act \
  --output_dir=outputs/train/act_sourccey_v1beta_towel_014 \
  --job_name=act_sourccey_v1beta_towel_014 \
  --policy.device=cuda \
  --policy.use_amp=true \
  --wandb.enable=false \
  --steps=100000 \
  --distributed_training=true \
  --num_gpus=2
```

```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=local/sourccey_v1beta_towel_010 \
  --policy.type=pi0 \
  --output_dir=outputs/train/pi0_sourccey_v1beta_towel_010 \
  --job_name=pi0_sourccey_v1beta_towel_010 \
  --policy.device=cuda \
  --policy.use_amp=true \
  --wandb.enable=false \
  --steps=100000 \
  --batch_size=2
```

```bash
torchrun --nproc_per_node=2 lerobot/scripts/train.py \
  --dataset.repo_id=local/sourccey_v1beta_towel_010 \
  --policy.type=pi0 \
  --output_dir=outputs/train/pi0_sourccey_v1beta_towel_010 \
  --job_name=pi0_sourccey_v1beta_towel_010 \
  --policy.device=cuda \
  --policy.use_amp=true \
  --wandb.enable=false \
  --steps=100000 \
  --batch_size=4 \
  --distributed_training=true \
  --num_gpus=2
```

```
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a towel with sourccey and attempt to fold it." \
  --control.repo_id=local/eval_act_sourccey_v1beta_towel_012 \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=500 \
  --control.reset_time_s=10 \
  --control.num_episodes=1 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/train/act_sourccey_v1beta_towel_012/checkpoints/100000/pretrained_model \
  --control.resume=true
```

```
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a towel with sourccey and attempt to fold it." \
  --control.repo_id=local/eval_pi0_sourccey_v1beta_towel_005 \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=500 \
  --control.reset_time_s=10 \
  --control.num_episodes=1 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/train/pi0_sourccey_v1beta_towel_010/checkpoints/100000/pretrained_model \
  --control.resume=true
```

python lerobot/scripts/remove_episodes.py \
 --repo-id=local/sourccey_v1beta_towel_006 \
 --episodes='12-16' \
 --push-to-hub=0

---

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

---

SO100

---

```
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=teleoperate \
  --control.fps=30 \
  --control.display_data=true
```
