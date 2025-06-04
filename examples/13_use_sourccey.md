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
  --control.fps=20 \
  --control.display_data=true
```

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=record \
  --control.fps=20 \
  --control.single_task="Grasp a towel with sourccey and fold it." \
  --control.repo_id=local/sourccey_v1beta_towel_010_c \
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
  --dataset.repo_id=local/sourccey_v1beta_towel_010_a \
  --policy.type=act \
  --output_dir=outputs/train/act_sourccey_v1beta_towel_010_a \
  --job_name=act_sourccey_v1beta_towel_010_a \
  --policy.device=cuda \
  --policy.use_amp=true \
  --wandb.enable=false \
  --steps=100000
```

```bash
torchrun --nproc_per_node=2 lerobot/scripts/train.py \
  --dataset.repo_id=local/sourccey_v1beta_towel_010_a  \
  --policy.type=act \
  --output_dir=outputs/train/act_sourccey_v1beta_towel_010_a \
  --job_name=act_sourccey_v1beta_towel_010_a  \
  --policy.device=cuda \
  --policy.use_amp=true \
  --wandb.enable=false \
  --steps=100000 \
  --distributed_training=true \
  --num_gpus=2
```

---- Train on Pretrained SmolVLA model

```bash
CUDA_VISIBLE_DEVICES=0 python lerobot/scripts/train.py \
 --dataset.repo_id=local/sourccey_v1beta_towel_010_a \
 --policy.path=lerobot/smolvla_base \
 --policy.device=cuda \
 --output_dir=outputs/train/smolvla_base_sourccey_v1beta_towel_010_a \
 --job_name=smolvla_base_sourccey_v1beta_towel_010_a \
 --wandb.enable=false \
 --steps=100000 \
 --batch_size=4
```

```bash
torchrun --nproc_per_node=2 lerobot/scripts/train.py \
 --dataset.repo_id=local/sourccey_v1beta_towel_010_a \
 --policy.path=lerobot/smolvla_base \
 --policy.device=cuda \
 --output_dir=outputs/train/smolvla_sourccey_v1beta_towel_050_a \
 --job_name=smolvla_sourccey_v1beta_towel_050_a \
 --wandb.enable=false \
 --steps=100000 \
 --batch_size=32 \
 --distributed_training=true \
 --num_gpus=2
```

---

--steps=200000 \
 --batch_size=64 \

------ Train on 0 base SmolVLA:


<!-- ```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=local/sourccey_v1beta_towel_010_a \
  --policy.type=smolvla \
  --output_dir=outputs/train/smolvla_sourccey_v1beta_towel_010_a \
  --job_name=smolvla_sourccey_v1beta_towel_010_a \
  --policy.device=cuda \
  --wandb.enable=false \
  --steps=100000 \
  --batch_size=4
```

```bash
torchrun --nproc_per_node=2 lerobot/scripts/train.py \
  --dataset.repo_id=local/sourccey_v1beta_towel_010_a\
  --policy.type=smolvla \
  --output_dir=outputs/train/smolvla_sourccey_v1beta_towel_010_4_a \
  --job_name=smolvla_sourccey_v1beta_towel_050_a \
  --policy.device=cuda \
  --wandb.enable=false \
  --steps=200000 \
  --distributed_training=true \
  --num_gpus=2 \
  --batch_size=32
``` -->


----- Combine Dataset functions

```bash
python lerobot/scripts/combine_dataset.py \
    --repo_ids local/sourccey_v1beta_towel_010_a local/sourccey_v1beta_towel_010_b local/sourccey_v1beta_towel_010_2b local/sourccey_v1beta_towel_010_c local/sourccey_v1beta_towel_010_2c local/sourccey_v1beta_towel_010_3c \
    --output_repo_id=local/sourccey_v1beta_towel_010_a_2b_3c_combined \
    --push_to_hub=0
```

```bash
python lerobot/scripts/combine_dataset.py \
    --repo_ids local/sourccey_v1beta_towel_010_d local/sourccey_v1beta_towel_010_e \
    --output_repo_id=local/sourccey_v1beta_towel_010_d_e_combined \
    --push_to_hub=0
```

```
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=record \
  --control.fps=20 \
  --control.single_task="Grasp a towel with sourccey and attempt to fold it." \
  --control.repo_id=local/eval_smolvla_sourccey_v1beta_towel_050_a \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=500 \
  --control.reset_time_s=10 \
  --control.num_episodes=1 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/train/smolvla_sourccey_v1beta_towel_050_a/checkpoints/100000/pretrained_model \
  --control.resume=true
```

```
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=record \
  --control.fps=20 \
  --control.single_task="Grasp a towel with sourccey and attempt to fold it." \
  --control.repo_id=local/eval_act_sourccey_v1beta_towel_010_a \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=500 \
  --control.reset_time_s=10 \
  --control.num_episodes=1 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/train/act_sourccey_v1beta_towel_010_a/checkpoints/100000/pretrained_model \
  --control.resume=true
```

```
python lerobot/scripts/combine_dataset.py \
    --repo_ids local/sourccey_v1beta_towel_010_a local/sourccey_v1beta_towel_010_b local/sourccey_v1beta_towel_010_c local/sourccey_v1beta_towel_010_2c local/sourccey_v1beta_towel_010_3c local/sourccey_v1beta_towel_010_d local/sourccey_v1beta_towel_010_e  \
    --output_repo_id local/sourccey_v1beta_towel_010_a_2b_3c_d_e_combined \
    --push_to_hub 0
```

```
python lerobot/scripts/combine_dataset.py \
    --repo_ids local/sourccey_v1beta_towel_010_a local/sourccey_v1beta_towel_010_d  \
    --output_repo_id local/sourccey_v1beta_towel_010_a_d_combined \
    --push_to_hub 0
```

python lerobot/scripts/remove_episodes.py \
 --repo-id=local/sourccey_v1beta_towel_010_c \
 --episodes='5' \
 --push-to-hub=0

---

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=teleoperate \
  --control.fps=20 \
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
  --control.fps=20
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
  --control.fps=20 \
  --control.display_data=true
```

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=20 \
  --control.single_task="Grasp a towel with sourccey and fold it." \
  --control.repo_id=local/so100_a \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=500 \
  --control.reset_time_s=10 \
  --control.num_episodes=1 \
  --control.push_to_hub=false \
  --control.resume=true
```
