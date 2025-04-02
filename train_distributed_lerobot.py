#!/usr/bin/env python

import os
import logging
import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.utils.data.distributed import DistributedSampler
from contextlib import nullcontext

from lerobot.common.datasets.factory import make_dataset
from lerobot.common.policies.factory import make_policy
from lerobot.common.optim.factory import make_optimizer_and_scheduler
from lerobot.common.envs.factory import make_env
from lerobot.common.utils.logging_utils import AverageMeter, MetricsTracker
from lerobot.common.utils.utils import get_safe_torch_device, init_logging
from lerobot.configs import parser
from lerobot.configs.train import TrainPipelineConfig
from lerobot.scripts.eval import eval_policy

from lerobot.common.utils.train_utils import (
    get_step_checkpoint_dir,
    get_step_identifier,
    load_training_state,
    save_checkpoint,
    update_last_checkpoint,
)

def setup_distributed(rank, world_size):
    """Initialize distributed training."""
    logging.info(f"Setting up distributed training for rank {rank}/{world_size}")
    
    # Set environment variables
    os.environ['MASTER_ADDR'] = '192.168.1.155'  # Master node IP
    os.environ['MASTER_PORT'] = '29500'
    
    # Initialize process group
    dist.init_process_group(
        backend="nccl",
        init_method=f"tcp://{os.environ['MASTER_ADDR']}:{os.environ['MASTER_PORT']}",
        world_size=world_size,
        rank=rank
    )
    
    # Set device
    torch.cuda.set_device(rank)
    device = torch.device(f"cuda:{rank}")
    
    return device

def cleanup():
    """Clean up distributed training."""
    if dist.is_initialized():
        dist.destroy_process_group()

def update_policy_distributed(
    train_metrics,
    policy,
    batch,
    optimizer,
    grad_clip_norm,
    grad_scaler,
    lr_scheduler=None,
    use_amp: bool = False,
):
    device = next(policy.parameters()).device
    policy.train()
    
    with torch.autocast(device_type=device.type) if use_amp else nullcontext():
        loss, output_dict = policy.forward(batch)

    grad_scaler.scale(loss).backward()
    grad_scaler.unscale_(optimizer)

    grad_norm = torch.nn.utils.clip_grad_norm_(
        policy.parameters(),
        grad_clip_norm,
        error_if_nonfinite=False,
    )

    grad_scaler.step(optimizer)
    grad_scaler.update()
    optimizer.zero_grad()

    if lr_scheduler is not None:
        lr_scheduler.step()

    train_metrics.loss = loss.item()
    train_metrics.grad_norm = grad_norm.item()
    train_metrics.lr = optimizer.param_groups[0]["lr"]
    
    return train_metrics, output_dict

@parser.wrap()
def train_distributed(rank, world_size, cfg: TrainPipelineConfig):
    try:
        # Set up distributed training
        device = setup_distributed(rank, world_size)
        cfg.validate()
        
        if rank == 0:
            logging.info(f"Creating dataset on rank {rank}")
        dataset = make_dataset(cfg)

        # Create distributed sampler
        sampler = DistributedSampler(dataset, num_replicas=world_size, rank=rank)
        
        dataloader = torch.utils.data.DataLoader(
            dataset,
            num_workers=cfg.num_workers,
            batch_size=cfg.batch_size,
            sampler=sampler,
            pin_memory=True,
        )

        # Create policy and wrap in DDP
        if rank == 0:
            logging.info("Creating policy")
        policy = make_policy(cfg=cfg.policy, ds_meta=dataset.meta)
        policy = policy.to(device)
        policy = DDP(policy, device_ids=[rank])

        # Create optimizer and scheduler
        optimizer, lr_scheduler = make_optimizer_and_scheduler(cfg, policy)
        grad_scaler = torch.cuda.amp.GradScaler(enabled=cfg.policy.use_amp)

        step = 0
        if cfg.resume:
            step, optimizer, lr_scheduler = load_training_state(
                cfg.checkpoint_path, optimizer, lr_scheduler
            )

        train_metrics = {
            "loss": AverageMeter("loss", ":.3f"),
            "grad_norm": AverageMeter("grdn", ":.3f"),
            "lr": AverageMeter("lr", ":0.1e"),
        }

        train_tracker = MetricsTracker(
            cfg.batch_size, 
            dataset.num_frames, 
            dataset.num_episodes, 
            train_metrics, 
            initial_step=step
        )

        if rank == 0:
            logging.info("Start distributed training")
            
        for epoch in range(cfg.steps):
            sampler.set_epoch(epoch)
            
            for batch in dataloader:
                # Move batch to device
                for key in batch:
                    if isinstance(batch[key], torch.Tensor):
                        batch[key] = batch[key].to(device, non_blocking=True)

                train_tracker, output_dict = update_policy_distributed(
                    train_tracker,
                    policy,
                    batch,
                    optimizer,
                    cfg.optimizer.grad_clip_norm,
                    grad_scaler=grad_scaler,
                    lr_scheduler=lr_scheduler,
                    use_amp=cfg.policy.use_amp,
                )

                step += 1
                train_tracker.step()

                # Logging and checkpointing only on rank 0
                if rank == 0:
                    is_log_step = cfg.log_freq > 0 and step % cfg.log_freq == 0
                    is_saving_step = step % cfg.save_freq == 0 or step == cfg.steps

                    if is_log_step:
                        logging.info(f"Step {step}: {train_tracker}")
                        train_tracker.reset_averages()

                    if cfg.save_checkpoint and is_saving_step:
                        logging.info(f"Checkpoint policy after step {step}")
                        checkpoint_dir = get_step_checkpoint_dir(cfg.output_dir, cfg.steps, step)
                        save_checkpoint(checkpoint_dir, step, cfg, policy, optimizer, lr_scheduler)
                        update_last_checkpoint(checkpoint_dir)

                if step >= cfg.steps:
                    break

            # Synchronize at the end of each epoch
            dist.barrier()

    except Exception as e:
        logging.error(f"Rank {rank} failed with error: {str(e)}")
        raise e
    finally:
        cleanup()

def main():
    init_logging()
    
    # Add distributed arguments to the parser
    parser.add_argument('--rank', type=int, required=True)
    parser.add_argument('--world_size', type=int, required=True)
    
    args = parser.parse_args()
    cfg = TrainPipelineConfig.from_args(args)
    
    train_distributed(args.rank, args.world_size, cfg)

if __name__ == "__main__":
    main() 