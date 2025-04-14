#!/usr/bin/env python

import os
import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.utils.data.distributed import DistributedSampler
import argparse
import logging
from pathlib import Path
import torch.multiprocessing as mp
from itertools import cycle

from simplified_robot.datasets.robot_dataset import RobotDataset
from simplified_robot.models.robot_policy import RobotPolicy
from simplified_robot.models.robot_policy_act import RobotPolicyACT

def setup_distributed(rank, world_size, master_addr=None, master_port=None):
    """Initialize distributed training."""
    logging.info(f"Setting up distributed training for rank {rank}/{world_size}")
    
    # Set environment variables
    os.environ['MASTER_ADDR'] = master_addr or '192.168.1.40'  # Default to master machine's IP
    os.environ['MASTER_PORT'] = master_port or '29500'
    
    # Initialize process group
    dist.init_process_group(
        backend="nccl",  # Using NCCL for GPU
        init_method=f"tcp://{os.environ['MASTER_ADDR']}:{os.environ['MASTER_PORT']}",
        world_size=world_size,
        rank=rank
    )
    
    # Set device
    torch.cuda.set_device(0)  # Each machine uses its first GPU
    device = torch.device("cuda:0")
    
    return device

def cleanup():
    if dist.is_initialized():
        dist.destroy_process_group()

def train(rank, world_size, args):
    device = setup_distributed(rank, world_size, args.master_addr, args.master_port)
    
    try:
        logging.info(f"[Rank {rank}] Starting data loading")
        dataset = RobotDataset(args.data_root)
        sampler = DistributedSampler(dataset, num_replicas=world_size, rank=rank)
        dataloader = torch.utils.data.DataLoader(
            dataset,
            batch_size=args.batch_size,
            sampler=sampler,
            pin_memory=True
        )
        # Create infinite data loader like in train.py
        dl_iter = iter(cycle(dataloader))
        
        logging.info(f"[Rank {rank}] Dataset size: {len(dataset)}, Batch size: {args.batch_size}")
        
        logging.info(f"[Rank {rank}] Creating model on {device}")
        if args.model == 'act':
            policy = RobotPolicyACT().to(device)
        else:
            policy = RobotPolicy().to(device)
        policy = DDP(policy, device_ids=[0])
        
        optimizer = torch.optim.Adam(policy.parameters(), lr=args.learning_rate)
        
        logging.info(f"[Rank {rank}] Starting training")
        step = 0
        total_loss = 0
        batch_count = 0
        avg_loss = 0  # Initialize avg_loss
        
        while step < args.steps:
            # Get next batch from infinite iterator
            batch = next(dl_iter)
            batch = {k: v.to(device) for k, v in batch.items()}
            
            optimizer.zero_grad()
            loss, output_dict = policy(batch)
            loss.backward()
            
            torch.nn.utils.clip_grad_norm_(policy.parameters(), max_norm=1.0)
            optimizer.step()
            
            total_loss += loss.item()
            batch_count += 1
            step += 1
            
            # Calculate avg_loss every time for checkpointing
            avg_loss = total_loss / batch_count
            
            if step % args.log_interval == 0:
                logging.info(f"[Rank {rank}] Step: {step}/{args.steps}, Loss: {loss.item():.4f}, Avg Loss: {avg_loss:.4f}")
            
            # Synchronize periodically
            if step % args.sync_interval == 0:
                dist.barrier()
                if rank == 0:
                    logging.info(f"All ranks completed step {step}")
                    
                    # Only save checkpoint at checkpoint_interval
                    if args.checkpoint_interval > 0 and step % args.checkpoint_interval == 0:
                        if args.save_dir:
                            save_dir = Path(args.save_dir)
                            save_dir.mkdir(parents=True, exist_ok=True)
                            torch.save({
                                'step': step,
                                'model_state_dict': policy.state_dict(),
                                'optimizer_state_dict': optimizer.state_dict(),
                                'loss': avg_loss,
                            }, save_dir / f'checkpoint_step_{step}.pt')

    except Exception as e:
        logging.error(f"Rank {rank} failed with error: {str(e)}")
        raise e
    finally:
        cleanup()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rank', type=int, required=True)
    parser.add_argument('--world_size', type=int, required=True)
    parser.add_argument('--master_addr', type=str, default='192.168.1.155')
    parser.add_argument('--master_port', type=str, default='29500')
    parser.add_argument('--data_root', type=str, default='/mnt/robotdata')
    parser.add_argument('--batch_size', type=int, default=32)
    parser.add_argument('--steps', type=int, default=18000)  # Instead of epochs
    parser.add_argument('--learning_rate', type=float, default=0.0001)
    parser.add_argument('--log_interval', type=int, default=200)
    parser.add_argument('--sync_interval', type=int, default=100)  # How often to sync between processes
    parser.add_argument('--save_dir', type=str, default='checkpoints')
    parser.add_argument('--model', type=str, default='simple', choices=['simple', 'act'],
                       help='Which model to use: simple (default) or act (transformer)')
    parser.add_argument('--checkpoint_interval', type=int, default=10000, 
                       help='Save checkpoints every N steps. Set to 0 to disable checkpoints.')
    
    args = parser.parse_args()
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    train(args.rank, args.world_size, args)

if __name__ == "__main__":
    main() 