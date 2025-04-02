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

from simplified_robot.datasets.robot_dataset import RobotDataset
from simplified_robot.models.robot_policy import RobotPolicy

def setup_distributed(rank, world_size):
    """Initialize distributed training."""
    logging.info(f"Setting up distributed training for rank {rank}/{world_size}")
    
    # Set environment variables
    os.environ['MASTER_ADDR'] = '192.168.1.155'  # Master machine's IP
    os.environ['MASTER_PORT'] = '29500'
    
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
    device = setup_distributed(rank, world_size)
    
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
        logging.info(f"[Rank {rank}] Dataset size: {len(dataset)}, Batch size: {args.batch_size}")
        
        logging.info(f"[Rank {rank}] Creating model on {device}")
        policy = RobotPolicy().to(device)
        policy = DDP(policy, device_ids=[0])
        
        optimizer = torch.optim.Adam(policy.parameters(), lr=args.learning_rate)
        
        logging.info(f"[Rank {rank}] Starting training")
        for epoch in range(args.epochs):
            sampler.set_epoch(epoch)
            total_loss = 0
            batch_count = 0
            
            for batch_idx, batch in enumerate(dataloader):
                batch = {k: v.to(device) for k, v in batch.items()}
                
                optimizer.zero_grad()
                loss, output_dict = policy(batch)
                loss.backward()
                
                # Add gradient clipping
                torch.nn.utils.clip_grad_norm_(policy.parameters(), max_norm=1.0)
                
                optimizer.step()
                
                total_loss += loss.item()
                batch_count += 1
                
                if batch_idx % args.log_interval == 0:
                    logging.info(f"[Rank {rank}] Epoch: {epoch}, Batch: {batch_idx}, Loss: {loss.item():.4f}")
            
            avg_loss = total_loss / batch_count
            logging.info(f"[Rank {rank}] Completed epoch {epoch}, Average loss: {avg_loss:.4f}")
            
            # Synchronize at epoch end
            dist.barrier()
            if rank == 0:
                logging.info(f"All ranks completed epoch {epoch}")
                
                # Save checkpoint
                if args.save_dir:
                    save_dir = Path(args.save_dir)
                    save_dir.mkdir(parents=True, exist_ok=True)
                    torch.save({
                        'epoch': epoch,
                        'model_state_dict': policy.state_dict(),
                        'optimizer_state_dict': optimizer.state_dict(),
                        'loss': avg_loss,
                    }, save_dir / f'checkpoint_epoch_{epoch}.pt')
    
    except Exception as e:
        logging.error(f"Rank {rank} failed with error: {str(e)}")
        raise e
    finally:
        cleanup()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rank', type=int, required=True)
    parser.add_argument('--world_size', type=int, required=True)
    parser.add_argument('--data_root', type=str, default='/mnt/so100_test3')
    parser.add_argument('--batch_size', type=int, default=32)
    parser.add_argument('--epochs', type=int, default=10)
    parser.add_argument('--learning_rate', type=float, default=0.0001)
    parser.add_argument('--log_interval', type=int, default=10)
    parser.add_argument('--save_dir', type=str, default='checkpoints')
    
    args = parser.parse_args()
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    train(args.rank, args.world_size, args)

if __name__ == "__main__":
    main() 