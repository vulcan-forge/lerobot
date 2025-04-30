#!/usr/bin/env python

import os
import torch
import torch.nn as nn
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.utils.data import Dataset, DataLoader
from torch.utils.data.distributed import DistributedSampler
import argparse
import logging
from datetime import datetime

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class SimpleModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(10, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1)
        )
    
    def forward(self, x):
        return self.net(x)

class SyntheticDataset(Dataset):
    def __init__(self, num_samples=1000):
        # Generate synthetic data
        self.data = torch.randn(num_samples, 10)
        self.targets = torch.randn(num_samples, 1)
    
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        return self.data[idx], self.targets[idx]

def setup_distributed(rank, world_size):
    """Initialize distributed training."""
    logging.info(f"Setting up distributed training for rank {rank}/{world_size}")
    
    # Set environment variables
    os.environ['MASTER_ADDR'] = 'localhost'
    os.environ['MASTER_PORT'] = '29500'
    
    # Initialize process group
    dist.init_process_group(
        backend="nccl",
        init_method=f"tcp://{os.environ['MASTER_ADDR']}:{os.environ['MASTER_PORT']}",
        world_size=world_size,
        rank=rank,
        timeout=datetime.timedelta(seconds=60)
    )
    
    # Set device
    torch.cuda.set_device(rank)
    device = torch.device(f"cuda:{rank}")
    
    return device

def cleanup():
    """Clean up distributed training."""
    if dist.is_initialized():
        dist.destroy_process_group()

def train(rank, world_size, args):
    """Main training function."""
    try:
        # Set up distributed training
        device = setup_distributed(rank, world_size)
        logging.info(f"Rank {rank}: Using device {device}")
        
        # Create model
        model = SimpleModel().to(device)
        model = DDP(model, device_ids=[rank])
        logging.info(f"Rank {rank}: Model created and wrapped in DDP")
        
        # Create dataset and dataloader
        dataset = SyntheticDataset(num_samples=args.num_samples)
        sampler = DistributedSampler(dataset, num_replicas=world_size, rank=rank)
        dataloader = DataLoader(
            dataset,
            batch_size=args.batch_size,
            sampler=sampler,
            num_workers=0,
            pin_memory=True
        )
        logging.info(f"Rank {rank}: Dataset and dataloader created")
        
        # Create optimizer
        optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
        
        # Training loop
        for epoch in range(args.epochs):
            sampler.set_epoch(epoch)
            model.train()
            
            for batch_idx, (data, target) in enumerate(dataloader):
                data, target = data.to(device), target.to(device)
                
                optimizer.zero_grad()
                output = model(data)
                loss = nn.MSELoss()(output, target)
                loss.backward()
                optimizer.step()
                
                if batch_idx % args.log_interval == 0:
                    logging.info(f"Rank {rank} - Epoch {epoch} - Batch {batch_idx} - Loss: {loss.item():.6f}")
            
            # Synchronize after each epoch
            dist.barrier()
            logging.info(f"Rank {rank}: Completed epoch {epoch}")
    
    except Exception as e:
        logging.error(f"Rank {rank} failed with error: {str(e)}")
        raise e
    finally:
        cleanup()

def main():
    parser = argparse.ArgumentParser(description='Distributed Training')
    parser.add_argument('--batch_size', type=int, default=32)
    parser.add_argument('--epochs', type=int, default=10)
    parser.add_argument('--learning_rate', type=float, default=0.001)
    parser.add_argument('--num_samples', type=int, default=1000)
    parser.add_argument('--log_interval', type=int, default=10)
    parser.add_argument('--rank', type=int, required=True)
    parser.add_argument('--world_size', type=int, required=True)
    
    args = parser.parse_args()
    train(args.rank, args.world_size, args)

if __name__ == "__main__":
    main() 