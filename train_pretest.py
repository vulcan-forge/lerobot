import os
import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.utils.data.distributed import DistributedSampler
import argparse
from model import SimpleModel
from data import generate_synthetic_data
import time
import datetime
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset

class SyntheticDataset(Dataset):
    def __init__(self, num_samples):
        X, y = generate_synthetic_data(num_samples)
        self.X = X
        self.y = y
        
    def __len__(self):
        return len(self.X)
    
    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

def print_system_info(rank, world_size):
    print(f"\n[Node {rank}] System Information:")
    print(f"  - Hostname: {os.uname().nodename}")
    print(f"  - CUDA available: {torch.cuda.is_available()}")
    
    if torch.cuda.is_available():
        print(f"  - CUDA version: {torch.version.cuda}")
        print(f"  - PyTorch version: {torch.__version__}")
        print(f"  - GPU count: {torch.cuda.device_count()}")
        for i in range(torch.cuda.device_count()):
            print(f"  - GPU {i}: {torch.cuda.get_device_name(i)}")
            print(f"    - Memory allocated: {torch.cuda.memory_allocated(i) / 1024**2:.2f} MB")
            print(f"    - Memory cached: {torch.cuda.memory_reserved(i) / 1024**2:.2f} MB")
    else:
        print("  - No CUDA devices available!")
    
    print(f"\n[Node {rank}] Network Information:")
    print(f"  - MASTER_ADDR: {os.environ.get('MASTER_ADDR', 'Not set')}")
    print(f"  - MASTER_PORT: {os.environ.get('MASTER_PORT', 'Not set')}")
    print(f"  - RANK: {rank}")
    print(f"  - WORLD_SIZE: {world_size}")

def setup_ddp(rank, world_size):
    print(f"\n[Node {rank}] Setting up DDP with rank {rank} and world_size {world_size}")
    os.environ['MASTER_ADDR'] = '192.168.1.155'  # Changed to your computer's IP
    os.environ['MASTER_PORT'] = '29500'
    
    # Print system information before attempting connection
    print_system_info(rank, world_size)
    
    try:
        print(f"\n[Node {rank}] Attempting to initialize process group...")
        dist.init_process_group(
            "nccl",
            rank=rank,
            world_size=world_size,
            timeout=datetime.timedelta(seconds=60)
        )
        print(f"[Node {rank}] Process group initialized successfully")
        
        # Test GPU communication
        if torch.cuda.is_available():
            print(f"\n[Node {rank}] Testing GPU communication...")
            tensor = torch.tensor([1.0], device=f"cuda:{0}")
            if rank == 0:
                print(f"[Node {rank}] Sending tensor to rank 1")
                dist.send(tensor, dst=1)
                print(f"[Node {rank}] Tensor sent successfully")
            else:
                print(f"[Node {rank}] Receiving tensor from rank 0")
                dist.recv(tensor, src=0)
                print(f"[Node {rank}] Tensor received successfully")
            
            print(f"[Node {rank}] GPU communication test successful")
        
        print(f"\n[Node {rank}] All setup completed successfully")
    except Exception as e:
        print(f"\n[Node {rank}] Setup failed with error: {str(e)}")
        raise e

def cleanup():
    if dist.is_initialized():
        dist.destroy_process_group()

def train(rank, world_size, args):
    print(f"\n[Node {rank}] Starting training on rank {rank}")
    setup_ddp(rank, world_size)
    
    try:
        # Create model and move it to GPU
        local_rank = 0
        torch.cuda.set_device(local_rank)
        model = SimpleModel().to(local_rank)
        model = DDP(model, device_ids=[local_rank], output_device=local_rank)
        print(f"[Node {rank}] Model created and moved to GPU {local_rank}")
        
        # Create data loaders with distributed sampler
        train_dataset = SyntheticDataset(args.num_samples)
        train_sampler = DistributedSampler(train_dataset, num_replicas=world_size, rank=rank)
        train_loader = DataLoader(train_dataset, batch_size=args.batch_size, sampler=train_sampler)
        print(f"[Node {rank}] Data loaders created with distributed sampler")
        print(f"[Node {rank}] Dataset size: {len(train_dataset)}, Sampler size: {len(train_sampler)}")
        
        optimizer = optim.Adam(model.parameters(), lr=args.learning_rate)
        
        for epoch in range(args.epochs):
            train_sampler.set_epoch(epoch)
            print(f"\n[Node {rank}] Starting epoch {epoch}")
            print(f"[Node {rank}] Sampler size for epoch {epoch}: {len(train_sampler)}")
            
            model.train()
            total_samples = 0
            for batch_idx, (data, target) in enumerate(train_loader):
                data, target = data.to(local_rank), target.to(local_rank)
                total_samples += len(data)
                
                optimizer.zero_grad()
                output = model(data)
                loss = F.mse_loss(output, target)
                loss.backward()
                optimizer.step()
                
                if batch_idx % args.log_interval == 0:
                    print(f"[Node {rank}] Train Epoch: {epoch} [{total_samples}/{len(train_loader.dataset)} ({100. * total_samples / len(train_loader.dataset):.0f}%)]\tLoss: {loss.item():.6f}")
                    print(f"[Node {rank}] Data shape: {data.shape}, Target shape: {target.shape}")
                    print(f"[Node {rank}] Batch {batch_idx + 1}/{len(train_loader)}")
            
            print(f"[Node {rank}] Completed epoch {epoch}")
            print(f"[Node {rank}] Total samples processed: {total_samples}")
            
            # Synchronize after each epoch
            dist.barrier()
            
    except Exception as e:
        print(f"[Node {rank}] Training failed with error: {str(e)}")
        raise e
    finally:
        dist.destroy_process_group()

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