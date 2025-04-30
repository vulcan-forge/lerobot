import os
import time
import datetime
import torch.distributed as dist

def log(msg):
    rank = os.environ.get("RANK", "?")
    print(f"[Rank {rank}] {msg}", flush=True)

def main():
    rank = int(os.environ["RANK"])
    world_size = int(os.environ["WORLD_SIZE"])

    log("Initializing process group with env://")
    dist.init_process_group(
        backend="gloo",
        init_method="env://",
        rank=rank,
        world_size=world_size,
        timeout=datetime.timedelta(seconds=60)
    )

    log("Process group initialized successfully.")
    dist.barrier()
    log("Passed barrier ✅")

    dist.destroy_process_group()
    log("Done.")

if __name__ == "__main__":
    main()
