import torch.multiprocessing as mp
from simplified_robot.scripts.train_distributed import train
import argparse
import tempfile
import shutil
from pathlib import Path
import logging

def main():
    # Create base directory for saved models inside simplified_robot
    base_dir = Path(__file__).parent.parent
    saved_models_dir = base_dir / "saved_models"
    saved_models_dir.mkdir(exist_ok=True)
    
    # Create temporary directory for training artifacts
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        checkpoints_dir = temp_path / "checkpoints"
        checkpoints_dir.mkdir(exist_ok=True)
        
        parser = argparse.ArgumentParser()
        parser.add_argument('--data_root', type=str, default='/mnt/so100_test3')
        parser.add_argument('--batch_size', type=int, default=32)
        parser.add_argument('--epochs', type=int, default=1)
        parser.add_argument('--learning_rate', type=float, default=0.001)
        parser.add_argument('--log_interval', type=int, default=10)
        parser.add_argument('--save_dir', type=str, default=str(checkpoints_dir))
        
        args = parser.parse_args()
        
        logging.info(f"Training artifacts will be saved temporarily to: {args.save_dir}")
        logging.info(f"Successful models will be saved to: {saved_models_dir}")
        
        try:
            # Launch two processes
            world_size = 2
            mp.spawn(
                train,
                args=(world_size, args),
                nprocs=world_size,
                join=True
            )
            
            # If training successful, save the final model
            checkpoints = sorted(checkpoints_dir.glob("checkpoint_epoch_*.pt"))
            if checkpoints:
                last_checkpoint = checkpoints[-1]
                shutil.copy2(
                    last_checkpoint, 
                    saved_models_dir / f"test_model_{last_checkpoint.name}"
                )
                logging.info(f"Saved final model to: {saved_models_dir / f'test_model_{last_checkpoint.name}'}")
            
        except Exception as e:
            logging.error(f"Training failed with error: {str(e)}")
            raise e
        finally:
            logging.info("Temporary training directory will be cleaned up automatically")

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    main() 