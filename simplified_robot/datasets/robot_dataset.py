import torch
from torch.utils.data import Dataset
import pandas as pd
from pathlib import Path

class RobotDataset(Dataset):
    def __init__(self, root_dir):
        self.root_dir = Path(root_dir)
        self.data_frames = []
        
        # Load all parquet files from chunks
        chunk_dir = self.root_dir / 'data' / 'chunk-000'
        for parquet_file in chunk_dir.glob('*.parquet'):
            df = pd.read_parquet(parquet_file)
            self.data_frames.append(df)
        
        self.data = pd.concat(self.data_frames, ignore_index=True)
        
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        # Get current state and action
        current_state = torch.tensor(self.data.iloc[idx][['observation.state']].values[0], dtype=torch.float32)
        current_action = torch.tensor(self.data.iloc[idx][['action']].values[0], dtype=torch.float32)
        
        return {
            'state': current_state,
            'action': current_action
        } 