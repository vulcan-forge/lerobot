import torch
from torch.utils.data import Dataset
import pandas as pd
from pathlib import Path
import numpy as np

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
        
        # Calculate mean and std for normalization
        self.state_mean = self.data['observation.state'].apply(lambda x: np.array(x)).mean()
        self.state_std = self.data['observation.state'].apply(lambda x: np.array(x)).std()
        self.action_mean = self.data['action'].apply(lambda x: np.array(x)).mean()
        self.action_std = self.data['action'].apply(lambda x: np.array(x)).std()
        
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        # Get current state and action
        current_state = torch.tensor(self.data.iloc[idx]['observation.state'], dtype=torch.float32)
        current_action = torch.tensor(self.data.iloc[idx]['action'], dtype=torch.float32)
        
        # Normalize
        current_state = (current_state - self.state_mean) / (self.state_std + 1e-8)
        current_action = (current_action - self.action_mean) / (self.action_std + 1e-8)
        
        return {
            'state': current_state,
            'action': current_action
        } 