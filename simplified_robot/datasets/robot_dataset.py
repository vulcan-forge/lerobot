import torch
from torch.utils.data import Dataset
import pandas as pd
import numpy as np
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
        
        # Convert the array data to numpy arrays first
        states = np.stack(self.data['observation.state'].values)
        actions = np.stack(self.data['action'].values)
        
        # Calculate statistics
        self.state_mean = states.mean(axis=0)
        self.state_std = states.std(axis=0)
        self.action_mean = actions.mean(axis=0)
        self.action_std = actions.std(axis=0)
        
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        # Get current state and action
        current_state = np.array(self.data.iloc[idx]['observation.state'])
        current_action = np.array(self.data.iloc[idx]['action'])
        
        # Convert to tensors and normalize
        current_state = torch.tensor(
            (current_state - self.state_mean) / (self.state_std + 1e-8),
            dtype=torch.float32
        )
        current_action = torch.tensor(
            (current_action - self.action_mean) / (self.action_std + 1e-8),
            dtype=torch.float32
        )
        
        return {
            'state': current_state,
            'action': current_action
        } 