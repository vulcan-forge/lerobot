import torch
import torch.nn as nn

class RobotPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(6, 128),  # 6 state dimensions
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 6)     # 6 action dimensions
        )
    
    def forward(self, batch):
        states = batch['state']
        predicted_actions = self.net(states)
        loss = nn.MSELoss()(predicted_actions, batch['action'])
        return loss, {'predicted_actions': predicted_actions} 