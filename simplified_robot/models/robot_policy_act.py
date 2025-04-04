import torch
import torch.nn as nn
import math

class MultiHeadAttention(nn.Module):
    def __init__(self, d_model, num_heads):
        super().__init__()
        assert d_model % num_heads == 0
        
        self.d_model = d_model
        self.num_heads = num_heads
        self.head_dim = d_model // num_heads
        
        self.qkv = nn.Linear(d_model, 3 * d_model)
        self.proj = nn.Linear(d_model, d_model)
        
    def forward(self, x, mask=None):
        batch_size, seq_len, d_model = x.shape
        
        # (batch, seq_len, 3 * d_model) -> 3 tensors of shape (batch, seq_len, d_model)
        qkv = self.qkv(x).chunk(3, dim=-1)
        q, k, v = map(lambda t: t.view(batch_size, seq_len, self.num_heads, self.head_dim).transpose(1, 2), qkv)
        
        # Scaled dot-product attention
        scores = torch.matmul(q, k.transpose(-2, -1)) / math.sqrt(self.head_dim)
        if mask is not None:
            scores = scores.masked_fill(mask == 0, float('-inf'))
        attn = torch.softmax(scores, dim=-1)
        
        # Combine heads
        out = torch.matmul(attn, v)
        out = out.transpose(1, 2).contiguous().view(batch_size, seq_len, d_model)
        return self.proj(out)

class TransformerBlock(nn.Module):
    def __init__(self, d_model, num_heads, mlp_dim, dropout=0.1):
        super().__init__()
        self.attention = MultiHeadAttention(d_model, num_heads)
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.mlp = nn.Sequential(
            nn.Linear(d_model, mlp_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(mlp_dim, d_model),
            nn.Dropout(dropout)
        )
        
    def forward(self, x, mask=None):
        x = x + self.attention(self.norm1(x), mask)
        x = x + self.mlp(self.norm2(x))
        return x

class RobotPolicyACT(nn.Module):
    def __init__(self, 
                 state_dim=6,
                 action_dim=6,
                 d_model=256,
                 num_heads=8,
                 num_layers=6,
                 mlp_dim=512,
                 dropout=0.1,
                 max_seq_len=32):
        super().__init__()
        
        self.state_embedding = nn.Sequential(
            nn.Linear(state_dim, d_model),
            nn.LayerNorm(d_model),
            nn.GELU(),
            nn.Dropout(dropout)
        )
        
        # Positional encoding
        self.pos_embedding = nn.Parameter(torch.zeros(1, max_seq_len, d_model))
        
        # Transformer blocks
        self.transformer_blocks = nn.ModuleList([
            TransformerBlock(d_model, num_heads, mlp_dim, dropout)
            for _ in range(num_layers)
        ])
        
        # Output head
        self.output_head = nn.Sequential(
            nn.LayerNorm(d_model),
            nn.Linear(d_model, mlp_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(mlp_dim, action_dim)
        )
        
        self.apply(self._init_weights)
        
    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            torch.nn.init.normal_(module.weight, mean=0.0, std=0.02)
            if module.bias is not None:
                torch.nn.init.zeros_(module.bias)
        elif isinstance(module, nn.LayerNorm):
            torch.nn.init.zeros_(module.bias)
            torch.nn.init.ones_(module.weight)
            
    def forward(self, batch):
        states = batch['state']  # Expected shape: (batch_size, seq_len, state_dim)
        
        # If states is 2D, add sequence dimension
        if len(states.shape) == 2:
            states = states.unsqueeze(1)
        
        x = self.state_embedding(states)
        x = x + self.pos_embedding[:, :x.size(1), :]
        
        # Apply transformer blocks
        for block in self.transformer_blocks:
            x = block(x)
            
        # Get predictions for each timestep
        predicted_actions = self.output_head(x)
        
        # If input was 2D, squeeze output back
        if len(batch['state'].shape) == 2:
            predicted_actions = predicted_actions.squeeze(1)
            
        # Calculate loss
        loss = nn.MSELoss()(predicted_actions, batch['action'])
        
        return loss, {'predicted_actions': predicted_actions} 