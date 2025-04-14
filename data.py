import torch

def generate_synthetic_data(num_samples):
    # Generate random input features
    X = torch.randn(num_samples, 10)
    
    # Generate random target values
    y = torch.randn(num_samples, 1)
    
    return X, y 