import torch
import torch.nn as nn
import numpy as np
from typing import List

class TrajectoryPredictorLSTM(nn.Module):
    def __init__(self, input_dim: int = 3, hidden_dim: int = 64, output_dim: int = 3, num_layers: int = 2):
        super(TrajectoryPredictorLSTM, self).__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_dim, output_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_dim).to(x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_dim).to(x.device)
        
        out, _ = self.lstm(x, (h0, c0))
        out = self.fc(out[:, -1, :])
        return out

    def extrapolate(self, history: List[np.ndarray], steps_forward: int = 10) -> np.ndarray:
        if len(history) < 5:
            return history[-1] if len(history) > 0 else np.zeros(3)
            
        seq = np.array(history[-10:])
        tensor_seq = torch.tensor(seq, dtype=torch.float32).unsqueeze(0)
        
        self.eval()
        with torch.no_grad():
            pred = self.forward(tensor_frame=tensor_seq)
            
        final_pred = pred.numpy()[0]
        direction = final_pred - history[-1]
        return history[-1] + (direction * (steps_forward / 2.0))