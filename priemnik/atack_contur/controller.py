import torch
import torch.nn as nn
import numpy as np
from typing import Dict

class AlphaSelectorNetwork(nn.Module):
    def __init__(self, input_dim: int = 4):
        super(AlphaSelectorNetwork, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, 16),
            nn.ReLU(),
            nn.Linear(16, 8),
            nn.ReLU(),
            nn.Linear(8, 1),
            nn.Sigmoid()
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.network(x)


class NavigationController:
    def __init__(self, K_p: float = 2.5):
        self.K_p = K_p
        self.alpha_model = AlphaSelectorNetwork()

    def _path_following_law(self, pos: np.ndarray, vel: np.ndarray, target: np.ndarray) -> np.ndarray:
        desired_direction = target - pos
        norm = np.linalg.norm(desired_direction)
        if norm > 0:
            desired_direction /= norm
        desired_velocity = desired_direction * np.linalg.norm(vel)
        return desired_velocity - vel

    def _obstacle_avoidance_law(self, pos: np.ndarray, vel: np.ndarray, obstacle: np.ndarray) -> np.ndarray:
        repulsion_vector = pos - obstacle
        dist = np.linalg.norm(repulsion_vector)
        if dist > 0:
            repulsion_vector /= (dist ** 2)
        return repulsion_vector * self.K_p

    def get_control_command(self, pos: np.ndarray, vel: np.ndarray, target: np.ndarray, 
                            obstacle: np.ndarray, metrics: Dict[str, float]) -> np.ndarray:
        
        feature_vector = torch.tensor([
            metrics.get("distance_to_obstacle", 10.0),
            metrics.get("lateral_deviation", 0.0),
            metrics.get("velocity_magnitude", 5.0),
            metrics.get("surface_friction", 0.8)
        ], dtype=torch.float32).unsqueeze(0)
        
        self.alpha_model.eval()
        with torch.no_grad():
            alpha = float(self.alpha_model(feature_vector).item())
            
        u_pf = self._path_following_law(pos, vel, target)
        u_oa = self._obstacle_avoidance_law(pos, vel, obstacle)
        
        u_final = (1.0 - alpha) * u_pf + alpha * u_oa
        return u_final, alpha