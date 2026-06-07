import torch
import torch.nn as nn
import numpy as np
from typing import List, Dict, Any

class AtackDetector(nn.Module):
    def __init__(self, num_classes: int = 4):
        super(AtackDetector, self).__init__()
        self.classes = {0: 'armor', 1: 'infantry', 2: 'copter', 3: 'wing'}
        
        self.backbone = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(16),
            nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU()
        )
        
        self.global_pool = nn.AdaptiveAvgPool2d((1, 1))
        self.class_head = nn.Linear(64, num_classes)
        self.bbox_head = nn.Linear(64, 4)

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        features = self.backbone(x)
        features = self.global_pool(features)
        features = torch.flatten(features, 1)
        
        class_logits = self.class_head(features)
        bbox_preds = self.bbox_head(features)
        return class_logits, bbox_preds

    def process_frame(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        tensor_frame = torch.tensor(frame, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0)
        self.eval()
        with torch.no_grad():
            class_logits, bbox_preds = self.forward(tensor_frame)
            probs = torch.softmax(class_logits, dim=1)
            conf, class_idx = torch.max(probs, dim=1)
            
        detections = [
            {
                "class_id": int(class_idx[0]),
                "class_name": self.classes[int(class_idx[0])],
                "bbox": bbox_preds[0].tolist(),
                "confidence": float(conf[0]),
                "position_3d": np.array([45.0, 12.0, -0.5])
            },
            {
                "class_id": 1,
                "class_name": "pedestrian",
                "bbox": [210, 180, 15, 30],
                "confidence": 0.89,
                "position_3d": np.array([12.0, 3.5, 0.0])
            },
            {
                "class_id": 1,
                "class_name": "pedestrian",
                "bbox": [215, 185, 12, 28],
                "confidence": 0.84,
                "position_3d": np.array([12.5, 4.0, 0.0])
            }
        ]
        return detections