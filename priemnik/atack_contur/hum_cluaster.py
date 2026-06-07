import numpy as np
from sklearn.cluster import MeanShift
from typing import List, Tuple, Optional

class InfantryClusterer:

    def __init__(self, target_radius: float = 12.0, use_seeding: bool = True):
        self.target_radius = target_radius
        self.model = MeanShift(bandwidth=target_radius, bin_seeding=use_seeding)
        self.last_cluster_centers: Optional[np.ndarray] = None
        self.last_labels: Optional[np.ndarray] = None

    def calculate_optimal_node(self, 
                               coordinates: List[np.ndarray], 
                               weights: Optional[List[float]] = None) -> Tuple[np.ndarray, int]:
        
        if not coordinates or len(coordinates) == 0:
            return np.zeros(3, dtype=np.float32), 0

        data_matrix = np.array(coordinates, dtype=np.float32)
        sample_weights = np.array(weights, dtype=np.float32) if weights is not None else None

        try:
            self.model.fit(data_matrix, sample_weight=sample_weights)
            
            self.last_cluster_centers = self.model.cluster_centers_
            self.last_labels = self.model.labels_

            unique_labels, counts = np.unique(self.last_labels, return_counts=True)
            
            dominant_cluster_idx = np.argmax(counts)
            
            optimal_center = self.last_cluster_centers[dominant_cluster_idx]
            density_count = counts[dominant_cluster_idx]

            return optimal_center, int(density_count)

        except Exception as e:
            print(f"[ERROR] Ошибка в модуле пространственной кластеризации: {e}")
            return data_matrix[0], 1

    def get_cluster_meta(self) -> dict:
        if self.last_cluster_centers is None:
            return {"status": "no_data"}
        return {
            "total_clusters": len(self.last_cluster_centers),
            "centroids": self.last_cluster_centers.tolist()
        }