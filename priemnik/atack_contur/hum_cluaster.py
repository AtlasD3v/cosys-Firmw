import numpy as np
from sklearn.cluster import MeanShift

class InfantryClusterer:
    def __init__(self, bandwidth: float = 12.0):
        self.bandwidth = bandwidth
        self.cluster_model = MeanShift(bandwidth=bandwidth, bin_seeding=True)

    def analyze_density_center(self, coordinates: list[np.ndarray]) -> tuple[np.ndarray, int]:
        if not coordinates:
            return np.zeros(3), 0
            
        data_matrix = np.array(coordinates)
        self.cluster_model.fit(data_matrix)
        centers = self.cluster_model.cluster_centers_
        labels = self.cluster_model.labels_
        
        unique_labels, counts = np.unique(labels, return_counts=True)
        top_cluster_idx = np.argmax(counts)
        
        optimal_center = centers[top_cluster_idx]
        density_count = counts[top_cluster_idx]
        
        return optimal_center, int(density_count)