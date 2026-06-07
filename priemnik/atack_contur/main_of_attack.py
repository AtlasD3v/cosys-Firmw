import numpy as np
import torch
from atack_contur.detect import AtackDetector
from atack_contur.traj_pred import TrajectoryPredictorLSTM
from atack_contur.controller import NavigationController
from atack_contur.hum_cluaster import InfantryClusterer

class AutonomousVehicleCore:
    def __init__(self):
        # Инициализация моделей
        self.detector = AtackDetector(num_classes=4)
        self.predictor = TrajectoryPredictorLSTM(input_dim=3, hidden_dim=64, output_dim=3)
        self.controller = NavigationController(K_p=3.0)
        self.clusterer = InfantryClusterer(bandwidth=10.0)
        
        # Внутреннее состояние системы навигации
        self.drone_position = np.array([0.0, 0.0, 5.0])
        self.drone_velocity = np.array([15.0, 0.0, 0.1])
        self.dt = 0.04  # Шаг дискретизации (25 Гц)

    def process_navigation_cycle(self, camera_frame: np.ndarray) -> dict:
        # 1. Запуск детектора целей
        detections = self.detector.process_frame(camera_frame)
        
        # 2. Выделение пехоты и поиск центра их плотности
        pedestrian_positions = [
            obj["position_3d"] for obj in detections if obj["class_name"] == "infantry"
        ]
        safety_center, cluster_density = self.clusterer.analyze_density_center(pedestrian_positions)
        
        # 3. Прогнозирование траектории вражеского БПЛА\бронированной техники
        target_objects = [obj for obj in detections if obj["class_name"] == "armor"]
        
        if target_objects:
            primary_target = target_objects[0]
            # Генерация синтетической истории движения для демонстрации работы LSTM
            simulated_history = [
                primary_target["position_3d"] - np.array([1.0, 0.1, 0.0]) * i
                for i in range(5, 0, -1)
            ]
            # Вызов экстраполяции через скрытые состояния LSTM
            predicted_position = self.predictor.extrapolate(simulated_history, steps_forward=10)
        else:
            predicted_position = self.drone_position + self.drone_velocity * 2.0

        #признаки для селектора альфа
        distance_to_lead = float(np.linalg.norm(predicted_position - self.drone_position))
        current_speed = float(np.linalg.norm(self.drone_velocity))
        
        environment_metrics = {
            "distance_to_obstacle": distance_to_lead,
            "lateral_deviation": 0.25,
            "velocity_magnitude": current_speed,
            "surface_friction": 0.85
        }
        
        #расчет вектора управления и динамического коэффициента смешивания
        control_vector, alpha_value = self.controller.get_control_command(
            pos=self.drone_position,
            vel=self.drone_velocity,
            target=predicted_position,
            obstacle=safety_center if cluster_density > 0 else predicted_position,
            metrics=environment_metrics
        )
        
        return {
            "detections_count": len(detections),
            "cluster_density": cluster_density,
            "predicted_point": predicted_position,
            "alpha": alpha_value,
            "control_command": control_vector
        }

if __name__ == "__main__":
    print("[SYSTEM] Запуск интеграционного навигационного комплекса...")
    
    # Создание управляющего ядра
    navigation_core = AutonomousVehicleCore()
    
    # Генерация тестовой матрицы изображения (H=224, W=224, C=3)
    mock_camera_input = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
    
    # Выполнение итерации цикла управления
    telemetry = navigation_core.process_navigation_cycle(mock_camera_input)
    
    print("\n================ МЕТРИКИ ВЫЧИСЛИТЕЛЬНОГО СЛУЖЕБНОГО ЦИКЛА ================")
    print(f"Зарегистрировано вражеских объектов: {telemetry['detections_count']}")
    print(f"Обнаружено объектов в кластере пехоты: {telemetry['cluster_density']} чел.")
    print(f"Расчетная координата путевой точки (LSTM): {telemetry['predicted_point']}")
    print(f"Выставленный нейросетью весовой коэффициент Alpha для подбора траектории: {telemetry['alpha']:.4f}")
    print(f"Сформированный вектор ускорения моторов: {telemetry['control_command']}")
    print("=========================================================================")