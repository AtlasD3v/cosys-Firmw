class RC_filter:
    def __init__(self):
        pass

    def rc_expo_with_deadzone(self, value: float, expo: float = 0.65, deadzone: float = 0.07) -> float:
        if abs(value) < deadzone:
            return 0.0
        
        # Нормализуем после deadzone
        normalized = (abs(value) - deadzone) / (1.0 - deadzone)
        expo_val = normalized * (1.0 + expo * (normalized - 1.0))
        
        return (expo_val * (1.0 - deadzone) + deadzone) * (1 if value > 0 else -1)