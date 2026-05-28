
# class RC_translater:
#     def __init__(self, max_rates_rate, max_rate_attitude, max_rates_velocity, max_rates_position):
#         self.max_rates_rate = max_rates_rate
#         self.max_rates_attitude = max_rate_attitude
#         self.max_rates_velocity = max_rates_velocity
#         self.max_rates_position = max_rates_position
#
#     def translater(self, rc_signal, contur_name):
#         if contur_name == "Rate":
#             return rc_signal * self.max_rates_rate
#         elif contur_name == "Attitude":
#             return rc_signal * self.max_rates_attitude
#         elif contur_name == "Velocity":
#             return rc_signal * self.max_rates_velocity
#         elif contur_name == "Position":
#             return rc_signal * self.max_rates_position
#         else:
#             print("Был подан RC-сигна с неявного ПИД-контура")

def translater(rc_signal, max_rates):
    return rc_signal * max_rates