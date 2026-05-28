import math

from filters import pt as pt

# class PID:
#     def __init__(self, P_rate, I_rate, D_rate, P_angular, D_angular, P_vel, I_vel, hz, ang_hz, vel_hz, pos_hz, firmwmode_number):

#         self.firmware_mode = firmwmode_number
#         self.max_angular_vel = 180 #grad/sec

#         self.max_angle = 60 #grad

#         self.max_lin_vel = 20 #m/s

#         self.hz = hz

#         self.P_rate = P_rate
#         self.I_rate = I_rate
#         self.I_term_rate = 0.0
#         self.D_rate = D_rate

#         self.P_angular = P_angular
#         self.D_angular = D_angular

#         self.P_vel = P_vel
#         self.I_vel = I_vel
#         self.I_term_vel = 0.0


#         self.pos_hz = pos_hz
#         self.vel_hz = vel_hz
#         self.angle_hz = ang_hz

#         self.angle_dt = 0.0
#         self.vel_dt = 0.0

#         self.last_Fs_rate = hz
#         self.last_fc_rate = self.hz / 5

#         self.last_Fs_ang = self.hz / self.angle_hz
#         self.last_fc_ang = self.last_Fs_ang / 4

#         self.last_Fs_vel = self.hz / self.vel_hz
#         self.last_fc_vel = self.last_Fs_vel / 4

#         self.lpf_to_D_rate = pt.PT3(self.last_fc_rate, self.last_Fs_rate)
#         self.lpf_to_D_ang = pt.PT3(self.last_fc_ang, self.last_Fs_ang)
#         self.lpf_to_D_vel = pt.PT3(self.last_fc_vel, self.last_Fs_vel)


#         self.position_setpoint = None
#         self.velocity_setpoint = None
#         self.angle_setpoint = None
#         self.rate_setpoint = None

#         self.tick = 0

#         self.is_setpoint_from_pos = False
#         self.is_setpoint_from_vel = False
#         self.is_setpoint_from_ang = False

#         self.gyro_measurement = None
#         self.angle_measurement = None
#         self.velocity_measurement = None
#         self.position_measurement = None

#         self.last_gyro_measurement = 0.0
#         self.last_angle_measurement = 0.0
#         self.last_velocity_measurement = 0.0
#         self.last_position_measurement = 0.0

#         self.physics_setpoint_rate = 0.0
#         self.physics_setpoint_ang = 0.0
#         self.physics_setpoint_vel = 0.0
#         self.physics_setpoint_pos = 0.0

#         self.rate_ticks = 0
#         self.ang_ticks = 0
#         self.vel_ticks = 0
#         self.pos_ticks = 0



    
#     def cascade(self, setpoint, dt):
#         #сбрасываем флаги
#         # self.is_setpoint_from_vel = False 
#         # self.is_setpoint_from_ang = False

#         self.vel_dt += dt
#         self.angle_dt += dt
#         self.tick += 1
        
#         # if (self.tick % self.pos_hz == 0) and self.firmware_mode >= 3:
#         #     pass

#         if ((self.tick % self.vel_hz == 0) or self.tick == 1) and self.firmware_mode >= 2:

#             if not self.is_setpoint_from_pos: #если сетпоинт пришёл не с контура выше, а со стиков
#                 self.velocity_setpoint = setpoint * self.max_lin_vel
#                 self.physics_setpoint_vel = self.velocity_setpoint

#             self.angle_setpoint = self.velocity_contur(self.velocity_setpoint, self.vel_dt)
#             self.vel_dt = 0.0
#             self.is_setpoint_from_vel = True

#         if ((self.tick % self.angle_hz == 0) or self.tick == 1)and self.firmware_mode >= 1: 
            
#             if not self.is_setpoint_from_vel: #если сетпоинт пришёл не с контура выше, а со стиков
#                 self.angle_setpoint = setpoint * self.max_angle
#                 self.physics_setpoint_ang = self.angle_setpoint
                
#             self.rate_setpoint = self.angle_contur(self.angle_setpoint, self.angle_dt)
#             self.angle_dt = 0.0
#             self.is_setpoint_from_ang = True
            

#         if not self.is_setpoint_from_ang: #если сетпоинт пришёл не с контура выше, а со стиков
#             self.rate_setpoint = setpoint * self.max_angular_vel#переводим нормированный сетпоинт, полученный с RC, в физическую величину

#         self.physics_setpoint_rate = self.rate_setpoint
#         signal = self.rate_contur(self.rate_setpoint, dt)


#         return signal, self.physics_setpoint_vel, self.physics_setpoint_ang, self.physics_setpoint_rate, self.rate_ticks, self.ang_ticks

#     # def position_contur(self, setpoint):
#     #     error = 

#     def velocity_contur(self, setpoint, vel_dt):
#         error = setpoint - self.velocity_measurement
#         P_part = error

#         pre_result = self.compute_attitude_setpoint((P_part * self.P_vel) + (self.I_term_vel * self.I_vel))

#         result = max(-self.max_angle, min(pre_result, self.max_angle))

#         if abs(self.I_term_vel) <= (0.5 * self.max_lin_vel):
#             self.I_term_vel += error * (vel_dt)

#         self.vel_ticks += 1

#         return result

        


#     def angle_contur(self, setpoint, angle_dt): #все данные и вычисления происходят в град, град\сек
#         error = setpoint - self.angle_measurement

#         P_part = error
        
#         if angle_dt > 1e-6:
#             D_part = self.lpf_to_D_ang.pt1(
#                 (self.last_angle_measurement - self.angle_measurement) / angle_dt
#             )
#         else:
#             D_part = 0.0

#         pre_result = (P_part * self.P_angular) + (D_part * self.D_angular)
#         result = max(-self.max_angular_vel,  min(self.max_angular_vel, pre_result))

#         self.last_angle_measurement = self.angle_measurement
#         self.ang_ticks += 1

#         return result

#     def rate_contur(self, setpoint, dt):
        
#         error = setpoint - self.gyro_measurement #ошибка, которую нужно устанить, чтобы добиться setpoint. Все измерения в градусах

#         P_part = error

#         I_part = self.I_term_rate
#         # I_part = 0.0
#         D_part = (self.last_gyro_measurement - self.gyro_measurement) / dt


#         result = max(-1.0, min(1.0, (P_part * self.P_rate) + (I_part) + self.lpf_to_D_rate.pt1((D_part * self.D_rate)) ) )

#         self.I_term_rate += error * self.I_rate * dt
#         self.last_gyro_measurement = self.gyro_measurement
#         self.rate_ticks += 1

#         return result
    

    


#     def compute_attitude_setpoint(self, a_des_xy, a_des_z=0.0, max_angle_deg= 45.0):
#         """
#         a_des_xy : требуемое горизонтальное ускорение (м/с²)
#         a_des_z  : требуемое вертикальное ускорение (м/с²), обычно 0 в velocity-режиме
#         max_angle_deg : ограничение угла наклона
#         """
#         g = 9.81  # м/с²
        
#         # Полная формула с atan2
#         # atan2(y, x) = угол вектора (x, y). Здесь x = вертикаль, y = горизонталь
#         angle_rad = math.atan2(a_des_xy, g + a_des_z)
        
#         # Ограничение угла (физический лимит эффективности тяги)
#         max_angle_rad = math.radians(max_angle_deg)
#         angle_rad = max(-max_angle_rad, min(max_angle_rad, angle_rad))
        
#         return math.degrees(angle_rad)  # в радианах, или math.degrees(angle_rad) если нужно в градусах
#     # def back_calc(self):
#     #     pass

class PID:
    def __init__(self, P_rate, I_rate, D_rate, P_angular, D_angular, P_vel, I_vel, hz, ang_hz, vel_hz, pos_hz, firmwmode_number):
        self.firmware_mode = firmwmode_number
        self.max_angular_vel = 180 #grad/sec

        self.max_angle = 60 #grad

        self.max_lin_vel = 20 #m/s

        self.hz = hz

        #переменные и коэффициенты для rate-контура
        self.P_rate = P_rate
        self.I_rate = I_rate
        self.I_term_rate = 0.0
        self.D_rate = D_rate
        self.rate_ticks = 0
        self.rate_dt = 1e-6
        self.is_setpoint_from_angle = False
        #######################

        #переменные и коэффициенты для angle-контура
        self.P_angular = P_angular
        self.D_angular = D_angular
        self.angle_ticks = 0
        self.angle_dt = 1e-6
        #######################

        #переменные и коэффициенты для vel-контура
        self.P_vel = P_vel
        self.I_vel = I_vel
        self.I_term_vel = 0.0
        #######################


        #ЧАСТОТЫ
        self.pos_hz = pos_hz
        self.vel_hz = vel_hz
        self.angle_hz = ang_hz

        ###ПЕРЕМЕННЫЕ СЕТПОИНТОВ
        self.setpoint_rate = 0.0
        self.setpoint_angular = 0.0

        ###ПЕРЕМЕННЫЕ ИЗМЕРЕНИЙ (ПОЛУЧАЮТ ЗНАЧЕНИЯ ИЗ ГЛ. УПРАВ. ЦИКЛА)
        self.gyro_measurement = 0.0
        self.angle_measurement = 0.0

        #переменные для D-части rate-контура
        self.last_gyro_measurement = 0.0
        self.fc_to_D_rate = hz / 5
        self.Fs_to_D_rate = hz
        self.lpf_to_D_rate = pt.PT3(self.fc_to_D_rate, self.Fs_to_D_rate)

        #переменные для D-части angle-контура
        self.last_angle_measurement = 0.0
        self.Fs_to_D_angle = hz / ang_hz
        self.fc_to_D_angle = self.Fs_to_D_angle / 2
        self.lpf_to_D_angle = pt.PT3(self.fc_to_D_angle, self.Fs_to_D_angle)

        self.ticks = 0

    def cascade(self, setpoint, dt):

        self.angle_dt += dt
        self.rate_dt += dt
        

        if ((self.ticks % self.angle_hz  == 0) or self.ticks == 1) and self.firmware_mode >= 1:
            self.setpoint_angular = self.max_angle * setpoint #получаем угол
            self.setpoint_rate = self.angular_contur(self.setpoint_angular) #запускаем angle-контур, получаем setpoint для rate

            self.is_setpoint_from_angle = True

        if not self.is_setpoint_from_angle: #если сетпоинт для rate контура передаётся не из angle-контура
            #находим setpoint для rate-контура сами
            self.setpoint_rate = self.max_angular_vel * setpoint


        signal = self.rate_contur(self.setpoint_rate)
        
        self.ticks += 1
        return signal, 0.0, self.setpoint_angular, self.setpoint_rate, self.rate_ticks, self.angle_ticks

    
    def angular_contur(self, setpoint):
        error = setpoint - self.angle_measurement

        P_part = error * self.P_angular
        D_part = self.lpf_to_D_angle.pt1( self.D_angular * (self.last_angle_measurement - self.angle_measurement) * self.angle_dt )

        pre_result = P_part + D_part

        result = max(-self.max_angular_vel, min(pre_result, self.max_angular_vel))

        self.last_angle_measurement = self.angle_measurement
        self.angle_ticks += 1
        self.angle_dt = 1e-6

        return result

    def rate_contur(self, setpoint):
        error = setpoint - self.gyro_measurement

        P_part = error * self.P_rate
        D_part = self.lpf_to_D_rate.pt3( self.D_rate * ((self.last_gyro_measurement - self.gyro_measurement) * self.rate_dt) )

        pre_result = P_part + self.I_term_rate + D_part

        result = max(-1.0, min(pre_result, 1.0))

        self.I_term_rate += self.I_rate * error * self.rate_dt

        self.last_gyro_measurement = self.gyro_measurement
        self.rate_ticks += 1
        self.rate_dt = 1e-6

        return result
        