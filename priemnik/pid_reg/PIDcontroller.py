import math

from filters import pt as pt

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
        self.velocity_measurement = 0.0
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

    def velocity_contur(self, setpoint):
        P_part = setpoint - self.velocity_measurement
    
    def angular_contur(self, setpoint):
        error = setpoint - self.angle_measurement

        P_part = error * self.P_angular
        D_part = self.lpf_to_D_angle.pt1( self.D_angular * (self.last_angle_measurement - self.angle_measurement) / self.angle_dt )

        pre_result = P_part + D_part

        result = max(-self.max_angular_vel, min(pre_result, self.max_angular_vel))

        self.last_angle_measurement = self.angle_measurement
        self.angle_ticks += 1
        self.angle_dt = 1e-6

        return result

    def rate_contur(self, setpoint):
        error = setpoint - self.gyro_measurement

        P_part = error * self.P_rate
        D_part = self.lpf_to_D_rate.pt3( self.D_rate * ((self.last_gyro_measurement - self.gyro_measurement) / self.rate_dt) )

        pre_result = P_part + self.I_term_rate + D_part

        result = max(-1.0, min(pre_result, 1.0))

        self.I_term_rate += self.I_rate * error * self.rate_dt

        self.last_gyro_measurement = self.gyro_measurement
        self.rate_ticks += 1
        self.rate_dt = 1e-6

        return result
    