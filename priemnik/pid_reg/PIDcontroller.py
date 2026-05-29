import math

from filters import pt as pt

class PID:

    def __init__(self, P_rate, I_rate, D_rate, P_ang, D_ang, P_vel, I_vel, P_pos, hz, angle_hz, vel_hz, pos_hz, firmwmode_number):

        #МАКСИМАЛЬНЫЕ ЗНАЧЕНИЯ
        self.max_angular_velocity = 180 #град\сек
        self.max_angle = 60 #град
        self.max_lin_velocity = 20 #м\с

        #режим прошивки
        self.firmwmode = firmwmode_number #0 - rate, 1 - angle, 2 - vel, 3 - pos

        #ЧАСТОТЫ и счётчики тиков
        self.hz = hz
        self.angle_hz = angle_hz
        self.velosity_hz = vel_hz
        self.position_hz = pos_hz

        self.angle_ticks_counter = 0
        self.velocity_ticks_counter = 0
        self.position_ticks_counter = 0
        self.ticks_counter = 0
        
        #КОЭФФИЦИЕНТЫ, СЕТПОИНТЫ, ИЗМЕРЕНИЯ ДЛЯ RATE-КОНТУРА
        self.P_rate = P_rate
        self.I_rate = I_rate
        self.D_rate = D_rate
        self.I_Term_rate = 0.0
        self.gyro_measurement = 0.0
        self.last_gyro_measurement = 0.0
        self.rate_setpoint = 0.0
        self.rate_dt = 0.0
        self.rate_ticks = 0

        #КОЭФФИЦИЕНТЫ, СЕТПОИНТЫ, ИЗМЕРЕНИЯ ДЛЯ ANGLE-КОНТУРА
        self.P_angle = P_ang
        self.D_ang = D_ang
        self.angle_measurement = 0.0
        self.last_angle_measurement = 0.0
        self.angle_setpoint = 0.0
        self.angle_dt = 0.0
        self.angle_ticks = 0
        self.angle_work_in_ticks = int(self.hz / self.angle_hz) #раз в сколько тиков должен работать angle-контур
        print(self.angle_work_in_ticks)

        #КОЭФФИЦИЕНТЫ, СЕТПОИНТЫ, ИЗМЕРЕНИЯ ДЛЯ VELOCITY-КОНТУРА
        self.P_vel = P_vel
        self.I_vel = I_vel
        self.I_Term_vel = 0.0
        self.velocity_setpoint = 0.0
        self.velocity_measurement = 0.0
        self.vel_dt = 0.0
        self.velocity_work_in_ticks = int(self.hz / self.velosity_hz) #раз в сколько тиков должен работать velocity-контур

        #КОЭФФИЦИЕНТЫ, СЕТПОИНТЫ, ИЗМЕРЕНИЯ ДЛЯ POSITION-КОНТУРА
        self.P_pos = P_pos
        self.position_setpoint = 0.0
        self.position_measurement = 0.0
        self.pos_dt = 0.0
        self.position_work_in_ticks = int(self.hz / self.position_hz) #раз в сколько тиков должен работать angle-контур


        #ФИЛЬТР НИЖНИХ ЧАСТОТ ДЛЯ D_TERM КОНТУРОВ И СООТВЕТСТВУЮЩИЕ ПЕРЕМЕННЫЕ К НИМ (у меня диф. части используются в rate и angle контурах)
        self.lpf_for_D_part_in_rate_contur = pt.PT3(fc = 100, Fs= self.hz) #FS-частота, fc-частота среза
        self.lpf_for_D_part_in_angle_contur = pt.PT3(fc = self.angle_hz / 4, Fs = self.angle_hz)

        #ФЛАГИ, ПОКАЗЫВАЮЩИЕ ОТКУДА СЧИТЫВАТЬ СЕТПОИНТЫ
        self.is_setpoint_from_pos_contur = False
        self.is_setpoint_from_vel_contur = False
        self.is_setpoint_from_ang_contur = False

    
    def cascade(self, setpoint, dt):

        self.rate_dt += dt
        self.angle_dt += dt
        self.vel_dt += dt
        self.pos_dt += dt

        if ( (self.ticks_counter % self.position_work_in_ticks == 0) or self.ticks_counter == 0 ) and self.firmwmode >= 3:
            pass

        if ( (self.ticks_counter % self.velocity_work_in_ticks == 0) or self.ticks_counter == 0 ) and self.firmwmode >= 2:
            
            if not self.is_setpoint_from_pos_contur: #если сетпоинт для velocity-контура устанавливается не из position контура, то находим setpoint сами
                self.velocity_setpoint = self.max_lin_velocity * setpoint
            
            #результатом работы veloctity-контура является setpoint для angle-контура, поэтому сразу результат работы велосити-контура приравниваем к англ-сетпоинту
            self.angle_setpoint = self.velocity_contur()
            self.is_setpoint_from_vel_contur = True #устанавливаем флаг, означающий, что сетпоинт для англ-контура приходит из велосити-контура

        if ( (self.ticks_counter % self.angle_work_in_ticks == 0) or self.ticks_counter == 0) and self.firmwmode >= 1:

            if not self.is_setpoint_from_vel_contur: #если сетпоинт для angle-контура устанавливается не из velocity контура, то находим setpoint сами
                self.angle_setpoint = self.max_angle * setpoint
            
            #результатом работы angle-контура является setpoint для rate-контура, поэтому сразу результат работы велосити-контура приравниваем к англ-сетпоинту
            self.rate_setpoint = self.angle_contur()
            self.is_setpoint_from_ang_contur = True #устанавливаем флаг, означающий, что сетпоинт для rate-контура приходит из angle-контура

        if not self.is_setpoint_from_ang_contur: #если сетпоинт для rate-контура устанавливается не из angle контура, то находим setpoint сами
            self.rate_setpoint = self.max_angular_velocity * setpoint

        signal = self.rate_contur()

        self.ticks_counter += 1

        return signal, self.velocity_setpoint, self.angle_setpoint, self.rate_setpoint, self.rate_ticks, self.angle_ticks
        

    def position_contur(self):
        self.pos_dt = 0.0
        pass

    def velocity_contur(self):
        error = self.velocity_setpoint - self.velocity_measurement

        P_part = self.P_vel * error

        pre_result = self.compute_attitude_setpoint(P_part + self.I_Term_vel)#получаем угол из ускорения (ускорение получается в P_part)
        

        if abs(self.I_term_vel) <= (self.max_lin_vel / (1.0 / self.velosity_hz)):
            self.I_term_vel += self.I_vel * error * self.vel_dt

        result = max(-self.max_angle, min(pre_result, self.max_angle))
        self.vel_dt = 0.0

        return result

    def angle_contur(self):
        error = self.angle_setpoint - self.angle_measurement

        P_part = self.P_angle * error
        D_part = self.lpf_for_D_part_in_angle_contur.pt3( self.D_ang * (self.last_angle_measurement - self.angle_measurement) / self.angle_dt )

        pre_result = P_part + D_part

        result = max(-self.max_angular_velocity, min(pre_result, self.max_angular_velocity))

        self.last_angle_measurement = self.angle_measurement
        self.angle_dt = 0.0
        self.angle_ticks += 1

        return result

    def rate_contur(self):
        error = self.rate_setpoint - self.gyro_measurement
        P_part = self.P_rate * error

        D_part = self.lpf_for_D_part_in_rate_contur.pt3( self.D_rate * (self.last_gyro_measurement - self.gyro_measurement) / self.rate_dt )

        pre_result = P_part + self.I_Term_rate + D_part
        
        result = max(-1.0, min(pre_result, 1.0))

        self.I_Term_rate += self.I_rate * error * self.rate_dt

        self.last_gyro_measurement = self.gyro_measurement
        self.rate_dt = 0.0
        self.rate_ticks += 1

        return result
    


    def compute_attitude_setpoint(self, a_des_xy, a_des_z=0.0, max_angle_deg= 60.0):
        """
        a_des_xy : требуемое горизонтальное ускорение (м/с²)
        a_des_z  : требуемое вертикальное ускорение (м/с²), обычно 0 в velocity-режиме
        max_angle_deg : ограничение угла наклона
        """
        g = 9.81  # м/с²
        
        # Полная формула с atan2
        # atan2(y, x) = угол вектора (x, y). Здесь x = вертикаль, y = горизонталь
        angle_rad = math.atan2(a_des_xy, g + a_des_z)
        
        # Ограничение угла (физический лимит эффективности тяги)
        max_angle_rad = math.radians(max_angle_deg)
        angle_rad = max(-max_angle_rad, min(max_angle_rad, angle_rad))
        
        return math.degrees(angle_rad)  # в радианах, или math.degrees(angle_rad) если нужно в градусах
