import math
import threading
import sys
from upravlenie import keyboard_joy as j
from filters import pt as filt
import time
from mav_connector import mav as mavBridge
from pid_reg import allocator as alloc
from pid_reg import PIDcontroller as pid
from filters import filt_rc

from scipy.spatial.transform import Rotation
import numpy as np

import cosysairsim as airsim

DRONE_CONNECT  = None


class Control_loop:
    DRONE_NAME = "MyDrone"
    AIRSIM_IP = "127.0.0.1"

    def __init__(self):

        self.hz = 500
        self.angle_hz = 5
        self.vel_hz = 15
        self.pos_hz = 50
        self.dt = float( 1.0 / self.hz)
        
        ####         ПАРСИМ КОЭФФИЦИЕНТЫ ПИДов И РЕЖИМ ПРОШИВКИ      #####
        coeffs_arr = self.update_PID_coefs()
        self.P_rate, self.I_rate, self.D_rate = coeffs_arr[0], coeffs_arr[1], coeffs_arr[2]
        self.P_angular, self.D_angular = coeffs_arr[3], coeffs_arr[4]
        self.P_vel, self.I_vel = coeffs_arr[5], coeffs_arr[6]
        self.firmwmode = coeffs_arr[7]
        #####################################################################

        #---  ИНИЦИАЛИЗИРУЕМ ПИДЫ   -----
        self.pid_roll = pid.PID(self.P_rate, self.I_rate, self.D_rate, self.P_angular, self.D_angular, self.P_vel, self.I_vel, self.hz, self.angle_hz, self.vel_hz, self.pos_hz, self.firmwmode)
        self.pid_pitch = pid.PID(self.P_rate, self.I_rate, self.D_rate, self.P_angular, self.D_angular, self.P_vel, self.I_vel, self.hz, self.angle_hz, self.vel_hz, self.pos_hz, self.firmwmode)
        self.pid_yaw = pid.PID(self.P_rate, self.I_rate, self.D_rate, self.P_angular, self.D_angular, self.P_vel, self.I_vel, self.hz, self.angle_hz, self.vel_hz, self.pos_hz, self.firmwmode)
        #----------------------------------------

        self.rc_raw = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'throttle': 0.0, 'timespan': 0.0}
        self.orientation_euler = [0.0, 0.0, 0.0]
        
        self.rc_filter = filt_rc.RC_filter()

        self.rc_update_interval = 20

        self.airsim_client = airsim.MultirotorClient(self.AIRSIM_IP)
        self.airsim_client.reset()
        self.airsim_client.confirmConnection()
        print("[BRIDGE] Подключились к Cosys-AirSim")
        



    def control_loop_func(self):
        
        self.enable_airsim_api()
        self.arm_drone()
        
        
        pt_gyro_x = filt.PT3(fc = 100, Fs = 400)
        pt_gyro_y = filt.PT3(fc = 100, Fs = 400)
        pt_gyro_z = filt.PT3(fc = 100, Fs = 400)
       
        tick_counter_for_rc = 0 #считаем время работы в мс
        all_time_counter = 0

        allocator = alloc.Allocator(1) #инициализация аллокатора с конфигурацией ВМГ №0 


        dead_zone = [0.0, 0.0, 0.0, 0.0, 0.0]
        j.main_joy_func(dead_zone) #чтение джойстика

        dt_target = 1.0 / self.hz
        t_prev = time.perf_counter()

        while True:

            t_iter_start = time.perf_counter()
            actual_dt = t_iter_start - t_prev   # реальный dt для ПИД
            t_prev = t_iter_start
            
            if tick_counter_for_rc >= self.rc_update_interval or all_time_counter == 0:
                self.parallel_execute_filt_rc() #получаем новые значения с RC-пульта (данные сохраняются в переменных self.rc_raw['соответств. ось'])
                tick_counter_for_rc = 0 # сбрасываем таймер

            #ПОЛУЧАЕМ ДАННЫЕ С ДАТЧИКОВ
            imu = self.airsim_client.getMultirotorState(vehicle_name=self.DRONE_NAME)
            

            ####################################################################################
            ############ ЧИСТИМ И ПЕРЕВОДИМ ДАННЫЕ В НУЖНЫЕ ФИЗИЧЕСКИЕ ВЕЛИЧИНЫ
            
            

            gyro_x = float(imu.kinematics_estimated.angular_velocity.x_val * (180 / math.pi))
            gyro_y = float(imu.kinematics_estimated.angular_velocity.y_val * (180 / math.pi))
            # gyro_y = -float(imu.kinematics_estimated.angular_velocity.y_val * (180 / math.pi))
            gyro_z = float(imu.kinematics_estimated.angular_velocity.z_val * (180 / math.pi))

            
            clean_gyro_x  = pt_gyro_x.pt3(gyro_x) 
            clean_gyro_y  = pt_gyro_y.pt3(gyro_y) 
            clean_gyro_z  = pt_gyro_z.pt3(gyro_z)
            # clean_gyro_x  = gyro_x
            # clean_gyro_y  = gyro_y
            # clean_gyro_z  = gyro_z

            clean_acc_x = imu.kinematics_estimated.linear_acceleration.x_val
            clean_acc_y = imu.kinematics_estimated.linear_acceleration.y_val
            clean_acc_z = imu.kinematics_estimated.linear_acceleration.z_val
            ####################################################################################
            ####################################################################################


            #-----------------------------------------------------
            #------РАБОТАЕТ С ANGLE-контуром
            if (all_time_counter % self.angle_hz == 0) or all_time_counter == 0:

                NED_quat = np.array(
                    [
                        imu.kinematics_estimated.orientation.w_val,
                        imu.kinematics_estimated.orientation.x_val,
                        imu.kinematics_estimated.orientation.y_val,
                        imu.kinematics_estimated.orientation.z_val,
                    ]
                )
                euler_angles = self.from_quat_to_euler(NED_quat)

                self.pid_roll.angle_measurement = euler_angles[0] #X
                self.pid_pitch.angle_measurement = euler_angles[1] #Y
                self.pid_yaw.angle_measurement = euler_angles[2] #Z

                self.orientation_euler = euler_angles

            #-----------------------------------------------------
            #-----------------------------------------------------

            ####################################################
            ######ПЕРЕДАЁМ ОЧИЩЕННЫЕ ИЗМЕРЕНИЯ ГИРОСКОПОВ В ПИДы ПО СООТВЕТСТВУЮЩИМ ОСЯМ
            self.pid_roll.gyro_measurement = clean_gyro_x
            self.pid_pitch.gyro_measurement = clean_gyro_y
            self.pid_yaw.gyro_measurement = clean_gyro_z
            ####################################################
            ####################################################


            PID_roll, roll_vel_setpoint, roll_ang_setpoint, roll_rate_setpoint, roll_rate_ticks, roll_ang_ticks = self.pid_roll.cascade(self.rc_raw['roll'], actual_dt)
            PID_pitch, pitch_vel_setpoint, pitch_ang_setpoint, pitch_rate_setpoint, pitch_rate_ticks, pitch_ang_ticks = self.pid_pitch.cascade(self.rc_raw['pitch'], actual_dt)
            PID_yaw, yaw_vel_setpoint, yaw_ang_setpoint, yaw_rate_setpoint, yaw_rate_ticks, yaw_ang_ticks = self.pid_yaw.cascade(self.rc_raw['yaw'], actual_dt)
            PID_thrust = self.rc_raw['throttle']

            signals = [PID_roll, PID_pitch, PID_yaw, PID_thrust]

            pwm_to_esc = allocator.allocator(signals)#передаём в аллокатор полученные от ПИД-регуляторов требуемый ШИМ для каждой из осей, включая тягу
            total_pwm = [pwm_to_esc[0], pwm_to_esc[1], pwm_to_esc[2], pwm_to_esc[3], 0.0, 0.0, 0.0, 0.0] #первые 4 значения - ШИМ для каждой из осей, а остальные (0.0) - нуль.зн. для того, чтобы заполнить нужные параметры в сообщении MAVLink

            self.airsim_client.moveByMotorPWMsAsync(
                front_right_pwm=total_pwm[1],
                rear_left_pwm=total_pwm[3], 
                front_left_pwm=total_pwm[0], 
                rear_right_pwm=total_pwm[2],
                duration= actual_dt * 2, 
                vehicle_name=self.DRONE_NAME
            )

            tick_counter_for_rc += 1
            all_time_counter += 1

            # Точное ожидание до конца тика
            elapsed   = time.perf_counter() - t_iter_start
            remaining = dt_target - elapsed
            if remaining > 0.0005:          # если остаток > 0.5 мс — busy-wait
                t_end = time.perf_counter() + remaining
                while time.perf_counter() < t_end:
                    pass

            if all_time_counter % 100 == 0:
                acc_arr = [clean_acc_x, clean_acc_y, clean_acc_z]
                gyro_arr = [clean_gyro_x, clean_gyro_y, clean_gyro_z]
                rc = [self.rc_raw['roll'], self.rc_raw['pitch'], self.rc_raw['yaw'], self.rc_raw['throttle']]
                pwm_sended = [total_pwm[0], total_pwm[1], total_pwm[2], total_pwm[3]]
                pids = [PID_roll, PID_pitch, PID_yaw, PID_thrust]
                setp = [roll_vel_setpoint, pitch_vel_setpoint, yaw_vel_setpoint, roll_ang_setpoint, pitch_ang_setpoint, yaw_ang_setpoint, roll_rate_setpoint, pitch_rate_setpoint, yaw_rate_setpoint]
                ticks = [roll_rate_ticks, pitch_rate_ticks, yaw_rate_ticks, roll_ang_ticks, pitch_ang_ticks, yaw_ang_ticks]
                self._print_dashboard(acc_arr, gyro_arr, rc, pwm_sended, pids, setp, ticks)



    def parallel_execute_filt_rc(self):#функция, которая параллельно запускает четыре функции фильтрации и изменения кривой управления RC-сигнала по всем степеням свободы

        signal = j.get_signals() #читаем сигнал в этом потоке, чтобы мы получали сигнал не с разных временных промежутков
        timespan = time.time()


        sig_yaw = self.rc_filter.rc_expo_with_deadzone((signal[3]), deadzone=0.07)
        sig_throttle = self.rc_filter.rc_expo_with_deadzone(signal[2], deadzone=0.07)
        sig_roll = self.rc_filter.rc_expo_with_deadzone(signal[0], deadzone=0.07)
        sig_pitch = self.rc_filter.rc_expo_with_deadzone(signal[1], deadzone=0.07)


        self.rc_raw['roll'] = sig_roll
        self.rc_raw['pitch'] = sig_pitch
        self.rc_raw['yaw'] = sig_yaw
        self.rc_raw['throttle'] = sig_throttle
        self.rc_raw['timespan'] = timespan


    def enable_airsim_api(self):
        self.airsim_client.enableApiControl(is_enabled=True, vehicle_name=self.DRONE_NAME) #self.DRONE_NAME равен имени дрона, которое прописывается в файле settings.json
        print("+++++ ВКЛЮЧИЛИ API +++++")

    def arm_drone(self):
        print("****[BRIDGE] АРМИРУЕМ ДРОН****")
        self.airsim_client.armDisarm(arm=True, vehicle_name=self.DRONE_NAME)   
    
    
    def _print_dashboard(self, accel_b, gyro_b, rc, pwm, pids, setp, ticks):
        # Каждая строка — это строго ОДИН элемент списка. 
        # Если нужна пустая строка для читаемости, просто ставим ""
        lines = [
            "=== ТЕЛЕМЕТРИЯ ===",
            f"IMU Accel  : [{accel_b}]",
            f"IMU Gyro   : X: {round(float(gyro_b[0]), 2)}, Y: {round(float(gyro_b[1]), 2)}, Z: {round(float(gyro_b[2]), 2)}",
            "", # Пустая строка вместо \n
            f"Setpoints_vel  : X: {round(setp[0], 2) if setp[0] is not None else 0.0}, Y: {round(setp[1], 2) if setp[1] is not None else 0.0}, Z: {round(setp[2], 2) if setp[2] is not None else 0.0}",
            f"Setpoints_ang  : X: {round(setp[3], 2) if setp[3] is not None else 0.0}, Y: {round(setp[4], 2) if setp[4] is not None else 0.0}, Z: {round(setp[5], 2) if setp[5] is not None else 0.0}",
            f"Setpoints_rate : X: {round(setp[6], 5) if setp[6] is not None else 0.0}, Y: {round(setp[7], 5) if setp[7] is not None else 0.0}, Z: {round(setp[8], 5) if setp[8] is not None else 0.0}",
            "", # Пустая строка вместо \n
            f"УГЛЫ           : X: {round(self.orientation_euler[0], 2)}, Y: {round(self.orientation_euler[1], 2)}, Z: {round(self.orientation_euler[2], 2)}",
            "", # Пустая строка вместо \n
            f"roll_rate_tick : {ticks[0]}, pitch_rate_tick: {ticks[1]}, yaw_rate_ticks: {ticks[2]}",
            f"roll_ang_tick  : {ticks[3]}, pitch_ang_tick: {ticks[4]}, yaw_ang_ticks: {ticks[5]}",
            "", # Пустая строка вместо \n
            f"Отправленный PWM на ESC: m1: {pwm[0]}, m2: {pwm[1]}, m3: {pwm[2]}, m4: {pwm[3]}",
            "============================================"
        ]
        
        # \033[K очищает строку от курсора до конца (на случай, если новый текст короче старого)
        dashboard = "\n".join([f"\r{line}\033[K" for line in lines])

        sys.stdout.write(dashboard)
        
        # Поднимаем курсор ровно на N-1 строк вверх, возвращая его на "=== ТЕЛЕМЕТРИЯ ==="
        # Важно: делаем это через sys.stdout.write в один проход, чтобы терминал не мерцал
        sys.stdout.write(f"\033[{len(lines) - 1}A")
        sys.stdout.flush()

    def update_PID_coefs(self):

        filename = r"D:\cosysfirmw\priemnik\pid_reg\coeffs.txt"
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # Убираем пробелы и пустые строки, конвертируем в float
            numbers = []
            for line in lines:
                line = line.strip()          # убираем \n и пробелы
                if line:                     # пропускаем пустые строки
                    numbers.append(float(line))
            
            if len(numbers) != 8:
                print(f"Предупреждение: в файле найдено {len(numbers)} чисел, а ожидалось 8")
            
            return numbers
        
        except FileNotFoundError:
            print(f"Ошибка: файл '{filename}' не найден")
            return None
        except ValueError as e:
            print(f"Ошибка: не удалось преобразовать в float. Возможно, есть нечисловые символы. Детали: {e}")
            return None
        

    def from_quat_to_euler(self, quat):
        # Создаём объект вращения из кватерниона
        rot = Rotation.from_quat(quat, scalar_first=True)

        # Преобразуем в углы Эйлера с указанием порядка вращения (например, 'zyx')
        euler_angles = rot.as_euler('zyx', degrees=True)
        roll = float(euler_angles[2])
        pitch = float(euler_angles[1])
        yaw = float(euler_angles[0])
    
        
        return [roll, pitch, yaw]
    
