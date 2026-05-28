import threading
import sys
from sensor_readers import readers
from pid_reg import pid
# from upravlenie import joy as j
from upravlenie import keyboard_joy as j
from filters import pt as filt
from filters import notch as notch
from sensor_readers import rc_translater as rc_t
import time
from concurrent.futures import ThreadPoolExecutor

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink_common
from mav_connector import mav as mavBridge

from pid_reg import allocator as alloc

DRONE_CONNECT  = None

def connector():
    global DRONE_CONNECT
    connection = mavutil.mavlink_connection("udp:0.0.0.0:14550")
    # connection = mavutil.mavlink_connection("udp:0.0.0.0:14540")

    connection.wait_heartbeat(timeout=10)
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

    DRONE_CONNECT = connection

    set_data_Hz(connection)

    params = [0.9, 0.9, 0.9, 0.9, 0.0, 0.0, 0.0]
    send_motors_commands(connection, params)


def set_data_Hz(drone_conn):

    res = False
    drone_conn.mav.command_long_send( drone_conn.target_system,                # ID системы (обычно 1)
                                      drone_conn.target_component,             # ID компонента (обычно 1)
                                      mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # ID команды (511)
                                      0,                                   # confirmation
                                      105,                                 # param1: ID сообщения (HIGHRES_IMU)
                                      4000,                                # param2: интервал в мкс (1000 мкс = 1000 Гц)
                                      0, 0, 0, 0, 0                        # остальные параметры не используются)
                                      )
    set_result = drone_conn.recv_match(type="COMMAND_ACK", blocking=True)

    if set_result and set_result.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
        if set_result.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Частота датчиков успешно установлена")
            res = True
        else:
            print("Ошибка установления частоты публикации данных с датчиков")
    else:
        print("Какая-то ошибка в set_result and set_result.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:")

    return res


def send_motors_commands(drone, params):
    res = False
    drone.mav.command_long_send(  drone.target_system,                # ID системы (обычно 1)
                                  drone.target_component,             # ID компонента (обычно 1)
                                  187, # ID команды (187)
                                  0,                                   # confirmation
                                  params[0],                                 # param1: ID сообщения (HIGHRES_IMU)
                                  params[1],                                # param2: интервал в мкс (1000 мкс = 1000 Гц)
                                  params[2], params[3], params[4], params[5], params[6]                     # остальные параметры не используются)
                                  )
    set_result = drone.recv_match(type="COMMAND_ACK", blocking=True)

    if set_result and set_result.command == 187:
        if set_result.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Моторная команда успешно отправлена")
            res = True
        else:
            print("ошибка моторых команд")
    else:
        print("ошибка моторных команд 2")

    return res



class Control_loop:

    def __init__(self, dt, RC_filter_poryadok, expo_num, rc_dt, expo_alphas, expo_beta, low_amplitude_percent, max_val_in_percent_diapazon ):
        self.dt = dt
        self.critical_error = False
        self.RC_filter_poryadok = RC_filter_poryadok
        self.rc_dt = rc_dt
        self.expo_alpha_throttle, self.expo_alpha_roll, self.expo_alpha_pitch, self.expo_alpha_yaw = expo_alphas[0], expo_alphas[1], expo_alphas[2], expo_alphas[3]
        self.expo_beta_throttle, self.expo_beta_roll, self.expo_beta_pitch, self.expo_beta_yaw = expo_beta[0], expo_beta[1], expo_beta[2], expo_beta[3]
        self.low_amplitude_percent_throttle, self.low_amplitude_percent_roll, self.low_amplitude_percent_pitch, self.low_amplitude_percent_yaw = low_amplitude_percent[0], low_amplitude_percent[1], low_amplitude_percent[2], low_amplitude_percent[3]
        self.max_val_in_percent_diapazon_throttle, self.max_val_in_percent_diapazon_roll, self.max_val_in_percent_diapazon_pitch, self.max_val_in_percent_diapazon_yaw = max_val_in_percent_diapazon[0], max_val_in_percent_diapazon[1], max_val_in_percent_diapazon[2], max_val_in_percent_diapazon[3]

        self.pt_to_RC_throttle, self.pt_to_RC_roll, self.pt_to_RC_pitch, self.pt_to_RC_yaw = RC_filter_poryadok[0], RC_filter_poryadok[1],RC_filter_poryadok[2],RC_filter_poryadok[3]
        self.expo_num_throttle, self.expo_num_roll, self.expo_num_pitch, self.expo_num_yaw  = expo_num[0], expo_num[1], expo_num[2], expo_num[3] #номера для выбора метода изменения кривой управления, где 0 - обычный Expo, 1 - custom_boost

        self.filt_read_throttle, self.filt_read_roll, self.filt_read_pitch, self.filt_read_yaw = (readers.RC_Reader(self.pt_to_RC_throttle, self.expo_num_throttle, self.expo_alpha_throttle, self.expo_beta_throttle, self.low_amplitude_percent_throttle, self.max_val_in_percent_diapazon_throttle),
                                                                                                  readers.RC_Reader(self.pt_to_RC_roll, self.expo_num_roll, self.expo_alpha_roll, self.expo_beta_roll, self.low_amplitude_percent_roll, self.max_val_in_percent_diapazon_roll),
                                                                                                  readers.RC_Reader(self.pt_to_RC_pitch, self.expo_num_pitch, self.expo_alpha_pitch, self.expo_beta_pitch, self.low_amplitude_percent_pitch, self.max_val_in_percent_diapazon_pitch),
                                                                                                  readers.RC_Reader(self.pt_to_RC_yaw, self.expo_num_yaw, self.expo_alpha_yaw, self.expo_beta_yaw, self.low_amplitude_percent_yaw, self.max_val_in_percent_diapazon_yaw))


        self.rc_raw = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'throttle': 0.0, 'timespan': 0.0}

        self.max_rates_rate_roll = 900 #deg\s
        self.max_rates_rate_pitch = 700 #deg\s
        self.max_rates_rate_yaw = 180 #deg\s
        # self.max_rates_attitude =

        # self.setpoint_raw = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        self.rc_update_interval = 20  #время, через которое запрашиваются новые данные с RC-пульта 0.02 с
        self.dynamic_notch_update_interval = 20 #0.02

        self.FILT_ACCEL_X = None
        self.FILT_ACCEL_Y = None
        self.FILT_ACCEL_Z = None

        self.FILT_GYRO_X = None
        self.FILT_GYRO_Y = None
        self.FILT_GYRO_Z = None

        # self.active_rc = j.init_joy(self.joy_id)
        self.sens_reader = None

        self.MAVBridge = None
        self._imu_thread = None



    def main_cylce_func(self):

        self.pt_to_RC_throttle, self.pt_to_RC_roll, self.pt_to_RC_pitch, self.pt_to_RC_yaw  = filt.PT3(), filt.PT3(), filt.PT3(), filt.PT3() #Тут инициализируется класс ФНЧ выбранного порядка для каждой из осей свободы

        self.expo_changer_throttle, self.expo_changer_roll, self.expo_changer_pitch, self.expo_changer_yaw = (filt.Expo(self.expo_alpha_throttle, self.expo_beta_throttle, self.low_amplitude_percent_throttle, self.max_val_in_percent_diapazon_throttle),
                                                                                          filt.Expo(self.expo_alpha_roll, self.expo_beta_roll, self.low_amplitude_percent_roll, self.max_val_in_percent_diapazon_roll),
                                                                                          filt.Expo(self.expo_alpha_pitch, self.expo_beta_pitch, self.low_amplitude_percent_pitch,self.max_val_in_percent_diapazon_pitch ),
                                                                                          filt.Expo(self.expo_alpha_yaw, self.expo_beta_yaw, self.low_amplitude_percent_yaw, self.max_val_in_percent_diapazon_yaw))#Тут инициализируется класс изменения кривой управления для каждой из степеней свободы

        self.MAVBridge = MAVBridge = mavBridge.MAVLinkBridge() #инициализируем класс, с помощью которого через MAVLink получаем\отправляем данные (при инициализации открывается подключение)

        pid_roll = pid.Standart_PID(0.00001, 0, 0, 0, 0.000005, 0, 0, 0, 0.0000005, 0, 0, 0) #инициалазируем класс ПИД-регуляторов для оси roll
        pid_pitch = pid.Standart_PID(0.00001, 0, 0, 0, 0.000005, 0, 0, 0, 0.0000005, 0, 0, 0) #инициалазируем класс ПИД-регуляторов для оси pitch
        pid_yaw = pid.Standart_PID(0.00001, 0, 0, 0, 0.000005, 0, 0, 0, 0.0000005, 0, 0, 0) #инициалазируем класс ПИД-регуляторов для оси yaw

        dyn_notch_gyr_x = notch.GyroDynamicNotch(1000, 0, 1.2) #Инициализация динамических режекторных фильтров для сигнала с гироскопа
        pt_to_gyro_x = filt.PT3(fc=150, Fs=1000)#ФНЧ первого порядка после динамического notch

        dyn_notch_gyr_y = notch.GyroDynamicNotch(1000, 0, 1.2)
        pt_to_gyro_y = filt.PT3(fc=150, Fs=1000)

        dyn_notch_gyr_z = notch.GyroDynamicNotch(1000, 0, 1.2)
        pt_to_gyro_z = filt.PT3(fc=150, Fs=1000)

        pt_gyro_x = filt.PT3(fc = 100, Fs = 1000)
        pt_gyro_y = filt.PT3(fc = 100, Fs = 1000)
        pt_gyro_z = filt.PT3(fc = 100, Fs = 1000)

        using_notch = False #Переменная-сигнализатор, которая показывает, были ли уже получены данные с RPM, а следовательно, были ли настроены notch (пока они не настроены, сигнал фильтроваться не должен)

        tick_counter_for_rc = 0 #считаем время работы в мс
        tick_counter_for_notch = 0
        all_time_counter = 0

        allocator = alloc.Allocator(1) #инициализация аллокатора с конфигурацией ВМГ №0 

        
        

        MAVBridge.heartbit_send()
        MAVBridge.wait_imu()

        self._imu_thread = threading.Thread(
            target=MAVBridge.get_imu_data_pool,
            name="IMU-RX",
            daemon=True
        )
        self._imu_thread.start() #запускаем поток бесконечного чтения данных с IMU

        dead_zone = [0.02, 0.02, 0.02, 0.02, 0.02]
        j.main_joy_func(dead_zone) #чтение джойстика


        while not self.critical_error:
            
            
            if tick_counter_for_rc >= self.rc_update_interval or all_time_counter == 0:
                self.parallel_execute_filt_rc() #получаем новые значения с RC-пульта (данные сохраняются в переменных self.rc_raw['соответств. ось'])
                tick_counter_for_rc = 0 # сбрасываем таймер

            # imu_data = MAVBridge.recv_imu_data() #читаем данные с IMU (частота 1000 Гц)
            imu_data = None

            while not MAVBridge.is_lock: #если is_lock = True, значит мы получили данные с IMU и нужно выйти из while, чтобы продолжить работу управ. цикла, иначе - не получили
                # print("ЖДЁМ ПОЛУЧЕНИЯ IMU")
                if MAVBridge.new_imu is not None:
                    # print("ПОЛУЧИЛИ IMU")
                    imu_data = MAVBridge.new_imu #помещаем только что полученные данные от IMU в переменную imu_data
                    MAVBridge.new_imu = None #установили MAVBridge.new_imu в None, чтобы далее иметь возможность получать и парсить новые данные IMU
                    MAVBridge.is_lock = True #установили True, чтобы выйти из цикла while

            MAVBridge.last_imu_data = imu_data
            MAVBridge.is_lock = False

            clean_gyro_x, clean_gyro_y, clean_gyro_z = None, None, None
            clean_acc_x, clean_acc_y, clean_acc_z = None, None, None
            if using_notch: #Если была произведена хотя бы первая настройка notch, то используем их

                notch_clean_gyro_x = dyn_notch_gyr_x.filter(imu_data['gyro_x'])
                notch_clean_gyro_y = dyn_notch_gyr_y.filter(imu_data['gyro_y'])
                notch_clean_gyro_z = dyn_notch_gyr_z.filter(imu_data['gyro_z'])

                clean_gyro_x = pt_to_gyro_x.pt1(notch_clean_gyro_x) #очищенный сигнал с помощью ФНЧ первого порядка после применения динамического нотч
                clean_gyro_y = pt_to_gyro_y.pt1(notch_clean_gyro_y)
                clean_gyro_z = pt_to_gyro_z.pt1(notch_clean_gyro_z)
            else:
                clean_gyro_x = pt_gyro_x.pt3(imu_data['gyro_x'])
                clean_gyro_y = pt_gyro_y.pt3(imu_data['gyro_y'])
                clean_gyro_z = pt_gyro_z.pt3(imu_data['gyro_z'])
                # clean_gyro_x = imu_data['gyro_x']
                # clean_gyro_y = imu_data['gyro_y']
                # clean_gyro_z = imu_data['gyro_z']
                clean_acc_x = imu_data['acc_x']
                clean_acc_y = imu_data['acc_y']
                clean_acc_z = imu_data['acc_z']

            pid_roll.gyro_measurement = clean_gyro_x #в ПИД-регуляторе, созданном для оси roll, заполняем переменную gyro_measurement (измерение с гироскопа по оси roll), так как она выступает в роли measurement внутри rate-контура
            pid_pitch.gyro_measurement = clean_gyro_y #в ПИД-регуляторе, созданном для оси pitch, заполняем переменную gyro_measurement (измерение с гироскопа по оси pitch), так как она выступает в роли measurement внутри rate-контура
            pid_yaw.gyro_measurement = clean_gyro_z #в ПИД-регуляторе, созданном для оси yaw, заполняем переменную gyro_measurement (измерение с гироскопа по оси yaw), так как она выступает в роли measurement внутри rate-контура

            
            PID_roll, roll_setpoint = pid_roll.release_cascade(self.rc_raw['roll']) #вызываем работу ПИД-регулятора по оси roll, передавая последнее полученное значение с RC-пульта
            PID_pitch, pitch_setpoint = pid_pitch.release_cascade(self.rc_raw['pitch']) #вызываем работу ПИД-регулятора по оси pitch, передавая последнее полученное значение с RC-пульта
            PID_yaw, yaw_setpoint = pid_yaw.release_cascade(self.rc_raw['yaw']) #вызываем работу ПИД-регулятора по оси yaw, передавая последнее полученное значение с RC-пульта
            PID_thrust = self.rc_raw['throttle'] 

            signals = [PID_roll, PID_pitch, PID_yaw, PID_thrust]

            pwm_to_esc = allocator.allocator(signals)#передаём в аллокатор полученные от ПИД-регуляторов требуемый ШИМ для каждой из осей, включая тягу
            total_pwm = [pwm_to_esc[0], pwm_to_esc[1], pwm_to_esc[2], pwm_to_esc[3], 0.0, 0.0, 0.0, 0.0] #первые 4 значения - ШИМ для каждой из осей, а остальные (0.0) - нуль.зн. для того, чтобы заполнить нужные параметры в сообщении MAVLink

            if tick_counter_for_notch >= self.dynamic_notch_update_interval:
                # self.get_RPM_update_coeffs(dyn_notch_gyr_x, dyn_notch_gyr_y, dyn_notch_gyr_z)
                # using_notch = True
                # tick_counter_for_notch = 0
                pass #пока что пропускаем

            
            MAVBridge.pwm_send(pwm_motor=total_pwm, time = time.time())
            
            

            tick_counter_for_rc += 1
            tick_counter_for_notch += 1
            all_time_counter += 1

            if all_time_counter % 100 == 0:
                acc_arr = [clean_acc_x, clean_acc_y, clean_acc_z]
                gyro_arr = [clean_gyro_x, clean_gyro_y, clean_gyro_z]
                rc = [self.rc_raw['roll'], self.rc_raw['pitch'], self.rc_raw['yaw'], self.rc_raw['throttle']]
                pwm_sended = [total_pwm[0], total_pwm[1], total_pwm[2], total_pwm[3]]
                pids = [PID_roll, PID_pitch, PID_yaw, PID_thrust]
                setpoints = [roll_setpoint, pitch_setpoint, yaw_setpoint, ]
                self._print_dashboard(acc_arr, gyro_arr, rc, pwm_sended, pids, setpoints)









    def parallel_execute_filt_rc(self):#функция, которая параллельно запускает четыре функции фильтрации и изменения кривой управления RC-сигнала по всем степеням свободы

        signal = j.get_signals() #читаем сигнал в этом потоке, чтобы мы получали сигнал не с разных временных промежутков
        timespan = time.time()

        # sig_yaw = self.filt_read_yaw.rc_reader_filter(signal[2])
        # sig_throttle = self.filt_read_throttle.rc_reader_filter(signal[3])
        # sig_roll = self.filt_read_roll.rc_reader_filter(signal[0])
        # sig_pitch = self.filt_read_pitch.rc_reader_filter(signal[1])

        sig_yaw = signal[3]
        sig_throttle = signal[2]
        sig_roll = signal[0]
        sig_pitch = signal[1]

        # sig_yaw = 0.0
        # sig_throttle = 0.620
        # sig_roll = 0.0
        # sig_pitch = 0.02


        self.rc_raw['roll'] = sig_roll
        self.rc_raw['pitch'] = sig_pitch
        self.rc_raw['yaw'] = sig_yaw
        self.rc_raw['throttle'] = sig_throttle
        self.rc_raw['timespan'] = timespan

        # print(self.rc_raw)


    def get_RPM_update_coeffs(self, notch1, notch2, notch3):
        data = self.MAVBridge.recv_rpm_data() #получение RPM со всех 4 моторов
        if data is not None:
            avg_rpm = sum(data) / len(data) #средние обороты в минуту
            f_noice = avg_rpm / 60.0 #средняя частота среза

            if f_noice <= 80.0: #ограничение минимума частоты среза
                f_noice = 80.0

            notch1.coefficient_updater(1000, f_noice, 1.2)
            notch2.coefficient_updater(1000, f_noice, 1.2)
            notch3.coefficient_updater(1000, f_noice, 1.2)




    def _print_dashboard(self, accel_b, gyro_b, rc, pwm, pids, setpoints):
            """Оптимизированный монолитный вывод данных в консоль с защитой от переносов."""

            # Формируем список строк (каждая строка теперь короче и не будет переноситься)
            lines = [
                f"=== ТЕЛЕМЕТРИЯ ===",
                # f"RPM1: {rpm1}, RPM2 {rpm2}, RPM3 {rpm3}, RPM4 {rpm4}",
                # f"Моторы (Н) : [{thr}] (Эталон: {thrust_per_motor:.3f})",
                # f"{thr[0]}  {thr[1]}",
                # f"{thr[2]}  {thr[3]}"
                f"IMU Accel  : [{accel_b}]",
                f"IMU Gyro   : [{gyro_b}]",
                f"Setpoints  : X: {setpoints[0]} - {gyro_b[0]}, Y: {setpoints[1]} - {gyro_b[1]}, Z: {setpoints[2]} - {gyro_b[2]}",
                f"Полученный RC: roll: {rc[0]}, ptich: {rc[1]}, yaw: {rc[2]}, throttle: {rc[3]}",
                f"PID-ROLL: {pids[0]}, PID-PITCH: {pids[1]}, PID_YAW: {pids[2]}, PID:THROTTLE: {pids[3]}",
                f"Отправленный PWM на ESC: m1: {pwm[0]}, m2: {pwm[1]}, m3: {pwm[2]}, m4: {pwm[3]}"
                # f"ИСТИНА Pos : [{pos}]",
                # f"ИСТИНА LVel: [{l_vel}]",
                # f"ИСТИНА AVel: [{a_vel}]",
                # f"Воздух (Н) : [{drags}]",
                # f"Батарея    : {battery_percent:.2%} | Масса: {self.TOTAL_MASS:.3f}",
                # f"RTF        : {self.RTF_VALUE} сек. реала за 1 сек. сим.",
                # f"Время сим. : {self.REAL_TIME_SEC:.2f} сек.",
                f"============================================"
            ]
            
            # Склеиваем с очисткой строки (\033[K)
            dashboard = "\n".join([f"\r{line}\033[K" for line in lines])
            # dashboard = "\n".join([f"\r{line}\n" for line in lines])
            
            # Печатаем
            sys.stdout.write(dashboard)
            
            # Поднимаем курсор строго на количество строк в нашем списке
            sys.stdout.write(f"\033[{len(lines)-1}A")
            sys.stdout.flush()