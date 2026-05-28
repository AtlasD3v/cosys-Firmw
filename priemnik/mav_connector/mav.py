import sys, os, time

#из документации по ссылке https://mavlink.io/en/mavgen_python/ устанавливаем переменным окружения нужные значения по инструкции
# os.environ["MAVLINK_DIALECT"] = "bpmav"
os.environ["MAVLINK20"] = "1"

#указываем путь, где лежит сгенерированная мной библиотека bestpilotmav на основании кастомного диалекта
# sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pymavlink import mavutil




class MAVLinkBridge:
    CONN_IN = "udpin:localhost:14561"
    CONN_OUT = "udpout:localhost:14560"
    def __init__(self):
        #открываем тут порт для получения данных от симуляции
        #также инициализируем подключение к симуляции
        self.conn_in = mavutil.mavlink_connection(self.CONN_IN) #порт для того, чтобы сюда шли сообщения
        self.conn_out = mavutil.mavlink_connection(self.CONN_OUT) #шлём данные симулятору (симулятор слушает на 14560)

        self.last_imu_data = None #последние известные данные, полученные с IMU 
        self.new_imu = None #данные с IMU, которые мы получили только что
        self.new_orientation = None
        # self.is_get_imu = False #получили ли мы данные с IMU? True - если в функции get_imu_data_pool мы получили данные, False - устанавливается в главном цикле управления, когда мы считали данные из переменной new_imu
        self._is_running = True
        self.is_lock = False
        self.is_lock_orient = False
        self.IMU_TIMEOUT = 0.001
        print(f"Инициализирована прослушка на {self.CONN_IN}\n Отправляться сообщения будут на {self.CONN_OUT}")





    def heartbit_send(self):
        self.conn_out.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0, 0, 0 )
        print("[FW] HEARTBEAT отправлен, жду данные от симуляции...")

    def wait_imu(self):
        """Ждём HEARTBEAT от симулятора перед стартом получения данных с IMU."""
        print("[SIM] Ожидание подключения симулятора...")
        self.conn_in.wait_heartbeat()
        print("[SIM] Симулятор подключен!")

    def pwm_send(self, pwm_motor, time):
        # print("Отправляем данные PWM на прошивку")
        self.conn_out.mav.actuator_control_target_send(time_usec = int(time * 1e6), group_mlx = 0, controls = pwm_motor)

    def get_imu_data_pool(self):

        while self._is_running: #бесконечный цикл получения данных IMU
            result = self.recv_imu_data() #функция получения данных с IMU

            if result is not None:
                self.new_imu = result
            
            if result is None:
                continue#если данные с IMU не получены - делаем timeout и  продолжаем пытаться их получить

    def get_orientation_data_pool(self):
        while self._is_running:
            result = self.recv_oreintation_data()

            if result is not None:
                self.new_orientation = result
            else:
                continue

    def recv_imu_data(self):

        msg = self.conn_in.recv_match(type = "HIGHRES_IMU", blocking = True, timeout=self.IMU_TIMEOUT)
        
        acc_x, acc_y, acc_z = 0.0, 0.0, 0.0
        gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0
        timespan = 0.0

        # print("ЖДЁМ IMU")
        if msg and msg.get_type() == "HIGHRES_IMU":

            acc_x, acc_y, acc_z = msg.xacc, msg.yacc, msg.zacc
            gyro_x, gyro_y, gyro_z = msg.xgyro, msg.ygyro, msg.zgyro
            timespan = msg.time_usec
            
            # print("ПОЛУЧИЛИ IMU")

            return {'acc_x': acc_x, 'acc_y': acc_y, 'acc_z': acc_z, 'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z, 'timespan': timespan}
        else:
            return None
        
    def recv_oreintation_data(self):
        msg = self.conn_in.recv_match(type = "ATTITUDE_QUATERNION", blocking=True, timeout= 0.001)
        w, x, y, z = 0.0, 0.0, 0.0, 0.0

        if msg and msg.get_type() == "ATTITUDE_QUATERNION":
            w, x, y, z = msg.q1, msg.q2, msg.q3, msg.q4
            rollspeed, pitchspeed, yawspeed = msg.rollspeed, msg.pitchspeed, msg.yawspeed

            return {'w': w, 'x': x, 'y': y, 'z': z, 'rollspeed': rollspeed, 'pitchspeed': pitchspeed, 'yawspeed': yawspeed}
        else:
            return None
        

    def recv_rpm_data(self):

        msg = self.conn_in.recv_match(type = "ACTUATOR_OUTPUT_STATUS", blocking = True)
        
        rpm = None
        timespan = 0.0

        if msg and msg.get_type() == "ACTUATOR_OUTPUT_STATUS":

            rpm = [msg.actuator[0], msg.actuator[1], msg.actuator[2], msg.actuator[3]]

            timespan = msg.time_usec

            return {'rpm': rpm, 'timespan': timespan}
        else:
            return None

        
        

