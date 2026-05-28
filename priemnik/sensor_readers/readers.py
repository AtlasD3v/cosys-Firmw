import sys

from upravlenie import joy as j
from filters import pt as pt
from filters import notch as n
import time
import threading
from pymavlink import mavutil

class RC_Reader:


    def __init__(self, pt_number, expo_num, alpha, beta,low_amplitude_percent, max_val_in_percent_diapazon ):
        self.pt = pt.PT3()
        self.alpha = alpha
        self.expo = pt.Expo(alpha, beta, low_amplitude_percent, max_val_in_percent_diapazon)
        self.pt_number = pt_number
        self.expo_num = expo_num
        self.JOY_VAL = []#сигнал с пульта
        self.SIG_VAL = []#отфильтрованный и сглаженный сигнал в RPM

    def rc_reader_filter(self, signal):
        res = None

        pt_res = None
        expo_res = None

        if self.pt_number == 1:
            pt_res = self.pt.pt1(signal)
        elif self.pt_number == 3:
            pt_res = self.pt.pt3(signal)
        else:
            print("---!!! Пока что такого порядка ФНЧ нет в прошивке! ---!!!\n")
            self.pt_number = 1 #чтобы программа не заканчивалась - ставим существующий порядок и продолжаем работу (вызываем заново функцию)
            self.rc_reader_filter(signal)

        if self.expo_num == 0:
            expo_res = self.expo.exp(pt_res, self.alpha)
        elif self.expo_num == 1:
            expo_res = self.expo.custom_bust(pt_res)
        else:
            print("---!!! НЕТ ТАКОГО МОДИФИКАТОРА КРИВОЙ УПРАВЛЕНИЯ, СТАВИМ ДЕФОЛТНЫЙ ---!!!\n")
            self.expo_num = 1
            self.rc_reader_filter(signal)

        self.JOY_VAL.append(signal)
        self.SIG_VAL.append(expo_res)

        return expo_res



class PX4Reader:

    def __init__(self, drone):
        self.drone = drone
        self.last_acc_x, self.last_acc_y,self.last_acc_z,self.last_gyro_x,self.gyro_y,self.gyro_z = 0.0,0.0,0.0,0.0,0.0,0.0


    def reader_data(self, drone_conn):
        acc_x, acc_y, acc_z = 0.0, 0.0, 0.0
        gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0
        timespan = 0.0
        # msg = drone_conn.recv_match(type = "HIGHRES_IMU", blocking=True, timeout= 0.001)
        msg = self.drone.recv_match(type = "HIGHRES_IMU", blocking=True, timeout= 0.001)

        if msg is not None:
            acc_x, acc_y, acc_z = msg.xacc, msg.yacc, msg.zacc
            gyro_x, gyro_y, gyro_z = msg.xgyro, msg.ygyro, msg.zgyro
            self.last_acc_x, self.last_acc_y,self.last_acc_z,self.last_gyro_x,self.gyro_y,self.gyro_z = acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
            timespan = msg.time_usec
        else:
            print("СООБЩЕНИЕ ОТ IMU = None")
            acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z = self.last_acc_x, self.last_acc_y,self.last_acc_z,self.last_gyro_x,self.gyro_y,self.gyro_z

        return {'acc_x': acc_x, 'acc_y': acc_y, 'acc_z': acc_z, 'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z, 'timespan': timespan}


    def reader_RPM_data(self):
        rpm1, rpm2, rpm3, rpm4 = 0.0, 0.0, 0.0, 0.0

        msg = self.drone.recv_match(type = "RAW_RPM", blocking=True, timeout = 0.02)
        if msg is not None:
            print(f"Motor: {msg.index}, RPM: {msg.frequency}")
        else:
            print("НЕ НАШЛИ СООБЩЕНИЕ О СКОРОСТИ ВРАЩЕНИЯ МОТОРОВ")

        return [rpm1, rpm2, rpm3, rpm4]

