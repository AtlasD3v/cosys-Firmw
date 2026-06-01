# import numpy as np
# from scipy.spatial.transform import Rotation as R 

# class Quat:
#     def __init__(self):
#         self.w = 1.0
#         self.x = 0.0
#         self.y = 0.0
#         self.z = 0.0

#     def fill_quat(self, w, x, y, z, quat_arr = None):
#         if quat_arr is None:
#             self.w = w
#             self.x = x
#             self.y = y
#             self.z = z
#         else:
#             self.w = quat_arr[0]
#             self.x = quat_arr[1]
#             self.y = quat_arr[2]
#             self.z = quat_arr[3]

#     def return_quat_as_arr(self):

#         return [self.w, self.x, self.y, self.z]


# class Mathmodel:
#     def __init__(self):
#         pass

#     def forward_movement(self, old_quat, pos, vel, acc_arr, last_acc, gyro_arr, acc_bias, gyro_bias, dt): #поступательное движение
#         gyro_corrected = [gyro_arr[0] - gyro_bias[0], gyro_arr[1] - gyro_bias[1], gyro_arr[2] - gyro_bias[2]] #так как мы вызываем forward_movement для каждой кубатурной точки, то и каждый раз мы получаем разные гиро и гиро биас, следоательно очищать гиро надо здесь
#         acc_corrected = [acc_arr[0] - acc_bias[0], acc_arr[1] - acc_bias[1], acc_arr[2] - acc_bias[2]]

#         actual_quat = self.angular_movement(old_quat, gyro_corrected, dt) #нашли текущий обновлённый кватернион
#         #теперь можно поворачивать вектор ускорений, а затем находить скорость, перемещение
#         rotated_acc = self.rotate_vector(actual_quat, acc_corrected)
#         rotated_acc[2] += 9.806

#         new_pos, new_vel  = self.trapezoid_method(pos, vel, last_acc, rotated_acc, dt)
#         new_last_acc = acc_corrected
        
#         return new_pos, new_vel, new_last_acc




#     def angular_movement(self, old_quat, gyro_arr, dt): #вращательное движение
#         #получаем угловое ускорение, время с последнего измерения, находим угловое "перемещение", обновляем кватернион ориентации с учётом текущих угловых скоростей
#         actual_quat = self.integrate_exp(old_quat=old_quat, gyro_arr=gyro_arr, dt=dt)
#         actual_quat = self.normalize_quat(actual_quat)

#         return actual_quat

#     def trapezoid_method(self, pos, vel, last_acc, current_acc, dt):
#         v_mid = []
#         for x in range(len(vel)):
#             v_mid[x] = vel[x] + (last_acc[x] * (dt / 2.0))

#         new_pos = []
#         for x in range(len(pos)):
#             new_pos[x] = pos[x] + (v_mid[x] * dt)

#         new_vel = []
#         for x in range(len(vel)):
#             new_vel[x] = v_mid[x] + (current_acc[x] * (dt / 2.0))

#         return new_pos, new_vel

#     def integrate_exp(self, old_quat, gyro_arr, dt): #экспоненциальное отображение
#         """
#         old_quat_arr : массив [w, x, y, z]
#         gyro_arr     : угловая скорость [wx, wy, wz] в rad/s
#         dt           : время шага
#         """


#         #считаем вектор приращения угла
#         angle_gain_x = gyro_arr[0] * dt #находим прирост угла x
#         angle_gain_y = gyro_arr[1] * dt #находим прирост угла y
#         angle_gain_z = gyro_arr[2] * dt #находим прирост угла z

#         #квадрат угла
#         theta_sq = angle_gain_x**2 + angle_gain_y**2 + angle_gain_z**2
#         theta = np.sqrt(theta_sq)

#         #Вычисляем дельта-кватернион с защитой от малых углов

#         dq = Quat() #создаём новый кватернион [1.0, 0.0, 0.0, 0.0] вида [w, x, y, z]

#         if theta_sq < 1e-8:
#             #Если вращение ничтожно мало, используем упрощенную форму (ряд Тейлора)
#             # dq.w = 1.0 - theta_sq / 8.0
#             dq.w = 1.0 - theta_sq / 8.0 + (theta_sq * theta_sq) / 384.0
#             dq.x = angle_gain_x * 0.5
#             dq.y = angle_gain_y * 0.5
#             dq.z = angle_gain_z * 0.5
#         else:
#             half_theta = theta * 0.5
#             s = np.sin(half_theta) / theta #коэффициент масштабирования оси
#             dq.w = np.cos(half_theta)
#             dq.x = angle_gain_x * s
#             dq.y = angle_gain_y * s
#             dq.z = angle_gain_z * s
        
#         new_quat = dq.return_quat_as_arr() #возвращаем кватернион в виде массива

#         result = self.quat_mult(old_quat, new_quat) #old_quat должен быть в виде массива, умножение идёт в таком порядке: старый кватернион * новый кватернион

#         return result


#     def rotate_vector(self, quat, vector):
#         """
#         Поворачивает вектор vec по кватерниону quat.
        
#         vec  : массив [x, y, z]
#         quat : кватернион в формате [w, x, y, z] (по умолчанию для scipy)
#         """
#         rotation = R.from_quat(quat, scalar_first=True)
#         return rotation.apply(vector)

#     def quat_mult(self, quat1, quat2): #принимает кватернионы в виде массива
#         rot1 = R.from_quat(quat1, scalar_first=True)
#         rot2 = R.from_quat(quat2, scalar_first=True)

#         return (rot1 * rot2).as_quat()
    
#     def normalize_quat(self, quat):


#         norm = np.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2)

#         return [quat[0] / norm, quat[1] / norm, quat[2] / norm, quat[3] / norm]


import math
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np


@dataclass
class Quat:
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def fill_quat(self, w, x, y, z, quat_arr=None):
        if quat_arr is None:
            self.w = float(w)
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
        else:
            self.w = float(quat_arr[0])
            self.x = float(quat_arr[1])
            self.y = float(quat_arr[2])
            self.z = float(quat_arr[3])

    def return_quat_as_arr(self):
        return [self.w, self.x, self.y, self.z]


class Mathmodel:
    """
    Кинематическая модель для CKF.

    Соглашение:
    - кватернион храним как [w, x, y, z] (scalar-first);
    - неподвижный акселерометр выдаёт [0, 0, -g] в body-frame;
    - ускорение поворачиваем в world-frame и прибавляем +g по Z
      (тогда у неподвижного аппарата мировое ускорение = 0).
    """

    GRAVITY = 9.81

    def __init__(self):
        pass

    @staticmethod
    def _to_list3(v: Sequence[float]) -> List[float]:
        if len(v) != 3:
            raise ValueError(f"Expected 3-vector, got length {len(v)}")
        return [float(v[0]), float(v[1]), float(v[2])]

    def forward_movement(self, old_quat, pos, vel, acc_arr, last_acc,
                         gyro_arr, acc_bias, gyro_bias, dt):
        gyro_arr = self._to_list3(gyro_arr)
        acc_arr = self._to_list3(acc_arr)
        acc_bias = self._to_list3(acc_bias)
        gyro_bias = self._to_list3(gyro_bias)
        pos = self._to_list3(pos)
        vel = self._to_list3(vel)
        last_acc = self._to_list3(last_acc)

        gyro_corrected = [gyro_arr[i] - gyro_bias[i] for i in range(3)]
        acc_corrected = [acc_arr[i] - acc_bias[i] for i in range(3)]

        actual_quat = self.angular_movement(old_quat, gyro_corrected, dt)

        rotated_acc = self.rotate_vector(actual_quat, acc_corrected)
        rotated_acc[2] += self.GRAVITY

        new_pos, new_vel = self.trapezoid_method(pos, vel, last_acc, rotated_acc, dt)

        # Возвращаем уже приведённое к world-frame ускорение для следующего шага.
        new_last_acc = rotated_acc

        return new_pos, new_vel, new_last_acc

    def angular_movement(self, old_quat, gyro_arr, dt):
        actual_quat = self.integrate_exp(old_quat=old_quat, gyro_arr=gyro_arr, dt=dt)
        actual_quat = self.normalize_quat(actual_quat)
        return actual_quat

    def trapezoid_method(self, pos, vel, last_acc, current_acc, dt):
        pos = self._to_list3(pos)
        vel = self._to_list3(vel)
        last_acc = self._to_list3(last_acc)
        current_acc = self._to_list3(current_acc)

        v_mid = [vel[i] + last_acc[i] * (dt * 0.5) for i in range(3)]
        new_pos = [pos[i] + v_mid[i] * dt for i in range(3)]
        new_vel = [v_mid[i] + current_acc[i] * (dt * 0.5) for i in range(3)]

        return new_pos, new_vel

    def integrate_exp(self, old_quat, gyro_arr, dt):
        """
        old_quat : [w, x, y, z]
        gyro_arr : [wx, wy, wz] в rad/s
        dt       : шаг интегрирования (с)
        """
        old_quat = self.normalize_quat(old_quat)
        gyro_arr = self._to_list3(gyro_arr)

        tx = gyro_arr[0] * dt
        ty = gyro_arr[1] * dt
        tz = gyro_arr[2] * dt

        theta_sq = tx * tx + ty * ty + tz * tz
        theta = math.sqrt(theta_sq)

        dq = Quat()

        if theta_sq < 1e-8:
            dq.w = 1.0 - theta_sq / 8.0 + (theta_sq * theta_sq) / 384.0
            dq.x = 0.5 * tx
            dq.y = 0.5 * ty
            dq.z = 0.5 * tz
        else:
            half_theta = 0.5 * theta
            s = math.sin(half_theta) / theta
            dq.w = math.cos(half_theta)
            dq.x = tx * s
            dq.y = ty * s
            dq.z = tz * s

        return self.quat_mult(old_quat, dq.return_quat_as_arr())

    def rotate_vector(self, quat, vector):
        quat = self.normalize_quat(quat)
        vector = self._to_list3(vector)

        w, x, y, z = quat
        vx, vy, vz = vector

        r00 = 1.0 - 2.0 * (y * y + z * z)
        r01 = 2.0 * (x * y - w * z)
        r02 = 2.0 * (x * z + w * y)

        r10 = 2.0 * (x * y + w * z)
        r11 = 1.0 - 2.0 * (x * x + z * z)
        r12 = 2.0 * (y * z - w * x)

        r20 = 2.0 * (x * z - w * y)
        r21 = 2.0 * (y * z + w * x)
        r22 = 1.0 - 2.0 * (x * x + y * y)

        return [
            r00 * vx + r01 * vy + r02 * vz,
            r10 * vx + r11 * vy + r12 * vz,
            r20 * vx + r21 * vy + r22 * vz,
        ]

    def quat_mult(self, quat1, quat2):
        quat1 = self.normalize_quat(quat1)
        quat2 = self.normalize_quat(quat2)

        w1, x1, y1, z1 = quat1
        w2, x2, y2, z2 = quat2

        return [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]

    def normalize_quat(self, quat):
        """
        Чистая нормировка к единичной норме.
        ВАЖНО: без принудительного приведения к полусфере w>=0 — иначе
        ломается симметрия кубатурных точек и портится ковариация.
        Выравнивание знака делается отдельно при усреднении среднего.
        """
        if isinstance(quat, Quat):
            q = np.array([quat.w, quat.x, quat.y, quat.z], dtype=float)
        else:
            q = np.asarray(quat, dtype=float).reshape(-1)

        if q.shape != (4,):
            raise ValueError(f"Expected quaternion of length 4, got shape {q.shape}")

        norm = float(np.linalg.norm(q))
        if not np.isfinite(norm) or norm < 1e-12:
            return [1.0, 0.0, 0.0, 0.0]

        q = q / norm
        return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]


# Backward-compatible alias
CKFMathModel = Mathmodel



