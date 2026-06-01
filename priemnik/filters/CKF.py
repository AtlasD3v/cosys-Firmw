# import numpy as np
# import scipy as sc
# from dataclasses import dataclass
# from typing import List
# from math_model import CKFMathModel as math_model



# @dataclass
# #В вектор состояния включаются только те величины, которые мы хотим оценить (т.е. получить наиболее точное и очищенное значение), и которые динамически связаны между собой
# #  через модель движения.
# #Мы не включаем в вектор состояния всё, что приходит с датчиков.
# #Сырые измерения (raw gyro, raw accel) используются как измерения (measurements) в фильтре, а не как часть состояния.

# class State:
#     # Кватернион (w, x, y, z) — порядок очень важен!
#     q: List[float] = None

#     # Положение (x, y, z)
#     pos: List[float] = None
    
#     # Линейная скорость (vx, vy, vz)
#     vel: List[float] = None
    
#     # Угловая скорость (wx, wy, wz)
#     ang_vel: List[float] = None
    
#     # Bias (смещения) гироскопа
#     gyro_bias: List[float] = None
    
#     # Bias акселерометра
#     acc_bias: List[float] = None

#     #скорость ветра
#     wind_vel: List[float] = None

#     def __post_init__(self):
#         """Инициализация значений по умолчанию"""
#         if self.q is None:
#             self.q = [1.0, 0.0, 0.0, 0.0]
#         if self.vel is None:
#             self.vel = [0.0, 0.0, 0.0]
#         if self.ang_vel is None:
#             self.ang_vel = [0.0, 0.0, 0.0]
#         if self.pos is None:
#             self.pos = [0.0, 0.0, 0.0]
#         if self.gyro_bias is None:
#             self.gyro_bias = [0.0, 0.0, 0.0]
#         if self.acc_bias is None:
#             self.acc_bias = [0.0, 0.0, 0.0]
#         if self.wind_vel is None:
#             self.wind_vel = [0.0, 0.0, 0.0]
    
#     def state_to_vector(self):

#         return [self.q[0], self.q[1], self.q[2], self.q[3], self.pos[0], self.pos[1], self.pos[2], self.vel[0], self.vel[1], self.vel[2],
#                  self.ang_vel[0], self.ang_vel[1], self.ang_vel[2], self.gyro_bias[0], self.gyro_bias[1], self.gyro_bias[2], self.acc_bias[0],
#                    self.acc_bias[1], self.acc_bias[2], self.wind_vel[0], self.wind_vel[1], self.wind_vel[2]]
    
#     def vector_to_state(self, vector):
#         self.q = vector[:4]
#         self.pos = vector[4:7]
#         self.vel = vector[7:10]
#         self.ang_vel = vector[10:13]
#         self.gyro_bias = vector[13:16]
#         self.acc_bias = vector[16:19]
#         self.wind_vel = vector[19:22]



# class CKF:
#     def __init__(self):
#         P_matrix = None #матрица ковариации состояний (ошибки оценки состояний). Эта матрица имеет размер n*n, где n - размерность вектора состояния
#         self.initialize_P_matrix() #вызываем функцию инициализации матрицы P


#         self.last_acc = [0.0, 0.0, 0.0]
#         self.math_model = math_model.Mathmodel()
#         self.state = State() #вектор состояний




#     def initialize_P_matrix(self):
#         #матрица P изначально инициализируется только дисперсиями (дисперсия - мера разброса величины от её мат.ожидания. Мат.ожидание - среднее) каждой из переменных в векторе состояния.
#         #Впжно отметить, что всё указывается в квадрате, то есть фактические величины будут равны sqrt(указанная_величина)
#         q_w = 0.01 #qw: - дисперсия кватерниона. Стандартное отклонение = ≈ 0.1 (неопределённость ~5.7° в ориентации)
#         q_x = 0.01
#         q_y = 0.01
#         q_z = 0.01

#         p_x = 0.25 #m дисперсия по позиции. Стандартное отклонение = 0.5 м
#         p_y = 0.25 
#         p_z = 0.25

#         v_x = 0.01 #m\s дисперсия по скорости. Стандартное отклонение = 0.1 м\с
#         v_y = 0.01
#         v_z = 0.01

#         gyro_x = 0.01 #rad\s дисп. гироскопа. Стандартное отклонение = 0.1 рад\сек
#         gyro_y = 0.01
#         gyro_z = 0.01

#         gyro_bias_x = 0.0001 #rad\s дисп. смещения гироскопа. Стандартное отклонение = 0.01 рад\с
#         gyro_bias_y = 0.0001
#         gyro_bias_z = 0.0001

#         acc_bias_x = 2.5e-3 #м\с^2 дисперсия смещения акселерометра. Стандартное отклонение = 0.05 м\с^2
#         acc_bias_y = 2.5e-3
#         acc_bias_z = 2.5e-3

#         self.P_matrix = np.diag([q_w, q_x, q_y, q_z, p_x, p_y, p_z, v_x, v_y, v_z, gyro_x, gyro_y, gyro_z, gyro_bias_x, gyro_bias_y, gyro_bias_z, acc_bias_x, acc_bias_y, acc_bias_z])

    
#     def predict_step(self, gyro, acc, dt):
#         #получаем матрицу S, применяя к матрице P разложение холецкого
#         S_matrix = np.linalg.cholesky(self.P_matrix, upper=False) #нам нужна нижнетреугольная матрица, поэтому upper=False
#         state_arr = self.state.state_to_vector() #получаем вектор состояния в виде массива
#         cube_points = self.generate_cubature_points(S_matrix, state_arr) #генерация кубатурных точек


#         for x in range(len(cube_points)):
#             #переводим состояния (кубатурные точки) в массив
#             state_from_cube_points = State()
#             state_x = state_from_cube_points.vector_to_state(cube_points[x])

#             new_state_with_cube_points = self.math_model.forward_movement(
#                 old_quat=state_from_cube_points.q,
#                 pos=state_from_cube_points.pos,
#                 vel=state_from_cube_points.vel,
#                 acc_arr=acc,
#                 last_acc=self.last_acc,
#                 gyro_arr=gyro,
#                 acc_bias=state_from_cube_points.acc_bias,
#                 gyro_bias=state_from_cube_points.gyro_bias,
#                 dt = dt
#             )
            



#     def correct_step(self):
#         pass
    

#     def generate_cubature_points(self, S_matrix, state):
#         #каждая кубатурная точка - отедльный вектор состояния такой же, как и обычный, но кубатурная точка отображает "гипотетическое состояние дрона",
#         # которое мы специально сконструировали, чтобы оно отражало неопределённость нашей оценки.

#         cubature_points = [] #размер будет 2 * len(state)
#         size = len(state)
#         sqrt_n = np.sqrt(size)

#         for col in range(size):
#             Xi_plus = []
#             Xi_minus = []

#             for row in range(size):
#                 Xi_plus[row] = state[row] + sqrt_n * S_matrix[row][col]
#                 Xi_minus[row] = state[row] - sqrt_n * S_matrix[row][col]
            

#             new_state_plus = State() #создаём новые состояния
#             new_state_minus = State()

#             new_state_plus.vector_to_state(Xi_plus)
#             new_state_minus.vector_to_state(Xi_minus)

#             #нормализуем кватернион
#             new_state_plus_quat_norm = self.math_model.normalize_quat(new_state_plus.q)
#             new_state_minus_quat_norm = self.math_model.normalize_quat(new_state_minus.q)

#             new_state_plus.q = new_state_plus_quat_norm
#             new_state_minus.q = new_state_minus_quat_norm


#             cubature_points[col] = new_state_plus.state_to_vector()
#             cubature_points[size + col] = new_state_minus.state_to_vector()
        
#         return cubature_points


        


import math
from dataclasses import dataclass
from typing import List

import numpy as np

from math_model.CKFMathModel import Mathmodel


@dataclass
class State:
    # Кватернион (w, x, y, z)
    q: List[float] = None
    # Линейная скорость (vx, vy, vz)
    vel: List[float] = None
    # Угловая скорость (wx, wy, wz)
    ang_vel: List[float] = None
    # Положение (x, y, z)
    pos: List[float] = None
    # Bias гироскопа
    gyro_bias: List[float] = None
    # Bias акселерометра
    acc_bias: List[float] = None
    # Зарезервировано на будущее (не входит в вектор состояния)
    wind_vel: List[float] = None

    def __post_init__(self):
        if self.q is None:
            self.q = [1.0, 0.0, 0.0, 0.0]
        if self.vel is None:
            self.vel = [0.0, 0.0, 0.0]
        if self.ang_vel is None:
            self.ang_vel = [0.0, 0.0, 0.0]
        if self.pos is None:
            self.pos = [0.0, 0.0, 0.0]
        if self.gyro_bias is None:
            self.gyro_bias = [0.0, 0.0, 0.0]
        if self.acc_bias is None:
            self.acc_bias = [0.0, 0.0, 0.0]
        if self.wind_vel is None:
            self.wind_vel = [0.0, 0.0, 0.0]

    def state_to_vector(self):
        # [q(4), vel(3), ang_vel(3), pos(3), gyro_bias(3), acc_bias(3)] = 19
        return [
            self.q[0], self.q[1], self.q[2], self.q[3],
            self.vel[0], self.vel[1], self.vel[2],
            self.ang_vel[0], self.ang_vel[1], self.ang_vel[2],
            self.pos[0], self.pos[1], self.pos[2],
            self.gyro_bias[0], self.gyro_bias[1], self.gyro_bias[2],
            self.acc_bias[0], self.acc_bias[1], self.acc_bias[2],
        ]

    def vector_to_state(self, vector):
        vec = np.asarray(vector, dtype=float).reshape(-1)
        if vec.shape != (19,):
            raise ValueError(f"State vector must have length 19, got shape {vec.shape}")

        self.q = vec[0:4].tolist()
        self.vel = vec[4:7].tolist()
        self.ang_vel = vec[7:10].tolist()
        self.pos = vec[10:13].tolist()
        self.gyro_bias = vec[13:16].tolist()
        self.acc_bias = vec[16:19].tolist()
        self.wind_vel = [0.0, 0.0, 0.0]
        return self


class CKF:
    STATE_SIZE = 19
    GPS_SIZE = 6

    def __init__(self, dt: float = 0.001):
        self.math_model = Mathmodel()
        self.state = State()
        self.last_acc = [0.0, 0.0, 0.0]
        self.dt = float(dt)

        self.P_matrix = None
        self.initialize_P_matrix()

        self.Q = self._make_Q(self.dt)
        self.R_GPS = self._make_R_gps()

    # ------------------------------------------------------------------ #
    #  Инициализация матриц
    # ------------------------------------------------------------------ #
    def initialize_P_matrix(self):
        # ВНИМАНИЕ: q_var держим маленьким. В наивном (аддитивном) кватернионном
        # CKF большая дисперсия ориентации создаёт паразитное ускорение из-за
        # нелинейности поворота. 1e-4 ~ малые углы -> линейный режим.
        q_var = 1e-4
        v_var = 0.01
        w_var = 0.01
        p_var = 0.25
        gyro_bias_var = 1e-4
        acc_bias_var = 2.5e-3

        self.P_matrix = np.diag([
            q_var, q_var, q_var, q_var,
            v_var, v_var, v_var,
            w_var, w_var, w_var,
            p_var, p_var, p_var,
            gyro_bias_var, gyro_bias_var, gyro_bias_var,
            acc_bias_var, acc_bias_var, acc_bias_var,
        ]).astype(float)

    def _make_Q(self, dt: float):
        Q = np.zeros((self.STATE_SIZE, self.STATE_SIZE), dtype=float)

        sigma_g = 0.005 * (math.pi / 180.0)  # рад/с/sqrt(Гц)
        sigma_a = 0.003                      # м/с²/sqrt(√Гц)

        q_quat = sigma_g * sigma_g * dt
        q_vel = sigma_a * sigma_a * dt
        q_w = sigma_g * sigma_g * dt
        q_pos = q_vel * dt * dt

        for i in range(4):
            Q[i, i] = q_quat
        for i in range(4, 7):
            Q[i, i] = q_vel
        for i in range(7, 10):
            Q[i, i] = q_w
        for i in range(10, 13):
            Q[i, i] = q_pos
        for i in range(13, 16):
            Q[i, i] = 1e-8 * max(dt / 0.001, 1.0)
        for i in range(16, 19):
            Q[i, i] = 1e-7 * max(dt / 0.001, 1.0)

        return Q

    def _make_R_gps(self):
        R = np.zeros((self.GPS_SIZE, self.GPS_SIZE), dtype=float)
        R[0, 0] = 9.0     # дисп_px
        R[1, 1] = 9.0     # дисп_py
        R[2, 2] = 25.0    # дисп_pz
        R[3, 3] = 0.01    # дисп_vx
        R[4, 4] = 0.01    # дисп_vy
        R[5, 5] = 0.04    # дисп_vz
        return R

    # ------------------------------------------------------------------ #
    #  Численные утилиты
    # ------------------------------------------------------------------ #
    @staticmethod
    def _symmetrize(mat):
        mat = np.asarray(mat, dtype=float)
        return 0.5 * (mat + mat.T)

    def _cholesky_lower(self, P):
        P = self._symmetrize(P)
        eye = np.eye(P.shape[0], dtype=float)

        jitter = 1e-12
        for _ in range(10):
            try:
                return np.linalg.cholesky(P + jitter * eye)
            except np.linalg.LinAlgError:
                jitter *= 10.0

        return np.linalg.cholesky(P + 1e-6 * eye)

    # ------------------------------------------------------------------ #
    #  Кубатурные точки
    # ------------------------------------------------------------------ #
    def generate_cubature_points(self, S_matrix, state):
        state = np.asarray(state, dtype=float).reshape(-1)
        n = state.size
        if n != self.STATE_SIZE:
            raise ValueError(f"Expected state size {self.STATE_SIZE}, got {n}")

        S_matrix = np.asarray(S_matrix, dtype=float)
        if S_matrix.shape != (n, n):
            raise ValueError(f"Expected S shape {(n, n)}, got {S_matrix.shape}")

        sqrt_n = math.sqrt(n)
        points = np.zeros((2 * n, n), dtype=float)

        for col in range(n):
            delta = sqrt_n * S_matrix[:, col]
            xi_plus = state + delta
            xi_minus = state - delta

            xi_plus[:4] = np.asarray(self.math_model.normalize_quat(xi_plus[:4]), dtype=float)
            xi_minus[:4] = np.asarray(self.math_model.normalize_quat(xi_minus[:4]), dtype=float)

            points[col] = xi_plus
            points[n + col] = xi_minus

        return points

    def compute_predicted_mean(self, propagated):
        propagated = np.asarray(propagated, dtype=float)
        if propagated.shape != (2 * self.STATE_SIZE, self.STATE_SIZE):
            raise ValueError(f"Unexpected propagated shape {propagated.shape}")

        # Выравниваем знак кватернионов перед арифметическим усреднением:
        # q и -q описывают один и тот же поворот, иначе они "схлопнутся".
        ref_q = propagated[0, :4].copy()
        aligned = propagated.copy()
        for i in range(1, aligned.shape[0]):
            if float(np.dot(aligned[i, :4], ref_q)) < 0.0:
                aligned[i, :4] *= -1.0

        mean_array = np.mean(aligned, axis=0)
        mean_array[:4] = np.asarray(self.math_model.normalize_quat(mean_array[:4]), dtype=float)
        return mean_array

    def compute_predicted_cov(self, propagated, mean):
        propagated = np.asarray(propagated, dtype=float)
        mean = np.asarray(mean, dtype=float)

        dx = propagated - mean
        weight = 1.0 / propagated.shape[0]

        P_pred = weight * (dx.T @ dx)
        P_pred += self.Q
        return self._symmetrize(P_pred)

    # ------------------------------------------------------------------ #
    #  ШАГ 1: ПРЕДСКАЗАНИЕ
    # ------------------------------------------------------------------ #
    def predict_step(self, gyro, acc, dt=None):
        gyro = np.asarray(gyro, dtype=float).reshape(-1)
        acc = np.asarray(acc, dtype=float).reshape(-1)
        if gyro.shape != (3,) or acc.shape != (3,):
            raise ValueError("gyro and acc must be 3-vectors")

        if dt is None:
            dt = self.dt
        dt = float(dt)
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        self.Q = self._make_Q(dt)

        S_matrix = self._cholesky_lower(self.P_matrix)
        state_arr = np.asarray(self.state.state_to_vector(), dtype=float)
        cube_points = self.generate_cubature_points(S_matrix, state_arr)

        propagated = np.zeros_like(cube_points)
        for i, cube_point in enumerate(cube_points):
            s = State().vector_to_state(cube_point)

            new_pos, new_vel, _ = self.math_model.forward_movement(
                old_quat=s.q,
                pos=s.pos,
                vel=s.vel,
                acc_arr=acc,
                last_acc=self.last_acc,
                gyro_arr=gyro,
                acc_bias=s.acc_bias,
                gyro_bias=s.gyro_bias,
                dt=dt,
            )

            gyro_corrected = (gyro - np.asarray(s.gyro_bias, dtype=float)).tolist()
            new_q = self.math_model.angular_movement(s.q, gyro_corrected, dt)

            s.q = new_q
            s.pos = new_pos
            s.vel = new_vel
            s.ang_vel = gyro_corrected
            propagated[i] = np.asarray(s.state_to_vector(), dtype=float)

        mean_state = self.compute_predicted_mean(propagated)
        self.P_matrix = self.compute_predicted_cov(propagated, mean_state)
        self.state = State().vector_to_state(mean_state.tolist())

        # Обновляем last_acc от центрального состояния
        acc_corrected = (acc - np.asarray(self.state.acc_bias, dtype=float)).tolist()
        acc_ned = self.math_model.rotate_vector(self.state.q, acc_corrected)
        acc_ned[2] += self.math_model.GRAVITY
        self.last_acc = acc_ned

        return self.state

    # ------------------------------------------------------------------ #
    #  Функции измерений
    # ------------------------------------------------------------------ #
    def h_gps(self, cub_state):
        return np.array([
            cub_state.pos[0], cub_state.pos[1], cub_state.pos[2],
            cub_state.vel[0], cub_state.vel[1], cub_state.vel[2],
        ], dtype=float)

    def h_baro(self, cub_state):
        return np.array([cub_state.pos[2]], dtype=float)

    def h_mag(self, cub_state):
        # Заглушка под будущий магнитометр.
        return np.array([0.0, 0.0, 0.0], dtype=float)

    # ------------------------------------------------------------------ #
    #  Ковариации измерения
    # ------------------------------------------------------------------ #
    def covariation_of_innovation(self, cubs, mean):
        cubs = np.asarray(cubs, dtype=float)
        mean = np.asarray(mean, dtype=float)

        dz = cubs - mean
        weight = 1.0 / cubs.shape[0]
        Szz = weight * (dz.T @ dz)
        Szz += self.R_GPS
        return self._symmetrize(Szz)

    def perekrest_cov_of_gps_data(self, array_state, cub_points,
                                  new_data_from_h_with_cub, mean_GPS):
        array_state = np.asarray(array_state, dtype=float)
        cub_points = np.asarray(cub_points, dtype=float)
        new_data_from_h_with_cub = np.asarray(new_data_from_h_with_cub, dtype=float)
        mean_GPS = np.asarray(mean_GPS, dtype=float)

        dx = cub_points - array_state
        dz = new_data_from_h_with_cub - mean_GPS
        weight = 1.0 / cub_points.shape[0]
        return weight * (dx.T @ dz)

    def _compute_K(self, Pxz, Szz):
        # K = Pxz * Szz^{-1}; solve устойчивее явного обращения.
        Szz = self._symmetrize(np.asarray(Szz, dtype=float))
        Pxz = np.asarray(Pxz, dtype=float)

        jitter = 1e-12
        eye = np.eye(Szz.shape[0], dtype=float)
        for _ in range(10):
            try:
                return np.linalg.solve(Szz + jitter * eye, Pxz.T).T
            except np.linalg.LinAlgError:
                jitter *= 10.0

        return np.linalg.solve(Szz + 1e-6 * eye, Pxz.T).T

    def _correct_state(self, GPS_array, z_mean, K):
        x_total = np.asarray(self.state.state_to_vector(), dtype=float)
        innovation = np.asarray(GPS_array, dtype=float) - np.asarray(z_mean, dtype=float)
        x_total = x_total + K @ innovation
        x_total[:4] = np.asarray(self.math_model.normalize_quat(x_total[:4]), dtype=float)
        self.state = State().vector_to_state(x_total.tolist())

    def _correct_covariance(self, K, Szz):
        self.P_matrix = self.P_matrix - K @ Szz @ K.T
        self.P_matrix = self._symmetrize(self.P_matrix)

        eig_min = float(np.min(np.linalg.eigvalsh(self.P_matrix)))
        if not np.isfinite(eig_min):
            self.P_matrix = np.eye(self.STATE_SIZE, dtype=float) * 1e-6
        elif eig_min <= 1e-12:
            self.P_matrix += np.eye(self.STATE_SIZE, dtype=float) * (1e-6 - eig_min)

    # ------------------------------------------------------------------ #
    #  ШАГ 2: КОРРЕКЦИЯ (GPS)
    # ------------------------------------------------------------------ #
    def correction_step(self, GPS_data):
        GPS_data = np.asarray(GPS_data, dtype=float).reshape(-1)
        if GPS_data.shape != (self.GPS_SIZE,):
            raise ValueError(f"GPS_data must have shape {(self.GPS_SIZE,)}, got {GPS_data.shape}")

        S_matrix = self._cholesky_lower(self.P_matrix)
        cub_points = self.generate_cubature_points(S_matrix, self.state.state_to_vector())

        h_data = np.array(
            [self.h_gps(State().vector_to_state(cp)) for cp in cub_points],
            dtype=float,
        )
        mean_gps = np.mean(h_data, axis=0)
        Szz = self.covariation_of_innovation(h_data, mean_gps)
        Pxz = self.perekrest_cov_of_gps_data(
            self.state.state_to_vector(), cub_points, h_data, mean_gps
        )

        K = self._compute_K(Pxz, Szz)
        self._correct_state(GPS_data, mean_gps, K)
        self._correct_covariance(K, Szz)

        return self.state