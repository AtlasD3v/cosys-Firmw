import numpy as np


class Allocator:
    def __init__(self, number_motors_configuration):
        self.M_roll_max = 0.0 #Максимальные моменты
        self.M_pitch_max = 0.0
        self.M_yaw_max = 0.0
        self.M_thrust_max = 0.0

        self.k_tau = 0.109919#пока что этот параметр находится в хардкоде, так как его измеряют либо на стенде, либо специальным алгоритмом при калибровке

        # self.Allocate_matrix = self.find_A_matrix(1) #матрица аллокации (распределения тяги\моментов тяги по моторам)

        self.number_motors_configuration = number_motors_configuration #номер конфигурации моторов
        self.distance_from_center_of_mass_to_motor = 0.23 #m
        self.max_motor_thrust_physicaly = 4.179446268 #максимальная тяга (в Ньютонах), которая берётся из документации производителя роторов
        self.procent_of_max_physical_thrust = 0.95 #Эта переменная обозначает максимум (в процентах), который мы можем взять от тяги, которую может обеспечить мотор (берём не 1.0, чтобы мотор не работал на пределе своих физических возможностей)
        self.max_motor_thrust = self.max_motor_thrust_physicaly * self.procent_of_max_physical_thrust #Эта переменная является ограничителем тяги для работы каждого из моторов квадрокоптера (максимальная тяга одного мотора на программном уровне)
        self.min_motor_thrust = 1.0 #минимальная тяга мотора 1H (настраивается)
        self.four_motors_max_thrust = 4.0 * self.max_motor_thrust #максимальная тяга, которую могут создать моторы квадрокоптера (все 4 мотора совместно)
        self.M_max_array = []
        
        self.B_matrix = None
        self.B_pinv = None#запускаем функцию нахождения матрицы B, с параметров 1 для нахождения матрицы А (если мотор должен ослабиться\ничего не делать для момента по оси,


        # self.calculating_params_to_qp() #вызываем функцию, которая заполняет параметры canonic_P, canonic_P_cvxopt, canonic_q, B_matrix, self.G_cvxopt
        self.find_first_scaling_coeffs(self.number_motors_configuration) #запускаем код, который установить значения в коэффициенеты максимальных моментов тяг по осям

        self.precision = 4


    def find_first_scaling_coeffs(self, motor_config_number):
        #вычислять максимальные моменты будем динамически, исходя из коллективной тяги, которая запрашивается (то есть у нас есть [Mx, My, Mz, T] - где T - запрашиваемая коллективная тяга)

        self.find_B_matrix(motor_config_number) #вызываем функцию, которая находит матрицу B и B_pinv (переменные self.B_matrix и self.B_pinv заполняются)
        
        motor_thrust1, motor_thrust2, motor_thrust3, motor_thrust4 = None, None, None, None

        if motor_config_number == 1:
            motor_thrust1 = np.array([self.max_motor_thrust, self.min_motor_thrust, self.min_motor_thrust, self.max_motor_thrust])#левая ВМГ на максимум, правая - на минимум
            motor_thrust2 = np.array([self.max_motor_thrust, self.max_motor_thrust, self.min_motor_thrust, self.min_motor_thrust])#чтобы получить положительный момент по pitch, нужно, чтобы передние моторы работали на всю, а задние - на минимум
            motor_thrust3 = np.array([self.min_motor_thrust, self.max_motor_thrust, self.min_motor_thrust, self.max_motor_thrust])#моторы CW - минимум, CCW - максимум
            motor_thrust4 = np.array([self.max_motor_thrust, self.max_motor_thrust, self.max_motor_thrust, self.max_motor_thrust])#все моторы максимум
        elif motor_config_number == 2:
            motor_thrust1 = np.array([self.max_motor_thrust, self.min_motor_thrust, self.min_motor_thrust, self.max_motor_thrust])
            motor_thrust2 = np.array([self.max_motor_thrust, self.max_motor_thrust, self.min_motor_thrust, self.min_motor_thrust])
            motor_thrust3 = np.array([self.max_motor_thrust, self.min_motor_thrust, self.max_motor_thrust, self.min_motor_thrust])
            motor_thrust4 = np.array([self.max_motor_thrust, self.max_motor_thrust, self.max_motor_thrust, self.max_motor_thrust])

        all_motor_thrusts = [motor_thrust1, motor_thrust2, motor_thrust3, motor_thrust4]

        thrusts = np.zeros((4,4))
        thrusts[0, :] = motor_thrust1[:]
        thrusts[1, :] = motor_thrust2[:]
        thrusts[2, :] = motor_thrust3[:]
        thrusts[3, :] = motor_thrust4[:]

        thrusts = thrusts.T


        self.M_max_array = self.B_matrix @ thrusts
        
        self.M_roll_max = self.M_max_array[0,0]
        self.M_pitch_max = self.M_max_array[1,1]
        self.M_yaw_max = self.M_max_array[2,2]
        self.four_motors_max_thrust = self.M_max_array[3,3]



    def allocator(self, signals): #signals = [u_roll, u_pitch, u_yaw, u_thrust]
        #Задача аллокатора: распределить нужную\максимально возможную тягу, найденную из команд, полученных с ПИД-регуляторов (в основном, пропорциональная моментам, или же переводится в моменты) по всем осям
        #В силу того, что внутренний контур (rate-контур) регулирует угловую скорость квадрокоптера, внутри аллокатора мы должны оперировать изначально моментами тяги, и вот почему:
        #1)Так как тяга по своему определению (в контексте квадрокоптера) - сила, которая прикладывается вертикально, то есть изначально мы не можем задать тягу по крену, так как крен - вращение
        #2)Физически то, что заставляет тело изменять угловую скорость - угловое ускорение, а угловое ускорение создаёт момент tau = I * a (где I - матрица инерции, tay - момент, a - угловое ускорение)
        #3)Аллокатором мы решаем обратную физическую задачу: распределение достпуной тяги по моторам, чтобы получить желаемые суммарные силы и моменты, формально мы должны получить тяги на каждый из моторов
        # такие, чтобы удовлетворялось условие B * [f1, f2, f3, f4] = [T, Mx, My, Mz], то есть мы на вход получаем от ПИД-регулятора U, которая переводится в [T, Mx, My, Mz], а далее, зная матрицу B
        # и [T, Mx, My, Mz], мы находим удовлетворяющие вот этому выражению B * [f1, f2, f3, f4] = [T, Mx, My, Mz] тяги [f1, f2, f3, f4], так как, по-факту, передав на вход в аллокатор команды по всем
        # осям (которые переводятся в моменты, так как именно моменты влияют на регулирование уловой скорости) ПИД-регулятор устанавливает setpoint'ы (изначально массив u, потом моменты тяги по каждой оси),
        # которых мы должны достичь

        M = np.array([
            self.M_roll_max * signals[0],
            self.M_pitch_max * signals[1],
            self.M_yaw_max * signals[2],
            signals[3] * self.four_motors_max_thrust]
        , dtype=np.float64)
        

        f = self.B_pinv @ M

        
        f_total, M_total = self.desaturate(f, M)

        # print(f"различие между изначально запраш. моментом и скорректированным {M / M_total}")

        pwm_to_esc = self.thrust_to_pwm(f_total, 'linear')

        return pwm_to_esc
    

    def desaturate(self, f: np.ndarray, tau: np.ndarray):
        """
        Оркестратор десатурации.
        
        Принимает:
        f   — сырой вектор тяг из B_pinv @ tau [Н]
        tau — вектор желаемых моментов [M_roll, M_pitch, M_yaw, T]
        
        Возвращает:
        f_final — десатурированный вектор тяг [Н]
        """


        # Шаг 1: пробуем чистый сдвиг (сохраняет все моменты)
        f_shifted = self.first_desaturate_method(f)
        if f_shifted is not None:
            return f_shifted, self.B_matrix @ f_shifted

        # Шаг 2: жертвуем yaw, пересчитываем
        f_no_yaw_result = self.no_yaw_desaturation(tau)
        if f_no_yaw_result is not None:
            return f_no_yaw_result, self.B_matrix @ f_no_yaw_result

        # Шаг 3: масштабируем roll+pitch (здесь добавляется следующий уровень)
        f_no_roll_pitch = self.no_roll_pitch_desaturation(tau)
        if f_no_roll_pitch is not None:
            return f_no_roll_pitch, self.B_matrix @ f_no_roll_pitch
        

        # ── Шаг 4: Даже thrust один вызывает сатурацию ─────────────────
        # Используем сдвиг+масштаб непосредственно вектора f от thrust
        # (сохраняем пропорции моментов, уменьшаем магнитуду)

        tau_thrust_only = np.zeros(4, dtype=np.float64)
        tau_thrust_only[3] = tau[3]
        f_thrust_only = self.B_pinv @ tau_thrust_only

        f_thrust_shifted = self.first_desaturate_method(f_thrust_only)
        if f_thrust_shifted is not None:
            return f_thrust_shifted, self.B_matrix @ f_thrust_shifted

        # Крайний случай: масштабируем сам f от thrust
        return self._shift_and_scale(f_thrust_only, self.B_matrix @ f_thrust_only)
        


    def first_desaturate_method(self, f, precision = 4): #первый метод десатурации: находим максимальный выход за границу и отнимаем этот выход из всех величин тяг, проверяя резульаты на то, чтобы они тоже не выходили за границы 
        
        new_f = f.copy().astype(float)

        # f_max = np.max(f)
        # f_min = np.min(f)

        f_max = np.round(np.max(f), decimals=precision)
        f_min = np.round(np.min(f), decimals=precision)
        max_limit = np.round(self.max_motor_thrust, decimals=precision)
        min_limit = np.round(self.min_motor_thrust, decimals=precision)

        if f_max <= max_limit and f_min >= min_limit:
            return f.copy() #уже в норме
        
        if f_max > max_limit:

            delta = f_max - max_limit
            for x in range(len(new_f)):
                new_f[x] -= delta

            new_f_min = np.round(np.min(new_f), decimals=precision)

            if new_f_min >= min_limit:
                return new_f
        
        if f_min < min_limit:

            delta = min_limit - f_min
            for x in range(len(new_f)):
                new_f[x] += delta

            new_f_max = np.round(np.max(new_f), decimals=precision)

            if new_f_max <= max_limit:
                return new_f
        
        return None #если обе сатурации одновременно, то простой сдвиг не поможет, нужно масштибирование и сдвиг
    

            
    def no_yaw_desaturation(self, M):

        max_limit = np.round(self.max_motor_thrust, decimals=self.precision)
        min_limit = np.round(self.min_motor_thrust, decimals=self.precision)

        no_yaw_moments = M.copy()
        no_yaw_moments[2] = 0.0 #переделываем запрос на момент по yaw в 0.0

        f_no_yaw = self.B_pinv @ no_yaw_moments #находим силы, которые получаются без момента по yaw

        des_f_no_yaw = self.first_desaturate_method(f_no_yaw)

        if des_f_no_yaw is not None:
            only_yaw = np.zeros(4, dtype=np.float64)
            only_yaw[2] = M[2]

            delta_f_yaw = self.B_pinv @ only_yaw #получаем силы, которые были бы запрошены при условии, что момент yaw не был бы убран
            
            # Аналитически находим максимальный scale для yaw
            scaling_coef = self.find_scale(des_f_no_yaw, delta_f_yaw)
            
            momets_with_part_of_yaw = no_yaw_moments.copy()
            momets_with_part_of_yaw[2] = M[2] * scaling_coef

            f_result = self.B_pinv @ momets_with_part_of_yaw

            f_result_shifted = self.first_desaturate_method(f_result)

            return f_result_shifted if f_result_shifted is not None else np.clip(f_result, min_limit, max_limit)

    def no_roll_pitch_desaturation(self, M):
        # ── Шаг 4: Масштабируем Roll+Pitch, Thrust сохраняем ───────────
        #
        # f = f_от_thrust + scale · f_от_roll_pitch
        #
        # f_thrust_only:
        tau_thrust_only = np.zeros(4, dtype=np.float64)
        tau_thrust_only[3] = M[3]
        f_thrust_only = self.B_pinv @ tau_thrust_only

        # Δf_rp: вклад roll+pitch вместе взятых:
        tau_rp_only = np.zeros(4, dtype=np.float64)
        tau_rp_only[0] = M[0]   # M_roll
        tau_rp_only[1] = M[1]   # M_pitch
        delta_f_rp = self.B_pinv @ tau_rp_only

        # Ищем максимальный scale для roll+pitch:
        k_rp = self.find_scale(f_thrust_only, delta_f_rp)

        tau_scaled = np.zeros(4, dtype=np.float64)
        tau_scaled[0] = M[0] * k_rp   # M_roll  уменьшен
        tau_scaled[1] = M[1] * k_rp   # M_pitch уменьшен
        tau_scaled[2] = 0.0              # yaw пожертвован
        tau_scaled[3] = M[3]           # Thrust сохранён
        f_rp_scaled = self.B_pinv @ tau_scaled

        f_rp_shifted = self.first_desaturate_method(f_rp_scaled)
        if f_rp_shifted is not None:
            return f_rp_shifted


    def find_scale(self, main_forces, delta_forces):
        #у нас есть силы, распределённые на моторы, без учёта какого-либо\каких-либо моментов (мы самостоятельно их обнуляем), у этих сил может быть запас до сатурации
        #чтобы попытаться максимально сохранить занулённые до этого моменты, мы находим максимальный запас, который мы можем прибавить\вычесть из доступных сил, и делим на ту силу, которую нужно сохранить
        #  
        # нам нужно, чтобы выполнялось условие f_main + f_delta * scale <= self.max_motor_thrust или такое же условие, только с минимальной границей

        """
        Аналитически находит максимальный scale s ∈ [0, 1] такой, что:
          f_base + s · f_delta ∈ [f_min, f_max]  (поэлементно)

        Для каждого мотора i:
          Если f_delta[i] > 0:
            Верхнее ограничение: f_base[i] + s·f_delta[i] ≤ f_max
            → s ≤ (f_max - f_base[i]) / f_delta[i]

          Если f_delta[i] < 0:
            Нижнее ограничение: f_base[i] + s·f_delta[i] ≥ f_min
            → s ≤ (f_min - f_base[i]) / f_delta[i]
               (деление на отрицательное → неравенство сохраняется, т.к. обе части делятся на одно)

          Если f_delta[i] ≈ 0: ограничение не активно.

        s_max = min всех найденных ограничений.
        """

        max_limit = np.round(self.max_motor_thrust, decimals=self.precision)
        min_limit = np.round(self.min_motor_thrust, decimals=self.precision)

        s_max = 1.0

        for i in range(len(main_forces)):
            #верхнее ограничение
            if delta_forces[i] > 1e-9:
                #делаем всё по соответствующим элементам, так как силу delta_forces[i] мы можем добавить\вычесть только к силе main_forces[i]

                max_added_val = max_limit - main_forces[i] #максимальное значение, которое мы можем добавить, чтобы не выйти за лимит
                s_i = (max_added_val) / delta_forces[i]
                s_max = min(s_max, s_i)

            elif delta_forces[i] < -1e-9:
                # Нижнее ограничение
                # f_base[i] + s·delta_i ≥ f_min
                # s · delta_i ≥ f_min - f_base[i]
                # delta_i < 0 → делим, неравенство переворачивается:
                # s ≤ (f_min - f_base[i]) / delta_i
                min_unadded_val = min_limit - main_forces[i] #минимальное значение, которое мы можем вычесть, чтобы не выйти за лимит
                s_i = (min_unadded_val) / delta_forces[i]
                s_max = min(s_max, s_i)


        return float(max(0.0, s_max))
    

    def _shift_and_scale(self, f: np.ndarray) -> np.ndarray:
        """
        Универсальная функция: вписывает вектор f в [f_min, f_max]
        через масштабирование (с сохранением пропорций) и сдвиг.

        Если spread > available:
          → Масштабируем (все моменты уменьшаются на scale, пропорции сохранены)
        Иначе:
          → Только сдвиг (моменты сохраняются точно)

        Математика масштабирования:
          scale = (f_max - f_min) / spread
          center_now   = (max(f) + min(f)) / 2
          center_target = (f_max + f_min) / 2
          f'[i] = center_target + (f[i] - center_now) · scale

        Почему пропорции сохранены:
          f'[i] - f'[j] = (f[i] - f[j]) · scale
          Моменты = разности тяг → все уменьшились на scale, но соотношение то же.
        """
        f_max = np.round(self.max_motor_thrust, decimals=self.precision)
        f_min = np.round(self.min_motor_thrust, decimals=self.precision)

        max_f  = np.max(f)
        min_f  = np.min(f)
        spread = max_f - min_f
        available = f_max - f_min

        if spread < 1e-9:
            # Все моторы одинаковы → просто клипаем к центру диапазона
            center = (f_max + f_min) / 2.0
            return np.full(4, center, dtype=np.float64)

        if spread > available:
            # Масштабируем (момент уменьшается)
            scale        = available / spread
            center_now   = (max_f + min_f) / 2.0
            center_target = (f_max + f_min) / 2.0
            f_out = center_target + (f - center_now) * scale
        else:
            # Только сдвиг (момент сохраняется точно)
            if max_f > f_max:
                f_out = f - (max_f - f_max)
            else:
                f_out = f + (f_min - min_f)

        return np.clip(f_out, f_min, f_max)



    def find_B_matrix(self, motor_config_number): #функция, которая будет переводить нормированные команды от ПИД-регулятора в моменты.
        #моторная конфигурация 1 выглядит так: левый передний мотор (1) cw, правый передний мотор (2) ccw, задний правый мотор (3) cw, задний левый мотор (4) ccw
        m1_commands, m2_commands, m3_commands, m4_commands = None, None, None, None

        if motor_config_number == 1:
                                                            # roll, pitch, yaw, thrust
            m1_commands = np.array([1.0, 1.0, -1.0, 1.0])   # + + - +
            m2_commands = np.array([-1.0, 1.0, 1.0, 1.0])   # - + + +
            m3_commands = np.array([-1.0, -1.0, -1.0, 1.0]) # - - - +
            m4_commands = np.array([1.0, -1.0, 1.0, 1.0])   # + - + +


        elif motor_config_number == 2:
            #моторная конфигурация 2 выглядит так: левый передний мотор (1) cсw, правый передний мотор (2) cw, задний правый мотор (3) cсw, задний левый мотор (4) cw
            m1_commands = np.array([1.0, 1.0, 1.0, 1.0])   # + + + +
            m2_commands = np.array([-1.0, 1.0, -1.0, 1.0]) # - + - +
            m3_commands = np.array([-1.0, -1.0, 1.0, 1.0]) # - - + +
            m4_commands = np.array([1.0, -1.0, -1.0, 1.0]) # + - - +

        all_commands_arr = [m1_commands, m2_commands, m3_commands, m4_commands]

        B_matrix = np.vstack((all_commands_arr[0], all_commands_arr[1], all_commands_arr[2], all_commands_arr[3]))
        B_geometry_matrix = B_matrix.T

        B_physicaly = np.zeros((4,4))#инициализируем матрицу нулей 4*4

        #начинаем процесс заполнения корректными физическими значениями матрицы B
        B_physicaly[0, :] = B_geometry_matrix[0, :] * self.distance_from_center_of_mass_to_motor
        B_physicaly[1, :] = B_geometry_matrix[1, :] * self.distance_from_center_of_mass_to_motor
        B_physicaly[2, :] = B_geometry_matrix[2, :] * self.k_tau
        B_physicaly[3, :] = B_geometry_matrix[3, :] * 1.0

        self.B_matrix = B_physicaly
        self.B_pinv = np.linalg.pinv(self.B_matrix)


    # ═══════════════════════════════════════════════════════════════════
    # ПЕРЕВОД ТЯГИ В КОМАНДЫ ESC
    # ═══════════════════════════════════════════════════════════════════

    def thrust_to_pwm(self, f: np.ndarray, model: str = 'linear') -> np.ndarray:
        """
        Переводит тяги моторов [Н] в нормированные команды ESC [0, 1].

        Далее в main-цикле из [0, 1] получаем:
          PWM мкс: pwm_us = 1000 + cmd * 1000    → [1000, 2000] мкс
          DShot:   dshot  = 48 + cmd * 1951       → [48, 1999] (DShot2000)

        Два варианта:

        A) Линейная модель (по умолчанию):
             cmd = (f - f_min) / (f_max - f_min)
           Смысл: 0 = минимальная тяга, 1 = максимальная.
           Плюс: просто, не требует знания характеристики мотора.
           Минус: физически неточна (реально F ∝ ω², а не ω).

        B) Квадратно-корневая модель (физически точнее):
             F = k_T · ω²    →   ω = √(F / k_T)
             Если ω линейно зависит от команды ESC (большинство современных ESC):
               cmd ∝ ω = √(F / F_max)
             → cmd = √(f / f_max)
           Плюс: точнее отражает реальную динамику.
           Минус: требует, чтобы ESC действительно линейно управлял ω (не всегда).

        На практике для первой версии прошивки используй линейную модель.
        После настройки и тестирования можно перейти к sqrt.
        """
        f_max = self.max_motor_thrust
        f_min = self.min_motor_thrust

        f_clipped = np.clip(f, f_min, f_max)

        if model == 'linear':
            # Линейная нормировка: 0 при f_min, 1 при f_max
            f_range = f_max - f_min
            if f_range < 1e-9:
                return np.full(4, 0.5, dtype=np.float64)
            cmd = (f_clipped - f_min) / f_range

        elif model == 'sqrt':
            # Квадратно-корневая: учитывает F ∝ ω²
            # Нормируем f относительно абсолютного максимума
            cmd = np.sqrt(f_clipped / f_max)

        else:
            raise ValueError(f"Неизвестная модель: {model}. Используй 'linear' или 'sqrt'.")

        return np.clip(cmd, 0.0, 1.0)

    def pwm_to_us(self, cmd: np.ndarray,
                  pwm_min_us: float = 1000.0,
                  pwm_max_us: float = 2000.0) -> np.ndarray:
        """
        Переводит нормированные команды [0, 1] в PWM в микросекундах [pwm_min, pwm_max].

        Стандарт ESC:
          1000 мкс = минимальный газ (мотор не крутится или idle)
          2000 мкс = максимальный газ

        cmd = 0.0 → 1000 мкс
        cmd = 1.0 → 2000 мкс
        cmd = 0.5 → 1500 мкс
        """
        return pwm_min_us + cmd * (pwm_max_us - pwm_min_us)

    def pwm_to_dshot(self, cmd: np.ndarray) -> np.ndarray:
        """
        Переводит нормированные команды [0, 1] в DShot-значения.

        DShot диапазон: 48 (idle/min) ... 2047 (максимум).
        Значения 0-47 зарезервированы для специальных команд (arm, beep и т.п.).
        """
        DSHOT_MIN = 48
        DSHOT_MAX = 2047
        return (DSHOT_MIN + cmd * (DSHOT_MAX - DSHOT_MIN)).astype(int)




# def main():
#     alloc = Allocator(1)
#     pwm = [0.6, 0.0, 1.0, 0.5]
#     pwm_to_esc = alloc.allocator(pwm)

#     # pwm_to_esc = alloc.thrust_to_pwm(thrusts, 'linear')

#     print(pwm_to_esc)







# main()























    # def calculating_params_to_qp(self):
    #     #изначальная наша задача: решить задачу минимизации (M = B * f) при условии, что f_min <= fi <= f_max - задача квадратичного программирования,
    #     # то есть надо минимизировать квадратичную ошибку W|| B*f - M ||**2
    #     # Расскрывая скобки, с учётом матричных правил, получаем такие действия (f_t * B_t - M_t)W(B*f - M)
    #     #Далее вносим W в левую скобку и расскрываем все скобки, получаем: (f_t * B_t * W * B*f) - (f_t * B_t * W * M) - (M_t * B * f * W) + (M_t * W * M)
    #     #смотрим на подобне и видим, что 2 и 3 многочлены равны, а 4 член - константа, тогда всё выражение (f_t * B_t * W * B*f) - 2 * (f_t * B_t * W * M) + const, и, окончательно преобразовав,
    #     # пытаясь подогнать к канонической форме 1/2 * f_t * P * f + q_t * f, мы получаем f_t * (B_t * W * B) * f - f_t * (-2B_t * W * M) + const
    #     # Сопоставление квадратичной части:Наше: f^T (B^T W B) f Каноническое: 1/2 f^T P f . Чтобы они были равны, матрица P должна быть такой, чтобы при умножении на 1/2 получилось
    #     # (B^T W B). 1/2 P = B^T W B. Отсюда вывод:P = 2 B^T W B     ---- Гессиан (исходя из правил квадратичного программирования)
    #     #следовательно q = (-2B_t * W * M)
        
    #     self.canonic_P = 2.0 * np.dot(np.dot(self.B_matrix.T, self.W), self.B_matrix)
    #     self.canonic_P_cvxopt = cvxopt.matrix(self.canonic_P)

    #     #рассчитываем матрицу ограничений G
    #     I_matrix = np.eye(4,4)
    #     G_np = np.vstack([I_matrix, -I_matrix])
    #     self.G_cvxopt = cvxopt.matrix(G_np)
    #     # h ограничения будет считывать далее, так как они могут меняться, например, при просадке батареи


    
    # def find_first_scaling_coeffs(self,A_matrix_number):

    #     A_matrix = self.find_A_matrix(A_matrix_number)

    #     self.B_matrix = self.find_B_matrix(A_matrix)
    #     self.B_pinv = np.linalg.pinv(self.B_matrix)

    #     T1_max, T2_max, T3_max, T4_max = self.max_motor_thrust, self.max_motor_thrust, self.max_motor_thrust, self.max_motor_thrust #устанавливаем максимальные значения тяги для каждого из моторов
    #     max_T_commands = np.vstack((T1_max, T2_max, T3_max, T4_max)) # делаем из этого вектор-столбец
    #     # max_T_to_all_axis = np.dot(A_matrix.T, max_T_commands)
    #     max_T_to_all_axis = np.dot(np.hstack((T1_max, T2_max, T3_max, T4_max)), A_matrix)#чтобы найти максимальное значение момента тяги по какой-либо оси, мы должны сделать так, чтобы по этой оси
    #     # выдавалась максимальная тяга. Тогда мы просто делаем так, что максимальное значение тяги каждого из моторов подаётся в матрицу распределения тяги на моторы по осям.
    #     # Делается это, во-первых, потому, что мы от производителя, то есть с самого начала знаем максимальное значение тяги каждого из моторов. То есть мы имитируем поданную команду u = 1.0
    #     # на нужная нам ось, но вместо u в умножении мы используем сразу значение тяги, так как из него вытекает значение момента тяги (мы бы могли так же подавать максимальное значение u = 1.0
    #     # и умножать его на матрицу распределения A, но тогда, в итоге, нам бы пришлось полученный результат домножать на максимальное значение тяги, чтобы получить максимальную тягу по данной оси,
    #     # дабы далее получить максимальное значение момента по этой оси)
    #     scaling_factors = np.array([self.distance_from_center_of_mass_to_motor, self.distance_from_center_of_mass_to_motor, self.k_tau, 1.0])
    #     max_moments_to_all_axis = scaling_factors * max_T_to_all_axis

    #     self.M_roll_max = max_moments_to_all_axis[0] 
    #     self.M_pitch_max = max_moments_to_all_axis[1]
    #     self.M_yaw_max = max_moments_to_all_axis[2]
    #     self.M_thrust_max = max_moments_to_all_axis[3]




    # def find_A_matrix(self, alloc_or_coeffsFinder): #функция, которая будет переводить нормированные команды от ПИД-регулятора в моменты.
    #     #моторная конфигурация 1 выглядит так: левый передний мотор (1) cw, правый передний мотор (2) ccw, задний правый мотор (3) cw, задний левый мотор (4) ccw
    #     A_matrix = None
    #     m1_commands, m2_commands, m3_commands, m4_commands = None, None, None, None

    #     if self.number_motors_configuration == 1:
    #                                                      # roll, pitch, yaw, thrust
    #         m1_commands = np.array([1.0, 1.0, 0.0, 1.0]) # + + - +
    #         m2_commands = np.array([0.0, 1.0, 1.0, 1.0]) # - + + +
    #         m3_commands = np.array([0.0, 0.0, 0.0, 1.0]) # - - - +
    #         m4_commands = np.array([1.0, 0.0, 1.0, 1.0]) # + - + +


    #     elif self.number_motors_configuration == 2:
    #         #моторная конфигурация 2 выглядит так: левый передний мотор (1) cсw, правый передний мотор (2) cw, задний правый мотор (3) cсw, задний левый мотор (4) cw
    #         m1_commands = np.array([1.0, 1.0, 1.0, 1.0]) # + + + +
    #         m2_commands = np.array([0.0, 1.0, 0.0, 1.0]) # - + - +
    #         m3_commands = np.array([0.0, 0.0, 1.0, 1.0]) # - - + +
    #         m4_commands = np.array([1.0, 0.0, 0.0, 1.0]) # + - - +

    #     all_commands_arr = [m1_commands, m2_commands, m3_commands, m4_commands]

    #     if alloc_or_coeffsFinder == 0: #если значение alloc_or_coeffsFinder == 0, значит, мы делаем матрицу A для того, чтобы находить коэффициенты максимальных моментов (различием в том, что в этой
    #     # матрице на моторах, которые не вносят в момент тяги по оси никакие воздействия (то есть должны либо ослабляться, либо не трогаться) стоят нули, вместо -1.0
    #         pass
    #     else: #иначе делаем матрицу А такой, которая будет пригодна для вычисления распределения тяг на двигатели уже с реальными командами
    #         for x in range(len(all_commands_arr)):
    #             for i in range(len(all_commands_arr[x])):
    #                 if all_commands_arr[x][i] == 0.0:
    #                     all_commands_arr[x][i] = -1.0

    #     A_matrix = np.vstack((all_commands_arr[0], all_commands_arr[1], all_commands_arr[2], all_commands_arr[3]))

    #     return A_matrix

    # def find_B_matrix(self, A_matrix): #массив нормированных команд u, полученных от ПИД-регуляторов
    #     # A_matrix = self.find_A_matrix(A_matrix_number) #находим матрицу аллокации A, которая будет находить распределение тяг\моментов тяг по осям, пока ещё нормированных команд
    #     B_geometry_matrix = A_matrix.T #матрица B (физическая матрица) по стандартам соответствует: строки - оси, столбцы - моторы, а наша матрица простой аллокации А: строки - моторы, столбцы - оси

    #     B_physicaly = np.zeros((4,4))#инициализируем матрицу нулей 4*4
    #     #начинаем процесс заполнения корректными физическими значениями матрицы B
    #     B_physicaly[0, :] = B_geometry_matrix[0, :] * self.distance_from_center_of_mass_to_motor
    #     B_physicaly[1, :] = B_geometry_matrix[1, :] * self.distance_from_center_of_mass_to_motor
    #     B_physicaly[2, :] = B_geometry_matrix[2, :] * self.k_tau
    #     B_physicaly[3, :] = B_geometry_matrix[3, :] * 1.0
    #     # A_matrix = self.find_A_matrix(A_matrix_number) #находим матрицу аллокации A, которая будет находить распределение по осям ещё пока нормированных команд
    #     # B_matrix = self.distance_from_center_of_mass_to_motor * np.dot(A_matrix, np.vstack(u_commands)) #выдаёт вектор-столбец, в котором по осям распределены значения нормированных команд, умноженных на длину рычага

    #     return B_physicaly

