import math
from sensor_readers import readers as reader
from filters import pt as pt


class Standart_PID:

    def __init__(self, Kp_rate, Kp_angular, Kp_velocity, Kp_position, Ki_rate, Ki_angular, Ki_velocity, Ki_position, Kd_rate, Kd_angular, Kd_velocity, Kd_position):
        
        self.firm_mode = 0 #0 - acro, 1 - angle, 2 - vel, 3 - poshold
        
 
        self.max_output = 100 #максимальная выходная команда в rate-контуре (Rates значение) #град\сек
        self.min_output = -100 #минимальная выходная команда в rate-контуре (Rates значение) #град\сек

        self.max_hor_vel = 20 #м\с - макисмальная скорость по горизонтали
        self.min_hor_vel = 20

        self.max_ang = 120 #используются для ШИМ, полученных с пульта #град
        self.min_ang = -120 #град

        self.dt_rate = 0.001 #1000
        self.dt_ang = 0.005 #200
        self.dt_vel = 0.02 #50
        self.dt_pos = 0.1 #10

        self.last_Fs = 1000
        self.last_fc = 100

        self.Kp_rate = Kp_rate
        self.Kp_angular = Kp_angular
        self.Kp_velocity = Kp_velocity
        self.Kp_position = Kp_position


        self.Ki_rate = Ki_rate
        self.Kt_rate = self.Ki_rate #коэффициент Back-calculation (по умолчанию равен Ki)
        self.Ki_angular = Ki_angular
        self.Ki_velocity = Ki_velocity
        self.Ki_position = Ki_position

        self.Kd_rate = Kd_rate
        self.Kd_angular = Kd_angular
        self.Kd_velocity = Kd_velocity
        self.Kd_position = Kd_position

        self.integral_rate = 0.0
        self.integral_vel = 0.0
        self.anti_windup_correct_rate = 0.0 #переменная для установления коррекции интегральной части с помощью техники back-calculation
        self.last_rate_measurement = 0.0
        self.last_setpoint = 0.0
        self.lpf_to_last_setpoint = pt.PT3(self.last_setpoint, self.last_Fs) #ФНЧ для setpoint
        self.last_v_rich_setpoint = 0.0 #переменная, показывающая прошлую скорость достижения установленного setpoint


        self.lpf1_to_D_rate = pt.PT3(self.last_fc, self.last_Fs)

        self.position_setpoint = 0.0
        self.velocity_setpoint = 0.0
        self.angle_setpoint = 0.0
        self.rate_setpoint = 0.0

        self.pos_divider = 0
        self.vel_divider = 0
        self.ang_divider = 0
        self.rate_divider = 0
        self.tick_divider = 0 #счётчик тиков

        self.pid_hz = 1000
        self.rate_hz = 1000
        self.angle_hz = 200
        self.vel_hz = 50
        self.pos_hz = 10


        self.ticks_for_pos = self.pid_hz / self.pos_hz #количество тиков, через которое работает Pos-контур (раз в сколько тиков работает)
        self.ticks_for_vel = self.pid_hz / self.vel_hz #количество тиков, через которое работает Vel-контур (раз в сколько тиков работает)
        self.ticks_for_angle = self.pid_hz / self.angle_hz #количество тиков, через которое работает Angle-контур (раз в сколько тиков работает)


        self.get_vel_from_CKF = None #в эту переменную помещается измерение скорости для соответствующей оси из цикла управления
        self.get_angle_from_CKF = None #в эту переменную помещается измерение угла для соответствующей оси из цикла управления
        self.gyro_measurement = None

        self.dt_to_angle_contur = 1.0 / self.angle_hz
        self.dt_to_vel_contur = 1.0 / self.vel_hz #получаем промежуток времени, с которым работает контур

        self.from_rad_to_grad = 57.29578


        

    def release_cascade(self, pwm_setpoint):
        pwm = None
        
        
        if (self.tick_divider % self.ticks_for_pos == 0) and self.firm_mode >= 3:
            pass
        
        if (self.tick_divider % self.ticks_for_vel == 0) and self.firm_mode >= 2:
            vel_setpoint = None 

            if self.firm_mode == 2: #если прошивка в режиме velocity, то сетпоинтом является полученный ШИМ
                vel_setpoint = pwm_setpoint * self.max_hor_vel #переводим полученный ШИМ в сетпоинт скорости (так как работает скоростной контур)
                
            elif self.firm_mode > 2: #если прошивка в режиме выше velocity, то сетпоинтом является полученный результат работы от контура выше (position), который помещается в переменную self.velocity_setpoint 
                vel_setpoint = self.velocity_setpoint 

            #град\сек
            self.angle_setpoint = self.velocity_contur(vel_setpoint, self.dt_to_vel_contur, self.get_vel_from_CKF, self.get_angle_from_CKF) # результатом работы контура скорости является setpoint Для углового-контура
            # поэтому результат работы self.velocity_contur() сразу помещаем в сетпоинт для контура ниже (угловой контур)

        if (self.tick_divider % self.ticks_for_angle == 0) and self.firm_mode >= 1:
            angle_setpoint = None

            if self.firm_mode == 1:
                #град
                angle_setpoint = pwm_setpoint * self.max_ang #если прошивка в режиме angle, то сетпоинтом является полученный ШИМ

            elif self.firm_mode > 1: #если прошивка в режиме выше angle, то сетпоинтом является полученный результат работы от контура выше (velocity), который помещается в переменную self.angle_setpoint 
                angle_setpoint = self.angle_setpoint
            

            self.rate_setpoint = self.angle_contur(angle_setpoint, self.dt_to_angle_contur, self.get_angle_from_CKF)# результатом работы контура угла является setpoint Для rate-контура
            # поэтому результат работы self.angle_contur() сразу помещаем в сетпоинт для контура ниже (rate-контур)

        setpoint = None

        if self.firm_mode == 0:
            setpoint = pwm_setpoint * self.max_output
        elif self.firm_mode > 0:
            setpoint = self.rate_setpoint
        
        pwm = self.rate_contur(setpoint, self.dt_rate, self.gyro_measurement)


        self.tick_divider += 1

        return pwm
        



    def standart_position_contur(self, setpoint, dt, measurement): #обычный PID-контур. Мысль такова, что,если нам нужно сделать PI, P или иные вариации контура, то мы просто зануляем соответствующие коэффициенты Kd, Ki и тд.
        #делаем только P-контур
        e = setpoint - measurement
        u = self.Kp_position * e

        clamp_u = max(self.min_hor_vel, min(u, self.max_hor_vel))

        return clamp_u

    def velocity_contur(self, setpoint, dt, measurement, angle, a_z_des=0.0):
        #реализовывать будем PI-контур. D-часть будет тут бесполезна, так как будет создавать чудовищную ошибку при дискретизации и интегрировании

        e = setpoint - measurement #получаем скорость, которую мы должны нарастить\убавить

        p_part = self.Kp_velocity * e

        u_acc = p_part + self.integral_vel #тут получаем ускорение
        
        # I-term с анти-виндапом (упрощённо)
        # if not self.is_saturated or (e * self.integral_vel < 0):
        self.integral_vel += self.Ki_velocity * e * dt#обновляем интегральный член скорости

        # Преобразование ускорения → угол наклона
        angle_setpoint_rad = self.compute_attitude_setpoint(u_acc, a_z_des)
    
        return self.from_rad_to_grad * angle_setpoint_rad #град\сек

    def angle_contur(self, setpoint, dt, measurement):
        #реализовывать буду, скорее всего, чистый П-регулятор, так как контур ниже (Rate-конутр) имеет уже I составляющую - не нужно дублировать интегратор
        #данные measurement будем получать из фильтра Калмана В ГРАД\СЕК

        #переменная setpoint должна приходить сюда уже в физических величинах или конвертироваться тут
        #setpoint мы получаем с двух разных направлений - либо из velocity_contur, либо с пульта оператора.
        #setpoint с пульта оператора подвергаются ограничениям в виде переменных self.max_..., а setpoint, полученный из конутра выше - нет.
        e = setpoint - measurement

        # clamp_ang = max(self.min_ang, min(e, self.max_ang))
        
        u = self.Kp_angular * e #превращаем setpoint (град) в угловую скорость. Размерность Kp_angular = 1\с

        clamp_ang_vel = max(self.min_output, min(u, self.max_output))

        return clamp_ang_vel #возвращаем град\сек, т.к. далее это значение пойдёт в rate-контур
        

    def rate_contur(self, setpoint, dt, measurement = 0.0):
        chastota_diskret = 1.0 / dt

        if chastota_diskret != self.last_Fs:
            self.lpf1_to_D_rate.update_coefs(self.last_fc, chastota_diskret)
            self.last_Fs = chastota_diskret

        #setpoint, measurement- град\сек
        e = setpoint - measurement #находим ошибку между сетпоинтом и измерением с гироскопа, полученным в данный момент времени

        P_part = self.Kp_rate * e #П-часть

        D_part = self.Kd_rate * ((self.last_rate_measurement - measurement) / dt)
        filted_D_part = self.lpf1_to_D_rate.pt1(D_part) #фильтруем D-часть с помощью ФНЧ первого порядка


        integralErrorRate = self.Ki_rate * (e * dt)
        integralError = self.I_Term_Relax_Second(setpoint, self.integral_rate, integralErrorRate, 50) #treshold - настраиваемый параметр. Функция вызывает до высчитывания интегрального члена для того, чтобы обнаружить
        # резкий манёвр до того, как он сильно повлияет на интеграл. Иначе говоря - задетектили резкий манёвр -> ослабили интеграл


        self.integral_rate += (integralError + self.anti_windup_correct_rate) #интегральная часть (с back-calculation коррекцией)

        unset_result = P_part + self.integral_rate + filted_D_part

        saturated_result = self.back_calculation(unset_result, dt)


        self.last_rate_measurement = measurement #обновление значения последнего измерения с гироскопа (переменная для рассчёта D-части)
        self.last_setpoint = setpoint

        return saturated_result, setpoint

    def back_calculation(self, unset_result, dt):

        min_out = self.min_output / self.max_output #получаем нормированное минамальное значение ШИМ, которое может вернуться в аллокатор (-900/900)
        max_out = self.max_output / self.max_output #получаем нормированное максимальное значение ШИМ, которое может вернуться в аллокатор (900/900)

        saturated_result = max(min_out, min(max_out, unset_result))#насыщение выхода (выбираем наибольшее значние между ограничительным минимумом\максимумом и посчитанным кодом выходом,
        # в случае, если посчитанный нашей программой выход будет по модулю больше ограничительного минимума\максимума, то в качестве выхода будут установлены эти самые ограначительные значения, иначе - посчитанный нами выход

        result_error = saturated_result - unset_result #тут мы считаем насколько вышли (если вышли) мы нашим посчитанным значением выхода (unset_result) за пределы нормы (макисмума\минимума).
        # Результат должен получаться с противоположным знаком от текущего направления интегрирования (для разматывания интеграла)

        self.anti_windup_correct_rate = self.Kt_rate * result_error * dt #рассчитывается для следующего цикла коррекции (k + 1 цикла)

        return saturated_result


    def I_Term_Relax_First(self, setpoint, threshold, integral_val, error, cutoff = None):
        #доделать таким образом, чтобы cutoff передавался в фнч
        setpointLpf = self.lpf_to_last_setpoint.pt1(setpoint)
        setpointHpf = abs(setpoint - setpointLpf)

        max_error = 200 #deg\s

        I_Term_Relax_K = 0.0
        # is_Decreasing_I = (integral_val > 0 and integral_error < 0) or (integral_val < 0 and integral_error > 0) #если на данный момент знак интеграл противоположен знаку ошибки
        # # setpoint-measurement, то начинаем уменьшать интеграл, чтобы он разматывался, например: интегральное значение -10, сетпоинт 100, гиро 10, ошибка получается 90, а интеграл направлен
        # # в другую сторону -> его надо уменьшить
        isDecreasing = self.safe_sign(integral_val) !=  self.safe_sign(error)

        if (setpointHpf >= threshold): #в силу этого условия I_Term_Relax_K всегда будет 0, кроме случая, когда isDecreasing = True, то есть работает естественное разматывание

            if (isDecreasing):#если знак ошибки сетпоинта - межурмент не совпадает со знаком интеграла, то ничего не меняем,
                # так как происходит естественное "разматывание"
                I_Term_Relax_K = 1.0
            else: #если же ошибка и интеграл в одну сторону
                I_Term_Relax_K = (0, 1 - ((setpointHpf - threshold)/ (max_error - threshold)))

            return I_Term_Relax_K
        else:
            return I_Term_Relax_K

    def I_Term_Relax_Second(self, setpoint, integral, integralError, threshold, cutoff = None):
        setpointLpf = self.lpf_to_last_setpoint.pt1(setpoint)
        setpointHpf = abs(setpoint - setpointLpf) #находим высокочастотную составляющую сигнала

        ITermRelax_K = 0.0

        if ((self.safe_sign(integral) >= 0 and self.safe_sign(integralError) <= 0) or (self.safe_sign(integral)) <= 0 and self.safe_sign(integralError) >= 0):
            pass
            #ничего не делаем в силу того, что интеграл в случае, если ошибка между сетпоинтом и измерением и интегральное значение имеют разные знаки, то интеграл сам "естественно" разматывается

        elif ((self.safe_sign(integral) >= 0 and self.safe_sign(integralError) > 0) or (self.safe_sign(integral) <= 0 and self.safe_sign(integralError) < 0)): #если ошибка между сетпоинтом и измерением и
                                                                                                                    # интегральное значение имеют одинаковые знаки, то разматываем интеграл до нуля
            ITermRelax_K = max(0, 1.0 - (setpointHpf / threshold))
            integralError *= ITermRelax_K

        return integralError


    def compute_attitude_setpoint(self, a_des_xy, a_des_z=0.0, max_angle_deg= 45.0):
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
        
        return angle_rad  # в радианах, или math.degrees(angle_rad) если нужно в градусах

    #непонятно, действенна ли такая реализация, так как работает только один цикл (по событию, а не по состоянию)

    # def I_term_relax(self, setpoint, measurement, treshold, dt):
    #     is_multiply_integral = False #переменная, которая показыает нам, будем ли мы умножать интеграл на коэффициент (то есть "раскручивать" его) или же будем новое значение умножать на коэффициент
    #     I_Term_Relax_K = 0.0
    #     max_error = 200 #должно быть настраиваемым значение. Если ошибка превышает это значение, то полностью останавливаем интегрирование
    #     # last_Integral_val = last_I_val #фиксируем последнее значение интегрального члена перед манёвром
    #     setpoint_error = setpoint - self.last_setpoint #находим разницу между текущим и прошлым сетпоинтами
    #
    #     znak_setpoint = self.safe_sign(setpoint) #узнаём знак текущего сетпоинта
    #     znak_last_setpoint = self.safe_sign(self.last_setpoint) #узнаём знак предыдущего сетпоинта
    #     znak_measurement = self.safe_sign(measurement)#узнаём знак измерения гироскопа
    #
    #     if (abs(setpoint_error) >= treshold) and (abs(setpoint_error) <= max_error): #условие детекнирования резкого манёвра
    #
    #         if znak_setpoint != znak_last_setpoint:
    #             if znak_setpoint > znak_last_setpoint: #если знак текущего сетпоинта положительный, а прошлого - отрицательный
    #
    #                 if znak_measurement == znak_last_setpoint: #если знак текущего измерения с гироскопа совпадает со знаком прошлого сетпоинта
    #                     I_Term_Relax_K = 0.0 #при условии, если разные знаки у прошлого и нынешнего сетпоинтов - мы останавливаем интегрирование, так как измерение будет постепенно уходить в знак нужного сетпоинта
    #                     # и нам нет смысла накручивать I-член, так как ошибка накручиватется уже в другую сторону
    #                     is_multiply_integral = True #устанавливаем True для расскручивания интеграла (в случае, если значния сетпоинтов имеют разные знаки) (произойдёт 20 циклов раскрутки
    #                 # (self.integral_rate *= 0.98), так как частота обновления rc-сигнала 20Гц, а частота работы цикла рейт-контура 1000 Гц, следовательно is_multiply_integral = True вернётся 20 раз,
    #                 # пока не поменяется last_setpoint на текущее значение (которое установлено сейчас), следовательно (self.integral_rate *= 0.98) выполнится 20 раз. (При частоте 20 Гц и умножении интеграла на 0.98 интеграл размотается примерно на 33%)
    #
    #
    #
    #                 elif znak_measurement == znak_setpoint or (znak_setpoint == 0 or znak_measurement == 0):#если знак текущего измерения с гироскопа не совпадает со знаком прошлого сетпоинта, а также #если знак сетпоинта или измерения равны нулю - это никак не должно влиять на логику интегрирования (заморозки или ослабления)
    #                     I_Term_Relax_K = 1.0 - (setpoint_error / max_error )
    #                     is_multiply_integral = False#скорее всего, если нынещний сетпоинт и измерение - одного знака, даже с условием того, что прошлый сетпоинт другого знака, есть смысл начинать интегрирование,
    #                     #так как ошибка, по которой ищется интеграл, находится так: e = setpoint - measurement, следовательно, даже если все прошлые last_setpoint были отрицательными, то текущий интеграл
    #                     #будет делать естественное размытвание
    #
    #             if znak_setpoint < znak_last_setpoint: #если знак текущего сетпоинта отрицательный, а прошлого - положительный
    #                 if znak_measurement == znak_last_setpoint:
    #                     I_Term_Relax_K = 0.0
    #                     is_multiply_integral = True #устанавливаем True для расскручивания интеграла (в случае, если значния сетпоинтов имеют разные знаки) (произойдёт 50 циклов раскрутки
    #                 # (self.integral_rate *= 0.98), так как частота обновления rc-сигнала 50Гц, а частота работы цикла рейт-контура 1000 Гц, следовательно is_multiply_integral = True вернётся 50 раз,
    #                 # пока не поменяется last_setpoint на текущее значение (которое установлено сейчас), следовательно (self.integral_rate *= 0.98) выполнится 50 раз. (При частоте 50 Гц и умножении интеграла на 0.98 интеграл размотается примерно на 63%)
    #
    #
    #                 elif znak_measurement == znak_setpoint or (znak_setpoint == 0 or znak_measurement == 0):#если знак сетпоинта или измерения равны нулю - это никак не должно влиять на логику интегрирования (заморозки или ослабления)
    #                     I_Term_Relax_K = 1 - (abs(setpoint_error) / max_error)
    #                     is_multiply_integral = False
    #
    #
    #
    #         elif znak_setpoint == znak_last_setpoint: #если знаки ошибок совпадают
    #             if znak_measurement == znak_setpoint or (znak_setpoint == 0 or znak_measurement == 0):#если знак сетпоинта или измерения равны нулю - это никак не должно влиять на логику интегрирования (заморозки или ослабления)
    #                 I_Term_Relax_K = 1 - (abs(setpoint_error) / max_error)
    #             else:
    #                 I_Term_Relax_K = 0.0 #может быть такая ситуация, что, в силу того, что сетпоинты обновляются достаточно быстро, то на предыдущем шаге был last_setpoint и last_measurement Отрицательными,
    #                 # а на этом шагу уже setpoint и last_setpoint - одного знака, но знак измерения по-прежнему остался, например, отицательным
    #                 is_multiply_integral = True
    #
    #
    #
    #     elif (abs(setpoint_error) > max_error):
    #         if znak_measurement == znak_setpoint: #если ошибка огромная и знак измерения гироскопа совпадает с направлением сетпоинта, то просто замораживаем интеграл
    #             I_Term_Relax_K = 0.0
    #         else: #иначе раскручиваем интеграл
    #             is_multiply_integral = True
    #     else:
    #         I_Term_Relax_K = 1.0
    #
    #     return I_Term_Relax_K, is_multiply_integral


    def safe_sign(self, value):
        if value > 0.0:
            return 1.0
        if value < 0.0:
            return -1.0
        return 0.0