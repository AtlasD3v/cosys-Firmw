import numpy as np

class PT3:

    def __init__(self, fc = 30.0, Fs = 1000.0):
        self.Fs = Fs #частота дискретизации датчика\приёмника
        self.Ts = 1.0 / self.Fs #время дискретизации
        self.fc = fc #частота среза (Гц)

        self.wc = 2.0 * np.pi * self.fc # (рад\с) определяет, насколько быстро фильтр «успевает реагировать. Чем выше wc, тем быстрее фильтр реагирует на быстрые изменения сигнала (пропускает больше высоких частот)
        self.k = np.sqrt( 2.0 ** (1/3) - 1.0) #просто числовой коэффициент, который «подстраивает» τ для ФНЧ так, чтобы фильтр имел -3 дБ ровно на заданной частоте среза.
        self.tay = self.k / self.wc

        self.alpha = np.e ** -(self.Ts/self.tay)
        self.beta = 1.0 - self.alpha

        self.y1 = 0.0
        self.y2 = 0.0
        self.y3 = 0.0


    def pt3(self, sig): #прямоугольная аппроксимация ZOH-дискретизации для режима position

        self.y1 = self.alpha * self.y1 + self.beta * sig
        self.y2 = self.alpha * self.y2 + self.beta * self.y1
        self.y3 = self.alpha * self.y3 + self.beta * self.y2

        return self.y3

    def pt1(self, sig): # для acro
        self.y1 = self.alpha * self.y1 + self.beta * sig

        return self.y1

    def update_coefs(self, fc, Fs):
        self.Fs = Fs
        self.fc = fc
        self.Ts = 1.0 / Fs
        self.wc = 2.0 * np.pi * self.fc

        self.tay = self.k / self.wc
        self.alpha = np.e ** -(self.Ts/self.tay)
        self.beta = 1.0 - self.alpha
class Expo:
    def __init__(self, alpha, beta, low_percent_stat, max_val_in_percent_diapazon_stat):
        self.alpha_stat = alpha #коэффициент, определяющий долю смягчённого сигнала
        self.alpha = None #переменная, в которую динамические приходит установленное изменённое значение

        self.beta_stat = beta #от -1 до 1 (-1 - быстрый рост начала второго интервала и сглаживание ближе к 100%, 0 - линейное поведение, 1 - кубическое поведение)
        self.beta = None

        self.low_percent_stat = low_percent_stat #проценты диапазона отклонения стика по модулю, в котором движения сглаживаются с Expo и никак не бустятся (либо уменьшаюся, либо линейно)
        self.percent = None #переменная, в которую динамические приходит установленное изменённое значение

        self.max_val_in_percent_diapazon_stat = max_val_in_percent_diapazon_stat #эта переменная определяет потолок макс. значения в диапазоне self.percent отклонения стика,
        # то есть, например, percent = 0.3 = 30%, а max_val_in_percent_diapazon = 0.2 = 20% - это значит, что отклоняя стик на 30%
        # мы максимум можем достигнуть 20% от возможного максимального количества оборотов
        self.max_val_in_percent_diapazon = None

        self.max_deg_s = 900.0
        self.center_sens = 0.0
        self.after_center_sens = 0.0
        self.max_stik_val = 1.0
        self.min_stick_val = -1.0

        self.k_s = 0.0 #коэффициент нормировки сигнала внутри диапазона percent_stat
        self.SR = 0.99


    def exp(self, sig, e):
        result = (e * sig ** 3) + ((1 - e) * sig)

        return result

    def expo(self, sig):
        result = (self.alpha_stat * sig ** 3) + ((1 - self.alpha_stat) * sig)

        return result

    def custom_bust1(self, sig):
        res = 0.0
        change = False
        if (self.max_val_in_percent_diapazon_stat != self.max_val_in_percent_diapazon and self.max_val_in_percent_diapazon is not None) or (self.alpha_stat != self.alpha and self.alpha is not None) or (self.low_percent_stat != self.percent and self.percent is not None):
            change = True
        else:
            change = False

        if abs(sig) <= (self.max_stik_val * self.low_percent_stat):

            if change:
                self.max_val_in_percent_diapazon_stat = self.max_val_in_percent_diapazon
                self.alpha_stat = self.alpha
                self.low_percent_stat = self.percent

                max_signal_val_in_diap = self.max_stik_val * self.max_val_in_percent_diapazon_stat #максимальное значение, которое можем получить с команды стика в диапазоне percent
                max_signal_val_from_percent = self.max_stik_val * self.low_percent_stat #жёсткое ограничение на максимально значение, внутри которого происходит регулировка
                max_in_expo = self.expo(max_signal_val_from_percent)#максимальный сигнал, который мы можем получить с учётом expo без ограничений по max_signal_val_from_percent
                self.k_s = max_signal_val_in_diap / max_in_expo #коэффициент домножения значений сигнала для нормировки в нужном ограничении (

                # self.max_val_in_percent_diapazon, self.alpha, self.percent = None, None, None #переводим в начальные состояния, чтобы после изменения параметров произошёл пересчёт коэффициентов


            expo = self.exp(sig, self.alpha_stat)
            signal = self.k_s * expo

            res = signal * self.k_s

        else:
            expo = self.exp(sig, self.alpha_stat)


    def custom_bust(self, sig):

        sig += 1e-9

        res = 0.0
        if (self.max_val_in_percent_diapazon_stat != self.max_val_in_percent_diapazon and self.max_val_in_percent_diapazon is not None) or (self.alpha_stat != self.alpha and self.alpha is not None) or (self.low_percent_stat != self.percent and self.percent is not None) or (self.beta_stat != self.beta and self.beta is not None):

            self.max_val_in_percent_diapazon_stat = self.max_val_in_percent_diapazon
            self.alpha_stat = self.alpha
            self.low_percent_stat = self.percent
            self.beta_stat = self.beta

        znak = sig / abs(sig)

        if abs(sig) <= (self.max_stik_val * self.low_percent_stat):

            max_sig_val_from_amplitude_percent = self.max_stik_val * self.low_percent_stat #определяет амплитуду отклонения стика, в которой будет данное сглаживание
            norm_sig_to_expo = sig / max_sig_val_from_amplitude_percent #нормировка значения сигнала для полноценного цикла работы expo (макс. зн. = 1)
            max_sig_val_in_amplitude_percent = self.max_stik_val * self.max_val_in_percent_diapazon_stat #максимальное значение сигнала, которое мы можем получить при установленной амплитуде стика
            expo_res = max_sig_val_in_amplitude_percent * self.exp(norm_sig_to_expo, self.alpha_stat) #результат применения expo к нормированному сигналу и домножение на верхний ограничитель

            res = self.max_deg_s * expo_res

            # self.max_val_in_percent_diapazon, self.alpha, self.percent = None, None, None #переводим в начальные состояния, чтобы после изменения параметров произошёл пересчёт коэффициентов

        else:
            #тут продолжение для интервала от 0.2 до 1.0
            #логика такова: чтобы работал Expo - нужно, чтобы вход нормировался и принимал значения от 0 до 1, следовательно, так как у нас значения будут от 0.2 до 1
            min_amplitude = self.max_stik_val * self.low_percent_stat
            max_signal_in_center_amplitude = self.max_stik_val * self.max_val_in_percent_diapazon_stat #минимальное значение, которое мы будет прибавлять к итоговому сигналу (так как в центральном диапазоне оно является максимумом, а мы нормируем сигнал и может выйти, что мы получем 0)
            norm = (abs(sig) - min_amplitude) / (self.max_stik_val - min_amplitude) #нормируем сигнал с учётом нижнего ограничения 0.3 так, чтобы если поступивший сигнал был равен 0.3, то выдаваемое слаженное значение было равно 0

            expo_res = self.exp(znak * norm, self.beta_stat)

            # res =  znak * (self.max_deg_s * (max_signal_in_center_amplitude + (self.max_stik_val - max_signal_in_center_amplitude) * abs(expo_res)))
            res =  znak * (max_signal_in_center_amplitude + (self.max_stik_val - max_signal_in_center_amplitude) * abs(expo_res))

        return res