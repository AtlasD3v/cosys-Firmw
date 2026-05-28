import numpy as np

class GyroDynamicNotch:

    def __init__(self, Fs, fc, Q):
        self.Q = Q #Добротность. Чем больше Q, тем уже полоса (1.2)

        self.fc = fc#Частота среза
        self.Fs = Fs#Частота получения данных с гироскопа


        self.b0 = 0.0
        self.b1 = 0.0
        self.b2 = 0.0

        self.a0 = 0.0
        self.a1 = 0.0
        self.a2 = 0.0

        self.z1 = 0.0
        self.z2 = 0.0

        self.f_noice_filtered = None
        self.alpha_to_simple_filt = 0.8

        self.coefficient_updater(self.Fs, self.fc)


    def coefficient_updater(self, Fs, fc): #Fs - частота работы цикла (получения сигнала с гироскопа), fc - частота, которая должна вырезаться

        if self.f_noice_filtered is None and fc > 0:
            self.f_noice_filtered = fc

        if self.f_noice_filtered is not None:
            self.fc = self.alpha_to_simple_filt * self.f_noice_filtered + (1 - self.alpha_to_simple_filt) * fc
            self.f_noice_filtered = self.fc

            self.Fs = Fs

            w0 = 2.0 * np.pi * (self.fc / Fs) #нормализованная частота
            alpha = np.sin(w0) / (2.0 * self.Q)

            b0_ = 1.0
            b1_ = -2.0 * np.cos(w0)
            b2_ = 1.0

            a0_ = 1.0 + alpha #коеф. нормирования
            a1_ = -2.0 * np.cos(w0)
            a2_ = 1.0 - alpha

            self.b0 = b0_ / a0_
            self.b1 = b1_ / a0_
            self.b2 = b2_ / a0_
            self.a1 = a1_ / a0_
            self.a2 = a2_ / a0_




    def filter(self, x):
        y = self.b0 * x + self.z1
        z1_new = self.b1 * x - self.a1 * y + self.z2
        z2_new = self.b2 * x - self.a2 * y

        #Сдвиг состояний: z[k-1] = z[k]
        self.z1 = z1_new
        self.z2 = z2_new

        #Возвращаем отфильтрованный результат
        return y


    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0