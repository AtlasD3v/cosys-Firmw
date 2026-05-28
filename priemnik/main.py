from upravlenie import calibrate as cal
from pid_reg import control_loop as cont_loop

from  pid_reg import Control as control

def main():

    # res_cust = pt.Expo()
    # res_cust.custom_bust(-0.3001)
    print("---СИСТЕМА ЗАПУЩЕНА!---\n")
    # popravka, calibrating_res, joy_id = cal.calidrate_joy()
    # print("--КАЛИБРОВКА ПРОЙДЕНА-- \n")

    # print("---ЗАПУСКАЕМ СЧИТЫВАНИЕ, ФИЛЬТРАЦИЮ И СГЛАЖИВАНИЕ СИГНАЛА---")

    #rc_pt_count = int(input("Введите порядок фильтра ФНЧ (доступно: 1, 3): "))
    rc_pt_count = [3,3,3,3]
    throttle_signal_number = 1
    expo_number = [0,0,0,0]
    #expo_number = int(input("Введите номер метода модификации кривой управления, который вы хотите использовать (доступно: 0 (Expo), 1 (custom boost)): "))
    dt = 0.001

    #expo_alpha = 0.4
    #expo_beta = 0.9
    expo_alpha = [0.4,0.4, 0.4, 0.4 ]
    expo_beta = [0.9,0.9,0.9,0.9 ]
    #low_amplitude_percent = 0.3#амплитуда центра джойстика
    #max_val_in_percent_diapazon = 0.2
    low_amplitude_percent = [0.3,0.3,0.3,0.3]
    max_val_in_percent_diapazon = [0.2, 0.2,0.2,0.2]

    
    
    # control_loop = cont_loop.Control_loop(0.001,rc_pt_count,expo_number,dt,expo_alpha,expo_beta, low_amplitude_percent, max_val_in_percent_diapazon )
    # control_loop.main_cylce_func()
    control_loop = control.Control_loop()
    control_loop.control_loop_func()









if __name__ == '__main__':
    main()