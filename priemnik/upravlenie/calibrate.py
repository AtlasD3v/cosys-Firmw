import sys
import time
from upravlenie import joy as j
import threading

def calidrate_joy():

    print('ПОЖАЛУЙСТА, НЕ ТРОГАЙТЕ НИКАКИЕ СТИКИ В ТЕЧЕНИЕ 1 СЕКУНД НА ДЖОЙСТИКЕ')

    con_arr = j.check_joy() #проверяем подключённые джойстики

    id = int(input("Введите id джойстика, которым вы собираетесь управлять: "))
    conn_string = con_arr[id] #возвращаем массив доступных джойстиков

    time_start = time.time()
    time_end = time_start + 1 #интервал в котором замеряем погрешности
    js_to_read_commands = j.connect_joy(conn_string, True) #также в методе connect_joy активный джойстик устанавливается в переменную ACTIVE_JOY

    lx_arr, ly_arr, rx_arr, ry_arr, thr_arr = [], [], [], [], [] #массивы в которые вносятся данные во время замеров погрещностей
    popravka = [0,0,0,0,0]

    all_comms = None

    if not (js_to_read_commands is None):
        j.CAN_READ = True

        # j.read_commands(js_to_read_commands)
        read_thread = threading.Thread(target=j.read_commands_to_poprav, args=(js_to_read_commands,), daemon=True) #специальная функция чтения данных с джойстика без корректировки
        read_thread.start() # Запускаем поток
        print("Начинаем поток чтения команд джойстика во время калибровки")


        while time.time() < time_end:
            if j.L_X is not None: lx_arr.append(j.L_X)
            #if j.L_Y is not None: ly_arr.append(j.L_Y)
            if j.R_X is not None: rx_arr.append(j.R_X)
            if j.R_Y is not None: ry_arr.append(j.R_Y)
            if j.THROTTLE is not None: thr_arr.append(j.THROTTLE)

            time.sleep(0.01)

        j.CAN_READ = False #закончили поток чтения данных для калибровки
        js_to_read_commands.quit() #деинициализирцем джойстик для последующей возможности его чтения

        all_comms = [lx_arr, ly_arr, rx_arr, ry_arr, thr_arr] #массив массивов всех команд


        for i in range(len(all_comms)):
            if all_comms[i]:
                popravka[i] = sum(all_comms[i]) / len(all_comms[i])

        print(f"Получили массив поправок в размере: {len(popravka)}")
        print(f"Получен массив поправок: {popravka}")


        read_thread.join()  # дождаться завершения потока
        print("Поток чтения команд остановлен")

        j.CALIBRATING_IS_OK = True #переменная, разрешаюшая далее чтение данных с джойстика для фильтрации и сглаживания внутри файла joy.py
        return popravka, j.CALIBRATING_IS_OK, id

    else:
        print("Не получили строки для чтения данных с джойстика")

        return None



    