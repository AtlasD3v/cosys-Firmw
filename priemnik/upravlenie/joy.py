# import sys

# import matplotlib.pyplot as plt
# import pygame
# import pygame as pg
# import time
# from filters import pt
# import numpy as np
# import threading
# import time
# from sensor_readers import readers

# L_X = None
# L_Y = None
# R_X = None
# R_Y = None
# THROTTLE = None
# UNKNOWN = None
# SIGNALS = []

# CAN_READ = False
# POPRAVKI = [0,0,0,0,0]


# TEST_SIG_ARR = []

# CALIBRATING_IS_OK = False #булевая переменная, показывающая, пройдена ли калибровка (устанавливается в Calibrate.py)

# def check_joy():
#     pg.init()
#     pg.joystick.init()

#     j_c = pg.joystick.get_count()

#     print(f"Найдено джойстиков: {j_c}")

#     connections_arr = []
#     for x in range(j_c):
#         j_num = pg.joystick.get_count().bit_count()
#         js = pg.joystick.Joystick(x)

#         connections_arr.append(js)

#         j_name = js.get_name()
#         print(f"id джойстика: {x}, имя джойстика: {j_name}")
#     return connections_arr



# def init_joy(id):
#     pg.init()
#     js = pg.joystick.Joystick(id)
#     js.init()
#     print("---ПОВТОРНАЯ ИНИЦИАЛИЗАЦИЯ ДЖОЙСТИКА ДЛЯ ОСНОВНОГО РЕЖИМА---")

#     return js
# def connect_joy(js, go):


#     if go:
#         try:
#             js.init()
#             print(f"Успешно подключён джойстик {js.get_name()}, включаем прослушивание команд")

#             return js
#         except:
#             print("Ошибка инициализации джойстика")
#             return None
#         # read_commands(js)

#     else:
#         print("Не смогли иницализировать джойстик")
#         return None, False



# def apply_deadzone(value, deadzone):
#     """Стандартная обработка deadzone (мёртвой зоны)"""
#     if abs(value) < deadzone:
#         return 0.0
#     # Линейная компенсация deadzone (опционально, можно убрать if не нужна)
#     # return (abs(value) - deadzone) / (1.0 - deadzone) * (1.0 if value > 0 else -1.0)
#     return value

# def read_command(js, popravki):
#     global L_X, THROTTLE, R_X, R_Y, UNKNOWN, SIGNALS

#     # 1. Сначала обрабатываем очередь системных событий (чтобы программа не зависала)
#     for event in pygame.event.get():
#         if event.type == pg.QUIT:
#             pg.quit()
#             sys.exit()

#         # 2. ПРИНУДИТЕЛЬНО обновляем внутреннее состояние осей джойстика в памяти Pygame
#         pygame.event.pump()
    
#         # 3. Считываем значения СРАЗУ, без привязки к циклу event
#         raw_ax0 = js.get_axis(0)
#         raw_ax1 = js.get_axis(1)
#         raw_ax2 = js.get_axis(2)
#         raw_ax3 = js.get_axis(3)
#         raw_ax4 = js.get_axis(4) if js.get_numaxes() > 4 else 0.0

#         # 4. Применяем deadzone
#         L_X      = apply_deadzone(raw_ax0, popravki[0])
#         THROTTLE = apply_deadzone(raw_ax1, popravki[4]) 
#         R_X      = apply_deadzone(raw_ax2, popravki[2])
#         R_Y      = apply_deadzone(raw_ax3, popravki[3])
#         UNKNOWN  = raw_ax4

#         SIGNALS = [L_X, THROTTLE, R_X, R_Y, UNKNOWN]

#         print(SIGNALS)
#     # Возвращаем управление основному циклу
#     # return SIGNALS


# def main_joy_func(selected_dead_zone):
#     conn_arr = check_joy()

#     id = int(input("Введите id джойстика, которым вы собираетесь управлять: "))
#     conn_string = conn_arr[id] 

#     joy_connect = connect_joy(conn_string, True)

#     if joy_connect is not None:
#         print("--- Начинаем бесконечный тест опроса (нажмите Ctrl+C для выхода) ---")
#         try:
#             while True:
#                 # Теперь функция вызывается атомарно, обновляет данные и возвращает их
#                 read_command(joy_connect, selected_dead_zone)

#                 time.sleep(0.02)
#         except KeyboardInterrupt:
#             print("\nТест завершен.")



# def signal_imitator(min_val, max_val, step):
#     sig_arr = np.arange(min_val, max_val + step, step).tolist()
#     return sig_arr, len(sig_arr)

# def graph(JOY_VAL, SIG_VAL):
#     plt.figure(figsize=(10, 5))
#     plt.plot(JOY_VAL, SIG_VAL, 'o-', label="SIG vs JOY")
#     plt.xlabel("JOY (сигнал с пульта)")
#     plt.ylabel("SIG (фильтрованный сигнал / обороты)")
#     plt.title("Зависимость фильтрованного сигнала от исходного")
#     plt.grid(True)
#     plt.legend()
#     plt.show()

#     # --- 2. Динамика сигналов во времени (если хочется сравнить) ---
#     plt.figure(figsize=(10, 5))
#     plt.plot(JOY_VAL, label="JOY (сырой)")
#     plt.plot(SIG_VAL, label="SIG (фильтрованный)")
#     plt.xlabel("Время (отсчёты)")
#     plt.ylabel("Значение сигнала")
#     plt.title("Сравнение сигналов во времени")
#     plt.grid(True)
#     plt.legend()
#     plt.show()


import sys
import pygame
import pygame as pg
import time
import threading

# ── Глобальные переменные ─────────────────────────────────────────────────────
L_X      = 0.0
L_Y      = 0.0
R_X      = 0.0
R_Y      = 0.0
THROTTLE = 0.0
UNKNOWN  = 0.0
SIGNALS  = [0.0, 0.0, 0.0, 0.0, 0.0]

_lock        = threading.Lock()   # защита при одновременном чтении/записи
_joy_running = False              # флаг управления потоком

# ── Вспомогательные функции ───────────────────────────────────────────────────

def apply_deadzone(value: float, deadzone: float) -> float:
    """Мёртвая зона: если |value| < deadzone → 0, иначе пропускаем как есть."""
    return 0.0 if abs(value) < deadzone else value


def check_joy() -> list:
    pg.init()
    pg.joystick.init()
    count = pg.joystick.get_count()
    print(f"Найдено джойстиков: {count}")

    joysticks = []
    for i in range(count):
        js = pg.joystick.Joystick(i)
        js.init()
        joysticks.append(js)
        print(f"  [{i}] {js.get_name()}")
    return joysticks


# ── Поточная функция ──────────────────────────────────────────────────────────

def _reader_loop(js: pg.joystick.JoystickType, dead_zones: list, interval: float = 0.02):
    """
    Выполняется в отдельном потоке.
    Читает оси каждые `interval` секунд и пишет в глобальные переменные.
    """
    global L_X, L_Y, R_X, R_Y, THROTTLE, UNKNOWN, SIGNALS, _joy_running

    print("─── Поток джойстика запущен ───")
    num_axes = js.get_numaxes()

    while _joy_running:
        # 1. pump() — ОБЯЗАТЕЛЬНО до event.get(), обновляет внутреннее состояние pygame
        pygame.event.pump()

        # 2. Обрабатываем системные события (чтобы не копились в очереди)
        for event in pygame.event.get():
            if event.type == pg.QUIT:
                _joy_running = False
                return

        # 3. Читаем оси — СНАРУЖИ цикла событий
        def safe_axis(idx):
            return js.get_axis(idx) if idx < num_axes else 0.0

        raw = [safe_axis(i) for i in range(5)]

        lx      = apply_deadzone(raw[1], dead_zones[1]) #roll
        ly= apply_deadzone(raw[0], dead_zones[0]) #pitch
        throttle      = apply_deadzone(raw[3], dead_zones[3]) #throttle
        ry      = apply_deadzone(raw[2], dead_zones[2]) #yaw
        unknown = raw[4]  # ось 4 — без мёртвой зоны (триммер и т.п.)

        # 4. Атомарно обновляем глобалы под локом
        with _lock:
            L_X = lx
            L_Y = ly
            THROTTLE = throttle * (-1.0) #нужна инверсия, чтобы поднимая стик вверх было положительное значение
            R_Y      = ry
            UNKNOWN  = unknown
            SIGNALS  = [L_X, L_Y, THROTTLE, R_Y]

        # print(f"ROLL: {L_Y}, PITCH: {L_X}, THROTTLE: {THROTTLE}, YAW: {R_Y}")
        time.sleep(interval)

    # Сброс при остановке потока
    with _lock:
        L_X = L_Y = R_X = R_Y = THROTTLE = UNKNOWN = 0.0
        SIGNALS = [0.0] * 5
    print("─── Поток джойстика остановлен ───")


# ── Публичный API ─────────────────────────────────────────────────────────────

def start_joy_thread(js: pg.joystick.JoystickType, dead_zones: list,
                     interval: float = 0.02) -> threading.Thread:
    """Запускает поток чтения и возвращает объект Thread."""
    global _joy_running
    _joy_running = True
    t = threading.Thread(
        target=_reader_loop,
        args=(js, dead_zones, interval),
        daemon=True   # поток умрёт вместе с основной программой
    )
    t.start()
    return t


def stop_joy_thread():
    """Вежливо останавливает поток."""
    global _joy_running
    _joy_running = False


def get_signals() -> list:
    """Потокобезопасное чтение текущих значений из любого места программы."""
    with _lock:
        return list(SIGNALS)


# ── Основной режим (оставлен для обратной совместимости) ─────────────────────

def main_joy_func(dead_zones: list):
    joysticks = check_joy()
    if not joysticks:
        print("Джойстики не найдены!")
        return

    idx = int(input("Введите id джойстика: "))
    js  = joysticks[idx]

    thread = start_joy_thread(js, dead_zones)

    print("Нажмите Ctrl+C для выхода")
    # try:
        
    #     while True:
    #         pass        # основной поток свободен для другой работы
    # except KeyboardInterrupt:
    #     print("\nОстанавливаем...")
    #     stop_joy_thread()
    #     thread.join(timeout=1.0)