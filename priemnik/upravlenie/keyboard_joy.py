# keyboard_joy.py
"""
Эмулятор джойстика на клавиатуре.
API полностью совместим с joy.py — меняйте только строку импорта.

Управление:
  W / S        — тангаж      pitch  (R_Y)       ±
  A / D        — крен        roll   (R_X)        ±
  Q / E        — рыскание    yaw    (L_X)        ±
  Пробел       — тяга +      throttle (THROTTLE) ▲
  Left Shift   — тяга −      throttle (THROTTLE) ▼
"""

import pygame
import threading
import time

# ── Те же глобалы, что в joy.py ──────────────────────────────────────────────
L_X      = 0.0
L_Y      = 0.0
R_X      = 0.0
R_Y      = 0.0
THROTTLE = 0.0
UNKNOWN  = 0.0
SIGNALS  = [0.0, 0.0, 0.0, 0.0, 0.0]

_lock        = threading.Lock()
_joy_running = False

# ── Настройки динамики ────────────────────────────────────────────────────────
RATE        = 1.2   # скорость нарастания при зажатой клавише  (ед/сек)
RETURN_RATE = 2.5   # скорость возврата к нулю после отпускания (ед/сек)
#  → полное отклонение за ~0.83 с, возврат в ноль за ~0.4 с

# ── Вспомогалки ───────────────────────────────────────────────────────────────

def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


def _spring(val: float, dt: float) -> float:
    """Плавный возврат к нулю (для пружинных осей)."""
    if val > 0:
        return max(0.0, val - RETURN_RATE * dt)
    if val < 0:
        return min(0.0, val + RETURN_RATE * dt)
    return 0.0


def _draw_ui(screen, font, lx, rx, ry, th):
    """Маленькое информационное окошко."""
    screen.fill((25, 25, 35))

    rows = [
        ("Рыскание  Q / E",     lx,  (100, 180, 255)),
        ("Крен      A / D",     rx,  (100, 255, 160)),
        ("Тангаж    W / S",     ry,  (255, 220, 80)),
        ("Тяга   Spc / Shft",   th,  (255, 100, 100)),
    ]

    bar_w, bar_h = 160, 14
    for i, (label, val, color) in enumerate(rows):
        y = 18 + i * 46

        # Подпись
        surf = font.render(label, True, (200, 200, 210))
        screen.blit(surf, (10, y))

        # Фон полоски
        pygame.draw.rect(screen, (60, 60, 75), (10, y + 20, bar_w, bar_h), border_radius=4)

        # Заполненная часть (центр = 0)
        center = 10 + bar_w // 2
        fill   = int(val * bar_w / 2)
        rect_x = center if fill >= 0 else center + fill
        pygame.draw.rect(screen, color, (rect_x, y + 20, abs(fill), bar_h), border_radius=4)

        # Значение
        num = font.render(f"{val:+.3f}", True, color)
        screen.blit(num, (bar_w + 18, y + 20))

    pygame.display.flip()


# ── Основной поток ────────────────────────────────────────────────────────────

def _reader_loop(interval: float = 0.02):
    global L_X, R_X, R_Y, THROTTLE, UNKNOWN, SIGNALS, _joy_running

    pygame.init()
    screen = pygame.display.set_mode((260, 210))
    pygame.display.set_caption("⌨ Keyboard Joystick")
    font = pygame.font.SysFont("monospace", 13)

    ly = lx = rx = ry = th = 0.0
    prev = time.perf_counter()

    print("─── Клавиатурный эмулятор запущен ───")

    while _joy_running:
        now = time.perf_counter()
        dt  = min(now - prev, 0.1)   # защита от огромного dt при лагах
        prev = now

        # Системные события
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                _joy_running = False

        keys = pygame.key.get_pressed()

        # ── Рыскание (yaw) — Q / E ────────────────────────────────────────────
        if   keys[pygame.K_q]: ry = _clamp(ry - RATE * dt)
        elif keys[pygame.K_e]: ry = _clamp(ry + RATE * dt)
        # else:                  ry = _spring(ry, dt)

        # ── Крен (roll) — A / D ───────────────────────────────────────────────
        if   keys[pygame.K_a]: ly = _clamp(ly - RATE * dt)
        elif keys[pygame.K_d]: ly = _clamp(ly + RATE * dt)
        # else:                  ly = _spring(ly)

        # ── Тангаж (pitch) — W / S ────────────────────────────────────────────
        if   keys[pygame.K_w]: lx = _clamp(lx - RATE * dt)   # вперёд = −
        elif keys[pygame.K_s]: lx = _clamp(lx + RATE * dt)   # назад  = +
        # else:                  lx = _spring(lx, dt)

        # ── Тяга (throttle) — Пробел ▲ / Shift ▼  (не пружинит!) ────────────
        if   keys[pygame.K_SPACE]:    th = _clamp(th + RATE * dt)
        elif keys[pygame.K_LSHIFT]:   th = _clamp(th - RATE * dt)
        # иначе — тяга держится на месте

        # ── Атомарная запись в глобалы ────────────────────────────────────────
        with _lock:
            L_Y = ly
            L_X      = lx
            R_X      = rx
            R_Y      = ry
            THROTTLE = th
            UNKNOWN  = 0.0
            SIGNALS  = [L_Y, L_X, THROTTLE, R_Y]

        _draw_ui(screen, font, lx, rx, ry, th)
        time.sleep(interval)

    pygame.quit()
    print("─── Клавиатурный эмулятор остановлен ───")


# ── Публичный API (идентичен joy.py) ─────────────────────────────────────────

def start_joy_thread(_js=None, _dead_zones=None, interval: float = 0.02) -> threading.Thread:
    """
    Параметры _js и _dead_zones принимаются для совместимости с joy.py,
    но не используются — у клавиатуры нет мёртвой зоны.
    """
    global _joy_running
    _joy_running = True
    t = threading.Thread(target=_reader_loop, args=(interval,), daemon=True)
    t.start()
    return t


def stop_joy_thread():
    global _joy_running
    _joy_running = False


def get_signals() -> list:
    """Потокобезопасное чтение из любого места программы."""
    with _lock:
        return list(SIGNALS)


def main_joy_func(dead_zones=None):
    """Совместимый с joy.py точка входа."""
    print(__doc__)
    thread = start_joy_thread()
    # try:
    #     while True:
    #         time.sleep(0.1)
    # except KeyboardInterrupt:
    #     print("\nОстанавливаем...")
    #     stop_joy_thread()
    #     thread.join(timeout=1.0)