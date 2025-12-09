import math
import matplotlib.pyplot as plt


T_total = 420.0        # Увеличено для наблюдения полёта после падения
dt = 0.1              # Шаг интегрирования

# Параметры ступеней
stages = [
    # Ступень 1
    {
        "M_start": 640_100,
        "M_end": 339_100,
        "F_thrust": 6_903_500,
        "T_burn": 120.0
    },
    # Ступень 2
    {
        "M_start": 266_800,
        "M_end": 129_300,
        "F_thrust": 2_280_660,
        "T_burn": 164.0
    },
    # Ступень 3
    {
        "M_start": 84_900,
        "M_end": 41_600,
        "F_thrust": 1_270_460,
        "T_burn": 70.0
    }
]

# Аэродинамика
d = 6.7                   # Диаметр ракеты (м)
S = math.pi * (d/2)**2    # Площадь поперечного сечения (м кв.)
C = 0.25                  # Коэф. обтекаемости

beta = 0.0025             # Скорость изменения угла (рад/с)

g = 9.80665

H = 20000.0

N = int(T_total / dt) + 1
time = [i * dt for i in range(N)]

x = 0.0
y = 0.0
vx = 0.0
vy = 0.0

height = []
speed = []

current_stage = 0
m = stages[0]["M_start"]

for i, t in enumerate(time):

    if current_stage < len(stages) - 1:
        if t >= sum(stage["T_burn"] for stage in stages[:current_stage+1]):
            current_stage += 1
    
    stage = stages[current_stage]
    T_burn = stage["T_burn"]
    F_thrust = stage["F_thrust"]
    
    t_stage = t - sum(stage["T_burn"] for stage in stages[:current_stage])
    
    # Масса
    if t_stage <= T_burn:
        m = stage["M_start"] - (stage["M_start"] - stage["M_end"]) * (t_stage / T_burn)
        m = max(m, stage["M_end"])
    else:
        m = stage["M_end"]

    # Тангаж
    alpha = math.pi / 2 - beta * t
    if alpha < 0:
        alpha = 0.0

    h = max(y, 0.0)
    rho = math.exp(-h / H)

    V = math.sqrt(vx**2 + vy**2)

    if t_stage <= T_burn:
        F_thrust_x = F_thrust * math.cos(alpha)
        F_thrust_y = F_thrust * math.sin(alpha)
    else:
        F_thrust_x = 0.0
        F_thrust_y = 0.0

    if V < 1e-6:
        drag_x = 0.0
        drag_y = 0.0
    else:
        drag_force = 0.5 * C * S * rho * V**2
        drag_x = -drag_force * (vx / V)
        drag_y = -drag_force * (vy / V)

    F_gravity_x = 0.0
    F_gravity_y = -m * g

    Fx = F_thrust_x + drag_x
    Fy = F_thrust_y + drag_y + F_gravity_y

    ax = Fx / m
    ay = Fy / m


    vx += ax * dt
    vy += ay * dt
    x += vx * dt
    y += vy * dt


    height.append(y)
    speed.append(V)


    if V > 1e6 or y > 1e8:
        remaining = len(time) - i - 1
        height.extend([y] * remaining)
        speed.extend([V] * remaining)
        break


plt.figure(figsize=(8, 6))
plt.plot(time, height, 'b-', linewidth=1.5)
plt.title('Зависимость высоты от времени')
plt.xlabel('Время (с)')
plt.ylabel('Высота (м)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()

plt.figure(figsize=(8, 6))
plt.plot(time, speed, 'r-', linewidth=1.5)
plt.title('Зависимость скорости от времени')
plt.xlabel('Время (с)')
plt.ylabel('Скорость (м/с)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()

plt.show()