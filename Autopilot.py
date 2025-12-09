import math
import time
import krpc

turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 150000

conn = krpc.connect(name='Precision 3-stage launch by delta-V')
vessel = conn.space_center.active_vessel

ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')


class DeltaVTracker:
    def __init__(self, vessel):
        self.vessel = vessel
        self.start_velocity = self.get_velocity()
        self.total_dv_used = 0
        self.stage_start_velocity = self.get_velocity()
        self.stage_dv_used = 0
        self.last_update_time = time.time()
        
    def get_velocity(self):
        return vessel.flight(vessel.orbit.body.reference_frame).speed
    
    def update(self):
        current_velocity = self.get_velocity()
        current_time = time.time()
        vertical_speed = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
        horizontal_speed = vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed
        current_speed = math.sqrt(vertical_speed**2 + horizontal_speed**2)
        dv_since_last = abs(current_speed - self.get_velocity())
        self.total_dv_used += dv_since_last
        self.stage_dv_used = current_speed - self.stage_start_velocity
        self.last_update_time = current_time
        return self.total_dv_used, self.stage_dv_used
    
    def reset_stage(self):
        self.stage_start_velocity = self.get_velocity()
        self.stage_dv_used = 0

dv_tracker = DeltaVTracker(vessel)

def calculate_optimal_throttle(current_dv, target_dv, current_altitude, target_altitude):
    dv_remaining = target_dv - current_dv
    altitude_ratio = current_altitude / target_altitude
    if dv_remaining > 500:
        return 1.0
    elif dv_remaining > 200:
        return 0.7
    elif dv_remaining > 100:
        return 0.4
    elif dv_remaining > 50:
        return 0.2
    elif dv_remaining > 10:
        return 0.1
    else:
        return 0.05

def get_liquid_fuel(stage_api_number):
    resources = vessel.resources_in_decouple_stage(stage=stage_api_number, cumulative=False)
    fuel = conn.add_stream(resources.amount, 'LiquidFuel')
    oxidizer = conn.add_stream(resources.amount, 'Oxidizer')
    return min(fuel(), oxidizer())

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Запуск!')

vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

current_stage_game = 9
stages_separated = []
turn_angle = 0
last_dv_print = time.time()

while True:
    total_dv, stage_dv = dv_tracker.update()
    
    if time.time() - last_dv_print > 10:
        print(f"Ступень {current_stage_game + 1 + len(stages_separated)}: V={stage_dv:.0f}м/с, Высота={altitude()/1000:.1f}км, Апоцентр={apoapsis()/1000:.1f}км")
        last_dv_print = time.time()
    
    if turn_start_altitude < altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) / (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)
    
    if current_stage_game == 9 and 9 not in stages_separated:
        if get_liquid_fuel(9) < 0.1:
            print('Отделение ступени 1')
            vessel.control.activate_next_stage()  
            time.sleep(0.5)
            vessel.control.activate_next_stage()  
            current_stage_game = 7
            stages_separated.append(9)
            print('Ступень 2 запущена')
    
    elif current_stage_game == 7 and 7 not in stages_separated:
        if get_liquid_fuel(7) < 0.1 or altitude() >= 69000:
            print('Отделение ступени 2')
            vessel.control.activate_next_stage() 
            time.sleep(0.5)
            vessel.control.activate_next_stage()  
            current_stage_game = 5
            stages_separated.append(7)
            print('Ступень 3 запущена')
            vessel.control.throttle = 0.7
    
    if apoapsis() > target_altitude * 1.1:
        print(f'АВАРИЙНОЕ ОТКЛЮЧЕНИЕ: апоцентр {apoapsis()/1000:.1f}км > {target_altitude/1000*1.1:.1f}км')
        vessel.control.throttle = 0.0
        break

print(f"Всего ΔV: {total_dv:.0f}м/с")
print(f"Апоцентр: {apoapsis()/1000:.1f}км")
print(f"Перицентр: {vessel.orbit.periapsis_altitude/1000:.1f}км")

print('Планирование маневра выхода на круговую орбиту')

while altitude() < 70500:
    pass

print('Отделение ступени 2')
vessel.control.activate_next_stage() 
time.sleep(0.5)
vessel.control.activate_next_stage()
print('Ступень 3 запущена')

mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu * ((2.0 / r) - (1.0 / a1)))
v2 = math.sqrt(mu * ((2.0 / r) - (1.0 / a2)))
delta_v = v2 - v1

node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis,
    prograde=delta_v
)

F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

print('Ориентация для маневра')
print('Ориентация для маневра через SAS')
vessel.auto_pilot.disengage()
time.sleep(0.2)
vessel.control.sas = True
time.sleep(0.3)
print("Устанавливаем режим 'маневр'")
vessel.control.sas_mode = conn.space_center.SASMode.maneuver
print("Ожидание ориентации на маневрный узел...")
time.sleep(40)
print("Корабль готов к маневру")
print('Завершение маневра')
vessel.control.throttle = 0.0
node.remove()
vessel.auto_pilot.engage()

print('Ожидание времени маневра')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time / 2.0)
lead_time = 5
conn.space_center.warp_to(burn_ut - lead_time)

print('Выполнение маневра')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')

while time_to_apoapsis() - (burn_time / 2.0) > 0:
    pass

vessel.control.throttle = 1.0
time.sleep(burn_time - 0.5)

print('Точная доводка')
vessel.control.throttle = 0.05
time.sleep(1.5)

try:
    if node.remaining_delta_v > 0.1:
        print(f"Доводим остаток: {node.remaining_delta_v:.1f} м/с")
        time.sleep(node.remaining_delta_v / 10)
except:
    pass

vessel.control.throttle = 0.0
node.remove()

print('Миссия выполнена! Ракета на целевой орбите.')
print(f'Апоцентр: {apoapsis()/1000:.1f} км')
print(f'Перицентр: {vessel.orbit.periapsis_altitude/1000:.1f} км')
