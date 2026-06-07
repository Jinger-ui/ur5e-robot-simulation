"""
GaiaHand Webots Controller
Demonstrates finger joint control: open/close (fist) animation.
"""
from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little']

motors = {}
sensors = {}
joint_limits = {}

for finger in FINGERS:
    for j in range(1, 4):
        name = f"right_{finger}_joint_{j}"
        motor = robot.getDevice(name)
        sensor = robot.getDevice(f"{name}_sensor")
        if motor and sensor:
            sensor.enable(timestep)
            motors[name] = motor
            sensors[name] = sensor
            joint_limits[name] = (motor.getMinPosition(), motor.getMaxPosition())

print(f"[GaiaHand] Initialized {len(motors)} joint motors")
for name, (lo, hi) in sorted(joint_limits.items()):
    print(f"  {name}: [{lo:.3f}, {hi:.3f}] rad")

# Animation parameters
CLOSE_TARGETS = {}
OPEN_TARGETS = {}

for finger in FINGERS:
    j1 = f"right_{finger}_joint_1"
    j2 = f"right_{finger}_joint_2"
    j3 = f"right_{finger}_joint_3"

    if j1 in joint_limits:
        CLOSE_TARGETS[j1] = 0.0
        OPEN_TARGETS[j1] = 0.0

    if j2 in joint_limits:
        _, hi = joint_limits[j2]
        CLOSE_TARGETS[j2] = hi * 0.85
        OPEN_TARGETS[j2] = 0.0

    if j3 in joint_limits:
        _, hi = joint_limits[j3]
        CLOSE_TARGETS[j3] = hi * 0.85
        OPEN_TARGETS[j3] = 0.0

state = "closing"
phase_time = 0.0
PHASE_DURATION = 3.0  # seconds per phase
finger_delay = 0.4    # seconds between each finger starting

print(f"[GaiaHand] Starting animation: close/open cycle every {PHASE_DURATION*2:.0f}s")

while robot.step(timestep) != -1:
    dt = timestep / 1000.0
    phase_time += dt

    if phase_time >= PHASE_DURATION:
        phase_time = 0.0
        state = "opening" if state == "closing" else "closing"

    targets = CLOSE_TARGETS if state == "closing" else OPEN_TARGETS

    for i, finger in enumerate(FINGERS):
        finger_start = i * finger_delay
        if phase_time < finger_start:
            continue

        progress = min(1.0, (phase_time - finger_start) / (PHASE_DURATION - finger_start))
        smooth = 0.5 * (1.0 - math.cos(math.pi * progress))

        for j in range(1, 4):
            name = f"right_{finger}_joint_{j}"
            if name not in motors:
                continue

            target = targets[name]
            current_sensor_val = sensors[name].getValue()

            if state == "closing":
                pos = current_sensor_val + (target - current_sensor_val) * smooth
                pos = target  # direct target for closing
            else:
                pos = current_sensor_val + (target - current_sensor_val) * smooth
                pos = target  # direct target for opening

            # Smooth interpolation
            blend = smooth
            if state == "closing":
                pos = (1.0 - blend) * OPEN_TARGETS.get(name, 0.0) + blend * CLOSE_TARGETS.get(name, 0.0)
            else:
                pos = (1.0 - blend) * CLOSE_TARGETS.get(name, 0.0) + blend * OPEN_TARGETS.get(name, 0.0)

            motors[name].setPosition(pos)
