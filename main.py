import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import random
import math

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port
pi_puck_id = '18'

x = 0.0
y = 0.0

# Arena parameters
StartX     = 0.1
SweepEndX  = 1.9   # 2m arena minus 0.1 margin
ArenaMaxY  = 1.0

# Constants
forward_speed = 6.28
turn_speed    = 3.14
max_range     = 0.3
wheel_step_to_cm = 0.01288  # 1 step ≈ 0.01288 cm
axle_radius_cm = 2.65       # 53 mm between wheels → r = 2.65 cm

puck_pos_dict = {}
puck_dict = {}

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")
    client.subscribe("robots/all")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if msg.topic == "robot_pos/all":
            puck_pos_dict.update(data)
            x_self, y_self, _ = get_position()
            in_range = set()
            for robot_id, robot_data in data.items():
                if robot_id == pi_puck_id:
                    continue
                msg_x, msg_y = robot_data.get("x"), robot_data.get("y")
                if dist(x_self, y_self, msg_x, msg_y) < max_range:
                    puck_dict[robot_id] = robot_data
                    in_range.add(robot_id)
            for rid in list(puck_dict.keys()):
                if rid not in in_range:
                    del puck_dict[rid]
    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')

def normalize_angle_deg(a): return a % 360
def extract_int(s): return int(''.join(filter(str.isdigit, s)))
def dist(x1, y1, x2, y2): return math.hypot(x2 - x1, y2 - y1)

def publish_data(packet):
    client.publish("robots/all", json.dumps(packet))

def drive_forward_stepwise(tx, ty, spd=forward_speed):
    global start_position
    x, y, _ = get_position()
    if start_position is None:
        start_position = (x, y)
    d = dist(x, y, tx, ty)
    print(f"[{pi_puck_id}] Driving→ ({x:.2f},{y:.2f})→({tx:.2f},{ty:.2f}) d={d:.3f}")
    if d < 0.05:
        pipuck.epuck.set_motor_speeds(0, 0)
        start_position = None
        return True
    pipuck.epuck.set_motor_speeds(spd, spd)
    return False

def rotate_to_target_stepwise(x, y, ang, tx, ty, thresh=3.0):
    dx = tx - x
    dy = ty - y
    angle1 = math.degrees(math.atan2(dy, dx))
    targ_ang = (-angle1 + 90) % 360
    diff = (targ_ang - ang + 540) % 360 - 180
    print(f"[{pi_puck_id}] Rotating→ T:{targ_ang:.1f} C:{ang:.1f} Δ:{diff:.1f}")
    if abs(diff) < thresh:
        pipuck.epuck.set_motor_speeds(0, 0)
        return True
    spd = max(min(0.05 * diff, turn_speed), 100)
    if diff > 0:
        pipuck.epuck.set_motor_speeds(spd, -spd)
    else:
        pipuck.epuck.set_motor_speeds(-spd, spd)
    return False

# MQTT Setup
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(Broker, Port, 60)
client.loop_start()

pipuck = PiPuck(epuck_version=2)

def get_position(id=pi_puck_id):
    data = puck_pos_dict.get(id)
    if not data:
        print(f"No data for PiPuck ID: {id}")
        return None, None, None
    x = data.get("x")
    y = data.get("y")
    angle = data.get("angle")
    if x is None or y is None or angle is None:
        print("Incomplete data:", data)
        return None, None, None
    return x, y, angle

# FSM States
STATE_START         = 0
STATE_WAIT_FOR_NEIGHBORS = 1
STATE_START_ROTATE  = 2
STATE_START_DRIVE   = 3
STATE_LINE_REACHED  = 4
STATE_START_SWEEP   = 5
STATE_SWEEP_DRIVE   = 6
STATE_ADVANCE_ROW   = 7
STATE_ROW_ROTATE    = 8
STATE_ROW_DRIVE     = 9
STATE_DONE          = 10

current_state = STATE_START
role = "UNKNOWN"
target_x = None
target_y = None
start_position = None

# Row sweeping globals
rowY = 0.1  # ← FIXED: Initialized rowY to avoid NoneType error
spacing = None
sweep_direction = 1

try:
    for _ in range(1000):
        time.sleep(1)
        x, y, angle = get_position()
        if x is not None and y is not None:
            publish_data({
                pi_puck_id: {
                    "x": x,
                    "y": y,
                    "angle": angle,
                    "sensors": {
                        "temperature": random.randint(0,50),
                        "humidity": random.randint(0,100),
                        "light": random.randint(0,100)
                    },
                    "target_found": False
                }
            })
        else:
            continue

        all_ids = sorted(list(puck_dict.keys()) + [pi_puck_id], key=extract_int)

        if spacing is None and len(all_ids) > 0:
            spacing = min(max_range * 0.9, ArenaMaxY / len(all_ids))

        if current_state == STATE_START:
            print("Waiting for neighbors...")
            current_state = STATE_WAIT_FOR_NEIGHBORS

        elif current_state == STATE_WAIT_FOR_NEIGHBORS:
            if len(all_ids) == 0:
                continue
            role = "LEADER" if int(pi_puck_id) == min(map(int, all_ids)) else "FOLLOWER"
            idx = all_ids.index(pi_puck_id)
            target_x = StartX
            target_y = rowY + idx * spacing
            print(f"I am {role} idx={idx}, lineY={rowY:.2f}, target=({target_x:.2f},{target_y:.2f})")
            current_state = STATE_START_ROTATE

        elif current_state == STATE_START_ROTATE:
            if rotate_to_target_stepwise(x, y, angle, target_x, target_y):
                current_state = STATE_START_DRIVE

        elif current_state == STATE_START_DRIVE:
            if drive_forward_stepwise(target_x, target_y):
                print(f"{pi_puck_id} formed line.")
                target_x = SweepEndX if sweep_direction == 1 else StartX
                current_state = STATE_START_SWEEP

        elif current_state == STATE_START_SWEEP:
            if rotate_to_target_stepwise(x, y, angle, target_x, target_y):
                current_state = STATE_SWEEP_DRIVE

        elif current_state == STATE_SWEEP_DRIVE:
            if drive_forward_stepwise(target_x, target_y):
                print(f"{pi_puck_id} sweep row complete.")
                current_state = STATE_ADVANCE_ROW

        elif current_state == STATE_ADVANCE_ROW:
            if rowY + spacing > ArenaMaxY - spacing:
                current_state = STATE_DONE
            else:
                rowY += spacing
                idx = all_ids.index(pi_puck_id)
                target_y = rowY + idx * spacing
                current_state = STATE_ROW_ROTATE

        elif current_state == STATE_ROW_ROTATE:
            print(f"{pi_puck_id} rotating to rowY={target_y:.2f}")
            if rotate_to_target_stepwise(x, y, angle, x, target_y):
                current_state = STATE_ROW_DRIVE

        elif current_state == STATE_ROW_DRIVE:
            if drive_forward_stepwise(x, target_y):
                sweep_direction *= -1
                target_x = SweepEndX if sweep_direction == 1 else StartX
                current_state = STATE_START_SWEEP

        elif current_state == STATE_DONE:
            pipuck.epuck.set_motor_speeds(0, 0)
            print(f"{pi_puck_id} DONE sweeping.")
            break

except KeyboardInterrupt:
    print("Interrupt detected!!")

finally:
    pipuck.epuck.set_motor_speeds(0, 0)
    client.loop_stop()
