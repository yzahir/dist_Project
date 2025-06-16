from controller import Supervisor, Robot, InertialUnit
import math
import json

# Constants
forward_speed = 6.28
turn_speed = 3.14
max_range = 0.3

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

pi_puck_id = robot.getName()
puck_pos_dict = {}

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
gps = robot.getDevice("gps")
imu = robot.getDevice("inertial unit")

gps.enable(timestep)
imu.enable(timestep)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timestep)

# States
STATE_START = 0
STATE_WAIT_FOR_NEIGHBORS = 1
STATE_START_ROTATE = 2
STATE_START_DRIVE = 3
STATE_LINE_REACHED = 4
STATE_END =10

current_state = STATE_START
role = "UNKNOWN"
target_x = None
target_y = None
start_position = None  # For tracking drive distance

# Helper functions
def normalize_angle_deg(angle):
    return angle % 360

def get_position():
    gps_values = gps.getValues()
    world_x = gps_values[0]
    world_y = gps_values[1]
    #arena_x = world_x
    #arena_y = world_z
    yaw_deg = math.degrees(imu.getRollPitchYaw()[2])
    yaw_deg = normalize_angle_deg(yaw_deg)
    return world_x, world_y, yaw_deg

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def publish_position():
    position = get_position()
    data = {
        str(pi_puck_id): {
            "position": position[:2],
            "angle": position[2]
        }
    }
    emitter.send(json.dumps(data).encode('utf-8'))

def on_message():
    rbt_in_range = set()
    x_self, y_self, _ = get_position()
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        data = json.loads(msg)
        for robot_id, robot_data in data.items():
            if robot_id == pi_puck_id:
                continue
            position = robot_data["position"]
            angle = robot_data["angle"]
            dist = distance(x_self, y_self, position[0], position[1])
            if dist <= max_range:
                puck_pos_dict[robot_id] = {"position": position, "angle": angle}
                rbt_in_range.add(robot_id)
        receiver.nextPacket()
    for rid in list(puck_pos_dict.keys()):
        if rid not in rbt_in_range:
            del puck_pos_dict[rid]

def extract_int(id_str):
    return int(''.join(filter(str.isdigit, id_str)))

# --- Modular Movement Logic ---

def rotate_to_target_stepwise(x, y, angle, target_x, target_y, threshold=5.0):
    dx = target_x - x
    dy = target_y - y
    angle_to_target = math.degrees(math.atan2(dy, dx))
    target_angle = normalize_angle_deg(angle_to_target)
    angle_diff = (target_angle - angle + 540) % 360 - 180

    print(f"[{pi_puck_id}] Rotating: Target {target_angle:.2f}, Current {angle:.2f}, Diff {angle_diff:.2f}")
    
    if abs(angle_diff) < threshold:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        return True

    speed = max(min(0.5 * angle_diff, turn_speed), -turn_speed)
    left_motor.setVelocity(speed if angle_diff > 0 else -speed)
    right_motor.setVelocity(-speed if angle_diff > 0 else speed)
    return False

def drive_forward_stepwise(target_x, target_y, speed=forward_speed * 0.5):
    global start_position
    x, y, _ = get_position()

    if start_position is None:
        start_position = (x, y)

    dist = distance(x, y, target_x, target_y)
    print(f"[{pi_puck_id}] Driving: Current ({x:.2f}, {y:.2f}) → Target ({target_x:.2f}, {target_y:.2f}) Dist: {dist:.3f}")

    if dist < 0.05:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        start_position = None
        return True

    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)
    return False
#jj
while robot.step(timestep) != -1:
    publish_position()
    on_message()
    x, y, angle = get_position()

    if current_state == STATE_START:
        print("Waiting for neighbors to arrive...")
        current_state = STATE_WAIT_FOR_NEIGHBORS

    elif current_state == STATE_WAIT_FOR_NEIGHBORS:
        all_ids = list(puck_pos_dict.keys()) + [pi_puck_id]
        if len(all_ids) <= 0:
            print("Still alone... waiting")
            continue
        else:
            sorted_ids = sorted(all_ids, key=extract_int)
            my_index = sorted_ids.index(pi_puck_id)
            role = "LEADER" if my_index == 0 else "FOLLOWER"
            print(f"I am {role} with ID {pi_puck_id}, index {my_index}")
            total_robots = len(sorted_ids)
            spacing = min(max_range * 0.9, 1.0 / total_robots)
            y_line_start = 0.1
            target_y = y_line_start + my_index * spacing
            target_x = 0.1
            print(f"Target arena position: ({target_x:.2f}, {target_y:.2f})")
            current_state = STATE_START_ROTATE

    elif current_state == STATE_START_ROTATE:
        if rotate_to_target_stepwise(x, y, angle, target_x, target_y):
            current_state = STATE_START_DRIVE

    elif current_state == STATE_START_DRIVE:
        if drive_forward_stepwise(target_x, target_y):
            print(f"{pi_puck_id} reached target position.")
            current_state = STATE_LINE_REACHED

    elif current_state == STATE_LINE_REACHED:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if rotate_to_target_stepwise(x, y, angle, 2, target_y):
            current_state = STATE_END
    elif current_state == STATE_END:
        print(f"{pi_puck_id} has reached the end of the simulation.")
        break
        
