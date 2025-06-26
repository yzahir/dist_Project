from controller import Supervisor, Robot, InertialUnit
import math, json, random
from collections import defaultdict




# Constants
forward_speed = 6.28
turn_speed    = 3.14
max_range     = 0.3
x = 0.0
y = 0.0
angle = 0.0
is_leader = False
target_x = 0.1
target_y = 0.1
ready = False
step_count = 0

# Arena parameters
StartX     = 0.1
SweepEndX  = 1.9    # 2m arena minus 0.1 margin
ArenaMaxY  = 1.0




# Initialize robot
robot        = Robot()
pi_puck_id  = robot.getName()
timestep    = int(robot.getBasicTimeStep())




# Devices
left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
gps          = robot.getDevice("gps")
imu          = robot.getDevice("inertial unit")
emitter     = robot.getDevice("emitter")
receiver    = robot.getDevice("receiver")




gps.enable(timestep)
imu.enable(timestep)
receiver.enable(timestep)
for m in (left_motor, right_motor):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)




# States
STATE_START          = 0
STATE_WAIT_FOR_puck_dict = 1
STATE_START_ROTATE  = 2
STATE_START_DRIVE    = 3
STATE_LINE_REACHED  = 4
STATE_START_SWEEP    = 5
STATE_SWEEP_DRIVE    = 6
STATE_ADVANCE_ROW    = 7
STATE_ROW_ROTATE    = 8
STATE_ROW_DRIVE     = 9
STATE_DONE           = 10
STATE_WAIT_FOR_NEIGHBORS_READY = 11


current_state = STATE_START
role           = "UNKNOWN"
target_x       = None
target_y       = None
start_position = None
ready = False
rows_swept = 0

# Row-sweeping globals
rowY       = None
spacing    = None
sweep_direction = 1  # 1=right, -1=left
puck_dict = {}
ack_received_from = set()
ready_counts = defaultdict(lambda: 0)  # counts how many times each robot sent ready=True
seen_ready_this_row = set()

# Helpers
def normalize_angle_deg(a): return a % 360
def distance(x1,y1,x2,y2): return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
def extract_int(s):     return int(''.join(filter(str.isdigit, s)))






def on_message():
    global step_count
    x_self, y_self, _  = get_position()
    while receiver.getQueueLength():
        data = json.loads(receiver.getString())
        receiver.nextPacket()
        for robot_id,robot_data in data.items():
            if robot_id==pi_puck_id: continue
            msg_x = robot_data.get("x")
            msg_y = robot_data.get("y")
            if distance(x_self,y_self,msg_x,msg_y) <= max_range + 0.02:
                puck_dict[robot_id] = robot_data
                #puck_dict[robot_id]["last_seen"] = step_count
                if robot_data.get("ready", False) and robot_id not in seen_ready_this_row:
                    ready_counts[robot_id] += 1
                    seen_ready_this_row.add(robot_id)


def remove_outdated_puck_dict():
    global puck_dict
    puck_dict = {
    rid: data for rid, data in puck_dict.items()
    if data.get("last_seen", 0) >= step_count - 50
}



def get_position():
    x,y,ang = gps.getValues()[0], gps.getValues()[1], normalize_angle_deg(math.degrees(imu.getRollPitchYaw()[2]))
    return x, y, ang
   
def publish_data(packet):
    emitter.send(json.dumps(packet).encode())
   
def rotate_to_target_stepwise(x, y, ang, tx, ty, thresh=1.0):
    dx = tx - x
    dy = ty - y
    targ_ang = normalize_angle_deg(math.degrees(math.atan2(dy, dx)))
    diff = (targ_ang - ang + 540) % 360 - 180
    print(f"[{pi_puck_id}] Rotating→ T:{targ_ang:.1f} C:{ang:.1f} Δ:{diff:.1f}")
    if abs(diff) < thresh:
        left_motor.setVelocity(0); right_motor.setVelocity(0)
        return True
    spd = max(min(0.05*diff, turn_speed), -turn_speed)
    if diff > 0:
        left_motor.setVelocity(spd)
        right_motor.setVelocity(-spd)
    else:
        left_motor.setVelocity(-spd)
        right_motor.setVelocity(spd)
    return False




def drive_forward_stepwise(tx, ty, spd=forward_speed*0.5):
    global start_position
    x,y, _ = get_position()
    if start_position is None:
        start_position = (x,y)
    d = distance(x,y,tx,ty)
    print(f"[{pi_puck_id}] Driving→ ({x:.2f},{y:.2f})→({tx:.2f},{ty:.2f}) d={d:.3f}")
    if d < 0.05:
        left_motor.setVelocity(0); right_motor.setVelocity(0)
        start_position = None
        return True
    left_motor.setVelocity(spd); right_motor.setVelocity(spd)
    return False


def neighbors_ready(puck_dict, all_ids, pi_puck_id):
    idx = all_ids.index(pi_puck_id)
    left_id = all_ids[idx - 1] if idx > 0 else None
    right_id = all_ids[idx + 1] if idx < len(all_ids) - 1 else None


    left_ready = True
    right_ready = True


    if left_id is not None and left_id in puck_dict:
        left_ready = puck_dict[left_id].get("ready", False)
    if right_id is not None and right_id in puck_dict:
        right_ready = puck_dict[right_id].get("ready", False)


    return left_ready and right_ready

def neighbors_ready_confirmed(all_ids, pi_puck_id, min_ready_count=3):
    idx = all_ids.index(pi_puck_id)
    left_id = all_ids[idx - 1] if idx > 0 else None
    right_id = all_ids[idx + 1] if idx < len(all_ids) - 1 else None

    left_ready = (left_id is None or ready_counts[left_id] >= min_ready_count)
    right_ready = (right_id is None or ready_counts[right_id] >= min_ready_count)

    return left_ready, right_ready

# Main loop
while robot.step(timestep) != -1:
   
    on_message()
    #remove_outdated_puck_dict()
    #step_count += 1
    #print(puck_dict)
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
                "target_found": False,
                "ready": ready
            }
        })
    else:
        print("Position data not available.")
       
    all_ids = sorted(list(set(puck_dict.keys()) | {pi_puck_id}), key=extract_int)
    print(all_ids)
    #time.sleep(1)
   
    #if spacing is None and len(all_ids) >= 1:
    #    spacing = min(max_range*0.9, ArenaMaxY/len(all_ids))
    #    rowY    = 0.1
    #    total_sweeps = math.ceil(ArenaMaxY / spacing)
    #    sweeps_per_rbt = math.ceil(total_sweeps / len(all_ids))

    if current_state == STATE_START:
        print("Waiting for puck_dict...")
        current_state = STATE_WAIT_FOR_puck_dict


    elif current_state == STATE_WAIT_FOR_puck_dict:
        print(f"Waiting for neighbors... {len(puck_dict)} found.")
        if len(all_ids) < 2:
            current_state = STATE_START
            continue
        else:
            if spacing is None and len(all_ids) >= 1:
                spacing = min(max_range*0.9, ArenaMaxY/len(all_ids))
                rowY    = 0.1
                total_sweeps = math.ceil(ArenaMaxY / spacing)
                sweeps_per_rbt = math.ceil(total_sweeps / len(all_ids))
                print(f"total_sweeps: {total_sweeps}, sweeps_per_rbt: {sweeps_per_rbt}, spacing: {spacing}")
               
            role      = "LEADER" if int(pi_puck_id)==min(map(int,all_ids)) else "FOLLOWER"
            idx       = all_ids.index(pi_puck_id)
            target_x  = StartX
            target_y  = rowY + idx*spacing
            print(f"I am {role} idx={idx}, lineY={rowY:.2f}, target=({target_x:.2f},{target_y:.2f})")
            current_state = STATE_START_ROTATE




    elif current_state == STATE_START_ROTATE:
        print(f"{pi_puck_id} STATE_START_ROTATE at Y={target_y:.2f}, direction={sweep_direction}")
        if rotate_to_target_stepwise(x,y,angle,target_x,target_y):
            current_state = STATE_START_DRIVE




    elif current_state == STATE_START_DRIVE:
        print(f"{pi_puck_id} STATE_START_DRIVE at Y={target_y:.2f}, direction={sweep_direction}")
        if drive_forward_stepwise(target_x,target_y):
            print(f"{pi_puck_id} formed line.")
            target_x = SweepEndX if sweep_direction == 1 else StartX        
            current_state = STATE_START_SWEEP


    elif current_state == STATE_WAIT_FOR_NEIGHBORS_READY:
        # Wait for left and right neighbors to be ready
        left_ready, right_ready = neighbors_ready_confirmed(all_ids, pi_puck_id, min_ready_count=1)
        if left_ready and right_ready:
            print(f"{pi_puck_id} neighbors ready, start sweeping!")
            current_state = STATE_SWEEP_DRIVE
            ready = False  # Reset for next row
        else:
            print(f"{pi_puck_id} waiting for neighbors... ready_counts={dict(ready_counts)}")


    elif current_state == STATE_START_SWEEP:
       print(f"{pi_puck_id} STATE_START_SWEEP at Y={target_y:.2f}, direction={sweep_direction}")
       if rotate_to_target_stepwise(x,y,angle,target_x,target_y):
           ready = True
           current_state = STATE_WAIT_FOR_NEIGHBORS_READY




    elif current_state == STATE_SWEEP_DRIVE:
        print(f"{pi_puck_id} STATE_SWEEP_DRIVE at Y={target_y:.2f}, direction={sweep_direction}")
        if drive_forward_stepwise(target_x,target_y):
            print(f"{pi_puck_id} sweep row complete.")
            #sweeps_per_rbt -= 1
            rows_swept += 1
            current_state = STATE_ADVANCE_ROW



    
    elif current_state == STATE_ADVANCE_ROW:
        print(f"{pi_puck_id} STATE_ADVANCE_ROW.")
    
        idx = all_ids.index(pi_puck_id)
        next_row_index = idx + rows_swept * len(all_ids)
        next_row_y = next_row_index * spacing
    
        max_rows_possible = math.ceil(ArenaMaxY / spacing)
        highest_id = max(all_ids, key=extract_int)
    
        # Case 1: Next row exceeds arena → Done
        if next_row_index >= max_rows_possible:
            print(f"{pi_puck_id} → next_row_index={next_row_index} exceeds arena limits.")
            current_state = STATE_DONE
            continue
    
        # Case 2: This robot is NOT highest ID and would be sweeping the last row → skip
        if next_row_index == max_rows_possible - 1 and pi_puck_id != highest_id:
            print(f"{pi_puck_id} skips final row {next_row_index} — reserved for highest ID.")
            current_state = STATE_DONE
            continue
    
        # Case 3: Valid row → proceed
        target_y = next_row_y
        current_state = STATE_ROW_ROTATE




    elif current_state == STATE_ROW_ROTATE:
        # Turn to new row position (Y changes, X remains)
        print(f"{pi_puck_id} STATE_ROW_ROTATE at Y={target_y:.2f}")
        if abs(target_y - rowY) < 0.05:
            current_state = STATE_ROW_DRIVE
        else:
            if rotate_to_target_stepwise(x, y, angle, x, target_y):
                current_state = STATE_ROW_DRIVE


    elif current_state == STATE_ROW_DRIVE:
        print(f"{pi_puck_id} STATE_ROW_DRIVE at Y={target_y:.2f}, direction={sweep_direction}")
        if drive_forward_stepwise(x, target_y):
            rowY = target_y
            sweep_direction *= -1
            target_x = SweepEndX if sweep_direction == 1 else StartX
            current_state = STATE_START_SWEEP


    elif current_state == STATE_DONE:
        left_motor.setVelocity(0); right_motor.setVelocity(0)
        print(f"{pi_puck_id} DONE sweeping.")
        break







