import rcu
import _thread
import time

# ========================================
# ROBOCON HAI PHONG 2026 - R2 LOGISTICS
# Robot RFID + Servo Control
# ========================================
# Tasks:
# 1. START -> Grab first package
# 2. Move to RFID Zone (Mission 3) -> Read ID
# 3. Route to Station (1/2/3/4) based on ID
# 4. Place package at station
# 5. Grab new package from station
# 6. Return to RFID Zone -> Read new ID
# 7. Route to next station -> Repeat
# 8. Return to FINISH
# ========================================

# ===== CONSTANTS =====
# Servo Configuration
SERVO_ARM = 3           # Main arm (grab/drop)
SERVO_GEAR = 6          # Secondary mechanism (push/rotate)

# Servo Angles
ARM_UP = 90             # Lifted position
ARM_DOWN_GRAB = 0       # Grab package position
ARM_DOWN_PUSH = 180     # Push lever position
GEAR_NEUTRAL = 90       # Neutral gear position
GEAR_PUSH = 180         # Push gear position

# Motor Speeds
SPEED_FAST = 80
SPEED_NORMAL = 60
SPEED_SLOW = 40
SPEED_TURN = 30
SPEED_STOP = 0

# Timing
LOOP_TIME = 0.01        # 100Hz
SERVO_DELAY = 0.6
SENSOR_DEBOUNCE = 0.005

# PID Line Following
KP = 1.2
KI = 0.1
KD = 0.3
MAX_CORRECTION = 40

# ===== GLOBAL STATE =====
class RobotState:
    def __init__(self):
        self.current_station = 0  # 1, 2, 3, 4
        self.packages_collected = 0
        self.total_packages = 4   # 4 packages to collect
        self.last_rfid_id = 0
        self.mission_complete = False
        
        # Line following state
        self.last_error = 0
        self.integral = 0
        self.last_sensor_hit = 0

robot_state = RobotState()

# ===== MOTOR CONTROL =====
def set_motors(left, right):
    """Set motor speeds"""
    rcu.SetMotor(1, int(left))
    rcu.SetMotor(2, int(right))

def stop():
    """Stop motors"""
    set_motors(0, 0)

def forward(speed, duration=None):
    """Move forward"""
    set_motors(speed, speed)
    if duration:
        rcu.SetWaitForTime(duration)
        stop()

def backward(speed, duration=None):
    """Move backward"""
    set_motors(-speed, -speed)
    if duration:
        rcu.SetWaitForTime(duration)
        stop()

def turn_left(speed=SPEED_TURN, duration=None):
    """Turn left"""
    set_motors(-speed, speed)
    if duration:
        rcu.SetWaitForTime(duration)
        stop()

def turn_right(speed=SPEED_TURN, duration=None):
    """Turn right"""
    set_motors(speed, -speed)
    if duration:
        rcu.SetWaitForTime(duration)
        stop()

# ===== SERVO CONTROL =====
def set_servo(servo_id, angle, wait_time=SERVO_DELAY):
    """Set servo angle and wait"""
    rcu.SetServo(servo_id, angle)
    if wait_time > 0:
        rcu.SetWaitForTime(wait_time)

def grab_package():
    """Grab package with arm"""
    set_servo(SERVO_ARM, ARM_DOWN_GRAB, SERVO_DELAY)
    set_servo(SERVO_ARM, ARM_UP, 0.3)

def release_package():
    """Release package"""
    set_servo(SERVO_ARM, ARM_DOWN_GRAB, 0.2)
    set_servo(SERVO_ARM, ARM_UP, 0.3)

def push_lever():
    """Push lever for Mission 4"""
    set_servo(SERVO_ARM, ARM_DOWN_PUSH, SERVO_DELAY)
    set_servo(SERVO_ARM, ARM_UP, 0.3)

def activate_gear():
    """Activate secondary gear"""
    set_servo(SERVO_GEAR, GEAR_PUSH, SERVO_DELAY)
    set_servo(SERVO_GEAR, GEAR_NEUTRAL, 0.3)

# ===== LED CONTROL =====
def set_led(color):
    """Set LED: 1=Red, 2=Green, 3=Yellow, 4=Blue, 8=White"""
    rcu.Set3CLed(7, color)

def blink_led(color, times=2, delay=0.3):
    """Blink LED"""
    for _ in range(times):
        set_led(color)
        rcu.SetWaitForTime(delay)
        set_led(0)
        rcu.SetWaitForTime(delay)

# ===== LINE FOLLOWING - PID CONTROLLER =====
def read_sensors():
    """Read all light sensors"""
    return {
        1: rcu.GetLightSensorData(1),  # Left
        2: rcu.GetLightSensorData(2),  # Left-Mid
        3: rcu.GetLightSensorData(3),  # Center
        4: rcu.GetLightSensorData(4),  # Right-Mid
        5: rcu.GetLightSensorData(5),  # Right
    }

def calculate_line_pid(base_speed):
    """PID controller for line following"""
    sensors = read_sensors()
    
    # Calculate error
    error = 0
    if sensors[1]:
        error = -2
        robot_state.last_sensor_hit = 1
    elif sensors[2]:
        error = -1
        robot_state.last_sensor_hit = 2
    elif sensors[3]:
        error = 0
        robot_state.last_sensor_hit = 3
    elif sensors[4]:
        error = 1
        robot_state.last_sensor_hit = 4
    elif sensors[5]:
        error = 2
        robot_state.last_sensor_hit = 5
    else:
        error = robot_state.last_error
    
    # PID calculation
    robot_state.integral += error * LOOP_TIME
    robot_state.integral = max(-1, min(1, robot_state.integral))
    
    derivative = (error - robot_state.last_error) / LOOP_TIME if LOOP_TIME > 0 else 0
    robot_state.last_error = error
    
    correction = KP * error + KI * robot_state.integral + KD * derivative
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))
    
    left_speed = base_speed - correction
    right_speed = base_speed + correction
    
    left_speed = max(-100, min(100, left_speed))
    right_speed = max(-100, min(100, right_speed))
    
    return int(left_speed), int(right_speed)

def follow_line(speed=SPEED_NORMAL, duration=None):
    """Follow line with PID"""
    if duration:
        start_time = time.time()
        while time.time() - start_time < duration:
            left, right = calculate_line_pid(speed)
            set_motors(left, right)
            rcu.SetWaitForTime(LOOP_TIME)
        stop()
    else:
        left, right = calculate_line_pid(speed)
        set_motors(left, right)

def follow_until_center(speed=SPEED_SLOW, timeout=5.0):
    """Follow line until center sensor detects"""
    start_time = time.time()
    center_count = 0
    
    while time.time() - start_time < timeout:
        sensors = read_sensors()
        
        if sensors[3]:
            center_count += 1
            if center_count > 5:
                stop()
                return True
        else:
            center_count = 0
        
        follow_line(speed)
        rcu.SetWaitForTime(LOOP_TIME)
    
    stop()
    return False

def follow_until_intersection(speed=SPEED_NORMAL, timeout=10.0):
    """Follow line until intersection (multiple sensors active)"""
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        sensors = read_sensors()
        active_sensors = sum([sensors[i] for i in range(1, 6)])
        
        # Intersection: 3+ sensors active or T-junction pattern
        if active_sensors >= 3:
            stop()
            return True
        
        follow_line(speed)
        rcu.SetWaitForTime(LOOP_TIME)
    
    stop()
    return False

# ===== RFID CONTROL =====
def read_rfid():
    """Read RFID value"""
    try:
        rfid_id = rcu.get_rfid_value()
        print(f"[RFID] Read ID: {rfid_id}")
        robot_state.last_rfid_id = rfid_id
        return rfid_id
    except:
        print("[RFID] Failed to read")
        return 0

def wait_rfid(timeout=5.0):
    """Wait for RFID to be detected"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        rfid_id = read_rfid()
        if rfid_id > 0:
            return rfid_id
        rcu.SetWaitForTime(0.2)
    return 0

# ===== STATION ROUTING =====
def route_to_station(station_id):
    """
    Route to specific station (1, 2, 3, or 4)
    Stations are arranged in a line or pattern
    """
    print(f"[ROUTE] Moving to Station {station_id}")
    set_led(1)  # Red
    
    # Approach distance varies by station
    station_distances = {
        1: 1.5,  # Closest
        2: 2.0,
        3: 2.5,
        4: 3.0,  # Farthest
    }
    
    distance = station_distances.get(station_id, 2.0)
    follow_line(SPEED_NORMAL, distance)
    
    # Attempt to find intersection/marker for station
    if follow_until_intersection(SPEED_SLOW, 3.0):
        print(f"[ROUTE] Reached Station {station_id}")
        return True
    
    return False

def return_to_main_line():
    """Return from station to main line"""
    print("[RETURN] Going back to main line")
    backward(SPEED_SLOW, 0.5)
    follow_until_center(SPEED_SLOW, 3.0)

# ===== MISSION TASKS =====
def mission_1_start():
    """
    Mission 1: Start position
    Robot at START, grab first package
    """
    print("\n=== MISSION 1: START ===")
    set_led(2)  # Green
    
    # Move forward to grab position
    follow_line(SPEED_SLOW, 1.0)
    
    # Grab first package
    print("[GRAB] Package 1")
    grab_package()
    robot_state.packages_collected += 1
    
    blink_led(2, 2)  # Success blink

def mission_2_approach_rfid():
    """
    Mission 2: Move to RFID zone
    """
    print("\n=== MISSION 2: APPROACH RFID ===")
    set_led(1)  # Red
    
    # Follow line to RFID zone
    follow_line(SPEED_NORMAL, 2.0)
    follow_until_center(SPEED_SLOW, 3.0)
    
    print("[POSITION] At RFID zone")

def mission_3_read_rfid_and_route():
    """
    Mission 3: Read RFID and determine route
    This is the "System Authorization" step
    """
    print("\n=== MISSION 3: READ RFID ===")
    set_led(3)  # Yellow
    
    # Position above RFID sensor
    forward(SPEED_SLOW, 0.3)
    
    # Read RFID
    rfid_id = wait_rfid(3.0)
    
    if rfid_id == 0:
        print("[ERROR] RFID read failed, defaulting to Station 1")
        rfid_id = 1
    
    robot_state.current_station = rfid_id
    blink_led(4, 2)  # Blue blink for successful read
    
    return rfid_id

def mission_4_move_to_station():
    """
    Mission 4: Move to assigned station
    """
    print("\n=== MISSION 4: MOVE TO STATION ===")
    set_led(1)  # Red
    
    station = robot_state.current_station
    
    # Back up from RFID zone
    backward(SPEED_SLOW, 0.5)
    follow_until_center(SPEED_SLOW, 2.0)
    
    # Route to station
    if route_to_station(station):
        print(f"[SUCCESS] At Station {station}")
        blink_led(2, 2)
        return True
    else:
        print(f"[WARNING] Could not confirm Station {station}")
        return False

def mission_5_exchange_package():
    """
    Mission 5: Exchange package at station
    Release current package, grab new one
    """
    print("\n=== MISSION 5: EXCHANGE PACKAGE ===")
    set_led(1)  # Red
    
    # Position for drop
    forward(SPEED_SLOW, 0.3)
    
    # Release package
    print("[DROP] Package at station")
    release_package()
    rcu.SetWaitForTime(0.5)
    
    # Grab new package from station
    print("[GRAB] New package from station")
    grab_package()
    robot_state.packages_collected += 1
    
    blink_led(2, 2)
    
    # Back up
    backward(SPEED_SLOW, 0.5)

def mission_6_return_to_rfid():
    """
    Mission 6: Return to RFID zone for next ID
    """
    print("\n=== MISSION 6: RETURN TO RFID ===")
    set_led(1)  # Red
    
    return_to_main_line()
    follow_line(SPEED_NORMAL, 2.0)
    follow_until_center(SPEED_SLOW, 2.0)

def mission_7_loop():
    """
    Mission 7: Repeat cycle if packages remain
    """
    print("\n=== MISSION 7: CHECK LOOP ===")
    
    if robot_state.packages_collected < robot_state.total_packages:
        print(f"[CONTINUE] Packages: {robot_state.packages_collected}/{robot_state.total_packages}")
        set_led(3)  # Yellow
        return True
    else:
        print("[COMPLETE] All packages delivered")
        set_led(2)  # Green
        return False

def mission_8_return_to_finish():
    """
    Mission 8: Return to FINISH
    """
    print("\n=== MISSION 8: RETURN TO FINISH ===")
    set_led(1)  # Red
    
    # Back up from RFID
    backward(SPEED_SLOW, 0.5)
    follow_until_center(SPEED_SLOW, 2.0)
    
    # Follow line to FINISH
    follow_line(SPEED_FAST, 4.0)
    
    # Victory!
    stop()
    print("[SUCCESS] MISSION COMPLETE!")
    
    # Victory sequence
    for _ in range(5):
        set_led(2)
        rcu.SetWaitForTime(0.2)
        set_led(4)
        rcu.SetWaitForTime(0.2)
    
    robot_state.mission_complete = True

# ===== MAIN EXECUTION =====
def main():
    """Main execution loop"""
    print("\n" + "="*50)
    print("ROBOCON HAI PHONG 2026 - R2 LOGISTICS")
    print("="*50)
    
    try:
        # Initialize
        set_led(2)
        rcu.SetWaitForTime(1.0)
        
        # Mission 1: Start and grab first package
        mission_1_start()
        
        # Loop through packages
        while robot_state.packages_collected < robot_state.total_packages:
            # Mission 2: Approach RFID zone
            mission_2_approach_rfid()
            rcu.SetWaitForTime(0.5)
            
            # Mission 3: Read RFID and get station assignment
            station = mission_3_read_rfid_and_route()
            rcu.SetWaitForTime(0.5)
            
            # Mission 4: Move to assigned station
            mission_4_move_to_station()
            rcu.SetWaitForTime(0.5)
            
            # Mission 5: Exchange package
            mission_5_exchange_package()
            rcu.SetWaitForTime(0.5)
            
            # Mission 6: Return to RFID
            if robot_state.packages_collected < robot_state.total_packages:
                mission_6_return_to_rfid()
                rcu.SetWaitForTime(0.5)
        
        # Mission 8: Return to FINISH
        mission_8_return_to_finish()
        
        print("\n" + "="*50)
        print("MISSION COMPLETED SUCCESSFULLY!")
        print(f"Total packages: {robot_state.packages_collected}")
        print("="*50)
        
    except Exception as e:
        print(f"[ERROR] Exception occurred: {e}")
        stop()
        set_led(1)  # Red error

def idle():
    """Idle thread"""
    while True:
        rcu.SetWaitForTime(0.1)

# ===== START =====
if __name__ == "__main__":
    _thread.start_new_thread(main, ())
    _thread.start_new_thread(idle, ())
    
    while True:
        rcu.SetWaitForTime(0.1)
