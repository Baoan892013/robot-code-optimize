import rcu
import _thread
import time

# ========================================
# ROBOT LINE-FOLLOWING - OPTIMIZED FOR MAP
# ========================================
# Map: START -> 1(Check-in) -> 2(Info) -> 3(Stack) -> 4(Transfer) -> 5(Sort) -> FINISH
# Strategy: PID Line-Following + Smart Turn Detection
# ========================================

# ===== CONSTANTS =====
# Motor Speeds
SPEED_FAST = 80         # Maximum speed for straight sections
SPEED_NORMAL = 60       # Normal speed for line following
SPEED_SLOW = 40         # Slow speed for tasks
SPEED_TURN = 30         # Speed for turning
SPEED_STOP = 0

# Timing
LOOP_TIME = 0.01        # 100Hz loop (10ms)
SERVO_DELAY = 0.6       # Servo response time
SENSOR_DEBOUNCE = 0.005 # 5ms debounce

# PID Parameters for Line Following
KP = 1.2                # Proportional gain
KI = 0.1                # Integral gain  
KD = 0.3                # Derivative gain
MAX_CORRECTION = 40     # Max motor speed correction

# ===== LINE FOLLOWING STATE MACHINE =====
class RobotLineFollower:
    def __init__(self):
        self.last_error = 0
        self.integral = 0
        self.mode = "FOLLOWING"  # FOLLOWING, LOST, TURN_LEFT, TURN_RIGHT
        self.last_sensor_hit = 0
        self.turn_count = 0
    
    def read_sensors(self):
        """Read all 5 light sensors"""
        return {
            1: rcu.GetLightSensorData(1),  # Left
            2: rcu.GetLightSensorData(2),  # Left-Mid
            3: rcu.GetLightSensorData(3),  # Center
            4: rcu.GetLightSensorData(4),  # Right-Mid
            5: rcu.GetLightSensorData(5),  # Right
        }
    
    def calculate_pid(self, base_speed):
        """Calculate motor speeds using PID controller"""
        sensors = self.read_sensors()
        
        # Calculate error: -2 (left) to +2 (right)
        error = 0
        if sensors[1]:
            error = -2
            self.last_sensor_hit = 1
        elif sensors[2]:
            error = -1
            self.last_sensor_hit = 2
        elif sensors[3]:
            error = 0
            self.last_sensor_hit = 3
        elif sensors[4]:
            error = 1
            self.last_sensor_hit = 4
        elif sensors[5]:
            error = 2
            self.last_sensor_hit = 5
        else:
            error = self.last_error  # Use last known error
        
        # PID calculation
        self.integral += error * LOOP_TIME
        self.integral = max(-1, min(1, self.integral))  # Clamp integral
        
        derivative = (error - self.last_error) / LOOP_TIME if LOOP_TIME > 0 else 0
        self.last_error = error
        
        correction = KP * error + KI * self.integral + KD * derivative
        correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))
        
        # Calculate motor speeds
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
        # Clamp speeds
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        return int(left_speed), int(right_speed)
    
    def follow_line(self, speed):
        """Main line following function"""
        left, right = self.calculate_pid(speed)
        self.set_motors(left, right)
        return left, right
    
    def set_motors(self, left, right):
        """Set motor speeds"""
        rcu.SetMotor(1, left)
        rcu.SetMotor(2, right)
    
    def stop(self):
        """Stop motors"""
        self.set_motors(0, 0)
    
    def turn_left(self, speed=SPEED_TURN):
        """Turn left"""
        self.set_motors(-speed, speed)
    
    def turn_right(self, speed=SPEED_TURN):
        """Turn right"""
        self.set_motors(speed, -speed)
    
    def forward(self, speed, distance_time=1.0):
        """Move forward"""
        self.set_motors(speed, speed)
        rcu.SetWaitForTime(distance_time)
        self.stop()
    
    def backward(self, speed, distance_time=1.0):
        """Move backward"""
        self.set_motors(-speed, -speed)
        rcu.SetWaitForTime(distance_time)
        self.stop()

# Global robot instance
robot = RobotLineFollower()

# Global variables
CURRENT_TASK = 0
ROBOT_STATE = "START"
TASK_COMPLETED = [False] * 6  # Track completion: 1-5 + FINISH

# ===== SERVO CONTROL =====
def set_servo(angle, wait_time=SERVO_DELAY):
    """Set servo angle and wait"""
    rcu.SetServo(3, angle)
    rcu.SetWaitForTime(wait_time)
    rcu.SetServo(3, 0)
    rcu.SetWaitForTime(0.1)

def grab_item(angle=85):
    """Grab item with servo"""
    set_servo(angle, SERVO_DELAY)

def release_item():
    """Release item"""
    set_servo(0, SERVO_DELAY * 0.5)

# ===== LED CONTROL =====
def set_led(color):
    """Set LED color: 1=Red, 2=Green, 3=Yellow, 4=Blue, 8=White"""
    rcu.Set3CLed(7, color)

def blink_led(color, times=3, delay=0.3):
    """Blink LED"""
    for _ in range(times):
        set_led(color)
        rcu.SetWaitForTime(delay)
        set_led(0)
        rcu.SetWaitForTime(delay)

# ===== TURN & DETECT FUNCTIONS =====
def wait_for_line_sensor(sensor_id, timeout=3.0):
    """Wait for specific sensor to detect line"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        if rcu.GetLightSensorData(sensor_id):
            return True
        rcu.SetWaitForTime(LOOP_TIME)
    return False

def turn_until_center():
    """Turn until center sensor detects line"""
    timeout = time.time() + 3.0
    while time.time() < timeout:
        if rcu.GetLightSensorData(3):
            robot.stop()
            return True
        robot.follow_line(SPEED_SLOW)
        rcu.SetWaitForTime(LOOP_TIME)
    return False

def turn_left_to_line():
    """Turn left until find line"""
    timeout = time.time() + 2.0
    while time.time() < timeout:
        robot.turn_left(SPEED_TURN)
        if rcu.GetLightSensorData(2) or rcu.GetLightSensorData(3):
            rcu.SetWaitForTime(0.1)
            turn_until_center()
            return True
        rcu.SetWaitForTime(LOOP_TIME)
    robot.stop()
    return False

def turn_right_to_line():
    """Turn right until find line"""
    timeout = time.time() + 2.0
    while time.time() < timeout:
        robot.turn_right(SPEED_TURN)
        if rcu.GetLightSensorData(4) or rcu.GetLightSensorData(3):
            rcu.SetWaitForTime(0.1)
            turn_until_center()
            return True
        rcu.SetWaitForTime(LOOP_TIME)
    robot.stop()
    return False

# ===== FOLLOW LINE UNTIL EVENT =====
def follow_until_center_detected(speed=SPEED_NORMAL, timeout=10.0):
    """Follow line until center sensor stable"""
    start_time = time.time()
    center_count = 0
    
    while time.time() - start_time < timeout:
        sensors = robot.read_sensors()
        
        if sensors[3]:
            center_count += 1
            if center_count > 5:  # Stable center detection
                robot.stop()
                return True
        else:
            center_count = 0
        
        robot.follow_line(speed)
        rcu.SetWaitForTime(LOOP_TIME)
    
    robot.stop()
    return False

def follow_line_duration(speed=SPEED_NORMAL, duration=5.0):
    """Follow line for specific duration"""
    start_time = time.time()
    while time.time() - start_time < duration:
        robot.follow_line(speed)
        rcu.SetWaitForTime(LOOP_TIME)
    robot.stop()

# ===== TASK FUNCTIONS =====
def task_1_check_in():
    """Task 1: Check-in at zone 1"""
    global CURRENT_TASK, ROBOT_STATE
    
    set_led(1)  # Red
    
    # Approach zone 1
    follow_line_duration(SPEED_NORMAL, 3.0)
    
    # Stop and perform check-in
    robot.stop()
    blink_led(2, 2)  # Green blink
    
    # Grab item
    grab_item(80)
    
    # Move away
    robot.forward(SPEED_SLOW, 1.0)
    
    TASK_COMPLETED[1] = True
    CURRENT_TASK = 2
    set_led(3)  # Yellow

def task_2_get_info():
    """Task 2: Get info at zone 2"""
    global CURRENT_TASK, ROBOT_STATE
    
    set_led(1)  # Red
    
    # Follow line to zone 2
    follow_line_duration(SPEED_NORMAL, 2.0)
    
    # Stop at zone 2
    robot.stop()
    blink_led(2, 2)  # Green blink
    
    # Wait for info (simulate AI reading)
    rcu.SetWaitForTime(2.0)
    
    # Move away
    robot.forward(SPEED_SLOW, 1.0)
    
    TASK_COMPLETED[2] = True
    CURRENT_TASK = 3
    set_led(3)  # Yellow

def task_3_stack():
    """Task 3: Stack items at zone 3"""
    global CURRENT_TASK, ROBOT_STATE
    
    set_led(1)  # Red
    
    # Follow line to zone 3
    follow_line_duration(SPEED_NORMAL, 2.0)
    
    # Stop at zone 3
    robot.stop()
    blink_led(2, 2)  # Green blink
    
    # Stack operation
    grab_item(75)
    rcu.SetWaitForTime(1.0)
    release_item()
    
    # Move away
    robot.forward(SPEED_SLOW, 1.0)
    
    TASK_COMPLETED[3] = True
    CURRENT_TASK = 4
    set_led(3)  # Yellow

def task_4_transfer():
    """Task 4: Transfer at zone 4"""
    global CURRENT_TASK, ROBOT_STATE
    
    set_led(1)  # Red
    
    # Follow line to zone 4
    follow_line_duration(SPEED_NORMAL, 3.0)
    
    # Stop at zone 4
    robot.stop()
    blink_led(2, 2)  # Green blink
    
    # Transfer operation
    grab_item(70)
    rcu.SetWaitForTime(1.0)
    release_item()
    
    # Move away
    robot.forward(SPEED_SLOW, 1.0)
    
    TASK_COMPLETED[4] = True
    CURRENT_TASK = 5
    set_led(3)  # Yellow

def task_5_sort():
    """Task 5: Sort items at zone 5"""
    global CURRENT_TASK, ROBOT_STATE
    
    set_led(1)  # Red
    
    # Follow line to zone 5
    follow_line_duration(SPEED_NORMAL, 2.0)
    
    # Stop at zone 5
    robot.stop()
    blink_led(2, 2)  # Green blink
    
    # Sort operation (multiple items)
    for i in range(2):
        grab_item(80)
        rcu.SetWaitForTime(0.5)
        release_item()
        rcu.SetWaitForTime(0.5)
    
    # Move away
    robot.forward(SPEED_SLOW, 1.0)
    
    TASK_COMPLETED[5] = True
    CURRENT_TASK = 6  # Finish
    set_led(3)  # Yellow

def task_finish():
    """Task Finish: Return to finish zone"""
    global ROBOT_STATE
    
    set_led(1)  # Red
    
    # Follow line to finish
    follow_line_duration(SPEED_FAST, 4.0)
    
    # Stop at finish
    robot.stop()
    blink_led(4, 5)  # Blue blink - Success!
    
    # Victory sequence
    for _ in range(3):
        set_led(2)
        rcu.SetWaitForTime(0.3)
        set_led(4)
        rcu.SetWaitForTime(0.3)
    
    ROBOT_STATE = "FINISHED"
    TASK_COMPLETED[6] = True

# ===== MAIN LOOP =====
def main_loop():
    """Main execution loop"""
    global CURRENT_TASK, ROBOT_STATE
    
    # Initial state
    set_led(2)  # Green
    robot.stop()
    rcu.SetWaitForTime(1.0)
    
    # Wait for center line detection
    set_led(1)
    follow_until_center_detected(SPEED_SLOW, 5.0)
    
    # Execute tasks in sequence
    while CURRENT_TASK <= 5:
        if CURRENT_TASK == 1:
            task_1_check_in()
        elif CURRENT_TASK == 2:
            task_2_get_info()
        elif CURRENT_TASK == 3:
            task_3_stack()
        elif CURRENT_TASK == 4:
            task_4_transfer()
        elif CURRENT_TASK == 5:
            task_5_sort()
        
        rcu.SetWaitForTime(0.5)
    
    # Finish task
    task_finish()
    
    # Stay at finish
    while True:
        rcu.SetWaitForTime(0.5)

def idle_loop():
    """Idle thread"""
    while True:
        rcu.SetWaitForTime(0.1)

# ===== START THREADS =====
_thread.start_new_thread(main_loop, ())
_thread.start_new_thread(idle_loop, ())

# Keep program running
while True:
    rcu.SetWaitForTime(0.1)
