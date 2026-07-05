import rcu
import _thread
import time

# ========================================
# ROBOCON HAI PHONG 2026 - R2 LOGISTICS
# Chuẩn hóa API Robosim - Zmrobo
# ========================================

# ===== CONSTANTS =====
SERVO_ARM = 3           
SERVO_GEAR = 6          

ARM_UP = 90             
ARM_DOWN_GRAB = 0       
ARM_DOWN_PUSH = 180     
GEAR_NEUTRAL = 90       
GEAR_PUSH = 180         

SPEED_FAST = 80
SPEED_NORMAL = 60
SPEED_SLOW = 40
SPEED_TURN = 30

LOOP_TIME = 0.01        
SERVO_DELAY = 0.6

# PID Parameters
KP = 1.2
KI = 0.1
KD = 0.3
MAX_CORRECTION = 40

# ===== GLOBAL STATE =====
class RobotState:
    def __init__(self):
        self.current_station = 0  
        self.packages_delivered = 0  # Số hàng ĐÃ GIAO ĐẾN TRẠM
        self.total_packages = 4   
        self.last_rfid_id = 0
        self.mission_complete = False
        
        self.last_error = 0
        self.integral = 0
        self.last_sensor_hit = 3

robot_state = RobotState()

# ===== MOTOR CONTROL (Chuẩn hóa API Robosim) =====
def set_motors(left, right):
    """Set motor speeds - API chuẩn Robosim (chữ thường)"""
    rcu.set_motor(1, int(left))
    rcu.set_motor(2, int(right))

def stop():
    """Stop motors immediately"""
    set_motors(0, 0)

def forward(speed, duration=None):
    """Move forward"""
    set_motors(speed, speed)
    if duration:
        time.sleep(duration)
        stop()

def backward(speed, duration=None):
    """Move backward"""
    set_motors(-speed, -speed)
    if duration:
        time.sleep(duration)
        stop()

def turn_left(speed=SPEED_TURN, duration=None):
    """Turn left in place"""
    set_motors(-speed, speed)
    if duration:
        time.sleep(duration)
        stop()

def turn_right(speed=SPEED_TURN, duration=None):
    """Turn right in place"""
    set_motors(speed, -speed)
    if duration:
        time.sleep(duration)
        stop()

# ===== SERVO CONTROL (Chuẩn hóa API Robosim) =====
def set_servo(servo_id, angle, wait_time=SERVO_DELAY):
    """Set servo angle and wait - API chuẩn Robosim"""
    rcu.set_servo(servo_id, angle)
    if wait_time > 0:
        time.sleep(wait_time)

def grab_package():
    """Grab package with arm"""
    print("[SERVO] Hạ cánh tay gắp gói")
    set_servo(SERVO_ARM, ARM_DOWN_GRAB, SERVO_DELAY)
    set_servo(SERVO_ARM, ARM_UP, 0.3)
    print("[SERVO] Nâng cánh tay hoàn tất")

def release_package():
    """Release package"""
    print("[SERVO] Hạ cánh tay thả gói")
    set_servo(SERVO_ARM, ARM_DOWN_GRAB, 0.2)
    set_servo(SERVO_ARM, ARM_UP, 0.3)
    print("[SERVO] Thả gói hoàn tất")

def push_lever():
    """Push lever for Mission 4 (Thu thập thông tin)"""
    print("[SERVO] Đẩy cần gạt sang phải")
    set_servo(SERVO_ARM, ARM_DOWN_PUSH, SERVO_DELAY)
    set_servo(SERVO_ARM, ARM_UP, 0.3)

def activate_gear():
    """Activate secondary gear mechanism"""
    print("[SERVO] Kích hoạt cơ cấu bánh răng")
    set_servo(SERVO_GEAR, GEAR_PUSH, SERVO_DELAY)
    set_servo(SERVO_GEAR, GEAR_NEUTRAL, 0.3)

# ===== LED CONTROL (Chuẩn hóa API Robosim) =====
def set_led(color):
    """Set LED: 1=Red, 2=Green, 3=Yellow, 4=Blue, 8=White"""
    rcu.set_3c_led(7, color)

def blink_led(color, times=2, delay=0.2):
    """Blink LED pattern"""
    for _ in range(times):
        set_led(color)
        time.sleep(delay)
        set_led(0)
        time.sleep(delay)

# ===== LINE FOLLOWING - PID CONTROLLER =====
def read_sensors():
    """Read all 5 light sensors - API chuẩn Robosim"""
    return {
        1: rcu.get_light_sensor_data(1),  # Left
        2: rcu.get_light_sensor_data(2),  # Left-Mid
        3: rcu.get_light_sensor_data(3),  # Center
        4: rcu.get_light_sensor_data(4),  # Right-Mid
        5: rcu.get_light_sensor_data(5),  # Right
    }

def calculate_line_pid(base_speed):
    """Calculate motor speeds using PID algorithm"""
    sensors = read_sensors()
    error = 0
    
    # Determine position error
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
        error = robot_state.last_error  # Use last known error
    
    # PID calculation
    robot_state.integral += error * LOOP_TIME
    robot_state.integral = max(-1, min(1, robot_state.integral))
    
    derivative = (error - robot_state.last_error) / LOOP_TIME
    robot_state.last_error = error
    
    correction = KP * error + KI * robot_state.integral + KD * derivative
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))
    
    left_speed = max(-100, min(100, base_speed - correction))
    right_speed = max(-100, min(100, base_speed + correction))
    
    return int(left_speed), int(right_speed)

def follow_line(speed=SPEED_NORMAL, duration=None):
    """Follow line with PID control"""
    if duration:
        start_time = time.time()
        while time.time() - start_time < duration:
            left, right = calculate_line_pid(speed)
            set_motors(left, right)
            time.sleep(LOOP_TIME)
        stop()
    else:
        left, right = calculate_line_pid(speed)
        set_motors(left, right)

def follow_until_intersection(speed=SPEED_NORMAL, timeout=10.0):
    """Follow line until intersection detected (T-junction or branch)"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        sensors = read_sensors()
        active_sensors = sum([1 for i in range(1, 6) if sensors[i]])
        
        # Intersection: 3+ sensors active = T-junction detected
        if active_sensors >= 3:
            stop()
            print(f"[SENSOR] Phát hiện ngã rẽ (Active: {active_sensors})")
            return True
        
        follow_line(speed)
        time.sleep(LOOP_TIME)
    
    stop()
    print("[SENSOR] Timeout - Không phát hiện ngã rẽ")
    return False

def follow_until_center(speed=SPEED_SLOW, timeout=5.0):
    """Follow until center sensor detects line"""
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
        time.sleep(LOOP_TIME)
    
    stop()
    return False

# ===== RFID CONTROL (Chuẩn hóa API Robosim) =====
def read_rfid():
    """Read RFID value - API chuẩn Robosim"""
    try:
        rfid_id = rcu.get_rfid_value()
        if rfid_id > 0:
            print(f"[RFID] Đọc thành công ID: {rfid_id}")
            robot_state.last_rfid_id = rfid_id
            return rfid_id
    except Exception as e:
        print(f"[RFID] Lỗi kết nối cảm biến: {e}")
    return 0

def wait_rfid(timeout=3.0):
    """Wait for RFID detection within timeout"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        rfid_id = read_rfid()
        if rfid_id > 0:
            return rfid_id
        time.sleep(0.1)
    print(f"[RFID] Timeout {timeout}s - Không phát hiện thẻ")
    return 0

# ===== STATION ROUTING =====
def route_to_station(station_id):
    """
    Route to specific station (1, 2, 3, or 4)
    Distance varies by station position
    """
    print(f"[ROUTE] Đang đến Trạm {station_id}")
    set_led(1)  # Red
    
    # Khoảng cách ước tính đến các ngã rẽ của trạm (điều chỉnh theo bản đồ thực tế)
    station_distances = {
        1: 1.2,  # Trạm 1 gần nhất
        2: 1.8,
        3: 2.4,
        4: 3.0,  # Trạm 4 xa nhất
    }
    distance = station_distances.get(station_id, 1.8)
    
    # Follow line to estimated intersection
    follow_line(SPEED_NORMAL, distance)
    
    # Slow down and lock onto station marker
    if follow_until_intersection(SPEED_SLOW, 2.0):
        print(f"[ROUTE] Đã khóa mục tiêu Trạm {station_id}")
        blink_led(2, 2)  # Green success
        return True
    
    print(f"[ROUTE] CẢNH BÁO: Không xác định được Trạm {station_id}")
    return False

def return_to_main_line():
    """Return from station to main line"""
    print("[RETURN] Quay lại đường trục chính")
    backward(SPEED_SLOW, 0.5)
    # Alternative: rotate and search for line
    turn_left(SPEED_TURN, 0.3)

# ===== MISSION TASKS =====
def mission_1_start():
    """
    Mission 1: START position
    Robot grabs first package at start zone
    """
    print("\n" + "="*60)
    print("MISSION 1: BẮT ĐẦU - GẮP GÓI HÀng ĐẦU TIÊN")
    print("="*60)
    
    set_led(2)  # Green
    
    # Move forward slowly from START zone
    follow_line(SPEED_SLOW, 0.8)
    
    # Grab first package
    print("[ACTION] Gắp bưu kiện đầu tiên")
    grab_package()
    
    blink_led(2, 2)  # Success indicator
    print("[MISSION 1] Hoàn tất")

def mission_2_approach_rfid():
    """
    Mission 2: Move to RFID zone
    Navigate to the RFID sensor area
    """
    print("\n" + "="*60)
    print("MISSION 2: TIẾP CẬN VÙNG QUÉT RFID")
    print("="*60)
    
    set_led(1)  # Red
    
    # Follow main line to RFID zone
    follow_line(SPEED_NORMAL, 1.5)
    
    # Lock onto RFID intersection
    follow_until_intersection(SPEED_SLOW, 3.0)
    
    print("[MISSION 2] Hoàn tất")

def mission_3_read_rfid_and_route():
    """
    Mission 3: Read RFID and get station assignment
    This is the "System Authorization" step
    """
    print("\n" + "="*60)
    print("MISSION 3: QUÉT MÃ RFID - NHẬN CHỈ ĐỊNH TRẠM")
    print("="*60)
    
    set_led(3)  # Yellow
    
    # Position over RFID sensor
    print("[ACTION] Nhích nhẹ lên tâm quét RFID")
    forward(SPEED_SLOW, 0.2)
    
    # Read RFID
    print("[RFID] Bắt đầu quét...")
    rfid_id = wait_rfid(3.0)
    
    if rfid_id == 0:
        print("[WARNING] Không đọc được RFID! Sử dụng ID dự phòng = 1")
        rfid_id = 1
    
    robot_state.current_station = rfid_id
    print(f"[DECISION] Chỉ định: Đi đến Trạm {rfid_id}")
    
    blink_led(4, 2)  # Blue success
    print("[MISSION 3] Hoàn tất")
    
    return rfid_id

def mission_4_move_to_station():
    """
    Mission 4: Move to assigned station
    Execute routing based on RFID ID
    """
    print("\n" + "="*60)
    print(f"MISSION 4: DI CHUYỂN VÀO TRẠM {robot_state.current_station}")
    print("="*60)
    
    set_led(1)  # Red
    
    station = robot_state.current_station
    
    # Back up from RFID zone first
    print("[ACTION] Lùi từ vùng RFID")
    backward(SPEED_SLOW, 0.5)
    
    # Find main line again
    follow_until_center(SPEED_SLOW, 2.0)
    
    # Route to station
    if route_to_station(station):
        print(f"[MISSION 4] Đã tới Trạm {station} an toàn")
        return True
    else:
        print(f"[MISSION 4] CẢNH BÁO: Có thể chưa tới Trạm {station}")
        return False

def mission_5_exchange_package():
    """
    Mission 5: Exchange package at station
    Release current package, grab new one if available
    """
    print("\n" + "="*60)
    print("MISSION 5: GIAO HÀng VÀ ĐỔI GÓI HÀNG MỚI")
    print("="*60)
    
    set_led(1)  # Red
    
    # Position for drop operation
    print("[ACTION] Dịch chuyển vị trí để thực hiện giao hàng")
    forward(SPEED_SLOW, 0.2)
    
    # Release current package
    print("[ACTION] Thả bưu kiện cũ xuống tại trạm")
    release_package()
    
    # Increment delivered counter
    robot_state.packages_delivered += 1
    print(f"[COUNT] Số hàng đã giao: {robot_state.packages_delivered}/{robot_state.total_packages}")
    
    time.sleep(0.3)
    
    # If more packages to deliver, grab new one from station
    if robot_state.packages_delivered < robot_state.total_packages:
        print("[ACTION] Gắp bưu kiện mới từ trạm")
        grab_package()
    else:
        print("[ACTION] Đã giao xong tất cả bưu kiện! Chuẩn bị về đích")
    
    # Back away from station
    print("[ACTION] Lùi khỏi trạm")
    backward(SPEED_SLOW, 0.4)
    
    print("[MISSION 5] Hoàn tất")

def mission_6_return_to_main_line():
    """
    Mission 6: Return to main line for next cycle
    Navigate back from station to main line
    """
    print("\n" + "="*60)
    print("MISSION 6: QUAY LẠI ĐƯỜNG TRỤC CHÍNH")
    print("="*60)
    
    set_led(1)  # Red
    
    # Continuous backward until finding main line
    print("[ACTION] Lùi để tìm lại đường chính")
    backward(SPEED_NORMAL, 0.8)
    
    # Rotate and search for line center
    print("[ACTION] Xoay tìm kiếm vạch line")
    turn_left(SPEED_TURN, 0.5)
    
    # Lock onto center line
    follow_until_center(SPEED_SLOW, 2.0)
    
    print("[MISSION 6] Hoàn tất - Sẵn sàng cho chu kỳ tiếp theo")

def mission_8_return_to_finish():
    """
    Mission 8: Return to FINISH zone
    Navigate back to starting area and stop
    """
    print("\n" + "="*60)
    print("MISSION 8: VỀ ĐÍCH")
    print("="*60)
    
    set_led(8)  # White - Maximum brightness
    
    # High speed return to finish
    print("[ACTION] Tăng tốc về đích")
    follow_line(SPEED_FAST, 2.5)
    
    stop()
    print("[SUCCESS] ROBOT ĐÃ VỀ ĐÍCH AN TOÀN!")
    print(f"[SUMMARY] Tổng bưu kiện giao: {robot_state.packages_delivered}")

# ===== MAIN EXECUTION =====
def main():
    """Main execution loop - 8 Mission sequence"""
    try:
        print("\n" + "#"*60)
        print("#" + " "*58 + "#")
        print("# ROBOCON HẢI PHÒNG 2026 - BÀN R2: LOGISTICS")
        print("# Chuẩn hóa API Robosim - Zmrobo")
        print("#" + " "*58 + "#")
        print("#"*60)
        
        # Initialize
        set_led(2)
        time.sleep(1.0)
        
        # Mission 1: Grab first package
        mission_1_start()
        time.sleep(0.5)
        
        # Loop: Process packages until all 4 are delivered
        while robot_state.packages_delivered < robot_state.total_packages:
            # Mission 2: Go to RFID zone
            mission_2_approach_rfid()
            time.sleep(0.3)
            
            # Mission 3: Read RFID and get station assignment
            station = mission_3_read_rfid_and_route()
            time.sleep(0.3)
            
            # Mission 4: Navigate to assigned station
            mission_4_move_to_station()
            time.sleep(0.3)
            
            # Mission 5: Exchange package at station
            mission_5_exchange_package()
            time.sleep(0.3)
            
            # Mission 6: Return to main line (if more packages to deliver)
            if robot_state.packages_delivered < robot_state.total_packages:
                mission_6_return_to_main_line()
                time.sleep(0.3)
        
        # Mission 8: Return to FINISH
        print("\n[SEQUENCE] Tất cả bưu kiện đã được giao! Bắt đầu về đích...")
        time.sleep(1.0)
        mission_8_return_to_finish()
        
        # Victory sequence
        print("\n" + "#"*60)
        print("# MISSION COMPLETED SUCCESSFULLY!")
        print(f"# Total packages delivered: {robot_state.packages_delivered}")
        print("#"*60 + "\n")
        
        for _ in range(5):
            set_led(2)
            time.sleep(0.2)
            set_led(4)
            time.sleep(0.2)
        
        robot_state.mission_complete = True
        
    except Exception as e:
        print(f"\n[CRITICAL ERROR] Khẩn cấp - Lỗi hệ thống: {e}")
        import traceback
        traceback.print_exc()
        stop()
        set_led(1)  # Red error

def idle():
    """Idle thread - keeps program alive"""
    while True:
        time.sleep(0.1)

# ===== ENTRY POINT =====
if __name__ == "__main__":
    _thread.start_new_thread(main, ())
    _thread.start_new_thread(idle, ())
    
    # Keep main thread alive
    while True:
        time.sleep(0.1)
