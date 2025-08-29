import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)

# Configuration
CONFIG = {
    'ARDUINO_PORT': '/dev/ttyUSB0',
    'ARDUINO_BAUDRATE': 9600,
    'SERIAL_TIMEOUT': 1.0,
    'COMMAND_TIMEOUT': 2.0,  # Added missing parameter
    'RELAY_PIN': 17,
    'ALARM_PIN': 18,
    'TOLERANCE': 15,
    'TRACK_STEP_SIZE': 5,
    'MAX_CHUNK_SIZE': 50,
    'MAX_RETRIES': 3,
    'SCAN_LIMIT_STEPS': 1024,
    'SCAN_STEP_SIZE': 10,
    'SCAN_STEP_DELAY': 0.1,  # Added to control scan speed
    'HOMING_SPEED': 20,      # Steps per second for homing
    'AREA_THRESHOLD': 15000,
    'CONFIDENCE_THRESHOLD': 0.45,
    'NMS_THRESHOLD': 0.2,
    'RELAY_DURATION': 2,
    'RELAY_COOLDOWN_SECONDS': 4,
    'HOME_COOLDOWN_SECONDS': 5,
    'LOST_OBJECT_TIMEOUT': 5,
    'CAMERA_WIDTH': 640,
    'CAMERA_HEIGHT': 480,
    'INPUT_SIZE': (320, 320),
    'COCO_NAMES_PATH': "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names",
    'MODEL_PATH': "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb",
    'CONFIG_PATH': "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt",
    'TARGET_OBJECTS': ['cat', 'dog', 'cell phone']
}

# Global state
state = {
    'current_pos_x': 0,
    'current_pos_y': 0,
    'object_tracked': False,
    'last_trigger_time': 0,
    'lost_since': None,
    'is_homing': False,
    'home_cooldown_active': False,
    'scan_direction': "FORWARD",
    'scan_steps': 0,
    'known_home_x': 0,  # Track known home position
    'known_home_y': 0,  # Track known home position
    'max_steps_x': 2000,  # Estimated maximum steps for X axis
    'max_steps_y': 2000,  # Estimated maximum steps for Y axis
}

# Arduino Serial Setup
try:
    arduino = serial.Serial(
        CONFIG['ARDUINO_PORT'], 
        CONFIG['ARDUINO_BAUDRATE'], 
        timeout=CONFIG['SERIAL_TIMEOUT']
    )
    time.sleep(2)
    logger.info("Arduino connected successfully")
except Exception as e:
    logger.error(f"Failed to connect to Arduino: {e}")
    arduino = None

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(CONFIG['RELAY_PIN'], GPIO.OUT)
GPIO.setup(CONFIG['ALARM_PIN'], GPIO.OUT)
GPIO.output(CONFIG['RELAY_PIN'], GPIO.HIGH)  # Initially off
GPIO.output(CONFIG['ALARM_PIN'], GPIO.LOW)

# Object Detection Setup
classNames = []
with open(CONFIG['COCO_NAMES_PATH'], "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

net = cv2.dnn_DetectionModel(CONFIG['MODEL_PATH'], CONFIG['CONFIG_PATH'])
net.setInputSize(*CONFIG['INPUT_SIZE'])
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Arduino Communication Functions
def send_command(cmd, expected_response=None):
    """Send command to Arduino and wait for response"""
    if arduino is None:
        logger.error("Arduino not connected")
        return False
        
    try:
        arduino.reset_input_buffer()
        time.sleep(0.01)
        
        arduino.write(cmd.encode())
        arduino.flush()
        logger.debug(f"Sent: {cmd.strip()}")
        
        # If no expected response, return success
        if expected_response is None:
            return True
            
        # Wait for acknowledgment
        start_time = time.time()
        while time.time() - start_time < CONFIG['COMMAND_TIMEOUT']:
            if arduino.in_waiting > 0:
                response = arduino.readline().decode().strip()
                if response:
                    logger.debug(f"Arduino response: '{response}'")
                    if response == expected_response:
                        return True
                    elif response.endswith("_ERROR"):
                        logger.error(f"Arduino reported error: {response}")
                        return False
            time.sleep(0.001)
        
        logger.error(f"Timeout waiting for {expected_response} acknowledgment")
        return False
        
    except Exception as e:
        logger.error(f"Error sending command: {e}")
        return False

def send_step_command(axis, direction, steps):
    """Send step command to Arduino and wait for confirmation"""
    cmd = f"STEP {axis} {direction} {steps}\n"
    return send_command(cmd, f"{axis}_OK")

def step_x(direction, steps):
    """Move X axis and update position tracking"""
    logger.debug(f"Moving X axis: {direction} {steps} steps")
    if send_step_command("X", direction, steps):
        if direction == "FORWARD":
            state['current_pos_x'] += steps
        else:
            state['current_pos_x'] -= steps
        return True
    return False

def step_y(direction, steps):
    """Move Y axis and update position tracking"""
    logger.debug(f"Moving Y axis: {direction} {steps} steps")
    if send_step_command("Y", direction, steps):
        if direction == "FORWARD":
            state['current_pos_y'] += steps
        else:
            state['current_pos_y'] -= steps
        return True
    return False

def step_x_with_delay(direction, steps, delay=0.01):
    """Move X axis with delay between steps"""
    success = True
    for _ in range(steps):
        if not step_x(direction, 1):
            success = False
        time.sleep(delay)
    return success

def step_y_with_delay(direction, steps, delay=0.01):
    """Move Y axis with delay between steps"""
    success = True
    for _ in range(steps):
        if not step_y(direction, 1):
            success = False
        time.sleep(delay)
    return success

# Relay and Alarm Functions
def trigger_relay(duration=None):
    """Trigger relay for specified duration (non-blocking)"""
    if duration is None:
        duration = CONFIG['RELAY_DURATION']
    
    def relay_operation():
        logger.info("Relay ON")
        GPIO.output(CONFIG['RELAY_PIN'], GPIO.LOW)
        alarm_on()
        time.sleep(duration)
        GPIO.output(CONFIG['RELAY_PIN'], GPIO.HIGH)
        alarm_off()
        logger.info("Relay OFF")
    
    threading.Thread(target=relay_operation, daemon=True).start()

def alarm_on():
    GPIO.output(CONFIG['ALARM_PIN'], GPIO.HIGH)

def alarm_off():
    GPIO.output(CONFIG['ALARM_PIN'], GPIO.LOW)

# Object Detection
def detect_objects(img, draw=True, target_objects=None):
    """Detect objects and mark centers"""
    if target_objects is None:
        target_objects = CONFIG['TARGET_OBJECTS']
        
    try:
        classIds, confs, bbox = net.detect(
            img, 
            confThreshold=CONFIG['CONFIDENCE_THRESHOLD'], 
            nmsThreshold=CONFIG['NMS_THRESHOLD']
        )
        
        object_info = []
        
        if len(classIds) != 0:
            for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                if classId - 1 < len(classNames):
                    className = classNames[classId - 1]
                    if className in target_objects:
                        center = (box[0] + box[2]//2, box[1] + box[3]//2)
                        object_info.append([box, className, round(confidence*100, 2), center])
                        
                        if draw:
                            cv2.rectangle(img, box, (0, 255, 0), 2)
                            cv2.putText(img, f"{className.upper()} {round(confidence*100, 2)}%", 
                                       (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.circle(img, center, 5, (0, 255, 0), -1)
        
        return img, object_info
        
    except Exception as e:
        logger.error(f"Object detection failed: {e}")
        return img, []

# Tracking Functions
def track_object(frame_center, object_center):
    """Track object by calculating error and moving steppers"""
    error_x = frame_center[0] - object_center[0]
    error_y = frame_center[1] - object_center[1]
    
    logger.debug(f"ErrorX: {error_x}, ErrorY: {error_y}")
    
    moved = False
    tolerance = CONFIG['TOLERANCE']
    track_step_size = CONFIG['TRACK_STEP_SIZE']
    
    # Calculate proportional step size
    def calculate_step_size(error):
        abs_error = abs(error)
        if abs_error <= tolerance:
            return 0
        
        if abs_error > 100:
            proportion = 1.0
        else:
            proportion = abs_error / 100.0
            
        return max(1, min(round(track_step_size * proportion), track_step_size))
    
    # Move X axis if error is above tolerance
    x_step_size = calculate_step_size(error_x)
    if x_step_size > 0:
        direction = "FORWARD" if error_x > 0 else "BACKWARD"
        if step_x(direction, x_step_size):
            moved = True
            time.sleep(0.01)
    
    # Move Y axis if error is above tolerance
    y_step_size = calculate_step_size(error_y)
    if y_step_size > 0:
        direction = "FORWARD" if error_y > 0 else "BACKWARD"
        if step_y(direction, y_step_size):
            moved = True
            time.sleep(0.01)
    
    # Check if aligned
    aligned = abs(error_x) <= tolerance and abs(error_y) <= tolerance
    if aligned:
        logger.debug("ALIGNED")
    
    return aligned, moved

def go_home():
    """Return both axes to logical zero position (async)"""
    if state['is_homing']:
        logger.warning("Homing already in progress...")
        return False
    
    logger.info(f"Starting homing from position: ({state['current_pos_x']}, {state['current_pos_y']})")
    state['is_homing'] = True
    
    def home_thread():
        try:
            # Home Y axis first
            if state['current_pos_y'] != 0:
                logger.info("Homing Y axis...")
                home_axis("Y", state['current_pos_y'])
            
            # Then home X axis
            if state['current_pos_x'] != 0:
                logger.info("Homing X axis...")
                home_axis("X", state['current_pos_x'])
            
            logger.info(f"Home complete. Final position: ({state['current_pos_x']}, {state['current_pos_y']})")
            
            # Start home cooldown
            state['home_cooldown_active'] = True
            logger.info(f"Starting {CONFIG['HOME_COOLDOWN_SECONDS']}-second home cooldown...")
            time.sleep(CONFIG['HOME_COOLDOWN_SECONDS'])
            state['home_cooldown_active'] = False
            logger.info("Home cooldown complete")
            
        except Exception as e:
            logger.error(f"Exception during homing: {e}")
        finally:
            state['is_homing'] = False
    
    threading.Thread(target=home_thread, daemon=True).start()
    return True

def home_axis(axis, current_pos):
    """Home a single axis using software tracking"""
    direction = "BACKWARD" if current_pos > 0 else "FORWARD"
    steps = abs(current_pos)
    logger.info(f"Homing {axis} axis: {steps} steps {direction}")
    
    # Calculate delay for homing speed
    delay = 1.0 / CONFIG['HOMING_SPEED']
    
    if axis == "X":
        success = step_x_with_delay(direction, steps, delay)
        if success:
            state['current_pos_x'] = 0
    else:
        success = step_y_with_delay(direction, steps, delay)
        if success:
            state['current_pos_y'] = 0
            
    if success:
        logger.info(f"{axis} axis homed successfully")
        return True
    else:
        logger.error(f"{axis} homing failed!")
        return False

def perform_scan_step():
    """Perform one scan step if ready"""
    if state['is_homing'] or state['home_cooldown_active']:
        return False
        
    if step_x(state['scan_direction'], CONFIG['SCAN_STEP_SIZE']):
        state['scan_steps'] += CONFIG['SCAN_STEP_SIZE']
        
        if state['scan_steps'] >= CONFIG['SCAN_LIMIT_STEPS']:
            state['scan_direction'] = "BACKWARD" if state['scan_direction'] == "FORWARD" else "FORWARD"
            state['scan_steps'] = 0
            logger.info(f"Scan direction changed to: {state['scan_direction']}")
        
        time.sleep(CONFIG['SCAN_STEP_DELAY'])  # Added delay to control scan speed
        return True
    return False

def reset_scan():
    """Reset scan parameters"""
    state['scan_steps'] = 0
    state['scan_direction'] = "FORWARD"

def display_ui_info(img):
    """Display UI information on the image"""
    cv2.putText(img, f"Pos: X={state['current_pos_x']}, Y={state['current_pos_y']}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    status = "HOMING" if state['is_homing'] else "HOME_COOLDOWN" if state['home_cooldown_active'] else "READY"
    cv2.putText(img, f"Status: {status}", (10, 55), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    cv2.putText(img, f"Tolerance: {CONFIG['TOLERANCE']}", (10, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.putText(img, f"Step Size: {CONFIG['TRACK_STEP_SIZE']}", (10, 100), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.putText(img, "Controls: Q=Quit, R=Reset, H=Home, P=Info", 
                (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

# Main Function
def main():
    cap = cv2.VideoCapture(0)
    cap.set(3, CONFIG['CAMERA_WIDTH'])
    cap.set(4, CONFIG['CAMERA_HEIGHT'])
    
    frame_center = (CONFIG['CAMERA_WIDTH'] // 2, CONFIG['CAMERA_HEIGHT'] // 2)
    logger.info(f"Camera ready: {CONFIG['CAMERA_WIDTH']}x{CONFIG['CAMERA_HEIGHT']}")
    
    try:
        while True:
            success, img = cap.read()
            if not success:
                logger.error("Failed to read from camera")
                break
            
            display_ui_info(img)
            
            # Detect objects
            result_img, object_info = detect_objects(img)
            
            # Handle object tracking
            object_detected = False
            biggest_box = None
            max_area = 0
            
            for obj in object_info:
                box, name, conf, center = obj
                area = box[2] * box[3]
                if area > CONFIG['AREA_THRESHOLD']:
                    if area > max_area:
                        max_area = area
                        biggest_box = (box, center)
            
            cv2.circle(img, frame_center, 6, (0, 0, 255), -1)
            
            if biggest_box is not None:
                box, bbox_center = biggest_box
                aligned, moved = track_object(frame_center, bbox_center)
                
                alarm_on()
                state['object_tracked'] = True
                state['lost_since'] = None
                
                # Visual feedback
                alignment_color = (0, 255, 0) if aligned else (0, 0, 255)
                cv2.circle(img, bbox_center, 8, alignment_color, 2)
                cv2.line(img, frame_center, bbox_center, alignment_color, 1)
                
                # Display alignment info
                error_x = frame_center[0] - bbox_center[0]
                error_y = frame_center[1] - bbox_center[1]
                cv2.putText(img, f"Error X: {error_x}, Y: {error_y}", 
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(img, f"Aligned: {aligned}", 
                            (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, alignment_color, 1)
                
                # Trigger relay if aligned and cooldown passed
                current_time = time.time()
                if aligned and current_time - state['last_trigger_time'] > CONFIG['RELAY_COOLDOWN_SECONDS']:
                    trigger_relay(CONFIG['RELAY_DURATION'])
                    state['last_trigger_time'] = current_time
                
                object_detected = True
            
            # Handle object lost
            if not object_detected:
                alarm_off()
                current_time = time.time()
                
                if state['object_tracked']:
                    if state['lost_since'] is None:
                        state['lost_since'] = current_time
                        logger.info("Object lost. Starting countdown before going home...")
                    else:
                        elapsed = int(current_time - state['lost_since'])
                        remaining = CONFIG['LOST_OBJECT_TIMEOUT'] - elapsed
                        
                        if remaining <= 0:
                            logger.info("Countdown complete, initiating home sequence...")
                            if go_home():
                                state['object_tracked'] = False
                                state['lost_since'] = None
                                reset_scan()
                            else:
                                state['lost_since'] = current_time
                else:
                    # Scanning mode
                    if not state['is_homing'] and not state['home_cooldown_active']:
                        perform_scan_step()
            
            cv2.imshow("PawStopper Output", result_img)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("Quitting. Going home...")
                # Go home before quitting
                if not state['is_homing']:
                    go_home()
                    # Wait a bit for homing to start
                    time.sleep(1)
                break
            elif key == ord('r'):
                logger.info("Manually resetting position to (0,0)")
                state['current_pos_x'] = 0
                state['current_pos_y'] = 0
            elif key == ord('h'):
                logger.info("Manual home command")
                go_home()
            elif key == ord('p'):
                logger.info(f"Current position: X={state['current_pos_x']}, Y={state['current_pos_y']}")
                
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        if arduino:
            arduino.close()
        logger.info("Cleanup complete")

if __name__ == "__main__":
    main()