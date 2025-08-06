# Imports
import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading
import json
import os
from datetime import datetime
import numpy as np

# ============================
# CONFIGURATION PARAMETERS
# ============================

# Serial Communication
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUDRATE = 9600
SERIAL_TIMEOUT = 1

# GPIO Pins
RELAY_PIN = 17
ALARM_PIN = 18

# Camera Settings
CAMERA_ID = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Object Detection Model Paths
MODEL_COCO_NAMES = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names"
MODEL_WEIGHTS = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"
MODEL_CONFIG = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"

# Object Detection Settings
DETECTION_THRESHOLD = 0.45
NMS_THRESHOLD = 0.2
TARGET_OBJECTS = ['cat', 'dog']  # Objects to detect
AREA_THRESHOLD = 15000  # Minimum area for valid detection

# Motor Limits and Movement
PAN_LIMIT_STEPS = 100   # ±90 degrees
TILT_LIMIT_STEPS = 25   # ±45 degrees
SCAN_STEPS_PER_MOVE = 5
SCAN_HEIGHT_LEVELS = [-15, -5, 5, 15]  # Different tilt levels for scanning

# Timing Settings
COOLDOWN_SECONDS = 10   # Relay trigger cooldown
RELAY_DURATION = 5      # How long to keep relay on
SCAN_DELAY = 0.1        # Delay between scan steps
TILT_MOVE_DELAY = 0.2   # Delay when changing tilt levels

# Position Tracking
POSITION_FILE = "position_state.json"

# Tolerance for alignment
ALIGNMENT_TOLERANCE = 25

# ============================
# HARDWARE INITIALIZATION
# ============================

# Open serial port to Arduino
arduino = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
time.sleep(2)  # Wait for Arduino to reset

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.setup(ALARM_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.HIGH)  # Initially off; HIGH = OFF, LOW = ON

# ============================
# FUNCTIONS
# ============================

def trigger_relay(duration=RELAY_DURATION):
    print("Relay ON")
    GPIO.output(RELAY_PIN, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    print("Relay OFF")

def trigger_relay_async(duration=RELAY_DURATION):
    threading.Thread(target=trigger_relay, args=(duration,), daemon=True).start()

def alarm_on():
    GPIO.output(ALARM_PIN, GPIO.HIGH)

def alarm_off():
    GPIO.output(ALARM_PIN, GPIO.LOW)

# Load object detection model
classNames = []
with open(MODEL_COCO_NAMES, "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

net = cv2.dnn_DetectionModel(MODEL_WEIGHTS, MODEL_CONFIG)
net.setInputSize(320, 320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)

# Function to detect objects and mark centers
def getObjects(img, thres=DETECTION_THRESHOLD, nms=NMS_THRESHOLD, draw=True, objects=TARGET_OBJECTS):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    objectInfo = []
    if len(objects) == 0: objects = classNames
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                center = (box[0]+box[2]//2, box[1]+box[3]//2)
                objectInfo.append([box, className, round(confidence*100,2), center])
                if draw:
                    cv2.rectangle(img, box, (0,255,0),2)
                    cv2.putText(img, f"{className.upper()} {round(confidence*100,2)}%", 
                                (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.circle(img, center, 5, (0,255,0), -1)
    return img, objectInfo

# Position tracking functions
def load_position():
    if os.path.exists(POSITION_FILE):
        with open(POSITION_FILE, 'r') as f:
            data = json.load(f)
            return data.get('pan_steps', 0), data.get('tilt_steps', 0)
    return 0, 0

def save_position(pan_steps, tilt_steps):
    data = {
        'pan_steps': pan_steps,
        'tilt_steps': tilt_steps,
        'last_updated': datetime.now().isoformat()
    }
    with open(POSITION_FILE, 'w') as f:
        json.dump(data, f, indent=2)

def send_absolute_position(pan_steps, tilt_steps):
    data = f"POS,{pan_steps},{tilt_steps}\n"
    arduino.write(data.encode())
    print(f"Position command sent: {data.strip()}")

def home_robot():
    print("Homing robot to center position...")
    pan_steps, tilt_steps = load_position()
    
    if pan_steps != 0 or tilt_steps != 0:
        send_absolute_position(0, 0)
        save_position(0, 0)
        time.sleep(2)
    
    print("Robot homed to center position")

# Global state variables
object_tracked = False
last_trigger_time = 0
scanning_mode = True
scan_pan_direction = 1  # 1 for right, -1 for left
current_height_index = 0

def perform_scan():
    global scanning_mode, scan_pan_direction, current_height_index
    
    pan_steps, tilt_steps = load_position()
    target_tilt = SCAN_HEIGHT_LEVELS[current_height_index]
    
    # Move to target height if not already there
    if tilt_steps != target_tilt:
        tilt_delta = target_tilt - tilt_steps
        new_tilt = max(-TILT_LIMIT_STEPS, min(TILT_LIMIT_STEPS, tilt_steps + tilt_delta))
        send_absolute_position(pan_steps, new_tilt)
        save_position(pan_steps, new_tilt)
        time.sleep(TILT_MOVE_DELAY)
        return
    
    # Perform horizontal scan
    pan_delta = SCAN_STEPS_PER_MOVE * scan_pan_direction
    new_pan = pan_steps + pan_delta
    
    # Check if within limits
    if -PAN_LIMIT_STEPS <= new_pan <= PAN_LIMIT_STEPS:
        send_absolute_position(new_pan, tilt_steps)
        save_position(new_pan, tilt_steps)
    else:
        # Reached limit, change direction and move to next height
        scan_pan_direction *= -1
        current_height_index = (current_height_index + 1) % len(SCAN_HEIGHT_LEVELS)
        print(f"Scan: Changed direction, height level: {current_height_index}")
    
    time.sleep(SCAN_DELAY)

def check_alignment():
    if arduino.in_waiting > 0:
        msg = arduino.readline().decode().strip()
        if msg == "ALIGNED":
            return True
        elif msg.startswith("POS:"):
            # Update position from Arduino feedback
            try:
                parts = msg.split(":")
                if len(parts) == 2:
                    pan, tilt = map(int, parts[1].split(","))
                    save_position(pan, tilt)
            except ValueError:
                print(f"Invalid position feedback: {msg}")
    return False

# ============================
# MAIN PROGRAM
# ============================

if __name__ == "__main__":
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(3, CAMERA_WIDTH)
    cap.set(4, CAMERA_HEIGHT)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Initialize robot position
    home_robot()

    try:
        while True:
            success, img = cap.read()
            if not success: break

            frame_center = (frame_width//2, frame_height//2)
            cv2.circle(img, frame_center, 6, (0,0,255), -1)  # Red center dot

            result, objectInfo = getObjects(img)

            # Find first detection above threshold
            first_detection = None
            for obj in objectInfo:
                box, name, conf, center = obj
                area = box[2]*box[3]
                print(f"Detection: {name.upper()} | Confidence: {conf}% | Area: {area}")
                
                if area > AREA_THRESHOLD:
                    first_detection = box
                    print(f"First target selected: {name.upper()} (Area: {area})")
                    break

            current_time = time.time()

            if first_detection is not None:
                scanning_mode = False
                bbox_center = (first_detection[0]+first_detection[2]//2, first_detection[1]+first_detection[3]//2)
                data = f"{frame_center[0]},{frame_center[1]},{bbox_center[0]},{bbox_center[1]}\n"
                arduino.write(data.encode())
                print(f"Target detected at {bbox_center}")
                alarm_on()
                object_tracked = True

                if check_alignment() and current_time-last_trigger_time>COOLDOWN_SECONDS:
                    trigger_relay_async(RELAY_DURATION)
                    last_trigger_time=current_time

            else:
                alarm_off()
                arduino.reset_input_buffer()
                
                # Enter scanning mode if no target detected
                if not scanning_mode:
                    scanning_mode = True
                    print("Entering scanning mode")
                
                perform_scan()

            # Display scanning status
            status_text = "SCANNING" if scanning_mode else "TRACKING"
            cv2.putText(img, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            cv2.imshow("Output", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): 
                break
            elif key == ord('h'):
                home_robot()

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        arduino.close()
