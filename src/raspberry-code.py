import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading
import logging
import numpy as np
from enum import Enum, auto

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class Config:
    # Serial communication
    arduino_port = '/dev/ttyUSB0'
    arduino_baudrate = 9600
    serail_timeout = 1.0
    cmd_timeout = 2.0
    
    # GPIO
    relay_pin = 17
    alarm_pin = 18
    
    # scanning
    scan_steplimit = 1024
    scan_stepsize = 10
    scan_delay = 0.1
    
    # homing
    homing_speed = 50 # automatic
    homing_speed_fast = 50 # manual
    home_cooldown = 5
    
    # object detection
    area_thres = 15000
    confidence_thres = 0.45
    nms_thres = 0.2
    obj_lost_timeout = 5
    
    # tracking
    tolerance = 15
    track_stepsize = 5
    max_chunksize = 50
    tries = 3
    
    # relay
    relay_duration = 2
    relay_cooldown = 4
    
    # camera
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    INPUT_SIZE = (320, 320)
    
    # file paths
    COCO_NAMES_PATH = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names"
    MODEL_PATH = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"
    CONFIG_PATH = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    
    TARGET_OBJECTS = ['cat', 'dog', 'cell phone']

class SystemState(Enum):
    IDLE = auto()
    TRACKING = auto()
    HOMING = auto()
    HOME_COOLDOWN = auto()
    SCANNING = auto()
    QUITTING = auto()

class PawStopperController:
    def __init__(self, config):
        self.config = config
        self.state = SystemState.IDLE
        self.current_pos_x = 0
        self.current_pos_y = 0
        self.last_trigger_time = 0
        self.lost_since = None
        self.scan_direction = "FORWARD"
        self.scan_steps = 0
        self.manual_homing = False
        self.object_tracked = False
        
        self.setup_serial()
        self.setup_gpio()
        self.setup_object_detection()
        
    def setup_serial(self):
        
        try:
            self.arduino = serial.Serial(
                self.config.arduino_port, 
                self.config.arduino_baudrate, 
                timeout=self.config.serail_timeout
            )
            time.sleep(2)
            logger.info("Arduino connected successfully")
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.arduino = None
    
    def setup_gpio(self):
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.config.relay_pin, GPIO.OUT)
        GPIO.setup(self.config.alarm_pin, GPIO.OUT)
        GPIO.output(self.config.relay_pin, GPIO.HIGH)  # Initially off
        GPIO.output(self.config.alarm_pin, GPIO.LOW)
    
    def setup_object_detection(self):
        
        self.classNames = []
        with open(self.config.COCO_NAMES_PATH, "rt") as f:
            self.classNames = f.read().rstrip("\n").split("\n")

        self.net = cv2.dnn_DetectionModel(self.config.MODEL_PATH, self.config.CONFIG_PATH)
        self.net.setInputSize(*self.config.INPUT_SIZE)
        self.net.setInputScale(1.0 / 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)
    
    def check_serial_connection(self):
        
        if self.arduino is None or not self.arduino.is_open:
            try:
                self.arduino = serial.Serial(
                    self.config.arduino_port, 
                    self.config.arduino_baudrate, 
                    timeout=self.config.serail_timeout
                )
                time.sleep(2)
                logger.info("Reconnected to Arduino successfully")
                return True
            except Exception as e:
                logger.error(f"Failed to reconnect to Arduino: {e}")
                return False
        return True
    
    def send_command(self, cmd, expected_response=None):
        
        if not self.check_serial_connection():
            return False
            
        try:
            
            self.arduino.reset_input_buffer()
            time.sleep(0.02)
            
            
            self.arduino.write(cmd.encode())
            self.arduino.flush()
            logger.debug(f"Sent: {cmd.strip()}")
            
            if expected_response is None:
                return True
                
            start_time = time.time()
            response = ""
            
            while time.time() - start_time < self.config.cmd_timeout:
                if self.arduino.in_waiting > 0:
                    byte = self.arduino.read(1)
                    if byte:
                        response += byte.decode('utf-8', errors='ignore')
                        if '\n' in response:
                            response = response.strip()
                            logger.debug(f"Arduino response: '{response}'")
                            
                            if response == expected_response:
                                return True
                            elif response.endswith("_ERROR"):
                                logger.error(f"Arduino reported error: {response}")
                                return False
                            
                            response = ""
                time.sleep(0.001)
            
            logger.error(f"Timeout waiting for {expected_response}. No valid response received.")
            return False
            
        except serial.SerialException as e:
            logger.error(f"Serial communication error: {e}")
            return False
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False
    
    def step_x(self, direction, steps):
        
        if not self.check_serial_connection():
            return False
            
        logger.debug(f"Moving X axis: {direction} {steps} steps")
        cmd = f"STEP X {direction} {steps}\n"
        if self.send_command(cmd, "X_OK"):
            if direction == "FORWARD":
                self.current_pos_x += steps
            else:
                self.current_pos_x -= steps
            return True
        return False
    
    def step_y(self, direction, steps):
        
        if not self.check_serial_connection():
            return False
            
        logger.debug(f"Moving Y axis: {direction} {steps} steps")
        cmd = f"STEP Y {direction} {steps}\n"
        if self.send_command(cmd, "Y_OK"):
            if direction == "FORWARD":
                self.current_pos_y += steps
            else:
                self.current_pos_y -= steps
            return True
        return False
    
    def step_x_with_delay(self, direction, steps, delay=0.01, chunk_size=1):
        
        success = True
        remaining_steps = steps
        
        while remaining_steps > 0:
            if self.state == SystemState.HOMING and not self.manual_homing and self.object_tracked:
                logger.info("Object detected during homing - interrupting homing")
                return False
                
            current_chunk = min(chunk_size, remaining_steps)
            
            if not self.step_x(direction, current_chunk):
                success = False
                break
                
            remaining_steps -= current_chunk
            time.sleep(delay)
            
        return success
    
    def step_y_with_delay(self, direction, steps, delay=0.01, chunk_size=1):
        
        success = True
        remaining_steps = steps
        
        while remaining_steps > 0:
            if self.state == SystemState.HOMING and not self.manual_homing and self.object_tracked:
                logger.info("Object detected during homing - interrupting homing")
                return False
                
            
            current_chunk = min(chunk_size, remaining_steps)
            
            if not self.step_y(direction, current_chunk):
                success = False
                break
                
            remaining_steps -= current_chunk
            time.sleep(delay)
            
        return True
    
    def trigger_relay(self, duration=None):
        
        if duration is None:
            duration = self.config.relay_duration
        
        def relay_operation():
            logger.info("Relay ON")
            GPIO.output(self.config.relay_pin, GPIO.LOW)
            self.alarm_on()
            time.sleep(duration)
            GPIO.output(self.config.relay_pin, GPIO.HIGH)
            self.alarm_off()
            logger.info("Relay OFF")
        
        threading.Thread(target=relay_operation, daemon=True).start()
    
    def alarm_on(self):
        GPIO.output(self.config.alarm_pin, GPIO.HIGH)
    
    def alarm_off(self):
        GPIO.output(self.config.alarm_pin, GPIO.LOW)
    
    def detect_objects(self, img, draw=True, target_objects=None):
        
        if target_objects is None:
            target_objects = self.config.TARGET_OBJECTS
            
        try:
            classIds, confs, bbox = self.net.detect(
                img, 
                confThreshold=self.config.confidence_thres, 
                nmsThreshold=self.config.nms_thres
            )
            
            object_info = []
            
            if len(classIds) != 0:
                for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    if classId - 1 < len(self.classNames):
                        className = self.classNames[classId - 1]
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
    
    def track_object(self, frame_center, object_center):
        
        error_x = frame_center[0] - object_center[0]
        error_y = frame_center[1] - object_center[1]
        
        logger.debug(f"ErrorX: {error_x}, ErrorY: {error_y}")
        
        moved = False
        tolerance = self.config.tolerance
        track_step_size = self.config.track_stepsize
        
        def calculate_step_size(error):
            abs_error = abs(error)
            if abs_error <= tolerance:
                return 0
            
            if abs_error > 100:
                proportion = 1.0
            else:
                proportion = abs_error / 100.0
                
            return max(1, min(round(track_step_size * proportion), track_step_size))
        
        # move x
        x_step_size = calculate_step_size(error_x)
        if x_step_size > 0:
            direction = "FORWARD" if error_x > 0 else "BACKWARD"
            if self.step_x(direction, x_step_size):
                moved = True
                time.sleep(0.01)
        
        # move y
        y_step_size = calculate_step_size(error_y)
        if y_step_size > 0:
            direction = "FORWARD" if error_y > 0 else "BACKWARD"
            if self.step_y(direction, y_step_size):
                moved = True
                time.sleep(0.01)
        
        aligned = abs(error_x) <= tolerance and abs(error_y) <= tolerance
        if aligned:
            logger.debug("ALIGNED")
        
        return aligned, moved
    
    def go_home(self, manual=False):
        
        if self.state == SystemState.HOMING:
            logger.warning("Homing already in progress...")
            return False
        
        logger.info(f"Starting {'manual ' if manual else ''}homing from position: ({self.current_pos_x}, {self.current_pos_y})")
        self.state = SystemState.HOMING
        self.manual_homing = manual
        
        def home_thread():
            try:
                # Home Y axis first
                if self.current_pos_y != 0:
                    logger.info("Homing Y axis...")
                    self.home_axis("Y", self.current_pos_y, manual)
                
                # Then home X axis
                if self.current_pos_x != 0:
                    logger.info("Homing X axis...")
                    self.home_axis("X", self.current_pos_x, manual)
                
                logger.info(f"Home complete. Final position: ({self.current_pos_x}, {self.current_pos_y})")
                
                # Start home cooldown only for automatic homing
                if not manual:
                    self.state = SystemState.HOME_COOLDOWN
                    logger.info(f"Starting {self.config.home_cooldown}-second home cooldown...")
                    time.sleep(self.config.home_cooldown)
                    self.state = SystemState.IDLE
                    logger.info("Home cooldown complete")
                
            except Exception as e:
                logger.error(f"Exception during homing: {e}")
                self.state = SystemState.IDLE
            finally:
                self.manual_homing = False
                
        threading.Thread(target=home_thread, daemon=True).start()
        return True
    
    def home_axis(self, axis, current_pos, manual=False):
        direction = "BACKWARD" if current_pos > 0 else "FORWARD"
        steps = abs(current_pos)
        
        if manual:
            homing_speed = self.config.homing_speed_fast
            chunk_size = 50
            logger.info(f"Fast homing {axis} axis: {steps} steps {direction} (chunk size: {chunk_size})")
        else:
            homing_speed = self.config.homing_speed
            chunk_size = 10
            logger.info(f"Homing {axis} axis: {steps} steps {direction} (chunk size: {chunk_size})")
        
        delay = 1.0 / homing_speed
        
        if axis == "X":
            success = self.step_x_with_delay(direction, steps, delay, chunk_size)
            if success:
                self.current_pos_x = 0
        else:
            success = self.step_y_with_delay(direction, steps, delay, chunk_size)
            if success:
                self.current_pos_y = 0
                
        if success:
            logger.info(f"{axis} axis homed successfully")
            return True
        else:
            logger.info(f"{axis} homing interrupted for object tracking")
            return False
    
    def perform_scan_step(self):
        
        if self.state in [SystemState.HOMING, SystemState.HOME_COOLDOWN]:
            return False
            
        if self.step_x(self.scan_direction, self.config.scan_stepsize):
            self.scan_steps += self.config.scan_stepsize
            
            if self.scan_steps >= self.config.scan_steplimit:
                self.scan_direction = "BACKWARD" if self.scan_direction == "FORWARD" else "FORWARD"
                self.scan_steps = 0
                logger.info(f"Scan direction changed to: {self.scan_direction}")
            
            time.sleep(self.config.scan_delay())
            return True
        return False
    
    def reset_scan(self):
        
        self.scan_steps = 0
        self.scan_direction = "FORWARD"
    
    def display(self, img):
        
        cv2.putText(img, f"Pos: X={self.current_pos_x}, Y={self.current_pos_y}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        status = self.state.name
        cv2.putText(img, f"Status: {status}", (10, 55), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.putText(img, f"Tolerance: {self.config.TOLERANCE}", (10, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.putText(img, f"Step Size: {self.config.TRACK_STEP_SIZE}", (10, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.putText(img, "Controls: Q=Quit, R=Reset, H=Home, P=Info", 
                    (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    
    def run(self):
        
        cap = cv2.VideoCapture(0)
        cap.set(3, self.config.CAMERA_WIDTH)
        cap.set(4, self.config.CAMERA_HEIGHT)
        
        frame_center = (self.config.CAMERA_WIDTH // 2, self.config.CAMERA_HEIGHT // 2)
        logger.info(f"Camera ready: {self.config.CAMERA_WIDTH}x{self.config.CAMERA_HEIGHT}")
        
        try:
            while self.state != SystemState.QUITTING:
                success, img = cap.read()
                if not success:
                    logger.error("Failed to read from camera")
                    self.check_serial_connection()
                    continue
                
                self.display(img)
                
                result_img, object_info = self.detect_objects(img)
                
                object_detected = False
                biggest_box = None
                max_area = 0
                
                for obj in object_info:
                    box, name, conf, center = obj
                    area = box[2] * box[3]
                    if area > self.config.area_thres:
                        if area > max_area:
                            max_area = area
                            biggest_box = (box, center)
                
                cv2.circle(img, frame_center, 6, (0, 0, 255), -1)
                
                if biggest_box is not None:
                    box, bbox_center = biggest_box
                    
                    # If object detected during manual homing, cancel homing
                    if self.state == SystemState.HOMING and self.manual_homing:
                        logger.info("Object detected during manual homing - cancelling homing")
                        self.state = SystemState.IDLE
                        self.manual_homing = False
                    
                    aligned, moved = self.track_object(frame_center, bbox_center)
                    
                    self.object_tracked = True
                    self.lost_since = None
                    
                    alignment_color = (0, 255, 0) if aligned else (0, 0, 255)
                    cv2.circle(img, bbox_center, 8, alignment_color, 2)
                    cv2.line(img, frame_center, bbox_center, alignment_color, 1)
                    
                    error_x = frame_center[0] - bbox_center[0]
                    error_y = frame_center[1] - bbox_center[1]
                    cv2.putText(img, f"Error X: {error_x}, Y: {error_y}", 
                                (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(img, f"Aligned: {aligned}", 
                                (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, alignment_color, 1)
                    
                    current_time = time.time()
                    if aligned and current_time - self.last_trigger_time > self.config.relay_cooldown:
                        self.trigger_relay(self.config.relay_duration)
                        self.last_trigger_time = current_time
                    
                    object_detected = True
                    self.state = SystemState.TRACKING
                
                if not object_detected:
                    self.alarm_off()
                    current_time = time.time()
                    
                    if self.object_tracked:
                        if self.lost_since is None:
                            self.lost_since = current_time
                            logger.info("Object lost. Starting countdown before going home...")
                        else:
                            elapsed = int(current_time - self.lost_since)
                            remaining = self.config.obj_lost_timeout - elapsed
                            
                            if remaining <= 0:
                                logger.info("Countdown complete, initiating home sequence...")
                                if self.go_home(manual=False):  # Automatic homing
                                    self.object_tracked = False
                                    self.lost_since = None
                                    self.reset_scan()
                                else:
                                    self.lost_since = current_time
                    else:
                        # Scan
                        if self.state not in [SystemState.HOMING, SystemState.HOME_COOLDOWN]:
                            self.perform_scan_step()
                            self.state = SystemState.SCANNING
                
                cv2.imshow("PawStopper Output", result_img)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    logger.info("Quitting. Going home...")
                    self.state = SystemState.QUITTING
                    if self.state != SystemState.HOMING:
                        self.go_home(manual=True)
                    while self.state == SystemState.HOMING:
                        time.sleep(0.1)
                    break
                elif key == ord('r'):
                    logger.info("Manually resetting position to (0,0)")
                    self.current_pos_x = 0
                    self.current_pos_y = 0
                elif key == ord('h'):
                    logger.info("Manual home command")
                    self.go_home(manual=True)
                elif key == ord('p'):
                    logger.info(f"Current position: X={self.current_pos_x}, Y={self.current_pos_y}")
                    
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            # Cleanup
            cap.release()
            cv2.destroyAllWindows()
            GPIO.cleanup()
            if self.arduino and self.arduino.is_open:
                self.arduino.close()
            logger.info("Cleanup complete")    

if __name__ == "__main__":
    config = Config()
    controller = PawStopperController(config)
    controller.run()