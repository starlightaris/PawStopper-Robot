"""
PawStopper Robot - Simplified Version
Automated Pet Deterrent System with reduced code complexity
"""

import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading
import logging
from typing import Tuple, List, Optional, Any
from dataclasses import dataclass

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class Config:
    """Simplified configuration settings"""
    # Hardware
    ARDUINO_PORT: str = '/dev/ttyUSB0'
    ARDUINO_BAUDRATE: int = 9600
    RELAY_PIN: int = 17
    ALARM_PIN: int = 18
    
    # Control parameters
    TOLERANCE: int = 15
    TRACK_STEP_SIZE: int = 5
    SCAN_STEP_SIZE: int = 10
    SCAN_LIMIT: int = 1024
    
    # Detection
    AREA_THRESHOLD: int = 15000
    CONFIDENCE_THRESHOLD: float = 0.45
    TARGET_OBJECTS: List[str] = None
    
    # Timing
    RELAY_DURATION: int = 2
    COOLDOWN_SECONDS: int = 4
    LOST_TIMEOUT: int = 5
    
    # Camera
    CAMERA_WIDTH: int = 640
    CAMERA_HEIGHT: int = 480
    INPUT_SIZE: Tuple[int, int] = (320, 320)
    CAMERA_FPS: int = 30
    
    # File paths
    COCO_NAMES_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names"
    MODEL_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"
    CONFIG_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    
    def __post_init__(self):
        if self.TARGET_OBJECTS is None:
            self.TARGET_OBJECTS = ['cat', 'dog']

class StepperController:
    """Simplified stepper motor control"""
    
    def __init__(self, arduino: serial.Serial, config: Config):
        self.arduino = arduino
        self.config = config
        self.pos_x = self.pos_y = 0
        self.is_homing = False
        self._lock = threading.Lock()
        
    def send_command(self, axis: str, direction: str, steps: int) -> bool:
        """Send command to Arduino and wait for response"""
        cmd = f"STEP {axis} {direction} {steps}\n"
        try:
            with self._lock:
                self.arduino.reset_input_buffer()
                time.sleep(0.03)
                self.arduino.write(cmd.encode())
                self.arduino.flush()
                
                start_time = time.time()
                while time.time() - start_time < 3.0:
                    if self.arduino.in_waiting > 0:
                        response = self.arduino.readline().decode().strip()
                        if response == f"{axis}_OK":
                            self._update_position(axis, direction, steps)
                            return True
                        elif response.endswith("_ERROR"):
                            return False
                    time.sleep(0.005)
                return False
        except Exception as e:
            logger.error(f"Command failed: {e}")
            return False
    
    def _update_position(self, axis: str, direction: str, steps: int):
        """Update position tracking"""
        if axis == "X":
            self.pos_x += steps if direction == "FORWARD" else -steps
        else:
            self.pos_y += steps if direction == "FORWARD" else -steps
    
    def track_object(self, frame_center: Tuple[int, int], object_center: Tuple[int, int]) -> Tuple[bool, bool]:
        """Track object with proportional control"""
        error_x = frame_center[0] - object_center[0]
        error_y = frame_center[1] - object_center[1]
        moved = False
        
        def calc_steps(error):
            abs_error = abs(error)
            if abs_error <= self.config.TOLERANCE:
                return 0
            proportion = min(1.0, abs_error / 100.0)
            return max(1, round(self.config.TRACK_STEP_SIZE * proportion))
        
        # Move X axis
        x_steps = calc_steps(error_x)
        if x_steps > 0:
            direction = "FORWARD" if error_x > 0 else "BACKWARD"
            if self.send_command("X", direction, x_steps):
                moved = True
                time.sleep(0.005)
        
        # Move Y axis
        y_steps = calc_steps(error_y)
        if y_steps > 0:
            direction = "FORWARD" if error_y > 0 else "BACKWARD"
            if self.send_command("Y", direction, y_steps):
                moved = True
                time.sleep(0.005)
        
        if moved:
            time.sleep(0.01)
        
        aligned = abs(error_x) <= self.config.TOLERANCE and abs(error_y) <= self.config.TOLERANCE
        return aligned, moved
    
    def scan_step(self, direction: str) -> bool:
        """Perform one scan step"""
        return self.send_command("X", direction, self.config.SCAN_STEP_SIZE)
    
    def go_home(self):
        """Return to home position (async)"""
        if self.is_homing:
            return
        self.is_homing = True
        threading.Thread(target=self._home_thread, daemon=True).start()
    
    def _home_thread(self):
        """Home both axes"""
        try:
            # Home Y first
            if self.pos_y != 0:
                direction = "BACKWARD" if self.pos_y > 0 else "FORWARD"
                steps = abs(self.pos_y)
                while steps > 0:
                    chunk = min(steps, 50)
                    if self.send_command("Y", direction, chunk):
                        steps -= chunk
                    else:
                        break
                self.pos_y = 0
            
            # Then home X
            if self.pos_x != 0:
                direction = "BACKWARD" if self.pos_x > 0 else "FORWARD"
                steps = abs(self.pos_x)
                while steps > 0:
                    chunk = min(steps, 50)
                    if self.send_command("X", direction, chunk):
                        steps -= chunk
                    else:
                        break
                self.pos_x = 0
            
            logger.info("Homing complete")
        finally:
            self.is_homing = False

class ObjectDetector:
    """Simplified object detection"""
    
    def __init__(self, config: Config):
        self.config = config
        self.class_names = self._load_classes()
        self.net = self._init_model()
    
    def _load_classes(self) -> List[str]:
        """Load class names"""
        try:
            with open(self.config.COCO_NAMES_PATH, "rt") as f:
                return f.read().rstrip("\n").split("\n")
        except FileNotFoundError:
            logger.error(f"Class file not found: {self.config.COCO_NAMES_PATH}")
            return []
    
    def _init_model(self) -> cv2.dnn_DetectionModel:
        """Initialize detection model"""
        net = cv2.dnn_DetectionModel(self.config.MODEL_PATH, self.config.CONFIG_PATH)
        try:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception:
            pass
        net.setInputSize(*self.config.INPUT_SIZE)
        net.setInputScale(1.0/127.5)
        net.setInputMean((127.5, 127.5, 127.5))
        net.setInputSwapRB(True)
        return net
    
    def detect(self, img) -> Tuple[Any, List]:
        """Detect objects and return image with annotations"""
        try:
            classIds, confs, bbox = self.net.detect(img, confThreshold=self.config.CONFIDENCE_THRESHOLD, nmsThreshold=0.2)
            objects = []
            
            if len(classIds) > 0:
                for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    if classId - 1 < len(self.class_names):
                        class_name = self.class_names[classId - 1]
                        if class_name in self.config.TARGET_OBJECTS:
                            center = (box[0] + box[2]//2, box[1] + box[3]//2)
                            objects.append([box, class_name, round(conf*100, 2), center])
                            
                            # Draw detection
                            cv2.rectangle(img, box, (0, 255, 0), 2)
                            cv2.putText(img, f"{class_name.upper()} {round(conf*100, 2)}%", 
                                       (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.circle(img, center, 5, (0, 255, 0), -1)
            
            return img, objects
        except Exception as e:
            logger.error(f"Detection failed: {e}")
            return img, []

class PawStopperRobot:
    """Simplified main robot controller"""
    
    def __init__(self, config: Config = None):
        self.config = config or Config()
        GPIO.setmode(GPIO.BCM)
        
        # Initialize hardware
        self.arduino = self._init_arduino()
        self.stepper = StepperController(self.arduino, self.config)
        self.detector = ObjectDetector(self.config)
        self.cap = self._init_camera()
        
        # Setup GPIO
        GPIO.setup(self.config.RELAY_PIN, GPIO.OUT)
        GPIO.setup(self.config.ALARM_PIN, GPIO.OUT)
        GPIO.output(self.config.RELAY_PIN, GPIO.HIGH)
        GPIO.output(self.config.ALARM_PIN, GPIO.LOW)
        
        # State variables
        self.object_tracked = False
        self.last_trigger = 0
        self.lost_since = None
        self.scan_direction = "FORWARD"
        self.scan_steps = 0
    
    def _init_arduino(self) -> serial.Serial:
        """Initialize Arduino connection"""
        arduino = serial.Serial(self.config.ARDUINO_PORT, self.config.ARDUINO_BAUDRATE, timeout=1.0)
        time.sleep(2)
        logger.info("Arduino connected")
        return arduino
    
    def _init_camera(self) -> cv2.VideoCapture:
        """Initialize camera with optimizations"""
        cap = cv2.VideoCapture(0)
        cap.set(3, self.config.CAMERA_WIDTH)
        cap.set(4, self.config.CAMERA_HEIGHT)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FPS, self.config.CAMERA_FPS)
        except Exception:
            pass
        
        # Warm up camera
        warmup_end = time.time() + 0.5
        while time.time() < warmup_end:
            cap.read()
            time.sleep(0.005)
        
        logger.info("Camera initialized")
        return cap
    
    def _trigger_relay(self, duration: int = None):
        """Trigger relay and alarm"""
        if duration is None:
            duration = self.config.RELAY_DURATION
        
        def relay_thread():
            GPIO.output(self.config.RELAY_PIN, GPIO.LOW)
            GPIO.output(self.config.ALARM_PIN, GPIO.HIGH)
            time.sleep(duration)
            GPIO.output(self.config.RELAY_PIN, GPIO.HIGH)
            GPIO.output(self.config.ALARM_PIN, GPIO.LOW)
        
        threading.Thread(target=relay_thread, daemon=True).start()
    
    def _display_info(self, img):
        """Display status information on image"""
        cv2.putText(img, f"Pos: X={self.stepper.pos_x}, Y={self.stepper.pos_y}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(img, f"Homing: {self.stepper.is_homing}", (10, 55), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(img, "Q=Quit, R=Reset, H=Home", (10, img.shape[0] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    
    def _handle_tracking(self, img, objects) -> bool:
        """Handle object tracking logic"""
        # Find largest valid object
        best_box = None
        max_area = 0
        
        for obj in objects:
            box, name, conf, center = obj
            area = box[2] * box[3]
            if area > self.config.AREA_THRESHOLD and area > max_area:
                max_area = area
                best_box = box
        
        frame_center = (self.config.CAMERA_WIDTH//2, self.config.CAMERA_HEIGHT//2)
        cv2.circle(img, frame_center, 6, (0, 0, 255), -1)
        
        if best_box is not None:
            bbox_center = (best_box[0] + best_box[2]//2, best_box[1] + best_box[3]//2)
            aligned, moved = self.stepper.track_object(frame_center, bbox_center)
            
            self.object_tracked = True
            self.lost_since = None
            GPIO.output(self.config.ALARM_PIN, GPIO.HIGH)
            
            # Visual feedback
            color = (0, 255, 0) if aligned else (0, 0, 255)
            cv2.circle(img, bbox_center, 8, color, 2)
            cv2.line(img, frame_center, bbox_center, color, 1)
            
            # Trigger if aligned and cooldown passed
            current_time = time.time()
            if aligned and current_time - self.last_trigger > self.config.COOLDOWN_SECONDS:
                self._trigger_relay()
                self.last_trigger = current_time
                time.sleep(0.05)
            
            return True
        
        return False
    
    def _handle_lost(self):
        """Handle lost object logic"""
        current_time = time.time()
        
        if self.object_tracked:
            if self.lost_since is None:
                self.lost_since = current_time
                logger.info("Object lost, starting countdown...")
            elif current_time - self.lost_since > self.config.LOST_TIMEOUT:
                logger.info("Going home...")
                self.stepper.go_home()
                self.object_tracked = False
                self.lost_since = None
                self.scan_steps = 0
                self.scan_direction = "FORWARD"
        else:
            # Scan if not homing
            if not self.stepper.is_homing:
                if self.stepper.scan_step(self.scan_direction):
                    self.scan_steps += self.config.SCAN_STEP_SIZE
                    if self.scan_steps >= self.config.SCAN_LIMIT:
                        self.scan_direction = "BACKWARD" if self.scan_direction == "FORWARD" else "FORWARD"
                        self.scan_steps = 0
    
    def run(self):
        """Main control loop"""
        logger.info("Starting PawStopper Robot...")
        
        try:
            while True:
                # Drop queued frames for fresh feed
                try:
                    for _ in range(2):
                        self.cap.grab()
                except Exception:
                    pass
                
                success, img = self.cap.read()
                if not success:
                    break
                
                self._display_info(img)
                result_img, objects = self.detector.detect(img)
                object_detected = self._handle_tracking(result_img, objects)
                
                if not object_detected:
                    GPIO.output(self.config.ALARM_PIN, GPIO.LOW)
                    if not self.stepper.is_homing:
                        self.arduino.reset_input_buffer()
                    self._handle_lost()
                
                cv2.imshow("PawStopper Output", result_img)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    logger.info("Quitting...")
                    self.stepper.go_home()
                    time.sleep(2)  # Wait for homing
                    break
                elif key == ord('r'):
                    self.stepper.pos_x = self.stepper.pos_y = 0
                    logger.info("Position reset")
                elif key == ord('h'):
                    self.stepper.go_home()
                    logger.info("Manual home")
        
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up...")
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        if hasattr(self, 'arduino'):
            self.arduino.close()
        logger.info("Cleanup complete!")

def main():
    """Main entry point"""
    config = Config()
    robot = PawStopperRobot(config)
    robot.run()

if __name__ == "__main__":
    main()
