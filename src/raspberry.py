"""
PawStopper Robot - Automated Pet Deterrent System
Main control module for Raspberry Pi with Arduino stepper motor communication
"""

# Imports
import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading
import logging
from typing import Tuple, List, Dict, Optional, Any
from dataclasses import dataclass

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Configuration Constants
@dataclass
class Config:
    """Configuration settings for the PawStopper robot"""
    
    # Serial Communication
    ARDUINO_PORT: str = '/dev/ttyUSB0'
    ARDUINO_BAUDRATE: int = 9600
    SERIAL_TIMEOUT: float = 1.0
    COMMAND_TIMEOUT: float = 3.0
    
    # GPIO Pins
    RELAY_PIN: int = 17
    ALARM_PIN: int = 18
    
    # Stepper Motor Settings
    DEFAULT_TOLERANCE: int = 15  # Increase to 20-25 to reduce oscillation if needed
    DEFAULT_TRACK_STEP_SIZE: int = 5
    MAX_CHUNK_SIZE: int = 50
    MAX_RETRIES: int = 3
    
    # Scanning Parameters
    SCAN_LIMIT_STEPS: int = 1024  # 180 degrees
    SCAN_STEP_SIZE: int = 10 # Number of steps per scan
    
    # Object Detection
    AREA_THRESHOLD: int = 15000
    CONFIDENCE_THRESHOLD: float = 0.45
    NMS_THRESHOLD: float = 0.2
    TARGET_OBJECTS: List[str] = None
    
    # Timing
    RELAY_DURATION: int = 2
    RELAY_COOLDOWN_SECONDS: int = 4 # 2-4 = 2s cd in consecutive detection/obj staying still
    HOME_COOLDOWN_SECONDS: int = 5
    LOST_OBJECT_TIMEOUT: int = 5
    
    # Camera Settings
    CAMERA_WIDTH: int = 640
    CAMERA_HEIGHT: int = 480
    INPUT_SIZE: Tuple[int, int] = (320, 320)
    CAMERA_FPS: int = 30  # NEW
    
    # File Paths
    COCO_NAMES_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names"
    MODEL_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"
    CONFIG_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    
    def __post_init__(self):
        if self.TARGET_OBJECTS is None:
            self.TARGET_OBJECTS = ['cat', 'dog', 'cell phone'] #change the object here

class StepperController:
    """Handles stepper motor control and position tracking"""
    
    def __init__(self, arduino_serial: serial.Serial, config: Config):
        self.arduino = arduino_serial
        self.config = config
        self.current_pos_x: int = 0
        self.current_pos_y: int = 0
        self.tolerance: int = config.DEFAULT_TOLERANCE
        self.track_step_size: int = config.DEFAULT_TRACK_STEP_SIZE
        self.is_homing: bool = False
        self.home_cooldown_active: bool = False
        self._lock = threading.Lock()
        self._homing_complete_event = threading.Event()
        
    def send_step_command(self, axis: str, direction: str, steps: int) -> bool:
        """Send step command to Arduino and wait for confirmation"""
        cmd = f"STEP {axis} {direction} {steps}\n"
        
        try:
            with self._lock:
                # Clear any pending data before sending command
                self.arduino.reset_input_buffer()
                time.sleep(0.01)  # Reduced from 0.1 to 0.01
                
                self.arduino.write(cmd.encode())
                self.arduino.flush()  # Ensure data is sent
                logger.debug(f"Sent: {cmd.strip()}")
                
                # Wait for acknowledgment with better timeout handling
                start_time = time.time()
                responses_received = []
                
                while time.time() - start_time < self.config.COMMAND_TIMEOUT:
                    if self.arduino.in_waiting > 0:
                        try:
                            response = self.arduino.readline().decode().strip()
                            if response:  # Only process non-empty responses
                                responses_received.append(response)
                                logger.debug(f"Arduino response: '{response}'")
                                
                                if response == f"{axis}_OK":
                                    return True
                                elif response.endswith("_ERROR"):
                                    logger.error(f"Arduino reported error: {response}")
                                    return False
                        except UnicodeDecodeError:
                            logger.warning("Received malformed data from Arduino")
                            continue
                    time.sleep(0.001)  # Reduced from 0.01 to 0.001
                
                logger.error(f"Timeout waiting for {axis}_OK acknowledgment")
                logger.debug(f"All responses received: {responses_received}")
                return False
                
        except Exception as e:
            logger.error(f"Error sending step command: {e}")
            return False
    
    def _update_position(self, axis: str, direction: str, steps: int) -> None:
        """Update position tracking after successful movement"""
        if axis == "X":
            old_pos = self.current_pos_x
            if direction == "FORWARD":
                self.current_pos_x += steps
            else:
                self.current_pos_x -= steps
            logger.debug(f"X position updated: {old_pos} -> {self.current_pos_x}")
        elif axis == "Y":
            old_pos = self.current_pos_y
            if direction == "FORWARD":
                self.current_pos_y += steps
            else:
                self.current_pos_y -= steps
            logger.debug(f"Y position updated: {old_pos} -> {self.current_pos_y}")
    
    def step_x(self, direction: str, steps: int) -> bool:
        """Move X axis and update position tracking"""
        logger.debug(f"Moving X axis: {direction} {steps} steps. Current pos: {self.current_pos_x}")
        if self.send_step_command("X", direction, steps):
            self._update_position("X", direction, steps)
            return True
        else:
            logger.error("X step command failed - position not updated")
            return False
    
    def step_y(self, direction: str, steps: int) -> bool:
        """Move Y axis and update position tracking"""
        logger.debug(f"Moving Y axis: {direction} {steps} steps. Current pos: {self.current_pos_y}")
        if self.send_step_command("Y", direction, steps):
            self._update_position("Y", direction, steps)
            return True
        else:
            logger.error("Y step command failed - position not updated")
            return False
    
    def test_connection(self) -> bool:
        """Test Arduino connection with a simple command"""
        logger.info("Testing Arduino connection...")
        
        test_success = self.send_step_command("X", "FORWARD", 1)
        if test_success:
            logger.info("Connection test successful!")
            self.send_step_command("X", "BACKWARD", 1)
            return True
        else:
            logger.error("Connection test failed!")
            return False
    
    def go_home(self) -> bool:
        """Return both axes to logical zero position (async)"""
        if self.is_homing:
            logger.warning("Homing already in progress...")
            return False
        
        logger.info(f"Starting homing from position: ({self.current_pos_x}, {self.current_pos_y})")
        self._homing_complete_event.clear()
        threading.Thread(target=self._go_home_thread, daemon=True).start()
        return True
    
    def wait_for_homing_complete(self, timeout: float = 30.0) -> bool:
        """Wait for homing to complete with timeout"""
        return self._homing_complete_event.wait(timeout)
    
    def _go_home_thread(self) -> bool:
        """Private method to handle homing in a separate thread"""
        logger.info(f"Going home from position: ({self.current_pos_x}, {self.current_pos_y})")
        
        self.is_homing = True
        home_success = True
        
        try:
            # Test connection before starting
            if not self.test_connection():
                logger.error("Cannot proceed with homing - Arduino communication failed")
                home_success = False
                return False
            
            # Clear any pending data
            with self._lock:
                self.arduino.reset_input_buffer()
                time.sleep(0.05)  # Reduced from 0.2 to 0.05
            
            # Home Y axis first (usually safer)
            if self.current_pos_y != 0:
                logger.info("Homing Y axis first...")
                home_success = self._home_axis("Y", self.current_pos_y)
                if not home_success:
                    logger.error("Y axis homing failed!")
                    return False
                time.sleep(0.1)  # Reduced from 0.5 to 0.1
            
            # Then home X axis
            if self.current_pos_x != 0:
                logger.info("Homing X axis...")
                home_success = self._home_axis("X", self.current_pos_x)
                if not home_success:
                    logger.error("X axis homing failed!")
                    return False
            
            if home_success:
                logger.info(f"Home complete. Final position: ({self.current_pos_x}, {self.current_pos_y})")
                threading.Thread(target=self._home_cooldown_thread, daemon=True).start()
            else:
                logger.error("Homing failed! Position may be inaccurate.")
                
        except Exception as e:
            logger.error(f"Exception during homing: {e}")
            home_success = False
        finally:
            self.is_homing = False
            self._homing_complete_event.set()
            
        return home_success
    
    def _home_axis(self, axis: str, current_pos: int) -> bool:
        """Home a single axis with chunked movement and retry logic"""
        direction = "BACKWARD" if current_pos > 0 else "FORWARD"
        steps = abs(current_pos)
        logger.info(f"Homing {axis} axis: {steps} steps {direction}")
        
        remaining_steps = steps
        retry_count = 0
        consecutive_failures = 0
        
        while remaining_steps > 0 and retry_count < self.config.MAX_RETRIES:
            chunk_steps = min(remaining_steps, self.config.MAX_CHUNK_SIZE)
            logger.debug(f"Attempting {axis} movement: {chunk_steps} steps {direction} (attempt {retry_count + 1})")
            
            if axis == "X":
                success = self.step_x(direction, chunk_steps)
            else:
                success = self.step_y(direction, chunk_steps)
                
            if success:
                remaining_steps -= chunk_steps
                retry_count = 0
                consecutive_failures = 0
                logger.debug(f"{axis} homing progress: {steps - remaining_steps}/{steps} steps")
                time.sleep(0.01)  # Reduced from 0.1 to 0.01
            else:
                retry_count += 1
                consecutive_failures += 1
                logger.warning(f"{axis} step failed! Retry {retry_count}/{self.config.MAX_RETRIES}")
                
                if consecutive_failures >= 2:
                    # Try to reset communication
                    logger.warning("Multiple consecutive failures, resetting communication...")
                    time.sleep(0.1)  # Reduced from 0.5 to 0.1
                    with self._lock:
                        self.arduino.reset_input_buffer()
                        self.arduino.reset_output_buffer()
                    time.sleep(0.1)  # Reduced from 0.5 to 0.1
                    
                if retry_count >= self.config.MAX_RETRIES:
                    logger.error(f"{axis} homing failed after maximum retries!")
                    return False
                    
                time.sleep(0.1)  # Reduced from 0.5 to 0.1
        
        if remaining_steps == 0:
            # Verify position is actually zero
            if axis == "X":
                self.current_pos_x = 0
            else:
                self.current_pos_y = 0
            logger.info(f"{axis} axis homed successfully")
            return True
        else:
            logger.error(f"{axis} homing incomplete! {remaining_steps} steps remaining")
            return False
    
    def _home_cooldown_thread(self) -> None:
        """Handle home cooldown in a separate thread"""
        self.home_cooldown_active = True
        logger.info(f"Starting {self.config.HOME_COOLDOWN_SECONDS}-second home cooldown...")
        
        for i in range(self.config.HOME_COOLDOWN_SECONDS, 0, -1):
            logger.debug(f"Home cooldown: {i} seconds remaining...")
            time.sleep(1)
            
        self.home_cooldown_active = False
        logger.info("Home cooldown complete. Ready to resume operations.")
    
    def track_object(self, frame_center: Tuple[int, int], object_center: Tuple[int, int]) -> Tuple[bool, bool]:
        """Track object by calculating error and moving steppers accordingly"""
        error_x = frame_center[0] - object_center[0]
        error_y = frame_center[1] - object_center[1]
        
        logger.debug(f"ErrorX: {error_x}, ErrorY: {error_y}")
        
        moved = False
        
        # Calculate proportional step size - smaller steps as we get closer
        # Min of 1 step, max of track_step_size
        def calculate_step_size(error, axis):
            # Calculate a proportional step based on error magnitude
            # Larger errors = larger steps, smaller errors = smaller steps
            abs_error = abs(error)
            if abs_error <= self.tolerance:
                return 0
            
            # Scale factor to reduce step size as we get closer
            if abs_error > 100:
                proportion = 1.0  # Full step size for large errors
            else:
                proportion = abs_error / 100.0  # Proportionally smaller steps
                
            # Ensure minimum step of 1, maximum of track_step_size
            step = max(1, min(round(self.track_step_size * proportion), self.track_step_size))
            logger.debug(f"{axis} error: {error}, calculated step: {step}")
            return step
        
        # Move X axis if error is above tolerance with proportional step size
        x_step_size = calculate_step_size(error_x, "X")
        if x_step_size > 0:
            direction = "FORWARD" if error_x > 0 else "BACKWARD"
            if self.step_x(direction, x_step_size):
                moved = True
                time.sleep(0.01)  # Reduced from 0.05 to 0.01
        
        # Move Y axis if error is above tolerance with proportional step size
        y_step_size = calculate_step_size(error_y, "Y")
        if y_step_size > 0:
            direction = "FORWARD" if error_y > 0 else "BACKWARD"
            if self.step_y(direction, y_step_size):
                moved = True
                time.sleep(0.01)  # Reduced from 0.05 to 0.01
        
        # Check if aligned
        aligned = abs(error_x) <= self.tolerance and abs(error_y) <= self.tolerance
        if aligned:
            logger.debug("ALIGNED")
        elif moved:
            # If we made movements, give a brief pause to let the system stabilize
            time.sleep(0.01)  # Reduced from 0.1 to 0.01
            
        return aligned, moved
    
    def scan_step(self, direction: str, step_size: int) -> bool:
        """Perform one scan step if ready"""
        if not self.is_ready_for_operations():
            return False
        return self.step_x(direction, step_size)
    
    def is_ready_for_operations(self) -> bool:
        """Check if the stepper is ready for normal operations"""
        return not (self.is_homing or self.home_cooldown_active)
    
    def get_status(self) -> str:
        """Get current status of the stepper controller"""
        if self.is_homing:
            return "HOMING"
        elif self.home_cooldown_active:
            return "HOME_COOLDOWN"
        else:
            return "READY"
    
    def get_position(self) -> Tuple[int, int]:
        """Get current position"""
        return (self.current_pos_x, self.current_pos_y)
    
    def emergency_stop(self) -> None:
        """Emergency stop - clear all buffers and stop homing"""
        self.is_homing = False
        self.home_cooldown_active = False
        with self._lock:
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()
        logger.warning("Emergency stop activated - buffers cleared")
    
    def force_go_home_sync(self) -> None:
        """Force synchronous homing for shutdown - blocks until complete"""
        if self.is_homing:
            logger.info("Waiting for current homing to complete...")
            if not self.wait_for_homing_complete(30.0):
                logger.warning("Timeout waiting for homing to complete, forcing stop...")
                self.emergency_stop()
                time.sleep(1)
        
        logger.info("Force homing for shutdown...")
        self.is_homing = True
        try:
            self._go_home_thread()
        finally:
            self.is_homing = False

    def set_tracking_parameters(self, tolerance: Optional[int] = None, step_size: Optional[int] = None) -> None:
        """Update tracking parameters"""
        if tolerance is not None:
            self.tolerance = tolerance
            logger.info(f"Tolerance updated to: {self.tolerance}")
        if step_size is not None:
            self.track_step_size = step_size
            logger.info(f"Tracking step size updated to: {self.track_step_size}")
    
    def manual_reset_position(self, x: int = 0, y: int = 0) -> None:
        """Manually reset position counters"""
        self.current_pos_x = x
        self.current_pos_y = y
        logger.info(f"Position manually reset to: ({x}, {y})")
    
    def get_position_info(self) -> Dict[str, Any]:
        """Get detailed position information"""
        return {
            'x_position': self.current_pos_x,
            'y_position': self.current_pos_y,
            'tolerance': self.tolerance,
            'track_step_size': self.track_step_size,
            'status': self.get_status(),
            'is_homing': self.is_homing,
            'home_cooldown_active': self.home_cooldown_active
        }

class RelayController:
    """Handles relay control for the deterrent mechanism"""
    
    def __init__(self, config: Config):
        self.config = config
        self.relay_pin = config.RELAY_PIN
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.HIGH)  # Initially off, but values inverted. HIGH = 0 here (temp fix)
        
        self.alarm = AlarmController(self.config)
        
    def trigger_relay(self, duration: int = None) -> None:
        """Trigger relay for specified duration - NON-BLOCKING VERSION"""
        if duration is None:
            duration = self.config.RELAY_DURATION
            
        def relay_operation():
            logger.info("Relay ON")
            GPIO.output(self.relay_pin, GPIO.LOW)
            self.alarm.alarm_on()
            time.sleep(duration)
            GPIO.output(self.relay_pin, GPIO.HIGH)
            self.alarm.alarm_off()
            logger.info("Relay OFF")
        
        # Run in separate thread to avoid blocking
        threading.Thread(target=relay_operation, daemon=True).start()
    
    def trigger_relay_async(self, duration: int = None) -> None:
        """Trigger relay asynchronously - alias for consistency"""
        self.trigger_relay(duration)

class AlarmController:
    """Handles alarm system control"""
    
    def __init__(self, config: Config):
        self.config = config
        self.alarm_pin = config.ALARM_PIN
        GPIO.setup(self.alarm_pin, GPIO.OUT)
        
    def alarm_on(self) -> None:
        """Turn alarm on"""
        GPIO.output(self.alarm_pin, GPIO.HIGH)
        
    def alarm_off(self) -> None:
        """Turn alarm off"""
        GPIO.output(self.alarm_pin, GPIO.LOW)

class ObjectDetector:
    """Handles object detection using OpenCV DNN"""
    
    def __init__(self, config: Config):
        self.config = config
        self.class_names = self._load_class_names()
        self.net = self._initialize_model()
        
    def _load_class_names(self) -> List[str]:
        """Load class names from coco.names file"""
        try:
            with open(self.config.COCO_NAMES_PATH, "rt") as f:
                return f.read().rstrip("\n").split("\n")
        except FileNotFoundError:
            logger.error(f"Class names file not found: {self.config.COCO_NAMES_PATH}")
            return []
    
    def _initialize_model(self) -> cv2.dnn_DetectionModel:
        """Initialize the detection model"""
        try:
            net = cv2.dnn_DetectionModel(self.config.MODEL_PATH, self.config.CONFIG_PATH)
            # Explicit backend/target for consistency and perf
            try:
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            except Exception:
                pass
            net.setInputSize(*self.config.INPUT_SIZE)
            net.setInputScale(1.0/127.5)
            net.setInputMean((127.5, 127.5, 127.5))
            net.setInputSwapRB(True)
            logger.info("Object detection model initialized successfully")
            return net
        except Exception as e:
            logger.error(f"Failed to initialize detection model: {e}")
            raise
    
    def detect_objects(self, img, draw: bool = True, target_objects: List[str] = None) -> Tuple[Any, List[List]]:
        """Detect objects and mark centers"""
        if target_objects is None:
            target_objects = self.config.TARGET_OBJECTS
            
        try:
            classIds, confs, bbox = self.net.detect(
                img, 
                confThreshold=self.config.CONFIDENCE_THRESHOLD, 
                nmsThreshold=self.config.NMS_THRESHOLD
            )
            
            object_info = []
            
            if len(classIds) != 0:
                for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    if classId - 1 < len(self.class_names):
                        className = self.class_names[classId - 1]
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

class ScanController:
    """Handles scanning behavior when no object is tracked"""
    
    def __init__(self, config: Config, stepper: StepperController):
        self.config = config
        self.stepper = stepper
        self.scan_direction = "FORWARD"
        self.scan_steps = 0
        
    def perform_scan_step(self) -> bool:
        """Perform one scan step if ready"""
        if not self.stepper.is_ready_for_operations():
            status = self.stepper.get_status()
            if status == "HOME_COOLDOWN":
                logger.debug("Waiting for home cooldown to complete...")
            elif status == "HOMING":
                logger.debug("Homing in progress...")
            return False
            
        if self.stepper.scan_step(self.scan_direction, self.config.SCAN_STEP_SIZE):
            logger.debug(f"Scanning... Direction: {self.scan_direction}, Steps: {self.config.SCAN_STEP_SIZE}")
            self.scan_steps += self.config.SCAN_STEP_SIZE

            if self.scan_steps >= self.config.SCAN_LIMIT_STEPS:
                self.scan_direction = "BACKWARD" if self.scan_direction == "FORWARD" else "FORWARD"
                self.scan_steps = 0
                logger.info(f"Scan direction changed to: {self.scan_direction}")
            return True
        else:
            logger.warning("Scan step failed, retrying...")
            return False
    
    def reset_scan(self) -> None:
        """Reset scan parameters"""
        self.scan_steps = 0
        self.scan_direction = "FORWARD"

class PawStopperRobot:
    """Main robot controller class"""
    
    def __init__(self, config: Config = None):
        self.config = config or Config()
        GPIO.setmode(GPIO.BCM)
        self.arduino = self._initialize_arduino()
        
        # Initialize subsystems
        self.stepper = StepperController(self.arduino, self.config)
        self.relay = RelayController(self.config)
        self.alarm = AlarmController(self.config)
        self.detector = ObjectDetector(self.config)
        self.scanner = ScanController(self.config, self.stepper)
        
        # State variables
        self.object_tracked = False
        self.last_trigger_time = 0
        self.lost_since = None
        self.last_countdown_second = None
        
        # Initialize camera
        self.cap = self._initialize_camera()
        
    def _initialize_arduino(self) -> serial.Serial:
        """Initialize Arduino serial connection"""
        try:
            arduino = serial.Serial(
                self.config.ARDUINO_PORT, 
                self.config.ARDUINO_BAUDRATE, 
                timeout=self.config.SERIAL_TIMEOUT
            )
            time.sleep(2)  # Wait for Arduino to reset
            
            # Wait for Arduino ready signal
            logger.info("Waiting for Arduino...")
            start_time = time.time()
            while time.time() - start_time < 5.0:
                if arduino.in_waiting > 0:
                    response = arduino.readline().decode().strip()
                    if "Arduino ready" in response:
                        logger.info("Arduino connected successfully!")
                        break
            else:
                logger.warning("Arduino ready signal not received, continuing anyway...")
                
            return arduino
            
        except Exception as e:
            logger.error(f"Failed to initialize Arduino: {e}")
            raise
    
    def _initialize_camera(self) -> cv2.VideoCapture:
        """Initialize camera"""
        try:
            cap = cv2.VideoCapture(0)
            cap.set(3, self.config.CAMERA_WIDTH)
            cap.set(4, self.config.CAMERA_HEIGHT)
            # Reduce internal buffering if supported
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            # Try to set FPS (may be ignored on some platforms)
            try:
                if self.config.CAMERA_FPS > 0:
                    cap.set(cv2.CAP_PROP_FPS, self.config.CAMERA_FPS)
            except Exception:
                pass
            # Warm up and drop initial frames
            warmup_until = time.time() + 0.5
            while time.time() < warmup_until:
                cap.read()
                time.sleep(0.005)
            logger.info("Camera initialized successfully")
            return cap
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            raise
    
    def _display_ui_info(self, img) -> None:
        """Display UI information on the image"""
        pos_x, pos_y = self.stepper.get_position()
        status = self.stepper.get_status()
        
        cv2.putText(img, f"Pos: X={pos_x}, Y={pos_y}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(img, f"Status: {status}", (10, 55), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(img, f"Tolerance: {self.stepper.tolerance}", (10, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f"Step Size: {self.stepper.track_step_size}", (10, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, "Controls: Q=Quit, R=Reset, H=Home, P=Info", 
                    (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    
    def _handle_object_tracking(self, img, object_info) -> bool:
        """Handle object tracking logic"""
        biggest_box = None
        max_area = 0
        alarm_triggered = False

        for obj in object_info:
            box, name, conf, center = obj
            area = box[2] * box[3]
            if area > self.config.AREA_THRESHOLD:
                alarm_triggered = True
                if area > max_area:
                    max_area = area
                    biggest_box = box

        current_time = time.time()
        frame_center = (self.config.CAMERA_WIDTH//2, self.config.CAMERA_HEIGHT//2)
        cv2.circle(img, frame_center, 6, (0, 0, 255), -1)  # Red center dot

        if alarm_triggered and biggest_box is not None:
            bbox_center = (biggest_box[0] + biggest_box[2]//2, biggest_box[1] + biggest_box[3]//2)
            
            aligned, moved = self.stepper.track_object(frame_center, bbox_center)
            
            #self.alarm.alarm_on()
            self.object_tracked = True
            self.lost_since = None

            # Add some visual feedback for alignment status
            alignment_color = (0, 255, 0) if aligned else (0, 0, 255)  # Green if aligned, red if not
            cv2.circle(img, bbox_center, 8, alignment_color, 2)
            cv2.line(img, frame_center, bbox_center, alignment_color, 1)
            
            # Display alignment info on frame
            error_x = frame_center[0] - bbox_center[0]
            error_y = frame_center[1] - bbox_center[1]
            cv2.putText(img, f"Error X: {error_x}, Y: {error_y}", 
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(img, f"Aligned: {aligned}", 
                        (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, alignment_color, 1)

            # If aligned and cooldown period has passed, trigger relay
            if aligned and current_time - self.last_trigger_time > self.config.RELAY_COOLDOWN_SECONDS:
                self.relay.trigger_relay_async(self.config.RELAY_DURATION)
                
                self.last_trigger_time = current_time
                
                # Add a small delay after triggering to avoid immediate movement
                time.sleep(0.01)  # Reduced from 0.2 to 0.01
            
            return True
        
        return False
    
    def _handle_object_lost(self) -> None:
        """Handle logic when object is lost"""
        current_time = time.time()
        
        if self.object_tracked:
            if self.lost_since is None:
                self.lost_since = current_time
                logger.info("Object lost. Starting countdown before going home...")
                self.last_countdown_second = self.config.LOST_OBJECT_TIMEOUT
            else:
                elapsed = int(current_time - self.lost_since)
                remaining = self.config.LOST_OBJECT_TIMEOUT - elapsed
                if remaining != self.last_countdown_second and remaining > 0:
                    logger.info(f"Object lost countdown: {remaining}")
                    self.last_countdown_second = remaining
                elif remaining <= 0:
                    logger.info("Countdown complete, initiating home sequence...")
                    if self.stepper.go_home():
                        self.object_tracked = False
                        self.lost_since = None
                        self.scanner.reset_scan()
                    else:
                        logger.warning("Failed to start homing, will retry...")
                        self.lost_since = current_time  # Reset countdown
        else:
            # Only scan if stepper is ready
            if self.stepper.is_ready_for_operations():
                self.scanner.perform_scan_step()
    
    def _handle_keyboard_input(self) -> bool:
        """Handle keyboard input, returns True if should quit"""
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            logger.info("Quitting. Sending robot to home position.")
            self.stepper.force_go_home_sync()
            return True
        elif key == ord('r'):
            logger.info("Manually resetting position to (0,0)")
            self.stepper.manual_reset_position(0, 0)
        elif key == ord('h'):
            logger.info("Manual home command")
            self.stepper.go_home()
        elif key == ord('p'):
            info = self.stepper.get_position_info()
            logger.info(f"Current position info: {info}")
            
        return False
    
    def run(self) -> None:
        """Main robot control loop"""
        logger.info("Starting PawStopper Robot...")
        
        try:
            while True:
                # Drop a couple of queued frames (if any) to keep feed fresh
                try:
                    for _ in range(2):
                        self.cap.grab()
                except Exception:
                    pass
                success, img = self.cap.read()
                if not success:
                    logger.error("Failed to read from camera")
                    break

                self._display_ui_info(img)
                
                # Detect objects
                result_img, object_info = self.detector.detect_objects(img)
                
                # Handle object tracking or scanning
                object_detected = self._handle_object_tracking(result_img, object_info)
                
                if not object_detected:
                    self.alarm.alarm_off()
                    # Only reset buffer if not homing to avoid interference
                    if not self.stepper.is_homing:
                        self.arduino.reset_input_buffer()
                    self._handle_object_lost()

                cv2.imshow("PawStopper Output", result_img)
                
                # Handle keyboard input
                if self._handle_keyboard_input():
                    break

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
            self.stepper.force_go_home_sync()
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """Clean up resources"""
        logger.info("Cleaning up...")
        self.stepper.emergency_stop()
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        if hasattr(self, 'arduino'):
            self.arduino.close()
        logger.info("Cleanup complete!")

def main():
    """Main entry point"""
    try:
        config = Config()
        robot = PawStopperRobot(config)
        robot.run()
    except Exception as e:
        logger.error(f"Failed to start robot: {e}")
        raise

if __name__ == "__main__":
    main()