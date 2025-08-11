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
import queue
from concurrent.futures import ThreadPoolExecutor
import numpy as np

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
    DEFAULT_TOLERANCE: int = 15
    DEFAULT_TRACK_STEP_SIZE: int = 5
    MAX_CHUNK_SIZE: int = 50
    MAX_RETRIES: int = 3
    
    # Scanning Parameters
    SCAN_LIMIT_STEPS: int = 1024  # 180 degrees
    SCAN_STEP_SIZE: int = 10 # Number of steps per scan
    
    # Object Detection - Optimized for Pi 3
    AREA_THRESHOLD: int = 8000  # Reduced from 15000
    CONFIDENCE_THRESHOLD: float = 0.5  # Increased from 0.45 for better accuracy
    NMS_THRESHOLD: float = 0.3  # Increased from 0.2
    TARGET_OBJECTS: List[str] = None
    
    # Performance Optimization
    FRAME_SKIP: int = 2  # Process every 2nd frame
    MAX_DETECTION_THREADS: int = 1  # Single thread for Pi 3
    DETECTION_TIMEOUT: float = 0.5  # Timeout for detection processing
    
    # Timing
    RELAY_DURATION: int = 5
    COOLDOWN_SECONDS: int = 10
    HOME_COOLDOWN_SECONDS: int = 5
    LOST_OBJECT_TIMEOUT: int = 5
    
    # Camera Settings - Optimized for performance
    CAMERA_WIDTH: int = 320  # Reduced from 640
    CAMERA_HEIGHT: int = 240  # Reduced from 480
    INPUT_SIZE: Tuple[int, int] = (160, 160)  # Reduced from (320, 320)
    CAMERA_FPS: int = 15  # Limit FPS
    CAMERA_BUFFER_SIZE: int = 1  # Reduce buffer to prevent corruption
    FRAME_RETRY_COUNT: int = 3  # Retry corrupted frames
    CAMERA_WARMUP_FRAMES: int = 5  # Skip initial frames
    
    # File Paths
    COCO_NAMES_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names"
    MODEL_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"
    CONFIG_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    
    def __post_init__(self):
        if self.TARGET_OBJECTS is None:
            self.TARGET_OBJECTS = ['cell phone']

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
                time.sleep(0.1)  # Increased delay
                
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
                    time.sleep(0.01)
                
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
                time.sleep(0.2)
            
            # Home Y axis first (usually safer)
            if self.current_pos_y != 0:
                logger.info("Homing Y axis first...")
                home_success = self._home_axis("Y", self.current_pos_y)
                if not home_success:
                    logger.error("Y axis homing failed!")
                    return False
                time.sleep(0.5)  # Pause between axes
            
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
                time.sleep(0.1)  # Small delay between successful moves
            else:
                retry_count += 1
                consecutive_failures += 1
                logger.warning(f"{axis} step failed! Retry {retry_count}/{self.config.MAX_RETRIES}")
                
                if consecutive_failures >= 2:
                    # Try to reset communication
                    logger.warning("Multiple consecutive failures, resetting communication...")
                    time.sleep(0.5)
                    with self._lock:
                        self.arduino.reset_input_buffer()
                        self.arduino.reset_output_buffer()
                    time.sleep(0.5)
                    
                if retry_count >= self.config.MAX_RETRIES:
                    logger.error(f"{axis} homing failed after maximum retries!")
                    return False
                    
                time.sleep(0.5)  # Wait before retry
        
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
        
        # Move X axis if error is above tolerance
        if abs(error_x) > self.tolerance:
            direction = "FORWARD" if error_x > 0 else "BACKWARD"
            if self.step_x(direction, self.track_step_size):
                moved = True
        
        # Move Y axis if error is above tolerance
        if abs(error_y) > self.tolerance:
            direction = "FORWARD" if error_y > 0 else "BACKWARD"
            if self.step_y(direction, self.track_step_size):
                moved = True
        
        # Check if aligned
        aligned = abs(error_x) <= self.tolerance and abs(error_y) <= self.tolerance
        if aligned:
            logger.debug("ALIGNED")
            
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
        GPIO.output(self.relay_pin, GPIO.HIGH)  # Initially off
        
    def trigger_relay(self, duration: int = None) -> None:
        """Trigger relay for specified duration"""
        if duration is None:
            duration = self.config.RELAY_DURATION
            
        logger.info("Relay ON")
        GPIO.output(self.relay_pin, GPIO.LOW)
        time.sleep(duration)
        GPIO.output(self.relay_pin, GPIO.HIGH)
        logger.info("Relay OFF")
    
    def trigger_relay_async(self, duration: int = None) -> None:
        """Trigger relay asynchronously"""
        if duration is None:
            duration = self.config.RELAY_DURATION
        threading.Thread(target=self.trigger_relay, args=(duration,), daemon=True).start()

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
    """Handles object detection using OpenCV DNN with performance optimizations"""
    
    def __init__(self, config: Config):
        self.config = config
        self.class_names = self._load_class_names()
        self.net = self._initialize_model()
        
        # Performance optimization variables
        self.frame_count = 0
        self.last_detection_result = (None, [])
        self.detection_queue = queue.Queue(maxsize=1)
        self.result_queue = queue.Queue(maxsize=1)
        self.detection_thread = None
        self.detection_running = False
        self.executor = ThreadPoolExecutor(max_workers=config.MAX_DETECTION_THREADS)
        
        # Performance monitoring
        self.detection_times = []
        self.last_fps_time = time.time()
        self.fps_counter = 0
        
    def _load_class_names(self) -> List[str]:
        """Load class names from coco.names file"""
        try:
            with open(self.config.COCO_NAMES_PATH, "rt") as f:
                return f.read().rstrip("\n").split("\n")
        except FileNotFoundError:
            logger.error(f"Class names file not found: {self.config.COCO_NAMES_PATH}")
            return []
    
    def _initialize_model(self) -> cv2.dnn_DetectionModel:
        """Initialize the detection model with optimizations"""
        try:
            net = cv2.dnn_DetectionModel(self.config.MODEL_PATH, self.config.CONFIG_PATH)
            net.setInputSize(*self.config.INPUT_SIZE)
            net.setInputScale(1.0/127.5)
            net.setInputMean((127.5, 127.5, 127.5))
            net.setInputSwapRB(True)
            
            # Set backend to CPU and target to CPU for Pi 3
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            
            logger.info("Object detection model initialized with CPU optimizations")
            return net
        except Exception as e:
            logger.error(f"Failed to initialize detection model: {e}")
            raise
    
    def _detect_async(self, img_small, target_objects: List[str]) -> Tuple[Any, List[List]]:
        """Perform detection in a separate thread"""
        start_time = time.time()
        
        try:
            classIds, confs, bbox = self.net.detect(
                img_small, 
                confThreshold=self.config.CONFIDENCE_THRESHOLD, 
                nmsThreshold=self.config.NMS_THRESHOLD
            )
            
            object_info = []
            
            if len(classIds) != 0:
                for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    if classId - 1 < len(self.class_names):
                        className = self.class_names[classId - 1]
                        if className in target_objects:
                            # Scale coordinates back to original image size
                            scale_x = self.config.CAMERA_WIDTH / self.config.INPUT_SIZE[0]
                            scale_y = self.config.CAMERA_HEIGHT / self.config.INPUT_SIZE[1]
                            
                            scaled_box = [
                                int(box[0] * scale_x),
                                int(box[1] * scale_y),
                                int(box[2] * scale_x),
                                int(box[3] * scale_y)
                            ]
                            center = (scaled_box[0] + scaled_box[2]//2, scaled_box[1] + scaled_box[3]//2)
                            object_info.append([scaled_box, className, round(confidence*100, 2), center])
            
            detection_time = time.time() - start_time
            self.detection_times.append(detection_time)
            if len(self.detection_times) > 10:
                self.detection_times.pop(0)
                
            return img_small, object_info
            
        except Exception as e:
            logger.error(f"Async detection failed: {e}")
            return img_small, []
    
    def detect_objects(self, img, draw: bool = True, target_objects: List[str] = None) -> Tuple[Any, List[List]]:
        """Optimized object detection with frame skipping and threading"""
        if target_objects is None:
            target_objects = self.config.TARGET_OBJECTS
        
        self.frame_count += 1
        self.fps_counter += 1
        
        # Calculate and log FPS every 30 frames
        current_time = time.time()
        if current_time - self.last_fps_time >= 2.0:
            fps = self.fps_counter / (current_time - self.last_fps_time)
            avg_detection_time = sum(self.detection_times) / len(self.detection_times) if self.detection_times else 0
            logger.debug(f"Processing FPS: {fps:.1f}, Avg detection time: {avg_detection_time:.3f}s")
            self.last_fps_time = current_time
            self.fps_counter = 0
        
        # Frame skipping - only process every nth frame
        if self.frame_count % self.config.FRAME_SKIP != 0:
            # Use last detection result but still draw on current frame
            if self.last_detection_result[0] is not None:
                return self._draw_detections(img, self.last_detection_result[1], draw)
            return img, []
        
        # Resize image for faster processing
        img_small = cv2.resize(img, self.config.INPUT_SIZE)
        
        # Try to get result from previous async detection
        result_ready = False
        try:
            if not self.result_queue.empty():
                _, object_info = self.result_queue.get_nowait()
                self.last_detection_result = (img, object_info)
                result_ready = True
        except queue.Empty:
            pass
        
        # Start new async detection if not already running
        if not self.detection_running:
            self.detection_running = True
            future = self.executor.submit(self._detect_async, img_small.copy(), target_objects)
            
            def detection_callback(fut):
                try:
                    result = fut.result(timeout=self.config.DETECTION_TIMEOUT)
                    try:
                        self.result_queue.put_nowait(result)
                    except queue.Full:
                        # Remove old result and add new one
                        try:
                            self.result_queue.get_nowait()
                            self.result_queue.put_nowait(result)
                        except queue.Empty:
                            self.result_queue.put_nowait(result)
                except Exception as e:
                    logger.warning(f"Detection callback error: {e}")
                finally:
                    self.detection_running = False
            
            future.add_done_callback(detection_callback)
        
        # Use cached result or return empty
        if result_ready:
            return self._draw_detections(img, self.last_detection_result[1], draw)
        elif self.last_detection_result[0] is not None:
            return self._draw_detections(img, self.last_detection_result[1], draw)
        else:
            return img, []
    
    def _draw_detections(self, img, object_info: List[List], draw: bool) -> Tuple[Any, List[List]]:
        """Draw detection results on image"""
        if draw and object_info:
            for obj in object_info:
                box, className, conf, center = obj
                cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 2)
                cv2.putText(img, f"{className.upper()} {conf}%", 
                           (box[0]+5, box[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.circle(img, center, 3, (0, 255, 0), -1)
        
        return img, object_info
    
    def cleanup(self):
        """Clean up detection resources"""
        self.detection_running = False
        self.executor.shutdown(wait=True)

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
        
        # Camera frame validation variables
        self.consecutive_frame_failures = 0
        self.max_consecutive_failures = 10
        self.last_valid_frame = None
        self.frame_validation_enabled = True
        
        # Initialize camera with optimizations
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
        """Initialize camera with performance optimizations and corruption handling"""
        try:
            cap = cv2.VideoCapture(0)
            
            # Set camera properties with error checking
            properties = [
                (cv2.CAP_PROP_FRAME_WIDTH, self.config.CAMERA_WIDTH),
                (cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAMERA_HEIGHT),
                (cv2.CAP_PROP_FPS, self.config.CAMERA_FPS),
                (cv2.CAP_PROP_BUFFERSIZE, self.config.CAMERA_BUFFER_SIZE),
                (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')),
                (cv2.CAP_PROP_AUTO_EXPOSURE, 0.25),  # Reduce auto-exposure for stability
            ]
            
            for prop, value in properties:
                if not cap.set(prop, value):
                    logger.warning(f"Failed to set camera property {prop} to {value}")
            
            # Verify camera is working
            if not cap.isOpened():
                raise Exception("Camera failed to open")
            
            # Test frame capture
            test_success, test_frame = cap.read()
            if not test_success or test_frame is None:
                raise Exception("Camera test read failed")
            
            # Warm up camera by capturing and discarding initial frames
            logger.info("Warming up camera...")
            for i in range(self.config.CAMERA_WARMUP_FRAMES):
                cap.read()
                time.sleep(0.1)
            
            # Verify settings
            actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            
            logger.info(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps")
            logger.info("Camera warmup complete")
            return cap
            
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            raise
    
    def _validate_frame(self, frame) -> bool:
        """Validate frame to detect corruption"""
        if frame is None:
            return False
        
        # Check if frame has correct dimensions
        if len(frame.shape) != 3:
            return False
        
        height, width, channels = frame.shape
        if height != self.config.CAMERA_HEIGHT or width != self.config.CAMERA_WIDTH or channels != 3:
            return False
        
        # Check for completely black or white frames (common corruption signs)
        mean_brightness = np.mean(frame)
        if mean_brightness < 5 or mean_brightness > 250:
            return False
        
        # Check for reasonable variance (not a solid color)
        if np.std(frame) < 5:
            return False
        
        return True
    
    def _capture_valid_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Capture a frame with corruption handling and retry logic"""
        for attempt in range(self.config.FRAME_RETRY_COUNT):
            try:
                # Clear buffer before reading to get fresh frame
                for _ in range(self.config.CAMERA_BUFFER_SIZE + 1):
                    success, frame = self.cap.read()
                    if not success:
                        break
                
                if not success or frame is None:
                    logger.warning(f"Frame capture failed, attempt {attempt + 1}")
                    continue
                
                # Validate frame if validation is enabled
                if self.frame_validation_enabled and not self._validate_frame(frame):
                    logger.warning(f"Frame validation failed, attempt {attempt + 1}")
                    continue
                
                # Frame is valid
                self.consecutive_frame_failures = 0
                self.last_valid_frame = frame.copy()
                return True, frame
                
            except Exception as e:
                logger.warning(f"Exception during frame capture, attempt {attempt + 1}: {e}")
                continue
        
        # All attempts failed
        self.consecutive_frame_failures += 1
        logger.error(f"Failed to capture valid frame after {self.config.FRAME_RETRY_COUNT} attempts")
        
        # If too many consecutive failures, try to reinitialize camera
        if self.consecutive_frame_failures >= self.max_consecutive_failures:
            logger.error("Too many consecutive frame failures, attempting camera reset...")
            return self._handle_camera_failure()
        
        # Return last valid frame if available
        if self.last_valid_frame is not None:
            logger.info("Using last valid frame due to capture failure")
            return True, self.last_valid_frame.copy()
        
        return False, None
    
    def _handle_camera_failure(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Handle persistent camera failures by reinitializing"""
        try:
            logger.info("Attempting to reinitialize camera...")
            
            # Close current camera
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
                time.sleep(1)
            
            # Try to reinitialize
            self.cap = self._initialize_camera()
            self.consecutive_frame_failures = 0
            
            # Try to capture a frame
            success, frame = self.cap.read()
            if success and frame is not None:
                logger.info("Camera reinitialization successful")
                self.last_valid_frame = frame.copy()
                return True, frame
            else:
                logger.error("Camera reinitialization failed")
                return False, None
                
        except Exception as e:
            logger.error(f"Camera reinitialization failed: {e}")
            return False, None
    
    def _display_ui_info(self, img) -> None:
        """Display UI information on the image - optimized"""
        pos_x, pos_y = self.stepper.get_position()
        status = self.stepper.get_status()
        
        # Use smaller font and fewer text elements for performance
        font_scale = 0.4
        thickness = 1
        
        cv2.putText(img, f"X:{pos_x} Y:{pos_y}", (5, 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
        cv2.putText(img, f"{status}", (5, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 255), thickness)
        
        # Show frame failure count if any
        if self.consecutive_frame_failures > 0:
            cv2.putText(img, f"Frame Failures: {self.consecutive_frame_failures}", (5, 45), 
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), thickness)
        
        cv2.putText(img, "Q=Quit R=Reset H=Home V=Toggle Validation", 
                    (5, img.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
    
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
        cv2.circle(img, frame_center, 3, (0, 0, 255), -1)  # Smaller center dot

        if alarm_triggered and biggest_box is not None:
            bbox_center = (biggest_box[0] + biggest_box[2]//2, biggest_box[1] + biggest_box[3]//2)
            
            aligned, moved = self.stepper.track_object(frame_center, bbox_center)
            
            self.alarm.alarm_on()
            self.object_tracked = True
            self.lost_since = None

            # If aligned and cooldown period has passed, trigger relay
            if aligned and current_time - self.last_trigger_time > self.config.COOLDOWN_SECONDS:
                self.relay.trigger_relay_async(self.config.RELAY_DURATION)
                self.last_trigger_time = current_time
            
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
        elif key == ord('v'):
            self.frame_validation_enabled = not self.frame_validation_enabled
            logger.info(f"Frame validation {'enabled' if self.frame_validation_enabled else 'disabled'}")
        elif key == ord('c'):
            logger.info("Manual camera reset")
            self._handle_camera_failure()
            
        return False
    
    def run(self) -> None:
        """Main robot control loop - optimized with corruption handling"""
        logger.info("Starting PawStopper Robot with performance optimizations and corruption handling...")
        
        try:
            while True:
                # Use improved frame capture with corruption handling
                success, img = self._capture_valid_frame()
                if not success or img is None:
                    logger.error("Failed to capture valid frame, retrying...")
                    time.sleep(0.1)  # Brief pause before retry
                    continue

                # Only update UI every few frames for performance
                if self.detector.frame_count % 3 == 0:
                    self._display_ui_info(img)
                
                # Detect objects (now optimized with frame skipping)
                try:
                    result_img, object_info = self.detector.detect_objects(img)
                except Exception as e:
                    logger.warning(f"Object detection failed: {e}")
                    result_img, object_info = img, []
                
                # Handle object tracking or scanning
                object_detected = self._handle_object_tracking(result_img, object_info)
                
                if not object_detected:
                    self.alarm.alarm_off()
                    # Only reset buffer if not homing to avoid interference
                    if not self.stepper.is_homing:
                        self.arduino.reset_input_buffer()
                    self._handle_object_lost()

                # Safe display with error handling
                try:
                    cv2.imshow("PawStopper Output", result_img)
                except Exception as e:
                    logger.warning(f"Display error: {e}")
                
                # Handle keyboard input
                if self._handle_keyboard_input():
                    break

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
            self.stepper.force_go_home_sync()
        except Exception as e:
            logger.error(f"Unexpected error in main loop: {e}")
            # Try to continue running
            time.sleep(1)
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """Clean up resources"""
        logger.info("Cleaning up...")
        self.stepper.emergency_stop()
        self.detector.cleanup()  # Clean up detection resources
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        if hasattr(self, 'arduino') and self.arduino is not None:
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
