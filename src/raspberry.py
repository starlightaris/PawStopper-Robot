"""
PawStopper Robot Control System
This module implements the main control system for the PawStopper robot,
integrating camera feed, object detection, and hardware control components.
"""

import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading
import logging
import json
import os
from datetime import datetime
from typing import Tuple, List, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Configuration Constants
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUDRATE = 9600
SERIAL_TIMEOUT = 1

# Motor limits (in steps)
PAN_LIMIT_STEPS = 100   # ±90 degrees (200 steps = 360°, so 100 steps = 180°)
TILT_LIMIT_STEPS = 25   # ±45 degrees (200 steps = 360°, so 25 steps = 45°)

# Position tracking file
POSITION_FILE = "position_state.json"

class ArduinoCommunicator:
    """Handles serial communication with the Arduino."""
    
    def __init__(self, port: str = SERIAL_PORT, baudrate: int = SERIAL_BAUDRATE, timeout: int = SERIAL_TIMEOUT):
        """Initialize the Arduino communication."""
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for Arduino to reset
            logger.info("Arduino connection established successfully")
        except serial.SerialException as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            raise

    def send_coordinates(self, center_frame: Tuple[int, int], center_object: Tuple[int, int]) -> None:
        """Send coordinate data to Arduino.
        
        Args:
            center_frame: Tuple of (x, y) coordinates for frame center
            center_object: Tuple of (x, y) coordinates for object center
        """
        try:
            data = f"{center_frame[0]},{center_frame[1]},{center_object[0]},{center_object[1]}\n"
            self.arduino.write(data.encode())
            logger.info(f"Sent to Arduino: {data.strip()}")
        except serial.SerialException as e:
            logger.error(f"Failed to send coordinates to Arduino: {e}")
    
    def send_scan_command(self) -> None:
        """Send scan command to Arduino."""
        try:
            self.arduino.write(b'SCAN\n')
            logger.debug("Scan command sent to Arduino")
        except serial.SerialException as e:
            logger.error(f"Failed to send scan command: {e}")
    
    def send_home_command(self) -> None:
        """Send home command to Arduino."""
        try:
            self.arduino.write(b'HOME\n')
            logger.info("Home command sent to Arduino")
        except serial.SerialException as e:
            logger.error(f"Failed to send home command: {e}")
    
    def send_absolute_position(self, pan_steps: int, tilt_steps: int) -> None:
        """Send absolute position command to Arduino.
        
        Args:
            pan_steps: Target pan position in steps
            tilt_steps: Target tilt position in steps
        """
        try:
            data = f"POS,{pan_steps},{tilt_steps}\n"
            self.arduino.write(data.encode())
            logger.info(f"Position command sent: {data.strip()}")
        except serial.SerialException as e:
            logger.error(f"Failed to send position command: {e}")
    
    def read_message(self) -> Optional[str]:
        """Read a message from Arduino.
        
        Returns:
            The decoded message or None if no message available
        """
        try:
            if self.arduino.in_waiting > 0:
                return self.arduino.readline().decode().strip()
            return None
        except serial.SerialException as e:
            logger.error(f"Failed to read from Arduino: {e}")
            return None
    
    def clear_buffer(self) -> None:
        """Clear the input buffer."""
        try:
            self.arduino.reset_input_buffer()
        except serial.SerialException as e:
            logger.error(f"Failed to clear Arduino buffer: {e}")

    def close(self) -> None:
        """Close the serial connection."""
        try:
            self.arduino.close()
            logger.info("Arduino connection closed")
        except serial.SerialException as e:
            logger.error(f"Error closing Arduino connection: {e}")


class PositionTracker:
    """Tracks and manages robot position state."""
    
    def __init__(self, position_file: str = POSITION_FILE):
        """Initialize position tracker.
        
        Args:
            position_file: Path to position state file
        """
        self.position_file = position_file
        self.pan_steps = 0
        self.tilt_steps = 0
        self.load_position()
    
    def load_position(self) -> None:
        """Load position from file or create new if doesn't exist."""
        try:
            if os.path.exists(self.position_file):
                with open(self.position_file, 'r') as f:
                    data = json.load(f)
                    self.pan_steps = data.get('pan_steps', 0)
                    self.tilt_steps = data.get('tilt_steps', 0)
                    logger.info(f"Position loaded: Pan={self.pan_steps}, Tilt={self.tilt_steps}")
            else:
                self.save_position()
                logger.info("Position file created with default values")
        except Exception as e:
            logger.error(f"Error loading position: {e}")
            self.pan_steps = 0
            self.tilt_steps = 0
    
    def save_position(self) -> None:
        """Save current position to file."""
        try:
            data = {
                'pan_steps': self.pan_steps,
                'tilt_steps': self.tilt_steps,
                'last_updated': datetime.now().isoformat()
            }
            with open(self.position_file, 'w') as f:
                json.dump(data, f, indent=2)
            logger.debug(f"Position saved: Pan={self.pan_steps}, Tilt={self.tilt_steps}")
        except Exception as e:
            logger.error(f"Error saving position: {e}")
    
    def update_position(self, pan_delta: int, tilt_delta: int) -> None:
        """Update position by delta steps.
        
        Args:
            pan_delta: Change in pan steps
            tilt_delta: Change in tilt steps
        """
        self.pan_steps = max(-PAN_LIMIT_STEPS, min(PAN_LIMIT_STEPS, self.pan_steps + pan_delta))
        self.tilt_steps = max(-TILT_LIMIT_STEPS, min(TILT_LIMIT_STEPS, self.tilt_steps + tilt_delta))
        self.save_position()
    
    def reset_to_center(self) -> Tuple[int, int]:
        """Reset position to center and return steps needed.
        
        Returns:
            Tuple of (pan_steps_to_center, tilt_steps_to_center)
        """
        pan_to_center = -self.pan_steps
        tilt_to_center = -self.tilt_steps
        
        self.pan_steps = 0
        self.tilt_steps = 0
        self.save_position()
        
        logger.info(f"Reset to center: Pan={pan_to_center}, Tilt={tilt_to_center}")
        return pan_to_center, tilt_to_center
    
    def is_within_limits(self, pan_delta: int, tilt_delta: int) -> bool:
        """Check if movement would exceed limits.
        
        Args:
            pan_delta: Proposed pan movement
            tilt_delta: Proposed tilt movement
            
        Returns:
            True if movement is within limits
        """
        new_pan = self.pan_steps + pan_delta
        new_tilt = self.tilt_steps + tilt_delta
        
        return (-PAN_LIMIT_STEPS <= new_pan <= PAN_LIMIT_STEPS and 
                -TILT_LIMIT_STEPS <= new_tilt <= TILT_LIMIT_STEPS)
    
    def get_position(self) -> Tuple[int, int]:
        """Get current position.
        
        Returns:
            Tuple of (pan_steps, tilt_steps)
        """
        return self.pan_steps, self.tilt_steps


class GPIOController:
    """Controls GPIO components including relay and alarm."""
    
    def __init__(self, relay_pin: int = 17, alarm_pin: int = 18):
        """Initialize GPIO controller.
        
        Args:
            relay_pin: GPIO pin number for relay
            alarm_pin: GPIO pin number for alarm
        """
        self.relay_pin = relay_pin
        self.alarm_pin = alarm_pin
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.setup(self.alarm_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.HIGH)  # Initially off
        
        logger.info("GPIO components initialized")
    
    def trigger_relay(self, duration: float = 5) -> None:
        """Trigger the relay for a specified duration.
        
        Args:
            duration: Time in seconds to keep the relay on
        """
        logger.info("Relay ON")
        GPIO.output(self.relay_pin, GPIO.LOW)
        time.sleep(duration)
        GPIO.output(self.relay_pin, GPIO.HIGH)
        logger.info("Relay OFF")
    
    def trigger_relay_async(self, duration: float = 5) -> None:
        """Trigger the relay asynchronously.
        
        Args:
            duration: Time in seconds to keep the relay on
        """
        threading.Thread(
            target=self.trigger_relay,
            args=(duration,),
            daemon=True
        ).start()
    
    def alarm_on(self) -> None:
        """Turn on the alarm."""
        GPIO.output(self.alarm_pin, GPIO.HIGH)
    
    def alarm_off(self) -> None:
        """Turn off the alarm."""
        GPIO.output(self.alarm_pin, GPIO.LOW)
    
    def cleanup(self) -> None:
        """Clean up GPIO resources."""
        GPIO.cleanup()
        logger.info("GPIO cleanup completed")

# Model paths configuration
MODEL_CONFIG = {
    'COCO_NAMES': "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names",
    'CONFIG_PATH': "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt",
    'WEIGHTS_PATH': "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"
}

class ObjectDetector:
    """Handles object detection using OpenCV DNN model."""
    
    def __init__(self, config_path: str, weights_path: str, names_path: str):
        """Initialize the object detector.
        
        Args:
            config_path: Path to model configuration file
            weights_path: Path to model weights file
            names_path: Path to class names file
        """
        self.load_class_names(names_path)
        self.initialize_model(config_path, weights_path)
        logger.info("Object detector initialized successfully")
    
    def load_class_names(self, names_path: str) -> None:
        """Load class names from file.
        
        Args:
            names_path: Path to the class names file
        """
        try:
            with open(names_path, "rt") as f:
                self.class_names = f.read().rstrip("\n").split("\n")
            logger.info(f"Loaded {len(self.class_names)} class names")
        except FileNotFoundError:
            logger.error(f"Class names file not found: {names_path}")
            raise
    
    def initialize_model(self, config_path: str, weights_path: str) -> None:
        """Initialize the DNN model.
        
        Args:
            config_path: Path to model configuration
            weights_path: Path to model weights
        """
        try:
            self.net = cv2.dnn_DetectionModel(weights_path, config_path)
            self.net.setInputSize(320, 320)
            self.net.setInputScale(1.0 / 127.5)
            self.net.setInputMean((127.5, 127.5, 127.5))
            self.net.setInputSwapRB(True)
        except Exception as e:
            logger.error(f"Failed to initialize model: {e}")
            raise
    
    def detect_objects(self, img: np.ndarray, 
                      threshold: float = 0.45, 
                      nms_threshold: float = 0.2, 
                      draw: bool = True,
                      target_objects: Optional[List[str]] = None) -> Tuple[np.ndarray, List]:
        """Detect objects in the image and optionally draw bounding boxes.
        
        Args:
            img: Input image
            threshold: Confidence threshold for detection
            nms_threshold: Non-maximum suppression threshold
            draw: Whether to draw detection results on image
            target_objects: List of specific objects to detect
        
        Returns:
            Tuple of (annotated image, list of detected objects)
        """
        target_objects = target_objects or self.class_names
        object_info = []
        
        try:
            class_ids, confidences, boxes = self.net.detect(
                img, 
                confThreshold=threshold, 
                nmsThreshold=nms_threshold
            )
            
            if len(class_ids) != 0:
                for class_id, confidence, box in zip(
                    class_ids.flatten(), 
                    confidences.flatten(), 
                    boxes
                ):
                    class_name = self.class_names[class_id - 1]
                    if class_name in target_objects:
                        center_x = box[0] + box[2] // 2
                        center_y = box[1] + box[3] // 2
                        
                        object_info.append([
                            box,
                            class_name,
                            round(confidence * 100, 2),
                            (center_x, center_y)
                        ])
                        
                        if draw:
                            self._draw_detection(
                                img, 
                                box, 
                                class_name, 
                                confidence,
                                (center_x, center_y)
                            )
            
            return img, object_info
            
        except Exception as e:
            logger.error(f"Error during object detection: {e}")
            return img, []
    
    def _draw_detection(self, img: np.ndarray, 
                       box: tuple, 
                       class_name: str, 
                       confidence: float,
                       center: Tuple[int, int]) -> None:
        """Draw detection results on the image.
        
        Args:
            img: Image to draw on
            box: Bounding box coordinates
            class_name: Name of detected class
            confidence: Detection confidence
            center: Center point coordinates
        """
        cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
        cv2.putText(img, class_name.upper(), (box[0]+10, box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(img, f"{round(confidence * 100, 2)}%", 
                    (box[0]+200, box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
        cv2.circle(img, center, radius=5, color=(0, 255, 0), thickness=-1)


class CameraController:
    """Manages camera operations and frame processing."""
    
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        """Initialize camera controller.
        
        Args:
            camera_id: Camera device ID
            width: Frame width
            height: Frame height
        """
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(3, width)
        self.cap.set(4, height)
        
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_center = (self.frame_width // 2, self.frame_height // 2)
        
        logger.info(f"Camera initialized: {self.frame_width}x{self.frame_height}")
    
    def read_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read a frame from the camera.
        
        Returns:
            Tuple of (success flag, frame if successful)
        """
        success, frame = self.cap.read()
        if not success:
            logger.error("Failed to read from camera")
            return False, None
            
        # Draw center of the video feed
        cv2.circle(frame, self.frame_center, 6, (0, 0, 255), -1)
        return True, frame
    
    def release(self) -> None:
        """Release camera resources."""
        self.cap.release()
        cv2.destroyAllWindows()
        logger.info("Camera resources released")


class PawStopperController:
    """Main controller class for the PawStopper robot."""
    
    def __init__(self):
        """Initialize the PawStopper controller."""
        self.arduino = ArduinoCommunicator()
        self.gpio = GPIOController()
        self.position_tracker = PositionTracker()
        self.detector = ObjectDetector(
            MODEL_CONFIG['CONFIG_PATH'],
            MODEL_CONFIG['WEIGHTS_PATH'],
            MODEL_CONFIG['COCO_NAMES']
        )
        self.camera = CameraController()
        
        # Configuration
        self.AREA_THRESHOLD = 15000
        self.COOLDOWN_SECONDS = 10
        self.last_trigger_time = 0
        
        # Scanning configuration
        self.scanning_mode = True
        self.scan_pan_direction = 1  # 1 for right, -1 for left
        self.scan_steps_per_move = 5
        self.scan_height_levels = [-15, -5, 5, 15]  # Different tilt levels
        self.current_height_index = 0
        
        # Initialize robot position
        self.home_robot()
    
    def home_robot(self) -> None:
        """Reset robot to center position."""
        logger.info("Homing robot to center position...")
        pan_to_center, tilt_to_center = self.position_tracker.reset_to_center()
        
        if pan_to_center != 0 or tilt_to_center != 0:
            # Send absolute position command to center
            self.arduino.send_absolute_position(0, 0)
            time.sleep(2)  # Wait for movement to complete
        
        logger.info("Robot homed to center position")
    
    def process_frame(self, frame: np.ndarray) -> None:
        """Process a single frame.
        
        Args:
            frame: Input frame from camera
        """
        frame, detections = self.detector.detect_objects(
            frame,
            target_objects=['cat', 'dog']
        )
        
        first_detection = self._find_first_detection(detections)
        
        if first_detection:
            self.scanning_mode = False
            bbox_center = (
                first_detection[0] + first_detection[2] // 2,
                first_detection[1] + first_detection[3] // 2
            )
            logger.info(f"Target detected at {bbox_center}")
            
            self.arduino.send_coordinates(
                self.camera.frame_center,
                bbox_center
            )
            self.gpio.alarm_on()
            
            if self._check_alignment():
                self._handle_aligned_target()
        else:
            self.gpio.alarm_off()
            self.arduino.clear_buffer()
            
            # Enter scanning mode if no target detected
            if not self.scanning_mode:
                self.scanning_mode = True
                logger.info("Entering scanning mode")
            
            self._perform_scan()
    
    def _perform_scan(self) -> None:
        """Perform horizontal sweep scanning at different heights."""
        try:
            current_pan, current_tilt = self.position_tracker.get_position()
            target_tilt = self.scan_height_levels[self.current_height_index]
            
            # Move to target height if not already there
            if current_tilt != target_tilt:
                tilt_delta = target_tilt - current_tilt
                if self.position_tracker.is_within_limits(0, tilt_delta):
                    self.arduino.send_absolute_position(current_pan, target_tilt)
                    self.position_tracker.update_position(0, tilt_delta)
                    time.sleep(0.2)
                    return
            
            # Perform horizontal scan
            pan_delta = self.scan_steps_per_move * self.scan_pan_direction
            
            if self.position_tracker.is_within_limits(pan_delta, 0):
                new_pan = current_pan + pan_delta
                self.arduino.send_absolute_position(new_pan, current_tilt)
                self.position_tracker.update_position(pan_delta, 0)
            else:
                # Reached limit, change direction and move to next height
                self.scan_pan_direction *= -1
                self.current_height_index = (self.current_height_index + 1) % len(self.scan_height_levels)
                logger.debug(f"Scan: Changed direction, height level: {self.current_height_index}")
            
            time.sleep(0.1)  # Small delay between scan steps
            
        except Exception as e:
            logger.error(f"Error during scanning: {e}")
    
    def _find_first_detection(self, detections: List) -> Optional[tuple]:
        """Find the first detected object above threshold.
        
        Args:
            detections: List of detected objects
            
        Returns:
            Bounding box of first qualifying object or None
        """
        for box, name, confidence, center in detections:
            bbox_area = box[2] * box[3]
            logger.debug(
                f"Detection: {name.upper()} | "
                f"Confidence: {confidence}% | "
                f"Area: {bbox_area}"
            )
            
            if bbox_area > self.AREA_THRESHOLD:
                logger.info(f"First target selected: {name.upper()} (Area: {bbox_area})")
                return box
        
        return None
    
    def _check_alignment(self) -> bool:
        """Check if Arduino reports alignment."""
        msg = self.arduino.read_message()
        if msg == "ALIGNED":
            return True
        elif msg and msg.startswith("POS:"):
            # Update position from Arduino feedback
            try:
                parts = msg.split(":")
                if len(parts) == 2:
                    pan, tilt = map(int, parts[1].split(","))
                    self.position_tracker.pan_steps = pan
                    self.position_tracker.tilt_steps = tilt
                    self.position_tracker.save_position()
            except ValueError:
                logger.error(f"Invalid position feedback: {msg}")
        return False
    
    def _handle_aligned_target(self) -> None:
        """Handle aligned target detection."""
        current_time = time.time()
        if current_time - self.last_trigger_time > self.COOLDOWN_SECONDS:
            self.gpio.trigger_relay_async(5)
            self.last_trigger_time = current_time
    
    def run(self) -> None:
        """Run the main control loop."""
        try:
            while True:
                success, frame = self.camera.read_frame()
                if not success:
                    break
                    
                self.process_frame(frame)
                
                # Display scanning status
                status_text = "SCANNING" if self.scanning_mode else "TRACKING"
                cv2.putText(frame, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                
                cv2.imshow("Output", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                elif cv2.waitKey(1) & 0xFF == ord('h'):
                    self.home_robot()
                    
        except KeyboardInterrupt:
            logger.info("Program interrupted by user")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """Clean up resources."""
        self.gpio.cleanup()
        self.arduino.close()
        self.camera.release()


if __name__ == "__main__":
    import numpy as np
    
    controller = PawStopperController()
    controller.run()

