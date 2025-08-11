"""
PawStopper Robot - Ultra-Lightweight Detection Version
For maximum performance on Raspberry Pi 3 using motion detection instead of AI
"""

import cv2
import RPi.GPIO as GPIO
import time
import numpy as np
from raspberry import StepperController, RelayController, AlarmController, Config
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MotionDetector:
    """Lightweight motion-based detection for maximum performance"""
    
    def __init__(self, config):
        self.config = config
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(
            detectShadows=False,
            varThreshold=50,
            history=200
        )
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.min_area = 1000  # Minimum area for detection
        
    def detect_motion(self, frame):
        """Detect motion in frame - much faster than AI detection"""
        # Apply background subtraction
        fg_mask = self.background_subtractor.apply(frame)
        
        # Morphological operations to clean up the mask
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, self.kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel)
        
        # Find contours
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        motion_centers = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                center = (x + w//2, y + h//2)
                motion_centers.append({
                    'center': center,
                    'bbox': (x, y, w, h),
                    'area': area
                })
        
        return motion_centers, fg_mask

class LightweightPawStopper:
    """Ultra-lightweight version using motion detection"""
    
    def __init__(self):
        self.config = Config()
        # Reduce camera resolution for maximum performance
        self.config.CAMERA_WIDTH = 320
        self.config.CAMERA_HEIGHT = 240
        
        GPIO.setmode(GPIO.BCM)
        self.arduino = self._initialize_arduino()
        
        # Initialize subsystems
        self.stepper = StepperController(self.arduino, self.config)
        self.relay = RelayController(self.config)
        self.alarm = AlarmController(self.config)
        self.detector = MotionDetector(self.config)
        
        # State variables
        self.object_tracked = False
        self.last_trigger_time = 0
        
        # Initialize camera
        self.cap = self._initialize_camera()
        
    def _initialize_arduino(self):
        """Initialize Arduino connection"""
        import serial
        try:
            arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1.0)
            time.sleep(2)
            logger.info("Arduino connected successfully!")
            return arduino
        except Exception as e:
            logger.error(f"Failed to initialize Arduino: {e}")
            raise
    
    def _initialize_camera(self):
        """Initialize camera with minimal settings"""
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, 20)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        logger.info("Camera initialized for lightweight detection")
        return cap
    
    def run(self):
        """Main control loop - optimized for speed"""
        logger.info("Starting Lightweight PawStopper...")
        
        # Performance monitoring
        fps_counter = 0
        fps_start = time.time()
        
        try:
            while True:
                success, frame = self.cap.read()
                if not success:
                    break
                
                # Detect motion (much faster than AI detection)
                motion_centers, mask = self.detector.detect_motion(frame)
                
                # Calculate FPS
                fps_counter += 1
                if time.time() - fps_start >= 1.0:
                    fps = fps_counter / (time.time() - fps_start)
                    fps_counter = 0
                    fps_start = time.time()
                    
                    # Display FPS on frame
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Handle detection
                if motion_centers:
                    # Find largest motion area
                    largest_motion = max(motion_centers, key=lambda x: x['area'])
                    center = largest_motion['center']
                    bbox = largest_motion['bbox']
                    
                    # Draw detection
                    cv2.rectangle(frame, (bbox[0], bbox[1]), 
                                (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 255, 0), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    
                    # Track object
                    frame_center = (self.config.CAMERA_WIDTH//2, self.config.CAMERA_HEIGHT//2)
                    cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)
                    
                    aligned, moved = self.stepper.track_object(frame_center, center)
                    
                    self.alarm.alarm_on()
                    self.object_tracked = True
                    
                    # Trigger deterrent if aligned
                    current_time = time.time()
                    if aligned and current_time - self.last_trigger_time > 5:
                        self.relay.trigger_relay_async()
                        self.last_trigger_time = current_time
                        logger.info("DETERRENT TRIGGERED!")
                
                else:
                    self.alarm.alarm_off()
                    if self.object_tracked:
                        logger.info("Motion lost - going home")
                        self.stepper.go_home()
                        self.object_tracked = False
                
                # Display frames
                cv2.imshow("Lightweight PawStopper", frame)
                cv2.imshow("Motion Mask", mask)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    logger.info("Resetting position")
                    self.stepper.manual_reset_position(0, 0)
                elif key == ord('h'):
                    logger.info("Going home")
                    self.stepper.go_home()
                
        except KeyboardInterrupt:
            logger.info("Stopping...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        self.arduino.close()
        logger.info("Cleanup complete!")

if __name__ == "__main__":
    robot = LightweightPawStopper()
    robot.run()
