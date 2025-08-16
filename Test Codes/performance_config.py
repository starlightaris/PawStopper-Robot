"""
PawStopper Robot - Performance Configuration
Optimized settings for Raspberry Pi 3 performance
"""

from dataclasses import dataclass
from typing import Tuple, List

@dataclass
class PerformanceConfig:
    """Optimized configuration settings for better performance on Raspberry Pi 3"""
    
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
    
    # Object Detection - OPTIMIZED FOR PERFORMANCE
    AREA_THRESHOLD: int = 15000
    CONFIDENCE_THRESHOLD: float = 0.5  # Increased for fewer false positives
    NMS_THRESHOLD: float = 0.3  # Increased to reduce computation
    TARGET_OBJECTS: List[str] = None
    
    # Timing
    RELAY_DURATION: int = 5
    COOLDOWN_SECONDS: int = 10
    HOME_COOLDOWN_SECONDS: int = 5
    LOST_OBJECT_TIMEOUT: int = 5
    
    # Camera Settings - OPTIMIZED
    CAMERA_WIDTH: int = 480  # Reduced from 640
    CAMERA_HEIGHT: int = 360  # Reduced from 480
    INPUT_SIZE: Tuple[int, int] = (192, 192)  # Smaller input for faster detection
    
    # Performance Optimization - KEY SETTINGS
    FRAME_SKIP_COUNT: int = 3  # Process every 4th frame (adjust 0-5)
    DETECTION_RESIZE_FACTOR: float = 0.4  # Resize to 40% for detection (0.3-1.0)
    USE_THREADING: bool = True  # Enable threaded detection
    
    # File Paths
    COCO_NAMES_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names"
    MODEL_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"
    CONFIG_PATH: str = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    
    def __post_init__(self):
        if self.TARGET_OBJECTS is None:
            self.TARGET_OBJECTS = ['cell phone']

# Performance presets for different scenarios
class PerformancePresets:
    """Predefined performance configurations"""
    
    @staticmethod
    def get_maximum_performance():
        """Maximum performance, minimum accuracy"""
        config = PerformanceConfig()
        config.FRAME_SKIP_COUNT = 4
        config.DETECTION_RESIZE_FACTOR = 0.3
        config.INPUT_SIZE = (160, 160)
        config.CAMERA_WIDTH = 320
        config.CAMERA_HEIGHT = 240
        config.CONFIDENCE_THRESHOLD = 0.6
        return config
    
    @staticmethod
    def get_balanced():
        """Balanced performance and accuracy"""
        config = PerformanceConfig()
        config.FRAME_SKIP_COUNT = 2
        config.DETECTION_RESIZE_FACTOR = 0.5
        config.INPUT_SIZE = (224, 224)
        config.CAMERA_WIDTH = 480
        config.CAMERA_HEIGHT = 360
        config.CONFIDENCE_THRESHOLD = 0.5
        return config
    
    @staticmethod
    def get_maximum_accuracy():
        """Maximum accuracy, lower performance"""
        config = PerformanceConfig()
        config.FRAME_SKIP_COUNT = 0
        config.DETECTION_RESIZE_FACTOR = 0.8
        config.INPUT_SIZE = (320, 320)
        config.CAMERA_WIDTH = 640
        config.CAMERA_HEIGHT = 480
        config.CONFIDENCE_THRESHOLD = 0.45
        return config

# Performance tuning tips for Raspberry Pi 3:
"""
PERFORMANCE TUNING GUIDE:

1. FRAME_SKIP_COUNT (0-5):
   - 0: Process every frame (highest accuracy, lowest performance)
   - 3: Process every 4th frame (good balance)
   - 5: Process every 6th frame (highest performance, lowest accuracy)

2. DETECTION_RESIZE_FACTOR (0.3-1.0):
   - 0.3: Resize to 30% (fastest detection, lower accuracy)
   - 0.5: Resize to 50% (good balance)
   - 1.0: No resize (slowest detection, highest accuracy)

3. INPUT_SIZE:
   - (160, 160): Fastest
   - (224, 224): Balanced
   - (320, 320): Most accurate

4. Camera Resolution:
   - 320x240: Fastest
   - 480x360: Balanced
   - 640x480: Highest quality

5. Runtime Controls (while running):
   - '+' / '-': Adjust frame skip
   - 's': Cycle through resize factors
   - 't': Toggle threading on/off

6. System Optimizations (run on Pi):
   sudo raspi-config
   - Advanced Options > Memory Split > 128 (give more RAM to CPU)
   - Advanced Options > Expand Filesystem
   
   # Increase swap if needed
   sudo dphys-swapfile swapoff
   sudo nano /etc/dphys-swapfile  # Set CONF_SWAPSIZE=1024
   sudo dphys-swapfile setup
   sudo dphys-swapfile swapon

7. Consider using lighter model:
   - Replace ssd_mobilenet_v3_large with ssd_mobilenet_v2
   - Or use YOLOv4-tiny for even better performance
"""