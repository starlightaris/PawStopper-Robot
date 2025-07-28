"""
PawStopper Robot - Main Control Script
This script handles object detection, serial communication with Arduino, and relay control
for the PawStopper robot system.
"""

import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading
from typing import Tuple, List, Optional

# Constants
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
SERIAL_TIMEOUT = 1
RELAY_PIN = 17
ALARM_PIN = 18
AREA_THRESHOLD = 15000  # Minimum area to trigger detection
COOLDOWN_SECONDS = 10   # Time between relay triggers
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DETECTION_CONFIDENCE = 0.45
NMS_THRESHOLD = 0.2
TARGET_OBJECT = 'cell phone'

# Initialize serial connection to Arduino
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
    time.sleep(2)  # Wait for Arduino to reset
except serial.SerialException as e:
    print(f"Failed to open serial port: {e}")
    raise

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.setup(ALARM_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.HIGH)  # Ensure relay is off initially

def send_coordinates(center_frame: Tuple[int, int], center_object: Tuple[int, int]) -> None:
    """
    Send coordinate data to Arduino for object tracking.
    
    Args:
        center_frame: Tuple of (x, y) coordinates for frame center
        center_object: Tuple of (x, y) coordinates for object center
    """
    data = f"{center_frame[0]},{center_frame[1]},{center_object[0]},{center_object[1]}\n"
    arduino.write(data.encode())
    print(f"Sent to Arduino: {data.strip()}")

def trigger_relay(duration: int = 5) -> None:
    """
    Trigger the relay for a specified duration.
    
    Args:
        duration: Time in seconds to keep the relay on
    """
    print("Relay ON")
    GPIO.output(RELAY_PIN, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    print("Relay OFF")

def trigger_relay_async(duration: int = 5) -> None:
    """
    Trigger the relay asynchronously to prevent video feed lag.
    
    Args:
        duration: Time in seconds to keep the relay on
    """
    threading.Thread(target=trigger_relay, args=(duration,), daemon=True).start()

def alarm_on() -> None:
    """Turn on the alarm system."""
    GPIO.output(ALARM_PIN, GPIO.HIGH)

def alarm_off() -> None:
    """Turn off the alarm system."""
    GPIO.output(ALARM_PIN, GPIO.LOW)

# Model paths
MODEL_DIR = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files"
CLASS_FILE = f"{MODEL_DIR}/coco.names"
CONFIG_FILE = f"{MODEL_DIR}/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
WEIGHTS_FILE = f"{MODEL_DIR}/frozen_inference_graph.pb"

def load_class_names() -> List[str]:
    """
    Load and return the class names from COCO dataset.
    
    Returns:
        List of class names
    """
    try:
        with open(CLASS_FILE, "rt") as f:
            return f.read().rstrip("\n").split("\n")
    except FileNotFoundError as e:
        print(f"Error loading class names: {e}")
        raise

# Load class names
classNames = load_class_names()

def initialize_model() -> cv2.dnn_DetectionModel:
    """
    Initialize and configure the object detection model.
    
    Returns:
        Configured detection model
    """
    model = cv2.dnn_DetectionModel(WEIGHTS_FILE, CONFIG_FILE)
    model.setInputSize(320, 320)
    model.setInputScale(1.0 / 127.5)
    model.setInputMean((127.5, 127.5, 127.5))
    model.setInputSwapRB(True)
    return model

# Initialize detection model
net = initialize_model()

def detect_objects(
    img: cv2.Mat,
    confidence_threshold: float,
    nms_threshold: float,
    draw: bool = True,
    target_objects: Optional[List[str]] = None
) -> Tuple[cv2.Mat, List]:
    """
    Detect objects in the image and optionally draw detection visualization.
    
    Args:
        img: Input image
        confidence_threshold: Confidence threshold for detection
        nms_threshold: Non-maximum suppression threshold
        draw: Whether to draw detection visualization
        target_objects: List of object classes to detect, or None for all classes
    
    Returns:
        Tuple of (annotated image, list of detected object information)
    """
    target_objects = target_objects or classNames
    object_info = []
    
    # Detect objects
    class_ids, confidences, boxes = net.detect(
        img,
        confThreshold=confidence_threshold,
        nmsThreshold=nms_threshold
    )
    
    if len(class_ids) != 0:
        for class_id, confidence, box in zip(
            class_ids.flatten(),
            confidences.flatten(),
            boxes
        ):
            class_name = classNames[class_id - 1]
            if class_name in target_objects:
                # Calculate center of bounding box
                center_x = box[0] + box[2] // 2
                center_y = box[1] + box[3] // 2
                center = (center_x, center_y)
                
                # Store detection info
                confidence_percent = round(confidence * 100, 2)
                object_info.append([box, class_name, confidence_percent, center])
                
                if draw:
                    _draw_detection(img, box, class_name, confidence_percent, center)
    
    return img, object_info

def _draw_detection(
    img: cv2.Mat,
    box: Tuple[int, int, int, int],
    class_name: str,
    confidence: float,
    center: Tuple[int, int]
) -> None:
    """
    Draw detection visualization on the image.
    
    Args:
        img: Image to draw on
        box: Bounding box coordinates (x, y, w, h)
        class_name: Name of detected class
        confidence: Detection confidence percentage
        center: Center point coordinates
    """
    # Draw bounding box
    cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
    
    # Draw class name
    cv2.putText(img, class_name.upper(), (box[0]+10, box[1]+30),
                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    
    # Draw confidence
    cv2.putText(img, f"{confidence}%", (box[0]+200, box[1]+30),
                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    
    # Draw center point
    cv2.circle(img, center, radius=5, color=(0, 255, 0), thickness=-1)


def initialize_camera() -> Tuple[cv2.VideoCapture, int, int]:
    """
    Initialize and configure the video capture.
    
    Returns:
        Tuple of (VideoCapture object, frame width, frame height)
    """
    cap = cv2.VideoCapture(0)
    cap.set(3, FRAME_WIDTH)
    cap.set(4, FRAME_HEIGHT)
    
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Video feed size: {frame_width}x{frame_height}")
    
    return cap, frame_width, frame_height

def process_frame(
    img: cv2.Mat,
    frame_center: Tuple[int, int]
) -> Tuple[Optional[Tuple[int, int]], bool]:
    """
    Process a single frame for object detection and tracking.
    
    Args:
        img: Input frame
        frame_center: Center coordinates of the frame
    
    Returns:
        Tuple of (object center coordinates or None, alarm trigger status)
    """
    # Draw frame center
    cv2.circle(img, frame_center, 6, (0, 0, 255), -1)
    
    # Detect objects
    _, object_info = detect_objects(
        img,
        DETECTION_CONFIDENCE,
        NMS_THRESHOLD,
        objects=[TARGET_OBJECT]
    )
    
    # Take the first object above threshold
    for box, name, confidence, center in object_info:
        bbox_area = box[2] * box[3]
        print(f"Detected: {name.upper()} | Confidence: {confidence}% | Box: {box} | Center: {center}")
        print(f"BBox Area: {bbox_area}")
        
        if bbox_area > AREA_THRESHOLD:
            # Use the first object that meets the threshold
            object_center = (
                box[0] + box[2] // 2,
                box[1] + box[3] // 2
            )
            return object_center, True
            
    # No suitable object found
    
    return None, False

# Main program
if __name__ == "__main__":

    try:
        # Initialize camera and get frame dimensions
        cap, frame_width, frame_height = initialize_camera()
        last_trigger_time = 0
        
        while True:
            # Capture frame
            success, img = cap.read()
            if not success:
                print("Failed to read from camera.")
                break

            frame_center = (frame_width // 2, frame_height // 2)
            object_center, alarm_triggered = process_frame(img, frame_center)

            if alarm_triggered and object_center:
                # Object detected, handle tracking and response
                print(f"Sending coordinates to Arduino: FrameCenter={frame_center} ObjectCenter={object_center}")
                send_coordinates(frame_center, object_center)
                alarm_on()

                # Check alignment and trigger response
                if arduino.in_waiting > 0:
                    msg = arduino.readline().decode().strip()
                    print(f"From Arduino: {msg}")
                    if msg == "ALIGNED":
                        current_time = time.time()
                        if current_time - last_trigger_time > COOLDOWN_SECONDS:
                            trigger_relay_async(5)
                            last_trigger_time = current_time
            else:
                # No object detected, maintain passive scanning
                alarm_off()
                arduino.reset_input_buffer()
                arduino.write(b'SCAN\n')
                print("Passive scanning...")
                time.sleep(0.1)

            # Display and handle user input
            cv2.imshow("Output", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('h'):
                arduino.write(b'HOME\n')
                print("Sent HOME command to Arduino")


    except UnicodeDecodeError as e:
        print(f"Invalid serial data received: {e}")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    except Exception as e:
        print(f"Unexpected error occurred: {e}")
    
    finally:
        print("Cleaning up resources...")
        try:
            GPIO.output(RELAY_PIN, GPIO.HIGH)  # Ensure relay is off
            GPIO.cleanup()
            cap.release()
            cv2.destroyAllWindows()
            arduino.close()
        except Exception as e:
            print(f"Error during cleanup: {e}")

