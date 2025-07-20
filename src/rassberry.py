# Imports
import cv2
import RPi.GPIO as GPIO
import serial
import time

# Open serial port to Arduino
arduino = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)  # Change the port name
time.sleep(2)  # Wait for Arduino to reset

# Message sends to Arduino
def send_coordinates(center_frame, center_object):
    data = f"{center_frame[0]},{center_frame[1]},{center_object[0]},{center_object[1]}\n"
    arduino.write(data.encode())
    print(f"Sent to Arduino: {data.strip()}")


GPIO.setmode(GPIO.BCM)
# Setup relay
RELAY_PIN = 17
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW) # Initially off

def trigger_relay(duration=5):
    print("Relay ON")
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(RELAY_PIN, GPIO.LOW)
    print("Relay OFF")

# Setup the alarm system
ALARM_PIN = 18
GPIO.setup(ALARM_PIN, GPIO.OUT)

def alarm_on():
    GPIO.output(ALARM_PIN, GPIO.HIGH)

def alarm_off():
    GPIO.output(ALARM_PIN, GPIO.LOW)

# Load class names from coco names to a list named classNames
classNames = []
classFile = "/home/eutech/Desktop/PawStopper Robot/Object_Detection_Files/coco.names"
with open(classFile, "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

# Load model config and weights
configPath = "/home/eutech/Desktop/PawStopper Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/eutech/Desktop/PawStopper Robot/Object_Detection_Files/frozen_inference_graph.pb"

# Initialize detection model
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Function to detect objects and mark centers
def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)  #classId of the object, confs: thresholder value, bbox: (x, y, width, height)
    objectInfo = []

    if len(objects) == 0:
        objects = classNames

    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                # Compute center of bounding box
                center_x = box[0] + box[2] // 2
                center_y = box[1] + box[3] // 2

                # Append to print
                objectInfo.append([box, className, round(confidence * 100, 2), (center_x, center_y)])

                if draw:
                    # Draw rectangle
                    cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                    
                    # Draw class name
                    cv2.putText(img, className.upper(), (box[0]+10, box[1]+30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    
                    # Draw confidence
                    cv2.putText(img, str(round(confidence * 100, 2)) + '%', (box[0]+200, box[1]+30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    
                    # Draw small circle at the center
                    cv2.circle(img, (center_x, center_y), radius=5, color=(0, 255, 0), thickness=-1) #Green dot

    return img, objectInfo


# Main program
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(3, 640) #width
    cap.set(4, 480) #height

    # Optional: print frame size
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Video feed size: {frame_width}x{frame_height}")
    
    # Thresholder for the alarm system
    AREA_THRESHOLD = 15000 # tune for need

    try:
        # Setup the relay to cooldown
        last_trigger_time = 0
        COOLDOWN_SECONDS = 10
        
        while True:
            success, img = cap.read()
            if not success:
                print("Failed to read from camera.")
                break

            # Draw center of the video feed
            frame_center = (frame_width // 2, frame_height // 2)
            cv2.circle(img, frame_center, 6, (0, 0, 255), -1)  # Red dot
        
            # Detect objects and mark centers of their bounding boxes
            result, objectInfo = getObjects(img, 0.45, 0.2, objects=['cat'])
        
            # Set alarm state
            alarm_triggered = False
            alarm_off()
        
            # Setup to find the biggest bbox
            biggest_box = None
            max_area = 0
           
            # Print object info || trigger alarm
            for obj in objectInfo:
                box, name, confidence, center = obj
                print(f"Detected: {name.upper()} | Confidence: {confidence}% | Box: {box} | Center: {center}")
                bbox_area = box[2] * box[3]
                print(f"BBox Area: {bbox_area}")
            
                if bbox_area > AREA_THRESHOLD:
                    alarm_triggered = True
                    if bbox_area > max_area:
                        max_area = bbox_area
                        biggest_box = box

            if alarm_triggered and biggest_box is not None:
                bbox_center = (biggest_box[0] + biggest_box[2] // 2, biggest_box[1] + biggest_box[3] // 2)
            
                # Send to Arduino
                send_coordinates(frame_center, bbox_center)
            
                alarm_on()
            else:
                alarm_off()
        
            if arduino.in_waiting > 0:
                    msg = arduino.readline().decode().strip()
                    if msg:
                        print(f"From Arduino: {msg}")
                        if msg == "ALIGNED":
                            current_time = time.time()
                            if current_time - last_trigger_time > COOLDOWN_SECONDS:
                                trigger_relay(5)
                                last_trigger_time = current_time
        
            cv2.imshow("Output", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except UnicodeDecodeError:
        print("Invalid serial data received")
    
    except KeyboardInterrupt:
        print("Interrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        arduino.close()
