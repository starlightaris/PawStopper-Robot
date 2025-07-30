# Imports
import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading


# Component setup
# Open serial port to Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)    # The port, baud rate(should be the same as the arduino), waiting time out
time.sleep(2)  # Wait for Arduino to reset

# GPIO setup
GPIO.setmode(GPIO.BCM)

RELAY_PIN = 17  # Setting up the relay
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.HIGH)   # Initially off; HIGH = OFF, LOW = ON (temp fix inverted)--pavani

ALARM_PIN = 18  # Setting up the alarm system(Buzzer)
GPIO.setup(ALARM_PIN, GPIO.OUT)


# Image detection setup
# Loads class names from coco names to a list named classNames
classNames = []
classFile = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names"
with open(classFile, "rt") as f:    # rt: read as texts(Strings)
    classNames = f.read().rstrip("\n").split("\n")

# Loads model configs and weights
configPath = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"

# Initialize detection model
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)  # Resizes the input image to 320 x 320 pixels before passing it to the neural network. Higher resolution: better accuracy but become slower
net.setInputScale(1.0 / 127.5)  # Normalizes pixel values from [0, 255] range to [0, 2] range
net.setInputMean((127.5, 127.5, 127.5)) # Subtracts a mean value from each color channel (R, G, B)
net.setInputSwapRB(True)    # Swap color channels from BGR to RGB


# Functions
# Function: Sends the center codes to the arduino
def send_coordinates(frame_center, object_center):
    data = f"{frame_center[0]},{frame_center[1]},{object_center[0]},{object_center[1]}\n"   # 0:x cordination, 1:y cordination
    arduino.write(data.encode())    # Encode convert the string to binary
    print(f"Sent to Arduino: {data.strip()}")   # Prints the message to the rasberry console as well. .strip() remove the /n at the end

# Function: Triggers the relay
def trigger_relay(duration=5):
    GPIO.output(RELAY_PIN, GPIO.LOW)
    print("Relay ON")
    time.sleep(duration)
    GPIO.output(RELAY_PIN, GPIO.HIGH)   # Change HIGH and LOW if the relay misbehaves
    print("Relay OFF")
    
# Function: Fix for video feed lag/delay--pavani
def trigger_relay_async(duration=5):
    threading.Thread(target=trigger_relay, args=(duration,), daemon=True).start()   # A daemon thread will execute and be killed automatically

# Function: Turn alarm on
def alarm_on():
    GPIO.output(ALARM_PIN, GPIO.HIGH)

# Function: Turn alarm on
def alarm_off():
    GPIO.output(ALARM_PIN, GPIO.LOW)

# Function: Detects objects and marks the centers
def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)  # classId of the object, confs: thresholder value, bbox: (x, y, width, height)
    objectInfo = []

    if len(objects) == 0:
        objects = classNames # If not defined, this detects everything

    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                # Compute center of bounding box (object_center values)
                center_x = box[0] + box[2] // 2 # 0:x position, 2:width
                center_y = box[1] + box[3] // 2 # 1:y position, 3:height

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
                    cv2.circle(img, (center_x, center_y), radius=5, color=(0, 255, 0), thickness=-1)    # Green dot

    return img, objectInfo


# Main program
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(3, 640) # width
    cap.set(4, 480) # height

    # Prints frame size
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera On; Feed Size: {frame_width}x{frame_height}")
    
    # Check arduino connection
    print("Checking Arduino connection...")
    arduino.write(b'PING\n')  # Send a simple message to Arduino

    timeout_start = time.time()
    timeout_limit = 5
    connection_established = False
    
    # Thresholder for the alarm system
    AREA_THRESHOLD = 15000  # tune for need

    try:
        # Setup relay cooldown
        last_trigger_time = 0
        COOLDOWN_SECONDS = 10
        
        while time.time() - timeout_start < timeout_limit:
            if arduino.in_waiting > 0:
                msg = arduino.readline().decode().strip()
                if msg == "PONG":
                    print("Connection stable: Arduino ready to receive coordinates.")
                    connection_established = True
                    break
                else:
                    print(f"Unexpected response from Arduino: {msg}")
                    break
            time.sleep(0.1)

        if not connection_established:
            print("No response from Arduino. Please check the connection and restart.")
            arduino.close()
            GPIO.cleanup()
            cap.release()
            exit()
        
        while True:
            success, img = cap.read()
            if not success:
                print("Failed to read from camera.")
                break

            # Draw center of the video feed
            frame_center = (frame_width // 2, frame_height // 2)
            cv2.circle(img, frame_center, 6, (0, 0, 255), -1)  # Red dot

            # Detect objects and mark centers of their bounding boxes
            result, objectInfo = getObjects(img, 0.45, 0.2, objects=['cell phone'])

            # Find the biggest bbox (for objects above area threshold)
            biggest_box = None
            max_area = 0
            alarm_triggered = False

            for obj in objectInfo:
                box, name, confidence, center = obj
                bbox_area = box[2] * box[3]
                print(f"Detected: {name.upper()} | Confidence: {confidence}% | Box: {box} | Center: {center}")
                print(f"BBox Area: {bbox_area}")

                if bbox_area > AREA_THRESHOLD:
                    alarm_triggered = True
                    if bbox_area > max_area:
                        max_area = bbox_area
                        biggest_box = box

            if alarm_triggered and biggest_box is not None:
                # Object detected now, send coords
                bbox_center = (biggest_box[0] + biggest_box[2] // 2, biggest_box[1] + biggest_box[3] // 2)
                print(f"Sending coordinates to Arduino: FrameCenter={frame_center} ObjectCenter={bbox_center}")
                send_coordinates(frame_center, bbox_center)

                alarm_on()

                # Check if Arduino replies "ALIGNED"
                if arduino.in_waiting > 0:
                    msg = arduino.readline().decode().strip()
                    print(f"From Arduino: {msg}")
                    if msg == "ALIGNED":
                        current_time = time.time()
                        if current_time - last_trigger_time > COOLDOWN_SECONDS:
                            trigger_relay_async(5)
                            last_trigger_time = current_time
            else:
                # No object detected above threshold
                alarm_off()
                arduino.reset_input_buffer()  # clear any old "ALIGNED" messages

                # Passive scan
                arduino.write(b'SCAN\n')
                print("Passive scanning...")
                time.sleep(0.1)  # adjust delay if needed

            cv2.imshow("Output", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            elif cv2.waitKey(1) & 0xFF == ord('h'):
                arduino.write(b'HOME\n')
                print("Sent HOME command to Arduino")


    except UnicodeDecodeError:
        print("Invalid serial data received")
    
    except KeyboardInterrupt:
        print("Interrupted by user")
    
    finally:
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        arduino.close()

