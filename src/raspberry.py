# Imports
import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading

# Open serial port to Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

# Setup relay
GPIO.setmode(GPIO.BCM)
RELAY_PIN = 17
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.HIGH)  # Initially off; HIGH = OFF, LOW = ON (temp fix inverted)

def trigger_relay(duration=5):
    print("Relay ON")
    GPIO.output(RELAY_PIN, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    print("Relay OFF")

def trigger_relay_async(duration=5):
    threading.Thread(target=trigger_relay, args=(duration,), daemon=True).start()

# Alarm system
ALARM_PIN = 18
GPIO.setup(ALARM_PIN, GPIO.OUT)

def alarm_on():
    GPIO.output(ALARM_PIN, GPIO.HIGH)

def alarm_off():
    GPIO.output(ALARM_PIN, GPIO.LOW)

# Load object detection model
classNames = []
with open("/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names", "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

net = cv2.dnn_DetectionModel(
    "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb",
    "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
)
net.setInputSize(320, 320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)

# Function to detect objects and mark centers
def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    objectInfo = []
    if len(objects) == 0: objects = classNames
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                center = (box[0]+box[2]//2, box[1]+box[3]//2)
                objectInfo.append([box, className, round(confidence*100,2), center])
                if draw:
                    cv2.rectangle(img, box, (0,255,0),2)
                    cv2.putText(img, f"{className.upper()} {round(confidence*100,2)}%", 
                                (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.circle(img, center, 5, (0,255,0), -1)
    return img, objectInfo

# Tracking & scan state
object_tracked = False
AREA_THRESHOLD = 15000
COOLDOWN_SECONDS = 10
last_trigger_time = 0

scan_direction = "FORWARD"
scan_limit_steps = 100
scan_steps = 0
scan_step_size = 5

# Cooldown before going home after losing object
home_cooldown = 5  # seconds
lost_since = None

# File save state
last_save_time = 0
save_interval = 10  # seconds
scan_info_file = "/home/eutech/Desktop/PawStopper-Robot/scan_info.txt"

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    try:
        while True:
            success, img = cap.read()
            if not success: break

            frame_center = (frame_width//2, frame_height//2)
            cv2.circle(img, frame_center, 6, (0,0,255), -1)  # Red center dot

            result, objectInfo = getObjects(img,0.45,0.2,objects=['cup'])  # change object name here

            biggest_box = None
            max_area = 0
            alarm_triggered = False

            for obj in objectInfo:
                box, name, conf, center = obj
                area = box[2]*box[3]
                if area > AREA_THRESHOLD:
                    alarm_triggered = True
                    if area > max_area:
                        max_area = area
                        biggest_box = box

            current_time = time.time()

            if alarm_triggered and biggest_box is not None:
                bbox_center = (biggest_box[0]+biggest_box[2]//2, biggest_box[1]+biggest_box[3]//2)
                data = f"{frame_center[0]},{frame_center[1]},{bbox_center[0]},{bbox_center[1]}\n"
                arduino.write(data.encode())
                alarm_on()
                object_tracked = True
                lost_since = None

                if arduino.in_waiting>0:
                    msg = arduino.readline().decode().strip()
                    if msg=="ALIGNED" and current_time-last_trigger_time>COOLDOWN_SECONDS:
                        trigger_relay_async(5)
                        last_trigger_time=current_time

            else:
                alarm_off()
                arduino.reset_input_buffer()

                if object_tracked:
                    if lost_since is None:
                        lost_since = current_time
                        print("Object lost. Starting countdown before going home...")
                        last_countdown_second = home_cooldown
                    else:
                        elapsed = int(current_time - lost_since)
                        remaining = home_cooldown - elapsed
                        if remaining != last_countdown_second and remaining > 0:
                            print(remaining)
                            last_countdown_second = remaining
                        elif remaining <= 0:
                            print("Going home...")
                            arduino.write(b'HOME\n')
                            object_tracked = False
                            lost_since = None
                            scan_steps = 0
                else:
                    # Passive scan mode
                    step_cmd = f"STEP X {scan_direction} {scan_step_size}\n"
                    arduino.write(step_cmd.encode())
                    print(f"Scanning... Sent: {step_cmd.strip()}")
                    scan_steps += scan_step_size

                    # Save scan state every 10s
                    if current_time - last_save_time > save_interval:
                        with open(scan_info_file, "w") as f:
                            f.write(f"{scan_direction},{scan_steps}\n")
                        print(f"[INFO] Saved scan position: Direction={scan_direction} Steps={scan_steps}")
                        last_save_time = current_time

                    if scan_steps >= scan_limit_steps:
                        scan_direction = "BACKWARD" if scan_direction=="FORWARD" else "FORWARD"
                        scan_steps = 0

            cv2.imshow("Output", img)
            if cv2.waitKey(1)&0xFF == ord('q'): break

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        arduino.close()
