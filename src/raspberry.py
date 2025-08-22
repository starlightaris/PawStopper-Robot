# =========================================
# PawStopper Robot - Clean Full Behavior
# =========================================
import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading

# -------------------------------
# Arduino Serial Setup
# -------------------------------
try:
    arduino = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    time.sleep(2)
    print("[INFO] Arduino connected on /dev/ttyAMA0")
except Exception as e:
    print(f"[ERROR] Failed to connect to Arduino: {e}")
    arduino = None

# -------------------------------
# GPIO Setup
# -------------------------------
GPIO.setmode(GPIO.BCM)

RELAY_PIN = 17
ALARM_PIN = 18
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.setup(ALARM_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW)
GPIO.output(ALARM_PIN, GPIO.LOW)

def trigger_relay(duration=3):
    print("[RELAY] ON")
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(RELAY_PIN, GPIO.LOW)
    print("[RELAY] OFF")

def alarm_on():
    GPIO.output(ALARM_PIN, GPIO.HIGH)

def alarm_off():
    GPIO.output(ALARM_PIN, GPIO.LOW)

# -------------------------------
# Object Detection Setup
# -------------------------------
classNames = []
with open("/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names", "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

TARGET_OBJECTS = ['cat', 'dog']
AREA_THRESHOLD = 15000
COOLDOWN_SECONDS = 10
OBJECT_LOST_TIMEOUT = 5

# -------------------------------
# Object Detection
# -------------------------------
def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    objectInfo = []
    if len(objects) == 0:
        objects = classNames
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                cx = box[0] + box[2] // 2
                cy = box[1] + box[3] // 2
                objectInfo.append([box, className, round(confidence * 100, 2), (cx, cy)])
                if draw:
                    cv2.rectangle(img, box, (0, 255, 0), 2)
                    cv2.putText(img, f"{className.upper()} {round(confidence*100,1)}%",
                                (box[0], box[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.circle(img, (cx, cy), 5, (0,255,0), -1)
    return img, objectInfo

# -------------------------------
# Arduino Communication
# -------------------------------
def send_coordinates(frame_center, target_center):
    if arduino is None: return
    data = f"{frame_center[0]},{frame_center[1]},{target_center[0]},{target_center[1]}\n"
    arduino.write(data.encode())
    print(f"[TX] {data.strip()}")

def read_from_arduino():
    if arduino and arduino.in_waiting > 0:
        try:
            msg = arduino.readline().decode().strip()
            if msg:
                print(f"[RX] {msg}")
                return msg
        except UnicodeDecodeError:
            print("[ERROR] Bad serial data")
    return None

# -------------------------------
# Main
# -------------------------------
def main():
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_center = (frame_w // 2, frame_h // 2)
    print(f"[INFO] Camera ready: {frame_w}x{frame_h}")

    last_trigger_time = 0
    detection_result = []
    object_last_seen = time.time()
    scanning = False
    homing = False
    status_text = "IDLE"

    # Threaded detection
    def detection_loop():
        nonlocal detection_result
        while True:
            success, img = cap.read()
            if not success: continue
            _, detection_result = getObjects(img.copy(), 0.45, 0.2, objects=TARGET_OBJECTS)

    threading.Thread(target=detection_loop, daemon=True).start()

    try:
        while True:
            success, img = cap.read()
            if not success: break

            cv2.circle(img, frame_center, 6, (0,0,255), -1)

            objects = list(detection_result)
            alarm_triggered = False
            biggest_box = None
            max_area = 0

            # --- TRACKING ---
            if objects:
                for box, name, conf, center in objects:
                    area = box[2] * box[3]
                    if area > AREA_THRESHOLD and area > max_area:
                        alarm_triggered = True
                        max_area = area
                        biggest_box = (box, center)

                if biggest_box:
                    _, target_center = biggest_box
                    send_coordinates(frame_center, target_center)
                    alarm_on()
                    object_last_seen = time.time()
                    scanning = False
                    homing = False
                    status_text = "TRACKING"
                else:
                    alarm_off()
                    status_text = "IDLE"
            else:
                alarm_off()

            # --- OBJECT LOST ---
            if time.time() - object_last_seen > OBJECT_LOST_TIMEOUT:
                if not homing:
                    print("[INFO] Object lost. Going home...")
                    homing = True
                    scanning = False
                    status_text = "HOMING"
                    send_coordinates(frame_center, frame_center)

            # --- INTERRUPT HOMING ---
            if homing and objects:
                print("[INFO] Object detected during homing. Aborting home, resuming tracking...")
                homing = False
                status_text = "TRACKING"

            # --- SCANNING MODE ---
            if not objects and not homing and (time.time() - object_last_seen > 2):
                if not scanning:
                    print("[INFO] Starting scan...")
                    scanning = True
                status_text = "SCANNING"
                # simple scan oscillation
                scan_offset = int(100 * (1 + (time.time() % 4 - 2)))
                target_scan = (frame_center[0] + scan_offset, frame_center[1])
                send_coordinates(frame_center, target_scan)

            # --- RELAY TRIGGER ---
            msg = read_from_arduino()
            if msg == "ALIGNED":
                now = time.time()
                if now - last_trigger_time > COOLDOWN_SECONDS:
                    trigger_relay(3)
                    last_trigger_time = now

            # --- STATUS OVERLAY ---
            cv2.putText(img, f"STATUS: {status_text}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.imshow("PawStopper Output", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] Quitting. Going home...")
                send_coordinates(frame_center, frame_center)
                time.sleep(2)
                break

    except KeyboardInterrupt:
        print("[INFO] Stopped by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        if arduino: arduino.close()

# -------------------------------
# Run
# -------------------------------
if __name__ == "__main__":
    main()
