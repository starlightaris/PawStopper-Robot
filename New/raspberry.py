# Imports
import cv2
import RPi.GPIO as GPIO
import serial
import time
import threading

# Stepper Motor Controller Class
class StepperController:
    def __init__(self, arduino_serial):
        self.arduino = arduino_serial
        self.current_pos_x = 0
        self.current_pos_y = 0
        self.tolerance = 15
        self.track_step_size = 5
        self.is_homing = False
        self.home_cooldown_active = False
        
    def send_step_command(self, axis, direction, steps):
        """Send step command to Arduino and wait for confirmation"""
        cmd = f"STEP {axis} {direction} {steps}\n"
        
        # Clear any pending data before sending command
        self.arduino.reset_input_buffer()
        time.sleep(0.05)  # Small delay to ensure buffer is clear
        
        self.arduino.write(cmd.encode())
        print(f"Sent: {cmd.strip()}")
        
        # Wait for acknowledgment
        start_time = time.time()
        responses_received = []
        
        while time.time() - start_time < 3.0:  # Increased timeout to 3 seconds
            if self.arduino.in_waiting > 0:
                try:
                    response = self.arduino.readline().decode().strip()
                    responses_received.append(response)
                    print(f"Arduino response: '{response}'")
                    
                    if response == f"{axis}_OK":
                        return True
                    elif response in ["X_OK", "Y_OK"]:
                        print(f"Warning: Expected {axis}_OK but got {response}")
                        return response == f"{axis}_OK"
                except UnicodeDecodeError:
                    print("Warning: Received malformed data from Arduino")
                    continue
            time.sleep(0.01)  # Small delay to prevent busy waiting
        
        print(f"Timeout waiting for {axis}_OK acknowledgment")
        print(f"All responses received: {responses_received}")
        return False
    
    def step_x(self, direction, steps):
        """Move X axis and update position tracking"""
        print(f"Moving X axis: {direction} {steps} steps. Current pos: {self.current_pos_x}")
        if self.send_step_command("X", direction, steps):
            old_pos = self.current_pos_x
            if direction == "FORWARD":
                self.current_pos_x += steps
            else:
                self.current_pos_x -= steps
            print(f"X position updated: {old_pos} -> {self.current_pos_x}")
            return True
        else:
            print("X step command failed - position not updated")
            return False
    
    def step_y(self, direction, steps):
        """Move Y axis and update position tracking"""
        print(f"Moving Y axis: {direction} {steps} steps. Current pos: {self.current_pos_y}")
        if self.send_step_command("Y", direction, steps):
            old_pos = self.current_pos_y
            if direction == "FORWARD":
                self.current_pos_y += steps
            else:
                self.current_pos_y -= steps
            print(f"Y position updated: {old_pos} -> {self.current_pos_y}")
            return True
        else:
            print("Y step command failed - position not updated")
            return False
    
    def test_connection(self):
        """Test Arduino connection with a simple command"""
        print("Testing Arduino connection...")
        
        # Try a small movement to test communication
        test_success = self.send_step_command("X", "FORWARD", 1)
        if test_success:
            print("Connection test successful!")
            # Return to original position
            self.send_step_command("X", "BACKWARD", 1)
            return True
        else:
            print("Connection test failed!")
            return False
    
    def go_home(self):
        """Return both axes to logical zero position"""
        if self.is_homing:
            print("Homing already in progress...")
            return False
        
        print(f"Starting homing from position: ({self.current_pos_x}, {self.current_pos_y})")
        
        # Start homing in a separate thread to avoid blocking the main camera loop
        threading.Thread(target=self._go_home_thread, daemon=True).start()
        return True
    
    def _go_home_thread(self):
        """Private method to handle homing in a separate thread"""
        print(f"Going home from position: ({self.current_pos_x}, {self.current_pos_y})")
        
        # Test connection first
        if not self.test_connection():
            print("Cannot proceed with homing - Arduino communication failed")
            return False
        
        self.is_homing = True
        
        # Clear any pending data first
        self.arduino.reset_input_buffer()
        time.sleep(0.1)
        
        home_success = True
        
        # Home X axis
        if self.current_pos_x != 0:
            direction = "BACKWARD" if self.current_pos_x > 0 else "FORWARD"
            steps = abs(self.current_pos_x)
            print(f"Homing X axis: {steps} steps {direction}")
            
            # For large movements, break into smaller chunks
            max_chunk = 50  # Reduced chunk size for better reliability
            remaining_steps = steps
            retry_count = 0
            max_retries = 3
            
            while remaining_steps > 0 and home_success and retry_count < max_retries:
                chunk_steps = min(remaining_steps, max_chunk)
                print(f"Attempting X movement: {chunk_steps} steps {direction} (attempt {retry_count + 1})")
                
                if self.step_x(direction, chunk_steps):
                    remaining_steps -= chunk_steps
                    retry_count = 0  # Reset retry count on success
                    print(f"X homing progress: {steps - remaining_steps}/{steps} steps")
                else:
                    retry_count += 1
                    print(f"X step failed! Retry {retry_count}/{max_retries}")
                    if retry_count >= max_retries:
                        print("X homing failed after maximum retries!")
                        home_success = False
                        break
                    else:
                        time.sleep(0.5)  # Wait before retry
            
            if home_success and remaining_steps == 0:
                self.current_pos_x = 0
                print("X axis homed successfully")
            elif remaining_steps > 0:
                print(f"X homing incomplete! {remaining_steps} steps remaining")
                home_success = False
        
        # Home Y axis
        if self.current_pos_y != 0 and home_success:
            direction = "BACKWARD" if self.current_pos_y > 0 else "FORWARD"
            steps = abs(self.current_pos_y)
            print(f"Homing Y axis: {steps} steps {direction}")
            
            # For large movements, break into smaller chunks
            max_chunk = 50  # Reduced chunk size for better reliability
            remaining_steps = steps
            retry_count = 0
            max_retries = 3
            
            while remaining_steps > 0 and home_success and retry_count < max_retries:
                chunk_steps = min(remaining_steps, max_chunk)
                print(f"Attempting Y movement: {chunk_steps} steps {direction} (attempt {retry_count + 1})")
                
                if self.step_y(direction, chunk_steps):
                    remaining_steps -= chunk_steps
                    retry_count = 0  # Reset retry count on success
                    print(f"Y homing progress: {steps - remaining_steps}/{steps} steps")
                else:
                    retry_count += 1
                    print(f"Y step failed! Retry {retry_count}/{max_retries}")
                    if retry_count >= max_retries:
                        print("Y homing failed after maximum retries!")
                        home_success = False
                        break
                    else:
                        time.sleep(0.5)  # Wait before retry
            
            if home_success and remaining_steps == 0:
                self.current_pos_y = 0
                print("Y axis homed successfully")
            elif remaining_steps > 0:
                print(f"Y homing incomplete! {remaining_steps} steps remaining")
                home_success = False
        
        self.is_homing = False
        
        if home_success:
            print(f"Home complete. Final position: ({self.current_pos_x}, {self.current_pos_y})")
            print("Starting 5-second home cooldown...")
            # Start cooldown in a separate thread
            threading.Thread(target=self._home_cooldown_thread, daemon=True).start()
        else:
            print("Homing failed! Position may be inaccurate.")
            print("Consider using manual_reset_position() if you know the actual position.")

        return home_success
    
    def _home_cooldown_thread(self):
        """Private method to handle home cooldown in a separate thread"""
        self.home_cooldown_active = True
        for i in range(5, 0, -1):
            print(f"Home cooldown: {i} seconds remaining...")
            time.sleep(1)
        self.home_cooldown_active = False
        print("Home cooldown complete. Ready to resume operations.")
    
    def track_object(self, frame_center, object_center):
        """Track object by calculating error and moving steppers accordingly"""
        error_x = frame_center[0] - object_center[0]
        error_y = frame_center[1] - object_center[1]
        
        print(f"ErrorX: {error_x}, ErrorY: {error_y}")
        
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
        if abs(error_x) <= self.tolerance and abs(error_y) <= self.tolerance:
            print("ALIGNED")
            return True, moved
        
        return False, moved
    
    def scan_step(self, direction, step_size):
        """Perform one scan step"""
        # Don't scan if homing or in cooldown
        if self.is_homing or self.home_cooldown_active:
            return False
        return self.step_x(direction, step_size)
    
    def is_ready_for_operations(self):
        """Check if the stepper is ready for normal operations (not homing or in cooldown)"""
        return not (self.is_homing or self.home_cooldown_active)
    
    def get_status(self):
        """Get current status of the stepper controller"""
        if self.is_homing:
            return "HOMING"
        elif self.home_cooldown_active:
            return "HOME_COOLDOWN"
        else:
            return "READY"
    
    def get_position(self):
        """Get current position"""
        return (self.current_pos_x, self.current_pos_y)
    
    def emergency_stop(self):
        """Emergency stop - clear all buffers and stop homing"""
        self.is_homing = False
        self.home_cooldown_active = False
        self.arduino.reset_input_buffer()
        self.arduino.reset_output_buffer()
        print("Emergency stop activated - buffers cleared")
    
    def force_go_home_sync(self):
        """Force synchronous homing for shutdown - blocks until complete"""
        if self.is_homing:
            print("Waiting for current homing to complete...")
            # Wait for homing to finish
            while self.is_homing:
                time.sleep(0.1)
        
        # Now do a quick synchronous home
        print("Force homing for shutdown...")
        self._go_home_thread()
    
    def set_tracking_parameters(self, tolerance=None, step_size=None):
        """Update tracking parameters"""
        if tolerance is not None:
            self.tolerance = tolerance
            print(f"Tolerance updated to: {self.tolerance}")
        if step_size is not None:
            self.track_step_size = step_size
            print(f"Tracking step size updated to: {self.track_step_size}")
    
    def manual_reset_position(self, x=0, y=0):
        """Manually reset position counters (use if you know the actual position)"""
        self.current_pos_x = x
        self.current_pos_y = y
        print(f"Position manually reset to: ({x}, {y})")
    
    def get_position_info(self):
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

# Open serial port to Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

# Initialize stepper controller
stepper = StepperController(arduino)

# Wait for Arduino ready signal
print("Waiting for Arduino...")
start_time = time.time()
while time.time() - start_time < 5.0:  # 5 second timeout
    if arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        if "Arduino ready" in response:
            print("Arduino connected successfully!")
            break
else:
    print("Warning: Arduino ready signal not received, continuing anyway...")

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
scan_limit_steps = 1024 #1024 for 180 degree
scan_steps = 0
scan_step_size = 10 # 10 is smoother

# Cooldown before going home after losing object
home_cooldown = 5  # seconds
lost_since = None
last_countdown_second = None

# Add position display
def display_position_info(img, stepper):
    """Display current stepper position on the image"""
    pos_x, pos_y = stepper.get_position()
    status = stepper.get_status()
    
    cv2.putText(img, f"Pos: X={pos_x}, Y={pos_y}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(img, f"Status: {status}", (10, 55), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(img, f"Tolerance: {stepper.tolerance}", (10, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, f"Step Size: {stepper.track_step_size}", (10, 100), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Add control instructions
    cv2.putText(img, "Controls: Q=Quit, R=Reset, H=Home, P=Info", 
                (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

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
            
            # Display position information
            display_position_info(img, stepper)

            result, objectInfo = getObjects(img,0.45,0.2,objects=['cell phone'])  # change object name here

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
                
                # Use stepper controller to track object
                aligned, moved = stepper.track_object(frame_center, bbox_center)
                
                alarm_on()
                object_tracked = True
                lost_since = None

                # If aligned and cooldown period has passed, trigger relay
                if aligned and current_time - last_trigger_time > COOLDOWN_SECONDS:
                    trigger_relay_async(5)
                    last_trigger_time = current_time

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
                            stepper.go_home()
                            object_tracked = False
                            lost_since = None
                            scan_steps = 0
                else:
                    # Passive scan mode using stepper controller
                    # Only scan if stepper is ready (not homing or in cooldown)
                    if stepper.is_ready_for_operations():
                        if stepper.scan_step(scan_direction, scan_step_size):
                            print(f"Scanning... Direction: {scan_direction}, Steps: {scan_step_size}")
                            scan_steps += scan_step_size

                            if scan_steps >= scan_limit_steps:
                                scan_direction = "BACKWARD" if scan_direction=="FORWARD" else "FORWARD"
                                scan_steps = 0
                                print(f"Scan direction changed to: {scan_direction}")
                        else:
                            print("Scan step failed, retrying...")
                    else:
                        # Robot is homing or in cooldown, don't scan
                        status = stepper.get_status()
                        if status == "HOME_COOLDOWN":
                            print("Waiting for home cooldown to complete...")
                        elif status == "HOMING":
                            print("Homing in progress...")

            cv2.imshow("Output", img)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[INFO] Quitting. Sending robot to home position.")
                stepper.force_go_home_sync()  # Use synchronous version for shutdown
                break
            elif key == ord('r'):  # Reset position to (0,0)
                print("[INFO] Manually resetting position to (0,0)")
                stepper.manual_reset_position(0, 0)
            elif key == ord('h'):  # Go home
                print("[INFO] Manual home command")
                stepper.go_home()  # Use async version during operation
            elif key == ord('p'):  # Print position info
                info = stepper.get_position_info()
                print(f"[INFO] Current position info: {info}")


    except KeyboardInterrupt:
        print("Interrupted by user")
        stepper.force_go_home_sync()  # Use synchronous version for shutdown
    finally:
        print("Cleaning up...")
        stepper.emergency_stop()
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        arduino.close()
        print("Cleanup complete!")
