import RPi.GPIO as GPIO
import time

# Constants
RELAY_PIN = 3  # Note: GPIO3 (physical pin 5) on the Pi

# Setup
GPIO.setmode(GPIO.BCM)      # Use Broadcom pin-numbering scheme
GPIO.setup(RELAY_PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(RELAY_PIN, GPIO.HIGH)  # Turn relay ON
        time.sleep(1)                    # Wait 500ms
        GPIO.output(RELAY_PIN, GPIO.LOW)   # Turn relay OFF
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Program stopped by User")
finally:
    GPIO.cleanup()  # Reset GPIO pins when exiting
