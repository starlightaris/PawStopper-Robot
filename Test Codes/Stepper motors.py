import RPi.GPIO as GPIO
import time

motor1_pins = [5, 6, 12, 13]  # IN1–IN4
motor2_pins = [17, 18, 27, 22]   # IN5–IN8

halfstep_seq = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1]
]

GPIO.setmode(GPIO.BCM)
for pin in motor1_pins + motor2_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

def rotate_motor(pins, delay, steps, direction=1):
    sequence = halfstep_seq if direction == 1 else list(reversed(halfstep_seq))
    for _ in range(steps):
        for halfstep in sequence:
            for pin in range(4):
                GPIO.output(pins[pin], halfstep[pin])
            time.sleep(delay)

try:
    while True:
        print("Motor 1 Forward")
        rotate_motor(motor1_pins, 0.002, 512)
        print("Motor 2 Backward")
        rotate_motor(motor2_pins, 0.002, 512, direction=-1)
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
