# PawStopper Robot 🐾💦

An automated **smart pet deterrent system** built using **Raspberry Pi + Arduino**, computer vision, and stepper motors. The system detects animals (cats/dogs) using a camera, tracks them in real time, aligns a water gun using stepper motors, and triggers a relay to spray water as a humane deterrent.

---

## Project Overview

**Goal:**
Prevent pets from entering restricted areas by automatically detecting and tracking them, then activating a water spray when aligned.

**Key Features:**

* Real-time object detection using OpenCV (SSD MobileNet)
* Pan–tilt tracking using two 28BYJ stepper motors
* Arduino-based motor control with L293D motor shield
* Raspberry Pi vision + decision logic
* Relay-controlled water gun
* Alignment-based firing (prevents random spraying)
* Passive scan mode (180° sweep)
* Persistent home-position logic
* Cooldown & safety logic

---

## System Architecture

```
Camera ──> Raspberry Pi (Python + OpenCV)
            │
            │  (x,y coordinates via Serial)
            ▼
         Arduino (Stepper Control)
            │
            ├── Pan Stepper (X axis)
            ├── Tilt Stepper (Y axis)
            └── Relay (Water Gun)
```

---

## Hardware Used

* Raspberry Pi 4
* Arduino Uno
* L293D Motor Driver Shield
* 2 × 28BYJ-48 Stepper Motors (Pan & Tilt)
* USB Camera
* Relay Module (with level shifter)
* DIY Water gun / (water pump + clear pvc tubes)
* External 5V power supply (for motors)

---

## Software Stack

### Raspberry Pi (Python)

* OpenCV (Object Detection)
* pySerial (Arduino communication)
* RPi.GPIO (Relay & alarm control)

### Arduino (C++)

* AFMotor library
* Stepper motor control
* Serial command parser

---

## How It Works (Logic Flow)

1. Camera captures live video as it sweeps 180°
2. OpenCV detects target object (cat/dog)
3. Bounding box center is calculated
4. Frame center & object center sent to Arduino
5. Arduino moves steppers until error is within tolerance
6. Arduino sends `ALIGNED`
7. Raspberry Pi triggers relay (water spray)
8. Cooldown prevents continuous firing

---

## Serial Protocol

**Raspberry Pi → Arduino**

```
frameX,frameY,objectX,objectY
```

**Arduino → Raspberry Pi**

```
ErrorX: <val> ErrorY: <val>
ALIGNED
```

---

## Project Structure

```
PawStopper-Robot/
│
├── src/
│   ├── raspberry.py        # Main vision & control script
│   ├── arduino.ino         # Stepper + alignment logic
│
├── Object_Detection_Files/
│   ├── coco.names
│   ├── frozen_inference_graph.pb
│   ├── ssd_mobilenet.pbtxt
│
├── README.md
```

---

## Setup Instructions (High Level)

1. Flash Arduino with provided `.ino` code
2. Install Python dependencies on Raspberry Pi

   ```bash
   pip3 install opencv-python pyserial
   ```
3. Connect camera, Arduino (USB), motors, and relay
4. Update serial port in Python code (`/dev/ttyUSB0`)
5. Run:

   ```bash
   python3 raspberry.py
   ```

---

## Known Limitations

* No physical limit switches (position is relative)
* Requires good lighting for reliable detection
* Camera-based distance estimation only

---

## Safety Notes

* This system uses **water only** (non-harmful)
* Do NOT use high-pressure pumps
* Always power motors separately
* Test with non-living objects first

---

## Future Improvements

* Multi-target tracking
* Distance filtering improvements
* Web dashboard / logging

---

## Team Notes

Built as a university robotics / IoT project.

> "Works better than shouting at the cat." 😼

---

## License

Educational / Academic Use Only
