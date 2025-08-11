#include <AFMotor.h>

AF_Stepper stepperX(48, 2); // horizontal pan
AF_Stepper stepperY(48, 1); // vertical tilt

// Speed constants for different modes
const int SCAN_SPEED = 50;     // RPM for scanning
const int TRACK_SPEED = 20;    // RPM for tracking (slower for smooth alignment)

// Mode tracking
String currentMode = "SCAN";   // Default mode

void setup() {
  Serial.begin(9600);
  stepperX.setSpeed(SCAN_SPEED);  // Start with scan speed
  stepperY.setSpeed(SCAN_SPEED); 
  Serial.println("Arduino ready");
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // Handle MODE command: MODE SCAN or MODE TRACK
    if (data.startsWith("MODE")) {
      int firstSpace = data.indexOf(' ');
      if (firstSpace > 0) {
        String newMode = data.substring(firstSpace + 1);
        
        // Only change speed if mode is different
        if (newMode != currentMode) {
          currentMode = newMode;
          
          if (currentMode == "SCAN") {
            stepperX.setSpeed(SCAN_SPEED);
            stepperY.setSpeed(SCAN_SPEED);
            Serial.println("MODE_SCAN_OK");
          }
          else if (currentMode == "TRACK") {
            stepperX.setSpeed(TRACK_SPEED);
            stepperY.setSpeed(TRACK_SPEED);
            Serial.println("MODE_TRACK_OK");
          }
          else {
            Serial.println("MODE_ERROR");
          }
        } else {
          // Mode unchanged, send acknowledgment anyway
          Serial.println("MODE_" + currentMode + "_OK");
        }
      }
    }
    // Expected format: STEP X FORWARD 5 or STEP Y BACKWARD 10
    else if (data.startsWith("STEP")) {
      int firstSpace = data.indexOf(' ');
      int secondSpace = data.indexOf(' ', firstSpace + 1);
      int thirdSpace = data.indexOf(' ', secondSpace + 1);

      if (firstSpace > 0 && secondSpace > firstSpace && thirdSpace > secondSpace) {
        String axis = data.substring(firstSpace + 1, secondSpace);
        String dirStr = data.substring(secondSpace + 1, thirdSpace);
        int steps = data.substring(thirdSpace + 1).toInt();

        int dir = (dirStr == "FORWARD") ? FORWARD : BACKWARD;

        if (axis == "X") {
          stepperX.step(steps, dir, SINGLE);
          Serial.println("X_OK");
        }
        else if (axis == "Y") {
          stepperY.step(steps, dir, SINGLE);
          Serial.println("Y_OK");
        }
      }
    }
  }
}
