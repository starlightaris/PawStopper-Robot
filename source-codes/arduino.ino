#include <AFMotor.h>

// Define stepper motors: 48 steps/rev
AF_Stepper stepperX(48, 2); // horizontal pan
AF_Stepper stepperY(48, 1); // vertical tilt

// Scan parameters
const int SCAN_STEP_SIZE = 5;
const int SCAN_LIMIT = 2048; // total sweep limit
const int TOLERANCE = 25;   // alignment precision

// State variables
int scanSteps = 0;
int scanDirection = FORWARD;

void setup() {
  Serial.begin(9600);
  stepperX.setSpeed(50);
  stepperY.setSpeed(50);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // === Scan Command ===
    if (data == "SCAN") {
      stepperX.step(SCAN_STEP_SIZE, scanDirection, SINGLE);
      scanSteps += SCAN_STEP_SIZE;

      if (scanSteps >= SCAN_LIMIT) {
        scanDirection = (scanDirection == FORWARD) ? BACKWARD : FORWARD;
        scanSteps = 0;
      }
    }

    // === Home Command ===
    else if (data == "HOME") {
      stepperX.step(SCAN_LIMIT, BACKWARD, SINGLE); // adjust if your center is not truly BACKWARD
      scanSteps = 0;
      scanDirection = FORWARD;
    }

    // === Coordinate Alignment ===
    else {
      int sep1 = data.indexOf(',');
      int sep2 = data.indexOf(',', sep1 + 1);
      int sep3 = data.indexOf(',', sep2 + 1);

      if (sep1 > 0 && sep2 > sep1 && sep3 > sep2) {
        int frameX = data.substring(0, sep1).toInt();
        int frameY = data.substring(sep1 + 1, sep2).toInt();
        int objX = data.substring(sep2 + 1, sep3).toInt();
        int objY = data.substring(sep3 + 1).toInt();

        int errorX = frameX - objX;
        int errorY = frameY - objY;

        Serial.print("ErrorX: "); Serial.print(errorX);
        Serial.print("  ErrorY: "); Serial.println(errorY);

        // Adjust X axis (horizontal)
        if (abs(errorX) > TOLERANCE) {
          stepperX.step(SCAN_STEP_SIZE, errorX > 0 ? FORWARD : BACKWARD, SINGLE);
        }

        // Adjust Y axis (vertical)
        if (abs(errorY) > TOLERANCE) {
          stepperY.step(SCAN_STEP_SIZE, errorY > 0 ? FORWARD : BACKWARD, SINGLE);
        }

        // Aligned
        if (abs(errorX) <= TOLERANCE && abs(errorY) <= TOLERANCE) {
          Serial.println("ALIGNED");
        }
      }
    }
  }
}
