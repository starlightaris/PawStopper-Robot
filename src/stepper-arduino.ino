#include <AFMotor.h>

AF_Stepper stepperX(200, 2); // horizontal pan
AF_Stepper stepperY(200, 1); // vertical tilt

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  stepperX.setSpeed(50); 
  stepperY.setSpeed(50); 
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    static int scanSteps = 0;            // keep track of how far it scanned
    static int scanDirection = FORWARD;  // current scan direction
    const int SCAN_LIMIT = 100;          // adjust: number of steps before turning back

    if (data == "SCAN") {
      // Passive scan: sweep left and right
      stepperX.step(5, scanDirection, SINGLE);
      scanSteps += 5;

      if (scanSteps >= SCAN_LIMIT) {
        // reached end, reverse direction
        scanDirection = (scanDirection == FORWARD) ? BACKWARD : FORWARD;
        scanSteps = 0;
      }
    }
    else if (data == "HOME") {
      // Go back to center (move backward by SCAN_LIMIT, adjust as needed)
      stepperX.step(SCAN_LIMIT, BACKWARD, SINGLE);
      scanSteps = 0;
      scanDirection = FORWARD;
    }
    else {
      int sep1 = data.indexOf(',');
      int sep2 = data.indexOf(',', sep1+1);
      int sep3 = data.indexOf(',', sep2+1);

      if (sep1 > 0 && sep2 > sep1 && sep3 > sep2) {
        int frameX = data.substring(0, sep1).toInt();
        int frameY = data.substring(sep1+1, sep2).toInt();
        int objX = data.substring(sep2+1, sep3).toInt();
        int objY = data.substring(sep3+1).toInt();

        int errorX = frameX - objX;
        int errorY = frameY - objY;

        Serial.print("ErrorX: "); Serial.print(errorX);
        Serial.print(" ErrorY: "); Serial.println(errorY);

        int tolerance = 25; //15 thibbe

        if (abs(errorX) > tolerance) {
          if (errorX > 0) {
            stepperX.step(5, FORWARD, SINGLE);
          } else {
            stepperX.step(5, BACKWARD, SINGLE);
          }
        }

        if (abs(errorY) > tolerance) {
          if (errorY > 0) {
            stepperY.step(5, FORWARD, SINGLE);
          } else {
            stepperY.step(5, BACKWARD, SINGLE);
          }
        }

        if (abs(errorX) <= tolerance && abs(errorY) <= tolerance) {
          Serial.println("ALIGNED");
        }
      }
    }
  }
}
