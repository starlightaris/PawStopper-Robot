#include <AFMotor.h>

AF_Stepper stepperX(48, 2); // horizontal pan
AF_Stepper stepperY(48, 1); // vertical tilt

// Track current logical position
long currentPosX = 0;
long currentPosY = 0;

void setup() {
  Serial.begin(9600);
  stepperX.setSpeed(50);  //RPM
  stepperY.setSpeed(50); 
  Serial.println("Arduino ready");
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    if (data.startsWith("STEP")) {
      // Expected format: STEP X FORWARD 5
      int firstSpace = data.indexOf(' ');
      int secondSpace = data.indexOf(' ', firstSpace + 1);
      int thirdSpace = data.indexOf(' ', secondSpace + 1);

      String axis = data.substring(firstSpace + 1, secondSpace);
      String dirStr = data.substring(secondSpace + 1, thirdSpace);
      int steps = data.substring(thirdSpace + 1).toInt();

      int dir = (dirStr == "FORWARD") ? FORWARD : BACKWARD;

      if (axis == "X") {
        stepperX.step(steps, dir, SINGLE);
        currentPosX += (dir == FORWARD) ? steps : -steps;
        Serial.print("X Pos: "); Serial.println(currentPosX);
      }
      else if (axis == "Y") {
        stepperY.step(steps, dir, SINGLE);
        currentPosY += (dir == FORWARD) ? steps : -steps;
        Serial.print("Y Pos: "); Serial.println(currentPosY);
      }
    }
    else if (data == "HOME") {
      // Return both to logical zero
      if (currentPosX != 0) {
        int dirX = (currentPosX > 0) ? BACKWARD : FORWARD;
        stepperX.step(abs(currentPosX), dirX, SINGLE);
        currentPosX = 0;
      }
      if (currentPosY != 0) {
        int dirY = (currentPosY > 0) ? BACKWARD : FORWARD;
        stepperY.step(abs(currentPosY), dirY, SINGLE);
        currentPosY = 0;
      }
      Serial.println("Returned to logical home (0,0)");
    }
    else {
      // Old object tracking data: frame and obj coordinates
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

        int tolerance = 15;

        if (abs(errorX) > tolerance) {
          int dir = (errorX > 0) ? FORWARD : BACKWARD;
          stepperX.step(5, dir, SINGLE);
          currentPosX += (dir == FORWARD) ? 5 : -5;
        }
        if (abs(errorY) > tolerance) {
          int dir = (errorY > 0) ? FORWARD : BACKWARD;
          stepperY.step(5, dir, SINGLE);
          currentPosY += (dir == FORWARD) ? 5 : -5;
        }

        if (abs(errorX) <= tolerance && abs(errorY) <= tolerance) {
          Serial.println("ALIGNED");
        }
      }
    }
  }
}
