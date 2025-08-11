#include <AFMotor.h>

AF_Stepper stepperX(48, 2); // horizontal pan
AF_Stepper stepperY(48, 1); // vertical tilt

int currentSpeed = 50; // Default speed

void setup() {
  Serial.begin(9600);
  stepperX.setSpeed(currentSpeed);
  stepperY.setSpeed(currentSpeed);
  Serial.println("Arduino ready");
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // Handle speed command: SPEED 30
    if (data.startsWith("SPEED")) {
      int spaceIndex = data.indexOf(' ');
      if (spaceIndex > 0) {
        int newSpeed = data.substring(spaceIndex + 1).toInt();
        if (newSpeed > 0 && newSpeed <= 100) { // Validate speed range
          currentSpeed = newSpeed;
          stepperX.setSpeed(currentSpeed);
          stepperY.setSpeed(currentSpeed);
          Serial.println("SPEED_OK");
        } else {
          Serial.println("SPEED_ERROR");
        }
      } else {
        Serial.println("SPEED_ERROR");
      }
    }
    // Handle step command: STEP X FORWARD 5 or STEP Y BACKWARD 10
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
