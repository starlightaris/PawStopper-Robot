#include <AFMotor.h>

AF_Stepper stepperX(48, 2); // horizontal pan
AF_Stepper stepperY(48, 1); // vertical tilt

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

    // Expected format: STEP X FORWARD 5 or STEP Y BACKWARD 10
    if (data.startsWith("STEP")) {
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
