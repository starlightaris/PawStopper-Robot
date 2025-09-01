#include <AFMotor.h>

// Initialize the stepper motors
AF_Stepper stepperX(48, 2);  // Horizontal
AF_Stepper stepperY(48, 1);  // Vertical

void setup() {
  Serial.begin(9600);
  stepperX.setSpeed(50);  // RPM
  stepperY.setSpeed(50);
  Serial.println("Ready to receive commands (e.g., X F 100)");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove whitespace or newline characters

    if (command.length() > 0) {
      // Example command: X F 100
      char motorID;
      char directionChar;
      int steps;

      int matched = sscanf(command.c_str(), "%c %c %d", &motorID, &directionChar, &steps);

      if (matched == 3) {
        int dir = (directionChar == 'F' || directionChar == 'f') ? FORWARD : BACKWARD;

        if (motorID == 'X' || motorID == 'x') {
          stepperX.step(steps, dir, SINGLE);
        } else if (motorID == 'Y' || motorID == 'y') {
          stepperY.step(steps, dir, SINGLE);
        } else {
          Serial.println("Invalid motor ID. Use X or Y.");
        }
      } else {
        Serial.println("Invalid command format. Use: X F 100");
      }
    }
  }
}
