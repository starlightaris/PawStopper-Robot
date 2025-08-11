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
      handleSpeedCommand(data);
    }
    // Handle step command: STEP X FORWARD 5 or STEP Y BACKWARD 10
    else if (data.startsWith("STEP")) {
      handleStepCommand(data);
    }
  }
}

void handleSpeedCommand(String data) {
  int spaceIndex = data.indexOf(' ');
  if (spaceIndex > 0 && spaceIndex < data.length() - 1) {
    String speedStr = data.substring(spaceIndex + 1);
    speedStr.trim(); // Remove any whitespace
    
    int newSpeed = speedStr.toInt();
    
    // More robust validation
    if (newSpeed >= 1 && newSpeed <= 100 && speedStr.length() > 0) {
      currentSpeed = newSpeed;
      stepperX.setSpeed(currentSpeed);
      stepperY.setSpeed(currentSpeed);
      
      // Send immediate response
      Serial.println("SPEED_OK");
      Serial.flush(); // Ensure data is sent immediately
    } else {
      Serial.println("SPEED_ERROR");
      Serial.flush();
    }
  } else {
    Serial.println("SPEED_ERROR");
    Serial.flush();
  }
}

void handleStepCommand(String data) {
  int firstSpace = data.indexOf(' ');
  int secondSpace = data.indexOf(' ', firstSpace + 1);
  int thirdSpace = data.indexOf(' ', secondSpace + 1);

  if (firstSpace > 0 && secondSpace > firstSpace && thirdSpace > secondSpace) {
    String axis = data.substring(firstSpace + 1, secondSpace);
    String dirStr = data.substring(secondSpace + 1, thirdSpace);
    String stepsStr = data.substring(thirdSpace + 1);
    
    axis.trim();
    dirStr.trim();
    stepsStr.trim();
    
    int steps = stepsStr.toInt();

    if (steps > 0) { // Validate steps
      int dir = (dirStr == "FORWARD") ? FORWARD : BACKWARD;

      if (axis == "X") {
        stepperX.step(steps, dir, SINGLE);
        Serial.println("X_OK");
        Serial.flush();
      }
      else if (axis == "Y") {
        stepperY.step(steps, dir, SINGLE);
        Serial.println("Y_OK");
        Serial.flush();
      }
    }
  }
}
