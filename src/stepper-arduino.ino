#include <AFMotor.h>

AF_Stepper stepperX(200, 2); // horizontal pan
AF_Stepper stepperY(200, 1); // vertical tilt

// Position tracking
int currentPanSteps = 0;
int currentTiltSteps = 0;

// Motor limits (in steps)
const int PAN_LIMIT = 100;   // ±90 degrees
const int TILT_LIMIT = 25;   // ±45 degrees

// Scanning variables
int scanDirection = 1;       // 1 = forward, -1 = backward
const int SCAN_STEP_SIZE = 5;

void setup() {
  Serial.begin(9600);
  stepperX.setSpeed(50); 
  stepperY.setSpeed(50);
  
  // Send initial position
  reportPosition();
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    if (data == "SCAN") {
      performScan();
    }
    else if (data == "HOME") {
      homeToCenter();
    }
    else if (data.startsWith("POS,")) {
      handleAbsolutePosition(data);
    }
    else {
      // Handle coordinate tracking (existing functionality)
      handleCoordinateTracking(data);
    }
  }
}

void performScan() {
  // Simple horizontal scan
  int targetSteps = SCAN_STEP_SIZE * scanDirection;
  
  if (abs(currentPanSteps + targetSteps) <= PAN_LIMIT) {
    stepperX.step(abs(targetSteps), (targetSteps > 0) ? FORWARD : BACKWARD, SINGLE);
    currentPanSteps += targetSteps;
  } else {
    // Reverse direction when hitting limit
    scanDirection *= -1;
    targetSteps = SCAN_STEP_SIZE * scanDirection;
    if (abs(currentPanSteps + targetSteps) <= PAN_LIMIT) {
      stepperX.step(abs(targetSteps), (targetSteps > 0) ? FORWARD : BACKWARD, SINGLE);
      currentPanSteps += targetSteps;
    }
  }
  
  reportPosition();
}

void homeToCenter() {
  // Move to center position (0,0)
  moveToAbsolutePosition(0, 0);
  Serial.println("HOMED");
}

void handleAbsolutePosition(String data) {
  // Parse: "POS,pan_steps,tilt_steps"
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  
  if (firstComma > 0 && secondComma > firstComma) {
    int targetPan = data.substring(firstComma + 1, secondComma).toInt();
    int targetTilt = data.substring(secondComma + 1).toInt();
    
    // Enforce limits
    targetPan = constrain(targetPan, -PAN_LIMIT, PAN_LIMIT);
    targetTilt = constrain(targetTilt, -TILT_LIMIT, TILT_LIMIT);
    
    moveToAbsolutePosition(targetPan, targetTilt);
  }
}

void moveToAbsolutePosition(int targetPan, int targetTilt) {
  // Move pan motor
  int panDelta = targetPan - currentPanSteps;
  if (panDelta != 0) {
    stepperX.step(abs(panDelta), (panDelta > 0) ? FORWARD : BACKWARD, SINGLE);
    currentPanSteps = targetPan;
  }
  
  // Move tilt motor
  int tiltDelta = targetTilt - currentTiltSteps;
  if (tiltDelta != 0) {
    stepperY.step(abs(tiltDelta), (tiltDelta > 0) ? FORWARD : BACKWARD, SINGLE);
    currentTiltSteps = targetTilt;
  }
  
  reportPosition();
}

void handleCoordinateTracking(String data) {
  // Existing coordinate tracking functionality
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

    int tolerance = 25;
    bool moved = false;

    if (abs(errorX) > tolerance) {
      int moveSteps = 5;
      int newPanSteps = currentPanSteps;
      
      if (errorX > 0 && currentPanSteps + moveSteps <= PAN_LIMIT) {
        stepperX.step(moveSteps, FORWARD, SINGLE);
        newPanSteps += moveSteps;
        moved = true;
      } else if (errorX < 0 && currentPanSteps - moveSteps >= -PAN_LIMIT) {
        stepperX.step(moveSteps, BACKWARD, SINGLE);
        newPanSteps -= moveSteps;
        moved = true;
      }
      
      currentPanSteps = newPanSteps;
    }

    if (abs(errorY) > tolerance) {
      int moveSteps = 5;
      int newTiltSteps = currentTiltSteps;
      
      if (errorY > 0 && currentTiltSteps + moveSteps <= TILT_LIMIT) {
        stepperY.step(moveSteps, FORWARD, SINGLE);
        newTiltSteps += moveSteps;
        moved = true;
      } else if (errorY < 0 && currentTiltSteps - moveSteps >= -TILT_LIMIT) {
        stepperY.step(moveSteps, BACKWARD, SINGLE);
        newTiltSteps -= moveSteps;
        moved = true;
      }
      
      currentTiltSteps = newTiltSteps;
    }

    if (moved) {
      reportPosition();
    }

    if (abs(errorX) <= tolerance && abs(errorY) <= tolerance) {
      Serial.println("ALIGNED");
    }
  }
}

void reportPosition() {
  Serial.print("POS:");
  Serial.print(currentPanSteps);
  Serial.print(",");
  Serial.println(currentTiltSteps);
}
