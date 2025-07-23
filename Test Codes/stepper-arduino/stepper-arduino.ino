// Motor 1 pins (IN1–IN4)
const int motor1_pins[4] = {13, 12, 11, 10};

// Motor 2 pins (IN5–IN8)
const int motor2_pins[4] = {7, 6, 5, 4};  // Replace with valid Arduino digital pins

// Half-step sequence
const int halfstep_seq[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

void setup() {
  // Set motor pins as OUTPUT
  for (int i = 0; i < 4; i++) {
    pinMode(motor1_pins[i], OUTPUT);
    pinMode(motor2_pins[i], OUTPUT);
  }

  Serial.begin(9600);
}

void rotateMotor(const int pins[], int delayTime, int steps, int direction) {
  for (int step = 0; step < steps; step++) {
    for (int i = 0; i < 8; i++) {
      int index = direction == 1 ? i : 7 - i;  // Forward or backward
      for (int pin = 0; pin < 4; pin++) {
        digitalWrite(pins[pin], halfstep_seq[index][pin]);
      }
      delayMicroseconds(delayTime);
    }
  }
}

void loop() {
  Serial.println("Motor 1 Forward");
  rotateMotor(motor1_pins, 2000, 512, 1);  // 2000 µs = 2 ms delay

  Serial.println("Motor 2 Backward");
  rotateMotor(motor2_pins, 2000, 512, -1);

  delay(1000);  // 1 second delay
}
