/* Changelog [01/11/25]
Version - 0.09
This version is stable, and apart from wiring issues car is perfectly functional. 

Known list of issues:
1. Combined key presses ( F + L, etc have reversed turning)
2. Testing cannot be invoked dynamically
*/

#include <SoftwareSerial.h>

// =========================
// CONFIGURATION
// =========================
#define TEST_MODE false   // Set to true to run test sequence

// Bluetooth (HC-05) on D2 (TX) and D3 (RX)
SoftwareSerial BTSerial(3, 2); // RX, TX

// L298N Motor Driver pins
const int IN1 = 4;   // Left motor direction 1
const int IN2 = 5;   // Left motor direction 2
const int IN3 = 6;   // Right motor direction 1
const int IN4 = 7;   // Right motor direction 2
const int ENA = 9;   // Left motor enable (PWM)
const int ENB = 10;  // Right motor enable (PWM)

// Motor speed variables
int speedLeft = 150;
int speedRight = 150;

// =========================
// HELPER FUNCTIONS
// =========================

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

// =========================
// MOVEMENT FUNCTIONS
// =========================
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  setSpeed(speedLeft, speedRight);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  setSpeed(speedLeft, speedRight);
}

void turnRightPivot() {  // left pivot = left motor backward, right motor forward
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  setSpeed(speedLeft, speedRight);
}

void turnLeftPivot() { // right pivot = left motor forward, right motor backward
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  setSpeed(speedLeft, speedRight);
}

// Diagonal directions
void forwardLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  setSpeed(0, speedRight);  // Left motor off
}

void forwardRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  setSpeed(speedLeft, 0);  // Right motor off
}

void backwardLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  setSpeed(speedLeft, 0);
}

void backwardRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  setSpeed(0, speedRight);
}

// =========================
// TEST FUNCTION
// =========================
void runMotorTest() {
  Serial.println("Running Motor Test...");
  delay(1000);

  Serial.println("Forward");
  moveForward();
  delay(2000);
  stopMotors(); delay(1000);

  Serial.println("Backward");
  moveBackward();
  delay(2000);
  stopMotors(); delay(1000);

  Serial.println("Left Pivot");
  turnLeftPivot();
  delay(2000);
  stopMotors(); delay(1000);

  Serial.println("Right Pivot");
  turnRightPivot();
  delay(2000);
  stopMotors(); delay(1000);

  Serial.println("Test Complete");
}

// =========================
// SETUP
// =========================
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  Serial.begin(9600);
  BTSerial.begin(9600);
  Serial.println("RC Car Ready!");

  if (TEST_MODE) runMotorTest();
}

// =========================
// MAIN LOOP
// =========================
void loop() {
  if (TEST_MODE) return; // Skip normal operation if testing

  if (BTSerial.available()) {
    char command = BTSerial.read();
    Serial.print("Received: ");
    Serial.println(command);

    switch (command) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeftPivot(); break;
      case 'R': turnRightPivot(); break;
      case 'S': stopMotors(); break;
      case 'G': forwardLeft(); break;   // Forward + Left
      case 'H': forwardRight(); break;  // Forward + Right
      case 'I': backwardLeft(); break;  // Backward + Left
      case 'J': backwardRight(); break; // Backward + Right

      // Speed control (0–9)
      case '0' ... '9':
        {
          int level = command - '0';
          int newSpeed = map(level, 0, 9, 0, 255);
          speedLeft = newSpeed;
          speedRight = newSpeed;
          Serial.print("Speed set to: ");
          Serial.println(newSpeed);
          break;
        }

      default:
        stopMotors();
        break;
    }
  }
}
