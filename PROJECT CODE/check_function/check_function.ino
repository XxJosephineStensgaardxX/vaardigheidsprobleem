#define SERVO_NECK_PIN 8
#define GRIPPER_PIN 9
#define FRONT_TRIG_PIN 12
#define FRONT_ECHO_PIN 13
#define SIDE_TRIG_PIN A1 //analogPin
#define SIDE_ECHO_PIN A0

#define MOTOR_RIGHT_ROTATION_SENSOR 3
#define MOTOR_LEFT_ROTATION_SENSOR 2

#define MOTOR_RIGHT_FORWARD 10 
#define MOTOR_RIGHT_BACKWARD 5
#define MOTOR_LEFT_FORWARD 6
#define MOTOR_LEFT_BACKWARD 11 

#include <Adafruit_NeoPixel.h> 

const int rightSpeed = 230;
const int leftSpeed = 225;

volatile int rightPulseCount = 0;
volatile int leftPulseCount = 0;

int targetRotations = 20;
bool movingForward = false;
bool movingBackward = false;

void setup() {
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_ROTATION_SENSOR, INPUT);
  pinMode(MOTOR_LEFT_ROTATION_SENSOR, INPUT);

  Serial.begin(9600);
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(SIDE_TRIG_PIN, OUTPUT);
  pinMode(SIDE_ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ROTATION_SENSOR), rightRotationsUpdate, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ROTATION_SENSOR), leftRotationsUpdate, CHANGE);
}

// change rotations, try turn around and maybe if it doesnt work go back to backward
void loop() {
    moveForwardInRotations(targetRotations);

    if(millis() % 5000 == 0){
      check();
    }
}

void centerRobot() {
    long sideDistance = getDistanceSide();

    if (sideDistance > 9.2) { // Far from the side obstacle
        Serial.println("sideDistance > 9.2");
        analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
        analogWrite(MOTOR_LEFT_FORWARD, 240); 
        analogWrite(MOTOR_LEFT_BACKWARD, 0);
        analogWrite(MOTOR_RIGHT_BACKWARD, 0);
        
    } else if (sideDistance < 7.4) { // Close to the side obstacle
        Serial.println("sideDistance < 7.4");
        analogWrite(MOTOR_RIGHT_FORWARD, 255);
        analogWrite(MOTOR_LEFT_FORWARD, 215); 
        analogWrite(MOTOR_LEFT_BACKWARD, 0); 
        analogWrite(MOTOR_RIGHT_BACKWARD, 0);
        
    } else {
        Serial.println("sideDistance ELSE");
        analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
        analogWrite(MOTOR_LEFT_BACKWARD, 0);
        analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    }
}

void moveForwardInRotations(int rotations) {
    if (!movingForward) {
        resetRotations();
        movingForward = true;
    }

    while (rightPulseCount < rotations && leftPulseCount < rotations) {
        // Continuously sense distance while moving forward
        long distance = getDistanceForward();
        if (distance < 10) {
            stopRobot();
//            determineTurn(); // Call determineTurn when an obstacle is detected
            return; // Exit the function to stop moving forward
        }

        centerRobot();
//        sideIsFree();

//        if (rightPulseCount >= 2 * rotations) {
//    Serial.println("Calling check()");
//    check(); // Call the check function
//    rightPulseCount = 0; // Reset the pulse count
//    leftPulseCount = 0; // Reset the pulse count
//    Serial.println("check() called");
//}

    }

    stopRobot();
    movingForward = false;
}

void check() {
  long distanceForward = getDistanceForward();
  swivelNeck(90);
  swivelNeck(-90);

  if (distanceForward < 1 || distanceForward < 5) {
    moveBackwardInRotations(40);
  }

  swivelNeck(90);
  swivelNeck(-90);
//  determineTurn();
  
}


void moveBackwardInRotations(int rotations) {
    if (!movingBackward) {
        resetRotations();
        movingBackward = true;
    }

    while (rightPulseCount < rotations && leftPulseCount < rotations) {
        Serial.println(rightPulseCount);
        Serial.println(leftPulseCount);

        // Adjust motor outputs for moving backward
        analogWrite(MOTOR_RIGHT_FORWARD, LOW);
        analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
        analogWrite(MOTOR_LEFT_FORWARD, LOW);
        analogWrite(MOTOR_LEFT_BACKWARD, leftSpeed);
    }

    stopRobot();
    movingBackward = false;
}

void stopRobot() {
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
}

float pulse(int TRIG_PIN, int ECHO_PIN) {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration_us = pulseIn(ECHO_PIN, HIGH);
  return duration_us * .017;
}

// include this in determine turn 
float getDistanceForward() {
  return round(pulse(FRONT_TRIG_PIN, FRONT_ECHO_PIN) * 100.0) / 100.0;
}

float getDistanceSide() {
  return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;
}

void resetRotations() {
  rightPulseCount = 0;
  leftPulseCount = 0;
}


void rightRotationsUpdate() {
  noInterrupts();
  rightPulseCount++;
  interrupts();
}

void leftRotationsUpdate() {
  noInterrupts();
  leftPulseCount++;
  interrupts();
}

void swivelNeck(int angle) {
    for (int i = 0; i < 10; i++) {
        int pulseWidth = map(angle, -90, 90, 600, 2400); // Map the angle to pulse width
        digitalWrite(SERVO_NECK_PIN, HIGH);              // Set the pin high
        delayMicroseconds(pulseWidth);                   // Delay for the calculated pulse width
        digitalWrite(SERVO_NECK_PIN, LOW);               // Set the pin low
        wait(20);                                        // Add a small delay to ensure the servo has enough time to respond
    }
}

void wait(int timeToWait) {
    unsigned long startTime = millis(); // Get the current time

    // Loop until the desired time has passed
    while (millis() - startTime < timeToWait) {
        // Do nothing, just keep looping
    }
}
