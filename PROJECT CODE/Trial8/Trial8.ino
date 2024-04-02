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

// new additions
const int sensorCount = 6; // Number of sensors in your analog line sensor
const int sensorPins[sensorCount] = {A1, A2, A3, A4, A5, A6}; // Analog sensor pins (removed pins: A0 and A7)

int sensorValues[sensorCount]; // Array to store sensor values

bool waitingStart = true;
bool startSequence = false;
bool ending = false;

bool turnedRight = false;

//side is free issue, when its too far from wall, it thinks the side is free
//centerRobot 

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

  for (int i = 0; i < 4; i++) {
        gripOpen();
    }
    gripOpen();
}

void loop() {
    long distanceForward = getDistanceForward();
    // long rightSideDistance = getDistanceSide();

    if (waitingStart) {
      
        sensingBothSides();
        if (distanceForward < 25) {
          
            waitingStart = false;
            startSequence = true;
        }
        
        return wait(100);
    }

    // the start itself;
    // the robot ought to move, pick up the stick,
    // turn left, and move forward
    if (startSequence) {
      
        wait(2000);
        
        goForwardBasic(55);
        wait(250);
        
        gripClose();
        wait(250);
        
        turnLeft(13);
        wait(250);
        
        goForwardBasic(40);
        
        startSequence = false;
        
        return wait(250);
    }

    ending = blackDetected();

    // end sequence
    if (ending)
    {
        stopRobot();
        
        gripOpen();
        wait(150);
        
        goBackwardBasic(20);
        
        wait(150);
        gripClose();
        
        while (true);
    }

    moveForwardInRotations(targetRotations);
}

void goForwardBasic(int ticks)
// moves the car forward for a given number of ticks
{
    resetRotations();

    while (rightPulseCount < ticks)
    {
        analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
    }

    turnedRight = false;

    ending = blackDetected();

    stopRobot();
    gripClose();
}

void goBackwardBasic(int ticks)
{
    resetRotations();

    while (rightPulseCount < ticks)
    {
        analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
        analogWrite(MOTOR_LEFT_BACKWARD, leftSpeed);
    }

    turnedRight = false;

    stopRobot();
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
            determineTurn(); // Call determineTurn when an obstacle is detected
            return; // Exit the function to stopRobot moving forward
        }

        centerRobot();

        turnedRight = false;
        ending = blackDetected();

        
        sideIsFree();
    }

    stopRobot();
    movingForward = false;

    turnedRight = false;

    ending = blackDetected();
}


void moveBackwardInRotations(int rotations) {
    if (!movingBackward) {
        resetRotations();
        movingBackward = true;
    }

    while (rightPulseCount < rotations && leftPulseCount < rotations) {
        Serial.println(rightPulseCount);
        Serial.println(leftPulseCount);

        analogWrite(MOTOR_RIGHT_FORWARD, LOW);
        analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
        analogWrite(MOTOR_LEFT_FORWARD, LOW);
        analogWrite(MOTOR_LEFT_BACKWARD, leftSpeed);
    }

    stopRobot();
    movingBackward = false;
}


void sideIsFree() {
     long rightSideDistance = getDistanceSide();
     Serial.print(rightSideDistance);

    if (rightSideDistance > 15) { // If side distance is free
        Serial.println("forward free");
        moveForwardInRotations(36);
        Serial.println("side is free function");
        turnRight(12);
        
        turnedRight = true;
        moveForwardInRotations(40);
        
    }

    moveForwardInRotations(targetRotations);
}

void determineTurn() {
    long distanceForward = getDistanceForward();
    long rightSideDistance = getDistanceSide();

    if (rightSideDistance > 20) { // If side distance is free
        Serial.println("side is free DT");
        turnRight(12.5); // Turning right as an example
        moveForwardInRotations(40);
    } else if (distanceForward < 20) { // If obstacle detected in front
        swivelNeck(90);
        wait(1000);
        long leftDistance = getDistanceForward();
        Serial.print("Left distance: ");
        Serial.println(leftDistance);
        swivelNeck(0);

        if (leftDistance > 15) {
            Serial.println("Turning left");
            turnLeft(13); 
        } else if (leftDistance < 15) {
            Serial.println("Turning Around");
            turnRight(20);
        }
    }  
}

boolean blackDetected()
{
    short sum = 0;

    queryIRSensors();
    for (int i = 0; i < 6; i++)
    {
        if (sensorValues[i])
        {
            sum++;
        };
    }

    return sum == 6;
}

void gripOpen() {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(2000);
    digitalWrite(GRIPPER_PIN, LOW);
    delay(10);
}

void gripClose() {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(GRIPPER_PIN, LOW);
    delay(10);
}

void queryIRSensors() {
    for (int i = 0; i < 6; i++) {
        // irValues[i] = analogRead(irSensors[i]) > 800;
    }
}

void sensingBothSides() {
    long forwardDistance = getDistanceForward();
    long leftDistance = getDistanceSide();
}

//void check() {
//  long distanceForward = getDistanceForward();
//  Serial.println(distanceForward);
//  swivelNeck(90);
//  Serial.println(distanceForward);
//  wait(1000);
//  swivelNeck(-90);
//  Serial.println(distanceForward);
//  wait(1000);
//  swivelNeck(0);
//  Serial.println(distanceForward);
//
//  if (distanceForward < 8) {
//    Serial.println(distanceForward);
//    moveBackwardInRotations(40);
//  }
//
//  swivelNeck(90);
//  wait(1000);
//  swivelNeck(-90);
//  wait(1000);
//  swivelNeck(0);
////  determineTurn();
//  
//}


//void moveBackwardInRotations(int rotations) {
//    if (!movingBackward) {
//        resetRotations();
//        movingBackward = true;
//    }
//
//    while (rightPulseCount < rotations && leftPulseCount < rotations) {
//        Serial.println(rightPulseCount);
//        Serial.println(leftPulseCount);
//
//        // Adjust motor outputs for moving backward
//        analogWrite(MOTOR_RIGHT_FORWARD, 0);
//        analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
//        analogWrite(MOTOR_LEFT_FORWARD, 0);
//        analogWrite(MOTOR_LEFT_BACKWARD, leftSpeed);
//    }
//
//    stopRobotRobot();
//    movingBackward = false;
//}

void stopRobot() {
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
}

void turnLeft(int rotations) {
    stopRobot();
    wait(150);

    resetRotations();
    while(rightPulseCount < rotations) {
        analogWrite(MOTOR_LEFT_FORWARD, LOW);
        analogWrite(MOTOR_RIGHT_FORWARD, leftSpeed);
        analogWrite(MOTOR_RIGHT_BACKWARD, LOW);
        analogWrite(MOTOR_LEFT_BACKWARD, rightSpeed);
    }

    stopRobot();
    wait(150);

    // Continue moving forward if obstacle is cleared
    long distance = getDistanceForward();
    if (distance > 15) {
        wait(100);
        moveForwardInRotations(5);
    }
    
    wait(2000);
}

void turnRight(int rotations) {
    Serial.print("Right turn how many times");
    stopRobot();
    wait(150);
    
    resetRotations();
    while (leftPulseCount < rotations) {
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
        analogWrite(MOTOR_LEFT_BACKWARD, LOW);
        analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
        analogWrite(MOTOR_RIGHT_FORWARD, LOW);
        Serial.println("Rotations in turnRight: " + String(leftPulseCount));
    }
   
   stopRobot();
   wait(150);
   
    // Continue moving forward if obstacle is cleared
    long distance = getDistanceForward();
    if (distance > 15) {
        wait(100);
        moveForwardInRotations(5);
    }

    wait(2000);
}

void resetRotations() {
  rightPulseCount = 0;
  leftPulseCount = 0;
}

float pulse(int TRIG_PIN, int ECHO_PIN) {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration_us = pulseIn(ECHO_PIN, HIGH);
  return duration_us * .017;
}

float getDistanceSide() {
    return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;
}

float getDistanceForward() {
    return round(pulse(FRONT_TRIG_PIN, FRONT_ECHO_PIN) * 100.0) / 100.0;  
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
