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

void loop() {

    moveForwardInRotations(targetRotations);
}

void centerRobot() {
    long sideDistance = getDistanceSide();

    if (sideDistance > 9.2) {       // Far from the side obstacle
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
            return; // Exit the function to stop moving forward
        }

        centerRobot();
        sideIsFree();
    }

    stopRobot();
    movingForward = false;
}

//void check() {
//  long distanceForward = getDistanceForward();
//  swivelNeck(90);
//  swivelNeck(-90);
//
//  if (distanceForward < 1 || distanceForward < 5) {
//    moveBackwardInRotations(40);
//  }
//
//  swivelNeck(90);
//  swivelNeck(-90);
//  //  determineTurn();
//  
//}


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
        moveForwardInRotations(40);
        Serial.println("side is free function");
        turnRight(12);
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

// include this in determine turn 
float getDistanceForward() {
  return round(pulse(FRONT_TRIG_PIN, FRONT_ECHO_PIN) * 100.0) / 100.0;
}

float getDistanceSide() {
  return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;
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
