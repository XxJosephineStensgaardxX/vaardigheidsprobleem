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

const int sensorCount = 6; // Number of sensors in your analog line sensor
const int sensorPins[sensorCount] = {A1, A2, A3, A4, A5, A6}; // Analog sensor pins (removed pins: A0 and A7)

int sensorValues[sensorCount]; // Array to store sensor values

bool waitingStart = true;
bool startSequence = false;
bool ending = false;

bool turnRight = false;

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
        querySensors();
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
        goForwardInTicks(55);
        wait(250);
        gripClose();
        wait(250);
        basicTurnLeft();
        wait(250);
        goForwardInTicks(40);
        startSequence = false;
        return wait(250);
    }
}

void querySensors() {
    long forwardDistance = getDistanceForward();
    // leftDistance = getLeftDistance();
}

void goForwardInTicks(int ticks) {
    resetRotations();
    while (rightPulseCount < ticks) {
        analogWrite(MOTOR_RIGHT_FORWARD, 255);
        analogWrite(MOTOR_LEFT_FORWARD, 232);
    }
    turnRight = false;
    ending = allBlack();
    stopRobot();
    gripClose();
}



void resetRotations() {
  rightPulseCount = 0;
  leftPulseCount = 0;
}

void stopRobot() {
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
}



boolean allBlack()
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


void basicTurnLeft()
{
    stopRobot();

    resetRotations();
    while (rightPulseCount < 17)
    {
        analogWrite(MOTOR_RIGHT_FORWARD, 255);
        analogWrite(MOTOR_LEFT_FORWARD, 255);
    }

    stopRobot();
}


float getDistanceForward() {
    return round(pulse(FRONT_TRIG_PIN, FRONT_ECHO_PIN) * 100.0) / 100.0;  
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

float getDistanceSide() {
    return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;
}

float pulse(int TRIG_PIN, int ECHO_PIN) {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration_us = pulseIn(ECHO_PIN, HIGH);
  return duration_us * .017;
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
