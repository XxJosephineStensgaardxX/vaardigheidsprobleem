#include <Adafruit_NeoPixel.h>

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

//#define PIN 7         // Digital pin Neopixels (Pin: IO)
//#define NUM_PIXELS 4  // Number of Neopixels
//Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_RGB );

#define INTERRUPT_COUNTER_INTERVAL  15    // interval in ms for the interruptroutine to be executed

const int rightSpeed = 230;
const int leftSpeed = 225;

volatile int rightPulseCount = 0;
volatile int leftPulseCount = 0;

int targetRotations = 20;

bool movingForward = false;
bool movingBackward = false;

bool sideIsFreeEnabled = true; // Global flag to control sideIsFree function

// new additions
const int sensorCount = 6; // Number of sensors in your analog line sensor
const int sensorPins[sensorCount] = {A1, A2, A3, A4, A5, A6}; // Analog sensor pins (removed pins: A0 and A7)

int sensorValues[sensorCount]; // Array to store sensor values

bool waitingStart = true;
bool startSequence = false;
bool ending = false;

bool gripperTriggered = false; // Flag to track if the gripper has been triggered

void setup() {

  pixels.begin();  // Initialize NeoPixels
  pixels.show();   // Initialize all pixels to 'off'
  pixels.setBrightness(50);  // Set NeoPixel brightness
  
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_ROTATION_SENSOR, INPUT);
  pinMode(MOTOR_LEFT_ROTATION_SENSOR, INPUT);

  Serial.println("Setup pins initialized.");

  Serial.begin(9600);
  Serial.println("Serial initialized.");
  
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(SIDE_TRIG_PIN, OUTPUT);
  pinMode(SIDE_ECHO_PIN, INPUT);

  Serial.println("Ultrasonic pins initialized.");

  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ROTATION_SENSOR), rightRotationsUpdate, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ROTATION_SENSOR), leftRotationsUpdate, CHANGE);

  Serial.println("Interrupts attached.");
    
//  Serial.println(millis());
//  if (millis() < 1000){
//
////    pixels.clear();
////     pixels.setPixelColor(1, pixels.Color(0, 0, 255)); 
////     pixels.setPixelColor(0, pixels.Color(0, 0, 255)); 
////     pixels.show();
//    
//  }

  Serial.println("Gripper initialization starting.");
  for (int i = 0; i < 4; i++) {
    gripOpen();
  }
  
  Serial.println("Gripper initialized.");
  // Set the gripperTriggered flag to true after the gripper has been triggered once
  gripperTriggered = true;

}

void loop() {

  Serial.println("Entering loop.");
  
  long distanceForward = getDistanceForward();
  Serial.print("Distance Forward: ");
  Serial.println(distanceForward);

  if (waitingStart) {
    sensingBothSides();
    if (distanceForward < 25) {
      waitingStart = false;
      startSequence = true;
    }
    return wait(100);
  }
  
  if (startSequence) {
    Serial.println("Starting sequence.");
  
    sideIsFreeEnabled = false;
    wait(800);
    goForwardBasic(55);
    wait(250);
    gripClose();
    wait(250);
    turnLeft(13);
    wait(250);
    goForwardBasic(60);
    startSequence = false;

    sideIsFreeEnabled = true;
    return wait(250);
  }

  ending = blackDetected();
  Serial.print("Ending: ");
  Serial.println(ending);

  // end sequence
  if (ending) {
    Serial.println("black Loop");
    stopRobot();
    gripOpen();
    wait(150);
    goBackwardBasic(20);
    wait(150);
    gripClose();
    sideIsFreeEnabled = false; // Turn off sideIsFree functionality
    while (true);
  }

  if (isStuck()) {
    Serial.println("Stuck detected.");
    recoverFromStuck();
  } else {
    moveForwardInRotations(targetRotations);
  }

}

void goForwardBasic(int ticks) {
  Serial.println("Going forward basic.");
  resetRotations();

  while (rightPulseCount < ticks) {
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
  }

  ending = blackDetected();
  stopRobot();
  gripClose();
}

void goBackwardBasic(int ticks) {
  Serial.println("Going backward basic.");
  resetRotations();
  while (rightPulseCount < ticks) {
    analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_BACKWARD, leftSpeed);

//     pixels.clear();
//     pixels.setPixelColor(1, pixels.Color(0, 255, 0)); //
//     pixels.setPixelColor(0, pixels.Color(0, 255, 0)); //
//     pixels.show();
  }

  stopRobot();
}

void centerRobot() {
  Serial.println("Centering robot.");
  long sideDistance = getDistanceSide();

  if (sideDistance > 9.2) { // Far from the side obstacle
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_FORWARD, 240);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);

    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(255, 209, 220)); // 
    pixels.setPixelColor(3, pixels.Color(255, 209, 220)); //
    pixels.show();

  } else if (sideDistance < 7.4) { // Close to the side obstacle
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_FORWARD, 215);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);

    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(255, 209, 220)); // 
    pixels.setPixelColor(3, pixels.Color(255, 209, 220));
    pixels.show();

  } else {
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);

    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(255, 209, 220)); // 
    pixels.setPixelColor(3, pixels.Color(255, 209, 220));
    pixels.show();
  }
}

void moveForwardInRotations(int rotations) {
  Serial.println("Moving forward in rotations.");
  if (!movingForward) {
    resetRotations();
    movingForward = true;
  }

  while (rightPulseCount < rotations && leftPulseCount < rotations) {
    long distance = getDistanceForward();
    if (distance < 10) {
      stopRobot();
      determineTurn();
      return;
    }
    
    ending = blackDetected();
    centerRobot();
    if (sideIsFreeEnabled) {
      sideIsFree();
    }
  }

  stopRobot();
  movingForward = false;
}

bool isStuck() {
  Serial.println("Checking if stuck.");
  if (rightPulseCount == 0 && leftPulseCount == 0) {
    return true;
  }

  else if(rightPulseCount == 0 || leftPulseCount == 0)
  {
    return true;
  }
  
  return false;
}

void recoverFromStuck() {
  Serial.println("Recovering from stuck.");
  
  goBackwardBasic(60);
 
}

void sideIsFree() {
  Serial.println("Checking if side is free.");

  long rightSideDistance = getDistanceSide();
  Serial.print(rightSideDistance);

  if (rightSideDistance > 15 && sideIsFreeEnabled) { // If side distance is free
    Serial.println("forward free");
    moveForwardInRotations(36);
    Serial.println("side is free function");
    turnRight(12);
    moveForwardInRotations(40);

  }

  moveForwardInRotations(targetRotations);
}

void determineTurn() {
  Serial.println("Determining turn.");
  long distanceForward = getDistanceForward();
  long rightSideDistance = getDistanceSide();

  if (rightSideDistance > 20) { // If side distance is free
    Serial.println("side is free DT");
    turnRight(12.5);
    moveForwardInRotations(40);
  } else if (distanceForward < 20) { // If obstacle detected in front
    swivelNeck(90);
    wait(500);
    long leftDistance = getDistanceForward();
    Serial.print("Left distance: ");
    Serial.println(leftDistance);
    swivelNeck(0);

    if (leftDistance > 15) {
      Serial.println("Turning left");
      turnLeft(12.5);
    } else if (leftDistance < 15) {
      Serial.println("Turning Around");
      turnRight(20);
    }
  }
}

boolean blackDetected()
{
  Serial.println("Checking for black detection.");
  short sum = 0;

  Serial.println("black function");
  
  queryIRSensors();
  for (int i = 0; i < 6; i++)
  {
    if (sensorValues[i])
    {
      Serial.println("black if fucntion");
      sum++;
    };
  }

  Serial.print("Sum: ");
  Serial.println(sum);

  return sum <= 4; //4 coz it never really meets 6
}

void gripOpen() {
  Serial.println("Opening gripper.");
  digitalWrite(GRIPPER_PIN, HIGH);
  delayMicroseconds(2000);
  digitalWrite(GRIPPER_PIN, LOW);
  delay(10);
}

void gripClose() {
  Serial.println("Closing gripper.");
  digitalWrite(GRIPPER_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(GRIPPER_PIN, LOW);
  delay(10);
}

void queryIRSensors() {
   Serial.println("Querying IR sensors.");
  for (int i = 0; i < 6; i++) {
     sensorValues[i] = analogRead(sensorPins[i]) > 800;
  }
}

void sensingBothSides() {
  Serial.println("Sensing both sides.");
  long forwardDistance = getDistanceForward();
  long leftDistance = getDistanceSide();
}

void stopRobot() {
  Serial.println("Stoping robot.");
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);

  pixels.clear();
  pixels.fill(pixels.Color(255, 0, 0)); 
  pixels.show();
}

void turnLeft(int rotations) {
  Serial.println("Turning left.");
  stopRobot();
  wait(150);

  resetRotations();
  while (rightPulseCount < rotations) {
    analogWrite(MOTOR_LEFT_FORWARD, LOW);
    analogWrite(MOTOR_RIGHT_FORWARD, leftSpeed);
    analogWrite(MOTOR_RIGHT_BACKWARD, LOW);
    analogWrite(MOTOR_LEFT_BACKWARD, rightSpeed);

    pixels.clear(); 
    pixels.setPixelColor(3, pixels.Color(178, 172, 136)); 
    pixels.show();
  }

  stopRobot();
  wait(150);

  // Continue moving forward if obstacle is cleared
  long distance = getDistanceForward();
  if (distance > 15) {
    wait(100);
    moveForwardInRotations(5);
  }

  wait(1000);
}

void turnRight(int rotations) {
  Serial.println("Turning right.");
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

    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(174, 198, 207)); 
    pixels.show();
  }

  stopRobot();
  wait(150);

  // Continue moving forward if obstacle is cleared
  long distance = getDistanceForward();
  if (distance > 15) {
    wait(100);
    moveForwardInRotations(5);
  }

  wait(1000);
}

void resetRotations() {
  Serial.println("Resetting rotations.");
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
  Serial.println("Getting distance from side.");
  return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;
}

float getDistanceForward() {
  Serial.println("Getting distance forward.");
  return round(pulse(FRONT_TRIG_PIN, FRONT_ECHO_PIN) * 100.0) / 100.0;
}

void rightRotationsUpdate() {
  Serial.println("Right rotations update.");
  noInterrupts();
  static unsigned long timer;
  static bool lastState;
  if (millis() > timer) {
    bool state = digitalRead(MOTOR_RIGHT_ROTATION_SENSOR);
    if (state != lastState) {
       rightPulseCount++;
       lastState = state;
    }
    timer = millis() + INTERRUPT_COUNTER_INTERVAL;
  } 
  interrupts(); 
}

void leftRotationsUpdate() {  
  Serial.println("Left rotations update."); 
  noInterrupts();
  static unsigned long timer;
  static bool lastState;
  if (millis() > timer) {
    bool state = digitalRead(MOTOR_LEFT_ROTATION_SENSOR);
    if (state != lastState) {
       leftPulseCount++;
       lastState = state;
    }
    timer = millis() + INTERRUPT_COUNTER_INTERVAL;
  } 
  interrupts(); 
}

void swivelNeck(int angle) {
  Serial.println("Swiveling neck.");
  for (int i = 0; i < 10; i++) {
    int pulseWidth = map(angle, -90, 90, 600, 2400); // Map the angle to pulse width
    digitalWrite(SERVO_NECK_PIN, HIGH);              // Set the pin high
    delayMicroseconds(pulseWidth);                   // Delay for the calculated pulse width
    digitalWrite(SERVO_NECK_PIN, LOW);               // Set the pin low
    wait(20);                                        // Add a small delay to ensure the servo has enough time to respond
  }
}

void wait(int timeToWait) {
  Serial.println("Waiting.");
  unsigned long startTime = millis(); // Get the current time

  // Loop until the desired time has passed
  while (millis() - startTime < timeToWait) {
    // Do nothing, just keep looping
  }
}
