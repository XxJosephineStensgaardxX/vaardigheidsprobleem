#include <Adafruit_NeoPixel.h>


/// Pins definition
#define SERVO_NECK_PIN 8  // Pin for servo controlling the neck
#define GRIPPER_PIN 9  // Pin for controlling the gripper
#define FRONT_TRIG_PIN 12 // Pin connected to the trigger of the front ultrasonic sensor
#define FRONT_ECHO_PIN 13 // Pin connected to the echo of the front ultrasonic sensor
#define SIDE_TRIG_PIN A1  // Pin connected to the trigger of the side ultrasonic sensor
#define SIDE_ECHO_PIN A0  // Pin connected to the echo of the side ultrasonic sensor

#define MOTOR_RIGHT_ROTATION_SENSOR 3
#define MOTOR_LEFT_ROTATION_SENSOR 2
#define MOTOR_RIGHT_FORWARD 10
#define MOTOR_RIGHT_BACKWARD 5
#define MOTOR_LEFT_FORWARD 6
#define MOTOR_LEFT_BACKWARD 11

#define INTERRUPT_COUNTER_INTERVAL  15  // interval in ms for the interruptroutine to be executed

#define START_BUTTON_PIN 4   // Pin connected to the pushbutton

#define PIN 7   // Digital pin Neopixels (Pin: IO)
#define NUM_PIXELS 4   // Number of Neopixels


// NeoPixel object initialization
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_RGB );

// Constants
const int rightSpeed = 230;
const int leftSpeed = 225;

const int sensorCount = 6; // Number of sensors in analog line sensor


// Variables
volatile int rightPulseCount = 0;
volatile int leftPulseCount = 0;

int targetRotations = 20;

int buttonState = 0; // Variable to store the state of the pushbutton


// Sensor pins array
const int sensorPins[sensorCount] = {A1, A2, A3, A4, A5, A6}; // Analog sensor pins (removed pins: A0 and A7)
int sensorValues[sensorCount]; // Array to store sensor values


// Global flags
bool movingForward = false;  // Flag indicating if the robot is moving forward
bool movingBackward = false;   // Flag indicating if the robot is moving backward

bool sideIsFreeEnabled = true; // Global flag to control sideIsFree function

bool blackDetectedResult;  // boolean that tracks the result of the BlackDetected function for the ending

bool waitingStart = true;  // Flag indicating if the robot is waiting to start
bool startSequence = false;  // Flag indicating if the start sequence has begun

bool ending = false;  // Flag indicating if the robot is performing the ending actions

bool gripperTriggered = false; // Flag to track if the gripper has been triggered

void setup() {

  pixels.begin();  // Initialize NeoPixels
  pixels.show();  // Initialize all pixels to 'off'
  pixels.setBrightness(50);  // Set NeoPixel brightness

  bool blackDetectedResult = blackDetected();  // Detect black color

  pinMode(START_BUTTON_PIN, INPUT);  // Initialize the pushbutton pin as an input
  buttonState = digitalRead(START_BUTTON_PIN);

  // Initialize motor pins
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_ROTATION_SENSOR, INPUT);
  pinMode(MOTOR_LEFT_ROTATION_SENSOR, INPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize ultrasonic sensor pins
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(SIDE_TRIG_PIN, OUTPUT);
  pinMode(SIDE_ECHO_PIN, INPUT);

  // Attach interrupt routine to rotation sensors
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ROTATION_SENSOR), rightRotationsUpdate, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ROTATION_SENSOR), leftRotationsUpdate, CHANGE); 

  // Display blue pixels if startup time is less than 1 second
  if (millis() < 1000) {

    pixels.clear();
    pixels.setPixelColor(1, pixels.Color(0, 0, 255));
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();
  }

  // Close gripper in setup
  for (int i = 0; i < 4; i++) {
    gripClose();
  }
  
  gripperTriggered = true; 

}

void loop() {
  // Get distance from the forward sensor
  long distanceForward = getDistanceForward();

  // Check if the start button is pressed
   if (buttonState == LOW) {
    if (waitingStart) {
      sensingBothSides();
      
      // If an obstacle is detected close to the robot
      if (distanceForward < 25) {
        waitingStart = false;
        startSequence = true; // Enable the start sequence
        }
        
        return wait(100);
     }
   }

  // Execute the start sequence
  if (startSequence) {
    sideIsFreeEnabled = false; // Disable side sensing
    goForwardBasic(55);
    wait(250);
    gripClose();
    wait(250);
    turnLeft(13);
    goForwardBasic(60);
    startSequence = false; // End the start sequence
    sideIsFreeEnabled = true; // Enable side sensing
    return wait(250);
  }

  // Check if the robot is stuck
  if (isStuck()) {
    recoverFromStuck(); // Recover from being stuck
  } 
  
  else {
    moveForwardInRotations(targetRotations); // Move forward in rotations
  }

  // Check if black color is detected
  if (blackDetected()) {
      performEnding(); // Perform ending actions
  }
}


// Functions

// Function to perform ending actions
void performEnding() {
    Serial.println("ending");
    stopRobot();
    gripOpen();
    wait(150);
    moveBackwardInRotations(10);
    wait(150);
    gripClose();
    sideIsFreeEnabled = false; // Turn off sideIsFree functionality
    while (true);
}

// Function to move forward with ticks
void goForwardBasic(int ticks) {
  resetRotations();

  while (rightPulseCount < ticks) {
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
  }

  stopRobot();
  gripClose();
}

// Function to center the robot
void centerRobot() {
  long sideDistance = getDistanceSide();

  if (sideDistance > 9.2) {  // Far from the side obstacle
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_FORWARD, 245);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);

    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(255, 209, 220)); 
    pixels.setPixelColor(3, pixels.Color(255, 209, 220)); 
    pixels.show();

  } else if (sideDistance < 7.4) {  // Close to the side obstacle
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_FORWARD, 215);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);

    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(255, 209, 220));
    pixels.setPixelColor(3, pixels.Color(255, 209, 220));
    pixels.show();

  } else {
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);

    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(255, 209, 220)); 
    pixels.setPixelColor(3, pixels.Color(255, 209, 220));
    pixels.show();
  }
}

// Function to move forward in rotations
void moveForwardInRotations(int rotations) {
  
  if (!movingForward) {  // If not already moving forward
    resetRotations();   // Reset rotation counts
    movingForward = true;  // Set flag to indicate moving forward
  }

  while (rightPulseCount < rotations && leftPulseCount < rotations) { // Loop until desired rotations are reached
    long distance = getDistanceForward(); // Get distance forward
    
    if (distance < 10) {  // If obstacle detected in front
      stopRobot();
      determineTurn(); // Determine and execute turning action
      return; // Exit the function
    }

    ending = blackDetected(); // Check for black color to trigger ending
    centerRobot(); // Center the Robot while moving
    
    if (sideIsFreeEnabled) {
      sideIsFree(); // Check if the side is free and take action if necessary
    }
  }

  stopRobot(); // Stop the robot after reaching the desired rotations
  movingForward = false; // Reset moving forward flag
}

// Function to move backward in rotations
void moveBackwardInRotations(int rotations) {
    if (!movingBackward) {
      resetRotations();
      movingBackward = true;
    }

    while (rightPulseCount < rotations && leftPulseCount < rotations) {
      analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
      analogWrite(MOTOR_LEFT_BACKWARD, leftSpeed);

      pixels.setPixelColor(1, pixels.Color(0, 255, 0)); 
      pixels.setPixelColor(2, pixels.Color(0, 0, 0)); 
      pixels.setPixelColor(3, pixels.Color(0, 0, 0)); 
      pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
      pixels.show();

    }

    stopRobot();
}

// Function to check if the robot is stuck
bool isStuck() {
  // If both rotation counters are zero, the robot is stuck
  if (rightPulseCount == 0 && leftPulseCount == 0) {  
    return true;  
  } 
  // If either rotation counter is zero, the robot is stuck
  else if (rightPulseCount == 0 || leftPulseCount == 0){  
    return true;
  }
  // If neither rotation counter is zero, the robot is not stuck
  return false;
}

// Function to recover from being stuck
void recoverFromStuck() {
    moveBackwardInRotations(targetRotations);
}

// Function to check if the right side is free
void sideIsFree() {
  long rightSideDistance = getDistanceSide();

  // If the right side is sufficiently free and the sideIsFreeEnabled flag is set
  if (rightSideDistance > 15 && sideIsFreeEnabled) { 
    moveForwardInRotations(36);
    turnRight(12);
    moveForwardInRotations(40);
  }
  
  moveForwardInRotations(targetRotations);
}

// Function to determine the turn direction
void determineTurn() {
  // Get distances from sensors
  long distanceForward = getDistanceForward();
  long rightSideDistance = getDistanceSide();

  // If side distance is free, turn right and move forward
  if (rightSideDistance > 20) { 
    turnRight(12.5);
    moveForwardInRotations(40);
  } 
  
  // If obstacle detected in front
  else if (distanceForward < 20) {
    swivelNeck(90); // Swivel neck to check left side
    wait(500);
    long leftDistance = getDistanceForward();
    swivelNeck(0);

    // If space on the left, turn left
    if (leftDistance > 15) {
      Serial.println("Turning left");
      turnLeft(12.75);
    } 

    // If no space on the left, turn around
    else if (leftDistance < 15) {
      Serial.println("Turning Around");
      turnRight(20);
    }
  }
}

// Function to detect black color
boolean blackDetected() {
  short sum = 0; // Initialize sum variable to count black sensor readings
  readSensorValues();  // Read sensor values

  // Iterate through sensor values
  for (int i = 0; i < 6; i++){    
    if (sensorValues[i]){   // If sensor value indicates black, increment sum
      sum++;
    };
  }

  Serial.print("Sum: ");
  Serial.println(sum);

  return sum == 6; // Return true if sum equals 6 (indicating 6 sensors detected black)
}

// Function to open the gripper
void gripOpen() {
  digitalWrite(GRIPPER_PIN, HIGH);
  delayMicroseconds(2000);
  digitalWrite(GRIPPER_PIN, LOW);
  delay(10);
}

// Function to close the grippe
void gripClose() {
  digitalWrite(GRIPPER_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(GRIPPER_PIN, LOW);
  delay(10);
}

// Function to read sensor values
void readSensorValues() {
  for (int i = 0; i < 6; i++) {
    // Read the analog value from each sensor pin and determine if it exceeds a threshold
    sensorValues[i] = analogRead(sensorPins[i]) > 800;
  }
}

// Function to sense both sides
void sensingBothSides() {
  // Get the distance to the front and right sides
  long forwardDistance = getDistanceForward();
  long rightDistance = getDistanceSide();
}

// Function to stop Robot
void stopRobot() {
  // Set all motor pins to LOW to stop the robot
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);

  pixels.clear();
  pixels.fill(pixels.Color(255, 0, 0)); 
  pixels.show();
}

// Function to turn the robot left by a specified number of rotations
void turnLeft(int rotations) {
  stopRobot();
  wait(150);

  // Reset rotation counters to track the turn
  resetRotations();

  // Perform the left turn until the desired number of rotations is reached
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
  
  // Check the distance to the front after turning left 
  long distance = getDistanceForward();  
  if (distance > 15) {
    wait(100);
    moveForwardInRotations(5);
  }

  wait(1000);
}

// Function to turn the robot right by a specified number of rotations
void turnRight(int rotations) {
  stopRobot();
  wait(150);

  // Reset rotation counters to track the turn
  resetRotations();
  
  // Perform the right turn until the desired number of rotations is reached
  while (leftPulseCount < rotations) {
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
    analogWrite(MOTOR_LEFT_BACKWARD, LOW);
    analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
    analogWrite(MOTOR_RIGHT_FORWARD, LOW);

    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(174, 198, 207));
    pixels.show();
  }

  stopRobot();
  wait(150);

  // Check the distance to the front after turning right
  long distance = getDistanceForward();   
  if (distance > 15) {
    wait(100);
    moveForwardInRotations(5);
  }

  wait(1000);
}

// Function to reset rotation counters
void resetRotations() {
  rightPulseCount = 0;
  leftPulseCount = 0;
}

// Function to measure pulse width
float pulse(int TRIG_PIN, int ECHO_PIN) {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration_us = pulseIn(ECHO_PIN, HIGH);  // Measure the duration of the echo pulse using pulseIn function
  return duration_us * .017;  // Convert the duration to distance in centimeters (speed of sound is 0.034 cm per microsecond)
}

// Function to get side distance
float getDistanceSide() {
  return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;  // Measure distance to the side using ultrasonic sensor
}

// Function to get forward distance
float getDistanceForward() {
  return round(pulse(FRONT_TRIG_PIN, FRONT_ECHO_PIN) * 100.0) / 100.0;  // Measure distance forward using ultrasonic sensor
}

// Interrupt update routine for right rotations
void rightRotationsUpdate() {
  noInterrupts();
  static unsigned long timer;  
  static bool lastState;
  
  if (millis() > timer) {
    bool state = digitalRead(MOTOR_RIGHT_ROTATION_SENSOR); // Read current state of rotation sensor
    
    if (state != lastState) { // If state has changed
      rightPulseCount++;  // Increment pulse count
      lastState = state;  // Update last state
    }
    
    timer = millis() + INTERRUPT_COUNTER_INTERVAL;
  }
  
  interrupts();
}

// Interrupt update routine for left rotations
void leftRotationsUpdate() {
  noInterrupts();
  static unsigned long timer;
  static bool lastState;
  
  if (millis() > timer) {
    bool state = digitalRead(MOTOR_LEFT_ROTATION_SENSOR); // Read current state of rotation sensor
    
    if (state != lastState) {  // If state has changed
      leftPulseCount++;  // Increment pulse count
      lastState = state;  // Update last state
    }
    
    timer = millis() + INTERRUPT_COUNTER_INTERVAL;
  }
  
  interrupts();
}

// Function to swivel the neck
void swivelNeck(int angle) {
  for (int i = 0; i < 10; i++) {
    int pulseWidth = map(angle, -90, 90, 600, 2400); // Map the angle to pulse width for servo control
    digitalWrite(SERVO_NECK_PIN, HIGH);
    delayMicroseconds(pulseWidth); 
    digitalWrite(SERVO_NECK_PIN, LOW); 
    
    wait(20);
  }
}

// Function to wait for a specified time
void wait(int timeToWait) {
  unsigned long startTime = millis(); // Get the current time

  while (millis() - startTime < timeToWait) { // Loop until the desired time has passed
  }
}
