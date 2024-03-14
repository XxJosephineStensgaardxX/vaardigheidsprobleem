#define SERVO_NECK_PIN 9
#define FRONT_TRIG_PIN 12
#define FRONT_ECHO_PIN 13
#define SIDE_TRIG_PIN A1 //analogPin
#define SIDE_ECHO_PIN A0

//#define GRIPPER PIN 8

#define MOTOR_RIGHT_ROTATION_SENSOR 3
#define MOTOR_LEFT_ROTATION_SENSOR 2

#define MOTOR_RIGHT_FORWARD 7  
#define MOTOR_RIGHT_BACKWARD 5
#define MOTOR_LEFT_FORWARD 6
#define MOTOR_LEFT_BACKWARD 4

const int rightSpeed = 195;
const int leftSpeed = 247;

volatile int rightPulseCount = 0;
volatile int leftPulseCount = 0;

void setup() {
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_ROTATION_SENSOR, INPUT); // COME BACK TO THIS!!!!
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
  moveForwardInRotationsWithAdjustment(20);
}

void moveForwardInRotationsWithAdjustment(int rotations) {
  resetRotations();

  while (rightPulseCount < rotations && leftPulseCount < rotations) {
    long sideDistance = getDistanceSide();

    // Adjust motor speeds based on side distance
    if (sideDistance > 9.2) { // far
      analogWrite(MOTOR_RIGHT_FORWARD, 195);
      analogWrite(MOTOR_LEFT_FORWARD, 260); // Left speed more 250
    } else if (sideDistance < 7.2) { // close
      analogWrite(MOTOR_RIGHT_FORWARD, 200); // Right speed more 205
      analogWrite(MOTOR_LEFT_FORWARD, 247);
    } else {
      analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
      analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
    }
  }

  stopRobot();
}

void moveForward(int rightSpeed, int leftSpeed) {
  analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
}
void adjustPosition() {
  long sideDistance = getDistanceSide();

  // Adjust motor speeds based on side distance
  if (sideDistance > 9.2) { // far
    analogWrite(MOTOR_RIGHT_FORWARD, 195);
    analogWrite(MOTOR_LEFT_FORWARD, 0); // Left speed more 260
  } else if (sideDistance < 7.2) { // close
    analogWrite(MOTOR_RIGHT_FORWARD, 0); // Right speed more 200
    analogWrite(MOTOR_LEFT_FORWARD, 247);
  } else {
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
  }
}

void stopRobot() {
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
}

void moveForwardInRotations(int desiredRotations) {
  resetRotations();

  while (rightPulseCount < desiredRotations && leftPulseCount < desiredRotations) {
    moveForward(rightSpeed, leftSpeed);
    
    adjustPosition();
  }

  stopRobot();
}

void resetRotations() {
  rightPulseCount = 0;
  leftPulseCount = 0;
}

float pulse(int TRIG_PIN, int ECHO_PIN){
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    float duration_us = pulseIn(ECHO_PIN, HIGH);
    return duration_us * .017;
}

float getDistanceForward(){
    return round(pulse(FRONT_TRIG_PIN, FRONT_ECHO_PIN) * 100.0) / 100.0;
}

float getDistanceSide(){
    return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;
}

void rightRotationsUpdate() {
  noInterrupts();
  rightPulseCount++;
  interrupts();
  return;
}

void leftRotationsUpdate() {
  noInterrupts();
  leftPulseCount++;
  interrupts();
  return;
}
