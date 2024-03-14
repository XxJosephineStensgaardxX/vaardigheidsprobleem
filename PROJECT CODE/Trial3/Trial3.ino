//#define SERVO_NECK_PIN 2
//#define TRIG_PIN 12
//#define ECHO_PIN 13
//#define SIDE_TRIG_PIN 1 //analogPin
//#define SIDE_ECHO_PIN 0
//
//#define MOTOR_RIGHT_ROTATION_SENSOR 8 //rotation Sensor, motorRightSpeedPin
//#define MOTOR_LEFT_ROTATION_SENSOR 9 //rotation Sensor, motorLeftSpeedPin
//
//#define MOTOR_RIGHT_FORWARD 7  
//#define MOTOR_RIGHT_BACKWARD 5
//#define MOTOR_LEFT_FORWARD 6
//#define MOTOR_LEFT_BACKWARD 4
//
//void setup() {
//  
//  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
//  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
//  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
//  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
//  analogWrite(MOTOR_RIGHT_ROTATION_SENSOR, ??);
//  analogWrite(MOTOR_LEFT_ROTATION_SENSOR, ??);
//  Serial.begin(9600);
//  pinMode(TRIG_PIN, OUTPUT);
//  pinMode(ECHO_PIN, INPUT);
//  pinMode(SIDE_TRIG_PIN, OUTPUT);
//  pinMode(SIDE_ECHO_PIN, INPUT);
//}
//
//void moveForward(int rightSpeed, int leftSpeed) {
//  analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
//  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
////  analogWrite(MOTOR_RIGHT_ROTATION_SENSOR, ??);
//  analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
//  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
////  analogWrite(MOTOR_LEFT_ROTATION_SENSOR, ??);
//
//}
//
//long getUltrasonicDistance() {
//  long duration, distance;
//  digitalWrite(TRIG_PIN, LOW);
//  delayMicroseconds(2);
//
//  // Send a 10 microsecond pulse to trigger the sensor
//  digitalWrite(TRIG_PIN, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIG_PIN, LOW);
//  // Read the pulse from the echo pin
//  duration = pulseIn(ECHO_PIN, HIGH);
//  
//  // Calculate the distance (in cm)
//  distance = duration * 0.034 / 2;
//  
//  return distance;
//
//}
//
//void stopRobot() {
//  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
//  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
//  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
//  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
//}
//
//void loop() {
//  long distance = getUltrasonicDistance(); // Call the function to get the distance
//  
//  // Print the distance to the Serial Monitor
//  Serial.print("Distance: ");
//  Serial.print(distance);
//  Serial.println(" cm");
//
//  if (distance > 10) {
//    // Move forward if the distance is greater than 10 cm
//    moveForward();
//  } else {
//    // Stop the robot if the distance is 10 cm or less
//    stopRobot();
//  }
//  
//  delay(100);
//  
//}



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

volatile int rightPulseCount = 0;
volatile int leftPulseCount = 0;

}
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

  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ROTATION_SENSOR), rightRotationISR, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ROTATION_SENSOR), leftRotationISR, RISING);
}

void moveForward(int rightSpeed, int leftSpeed) {

  long distanceSide = getSideDistance();
  analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);

  if (distanceSide > 9.2){
    
  analogWrite(MOTOR_RIGHT_FORWARD, 205);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_FORWARD, 247);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  }

  else if(distanceSide < 7.2){
  analogWrite(MOTOR_RIGHT_FORWARD, 190);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_FORWARD, 247);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  }

  else{
  analogWrite(MOTOR_RIGHT_FORWARD, 195  ^
    // Stop the robot if the distance is 10 cm or less
    stopRobot();
  }
  
  delay(100);
}

void rightRotation() {
  noInterrupts();
  rightPulseCount++;
  interrupts();
  return;
}

void leftRotationISR() {
  noInterrupts();
  leftPulseCount++;
  interrupts();
  return;
}



//void updateRW()
//{
//    noInterrupts();
//    countRW++;
//    interrupts();
//}
//
//void updateLW()
//{
//    noInterrupts();
//    countLW++;
//    interrupts();
//}
//
//void resetCounters()
//{
//    countRW = 0;
//    countLW = 0;
//}



//rotation snesors
//
//
//#define SERVO_NECK_PIN 2
//#define TRIG_PIN 12
//#define ECHO_PIN 13
//
//#define MOTOR_RIGHT_ROTATION_SENSOR 8
//#define MOTOR_LEFT_ROTATION_SENSOR 9
//
//#define MOTOR_RIGHT_FORWARD 7  
//#define MOTOR_RIGHT_BACKWARD 5
//#define MOTOR_LEFT_FORWARD 6
//#define MOTOR_LEFT_BACKWARD 4
//
//volatile int rightPulseCount = 0;
//volatile int leftPulseCount = 0;
//
//void rightRotationISR() {
//  rightPulseCount++;
//}
//
//void leftRotationISR() {
//  leftPulseCount++;
//}
//
//void setup() {
//  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
//  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
//  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
//  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
//  pinMode(MOTOR_RIGHT_ROTATION_SENSOR, INPUT);
//  pinMode(MOTOR_LEFT_ROTATION_SENSOR, INPUT);
//
//  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ROTATION_SENSOR), rightRotationISR, RISING);
//  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ROTATION_SENSOR), leftRotationISR, RISING);
//
//  Serial.begin(9600);
//  pinMode(TRIG_PIN, OUTPUT);
//  pinMode(ECHO_PIN, INPUT);
//}
//
//void moveForward(int rightSpeed, int leftSpeed) {
//  analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
//  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
//  analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
//  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
//}
//
//long getUltrasonicDistance() {
//  long duration, distance;
//  digitalWrite(TRIG_PIN, LOW);
//  delayMicroseconds(2);
//
//  digitalWrite(TRIG_PIN, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIG_PIN, LOW);
//  
//  duration = pulseIn(ECHO_PIN, HIGH);
//  distance = duration * 0.034 / 2;
//  
//  return distance;
//}
//
//void stopRobot() {
//  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
//  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
//  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
//  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
//}
//
//void loop() {
//  long distance = getUltrasonicDistance();
//
//  if (distance > 20) {
//    // Calculate the number of rotations based on pulse counts (20 pulses per rotation)
//    int rightRotations = rightPulseCount / 20;
//    int leftRotations = leftPulseCount / 20;
//
//    // Calculate the maximum number of rotations needed to reach the target distance
//    int targetRotations = distance / 10; // Assuming 10 cm per rotation
//
//    // Determine the speed based on the difference between actual and target rotations
//    int speedDifference = targetRotations - max(rightRotations, leftRotations);
//    int rightSpeed = 195 + speedDifference;
//    int leftSpeed = 247 + speedDifference;
//
//    // Move forward at the calculated speeds
//    moveForward(rightSpeed, leftSpeed);
//  } else {
//    // Stop the robot if the distance is 20 cm or less
//    stopRobot();
//  }
//  
//  delay(100);
//}
