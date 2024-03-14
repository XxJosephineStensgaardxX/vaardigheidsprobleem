//#include <Ultrasonic.h>
//#include <Servo.h>
//
//const int neckServoPin = 2; // Neck swiveler servo signal pin
//#define trigPin 12
//#define echoPin 13
//
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
//const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4
//
//void setup() {
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//  neckSwiveler.attach(neckServoPin);
//  Serial.begin(9600);
//}
//
//void moveForward() {
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//}
//
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, LOW);
//}
//
//void turnLeft() {
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, HIGH);
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//  delay(1000); // Adjust duration as needed
//}
//
//void turnRight() {
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, HIGH);
//  delay(1000); // Adjust duration as needed
//}
//
//void loop() {
//  long distanceFront = ultrasonic.read(); // Use read() method instead of distanceRead()
//  Serial.print("Distance Front: ");
//  Serial.println(distanceFront);
//
//  if (distanceFront > 10) {
//    moveForward(); // Drive forward if no obstacle ahead
//  } else {
//    stopRobot(); // Stop if obstacle detected
//    Serial.println("Obstacle detected!");
//
//    // Determine which direction to turn
//    if (random(0, 2) == 0) { // 50% chance to turn left
//      Serial.println("Turning left");
//      turnLeft();
//    } else { // 50% chance to turn right
//      Serial.println("Turning right");
//      turnRight();
//    }
//
//    // Resume forward movement
//    moveForward();
//  }
//}


//move code
//const int trigPin = 12;
//const int echoPin = 13;
//const int neckServoPin = 2;
//
//const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
//const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4
//
//int servoPos = 90; // Initial position of the servo
//
//void setup() {
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//
//  Serial.begin(9600);
//}
//
//void moveForward() {
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//}
//
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, LOW);
//}
//
//void turnLeft() {
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, HIGH);
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//  delay(300); // Adjust duration as needed
//}
//
//void turnRight() {
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, HIGH);
//  delay(300); // Adjust duration as needed
//}
//
//long getDistance() {
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  
//  long duration = pulseIn(echoPin, HIGH);
//  long distance = duration * 0.034 / 2; // Speed of sound is 34 cm/ms, and we're measuring round trip, hence dividing by 2
//  return distance;
//}
//
//void loop() {
//  long distanceFront = getDistance();
//  Serial.print("Distance Front: ");
//  Serial.println(distanceFront);
//
//  if (distanceFront > 5) {
//    moveForward(); // Drive forward if no obstacle ahead
//  } else {
//    stopRobot(); // Stop if obstacle detected
//    Serial.println("Obstacle detected!");
//
//    // Determine which direction to turn
//    if (random(0, 2) == 0) { // 50% chance to turn left
//      Serial.println("Turning left");
//      turnLeft();
//    } else { // 50% chance to turn right
//      Serial.println("Turning right");
//      turnRight();
//    }
//
//    // Resume forward movement
//    moveForward();
//  }
//}

//move and snese but no turn
//#include <Ultrasonic.h>
//#include <Servo.h>
//
//// Define pins for the ultrasonic sensor
//#define trigPin 12
//#define echoPin 13
//
//// Motor A (Drive Motor)
//const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
//
//// Motor B (Steering Motor)
//const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4
//
//// Define pins for the neck swiveler servo
//const int neckServoPin = 2; // Neck swiveler servo signal pin
//
//// Create objects for Ultrasonic sensor and neck swiveler servo
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//void setup() {
//  // Set the motor control pins to outputs
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//
//  // Attach the neck swiveler servo to its pin
//  neckSwiveler.attach(neckServoPin);
//
//  // Set up the serial communication
//  Serial.begin(9600);
//}
//
//// Function to move the robot forward
//void moveForward() {
//  digitalWrite(motorAPin1, HIGH); // Motor A forward
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, HIGH); // Motor B forward
//  digitalWrite(motorBPin2, LOW);
//}
//
//// Function to stop the robot
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW); // Motor A stop
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW); // Motor B stop
//  digitalWrite(motorBPin2, LOW);
//}
//
//// Function to swivel the neck to the specified angle
//void swivelNeck(int angle) {
//  neckSwiveler.write(angle); // Set the angle for the neck swiveler
//  delay(1000); // Wait for the neck swiveler to reach the desired position
//}
//
//void loop() {
//  // Read the distance from the ultrasonic sensor
//  long distanceFront = ultrasonic.read();
//
//  // Print the distance to the serial monitor
//  Serial.print("Distance Front: ");
//  Serial.println(distanceFront);
//
//  // Make decision based on the distance readings
//  if (distanceFront > 10) {
//    // Move forward if no obstacle ahead
//    moveForward();
//  } else {
//    // Stop if obstacle detected
//    stopRobot();
//
//    // Swivel the neck to the left
//    swivelNeck(0); // Adjust the angle as needed
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceLeft = ultrasonic.read();
//
//    // Swivel the neck to the right
//    swivelNeck(180); // Adjust the angle as needed
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceRight = ultrasonic.read();
//
//    // Reset the neck swiveler to its center position
//    swivelNeck(90); // Adjust the angle as needed
//
//    // Decide the next move based on the distance readings
//    if (distanceLeft > distanceRight) {
//      // Turn left if more space on the left
//      // Add code to turn left
//    } else {
//      // Turn right if more space on the right
//      // Add code to turn right
//    }
//  }
//
//  delay(100); // Add a short delay for stability
//}
//


//good code ut witrh delays and nbot millis
//#include <Ultrasonic.h>
//#include <Servo.h>
//
//const int neckServoPin = 2; // Neck swiveler servo signal pin
//#define trigPin 12
//#define echoPin 13
//
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
//const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4
//
//void setup() {
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//  neckSwiveler.attach(neckServoPin);
//  Serial.begin(9600);
//}
//
//void moveForward() {
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//}
//
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, LOW);
//}
//
//void turnLeft() {
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, HIGH);
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//  delay(250); // Adjust duration as needed
//}
//
//void turnRight() {
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, HIGH);
//  delay(250); // Adjust duration as needed
//}
//
//void swivelNeck(int angle) {
//  neckSwiveler.write(angle); // Set the angle for the neck swiveler
//  delay(1000); // Wait for the neck swiveler to reach the desired position
//}
//
//void loop() {
//  long distanceFront = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//  Serial.print("Distance Front: ");
//  Serial.print(distanceFront);
//  Serial.println(" cm");
//
//  if (distanceFront > 15) {
//    moveForward(); // Drive forward if distance is greater than 10 cm
//  } else {
//    stopRobot(); // Stop if obstacle detected
//    Serial.println("Obstacle detected!");
//
//    // Swivel the neck to the left
//    swivelNeck(0); // Adjust the angle as needed
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceLeft = ultrasonic.read();
//    Serial.print("Distance Left: ");
//    Serial.print(distanceLeft);
//    Serial.println(" cm");
//
//    // Swivel the neck to the right
//    swivelNeck(180); // Adjust the angle as needed
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceRight = ultrasonic.read();
//    Serial.print("Distance Right: ");
//    Serial.print(distanceRight);
//    Serial.println(" cm");
//
//    // Reset the neck swiveler to its center position
//    swivelNeck(90); // Adjust the angle as needed
//
//    // Determine the direction with more free space
//    if (distanceLeft > distanceRight) {
//      // Turn left if more space on the left
//      turnLeft();
//    } else if (distanceRight > distanceLeft) {
//      // Turn right if more space on the right
//      turnRight();
//    } else {
//      // If both sides have obstacles, turn right fully (make a half circle)
//      turnRight();
//      delay(1000); // Adjust duration as needed for a half circle
//    }
//  }
//
//  delay(100); // Add a short delay for stability
//}

//OUR MAIN CODE
//#include <Ultrasonic.h>
//#include <Servo.h>
//
//const int neckServoPin = 2; // Neck swiveler servo signal pin
//#define trigPin 12
//#define echoPin 13
//
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//const int motorRightPWM = 100; // Adjust as needed
//const int motorLeftPWM = 25; // Adjust as needed
//const int motorRightSpeedPin = 8; // PWM pin for controlling motor A speed
//const int motorLeftSpeedPin = 9; // PWM pin for controlling motor B speed
//
//const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
//const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4
//
//void setup() {
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//  neckSwiveler.attach(neckServoPin);
//  Serial.begin(9600);
//  analogWrite(motorRightSpeedPin, motorRightPWM);
//  analogWrite(motorLeftSpeedPin, motorLeftPWM);
//}
//
//void moveForward() {
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  analogWrite(motorRightSpeedPin, motorRightPWM);
//
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//  analogWrite(motorLeftSpeedPin, motorLeftPWM);
//}
//
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, LOW);
//}
//
//void turnLeft() {
//  unsigned long startTime = millis(); // Record the start time of the turn
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, HIGH);
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//  
//  // Continue turning left while checking for obstacles
//  while (millis() - startTime < 400) { // Turn for 250 milliseconds
//    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//    if (distance < 10) { // If obstacle detected, stop turning
//      stopRobot();
//      Serial.println("Obstacle detected!");
//      return;
//    }
//  }
//  stopRobot(); // Stop turning after 250 milliseconds
//}
//
//void turnRight() {
//  unsigned long startTime = millis(); // Record the start time of the turn
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, HIGH);
//  
//  // Continue turning right while checking for obstacles
//  while (millis() - startTime < 400) { // Turn for 250 milliseconds
//    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//    if (distance < 10) { // If obstacle detected, stop turning
//      stopRobot();
//      Serial.println("Obstacle detected!");
//      return;
//    }
//  }
//  stopRobot(); // Stop turning after 250 milliseconds
//}
//
//void turnAround() {
//  unsigned long startTime = millis(); // Record the start time of the turn
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, HIGH);
//  
//  // Continue turning right while checking for obstacles
//  while (millis() - startTime < 1500) { // Turn for 250 milliseconds
//    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//    if (distance < 10) { // If obstacle detected, stop turning
//      stopRobot();
//      Serial.println("Obstacle detected!");
//      return;
//    }
//  }
//  stopRobot(); // Stop turning after 250 milliseconds
//}
//
//void swivelNeck(int angle) {
//  neckSwiveler.write(angle); // Set the angle for the neck swiveler
//  delay(1000); // Wait for the neck swiveler to reach the desired position
//}
//
//void loop() {
//  long distanceFront = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//  Serial.print("Distance Front: ");
//  Serial.print(distanceFront);
//  Serial.println(" cm");
//
//  if (distanceFront > 15) {
//    moveForward(); // Drive forward if distance is greater than 15 cm
//  } else {
//    stopRobot(); // Stop if obstacle detected
//    Serial.println("Obstacle detected!");
//
//    // Swivel the neck to the left
//    swivelNeck(0); // Adjust the angle as needed
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceLeft = ultrasonic.read();
//    Serial.print("Distance Left: ");
//    Serial.print(distanceLeft);
//    Serial.println(" cm");
//
//    // Swivel the neck to the right
//    swivelNeck(180); // Adjust the angle as needed
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceRight = ultrasonic.read();
//    Serial.print("Distance Right: ");
//    Serial.print(distanceRight);
//    Serial.println(" cm");
//
//    // Reset the neck swiveler to its center position
//    swivelNeck(90); // Adjust the angle as needed
//
//    // Determine the direction with more free space
//    if (distanceLeft > distanceRight) {
//      // Turn left if more space on the left
//      turnLeft();
//    } else if (distanceRight > distanceLeft) {
//      // Turn right if more space on the right
//      turnRight();
//    } else {
//      // If both sides have obstacles, turn right fully (make a half circle)
//      turnAround();
//    }
//  }
//
//  delay(100); // Add a short delay for stability
//}

//our final code
//#include <Ultrasonic.h>
//#include <Servo.h>
//
//const int neckServoPin = 2; // Neck swiveler servo signal pin
//#define trigPin 12
//#define echoPin 13
//
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//const int motorRightPWM = 100; // Adjust as needed
//const int motorLeftPWM = 25; // Adjust as needed
//const int motorRightSpeedPin = 8; // PWM pin for controlling motor A speed
//const int motorLeftSpeedPin = 9; // PWM pin for controlling motor B speed
//
//const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
//const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4
//
//void setup() {
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//  neckSwiveler.attach(neckServoPin);
//  Serial.begin(9600);
//  analogWrite(motorRightSpeedPin, motorRightPWM);
//  analogWrite(motorLeftSpeedPin, motorLeftPWM);
//}
//
//void moveForward() {
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  analogWrite(motorRightSpeedPin, motorRightPWM);
//
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//  analogWrite(motorLeftSpeedPin, motorLeftPWM);
//}
//
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, LOW);
//}
//
//void turnLeft() {
//  unsigned long startTime = millis(); // Record the start time of the turn
//  digitalWrite(motorAPin1, LOW);
//  digitalWrite(motorAPin2, HIGH);
//  digitalWrite(motorBPin1, HIGH);
//  digitalWrite(motorBPin2, LOW);
//  
//  // Continue turning left while checking for obstacles
//  while (millis() - startTime < 400) { // Turn for 400 milliseconds
//    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//    if (distance < 10) { // If obstacle detected, stop turning
//      stopRobot();
//      Serial.println("Obstacle detected!");
//      return;
//    }
//  }
//  stopRobot(); // Stop turning after 400 milliseconds
//  swivelNeck(90); // Adjust the angle to make the direction straight
//}
//
//void turnRight() {
//  unsigned long startTime = millis(); // Record the start time of the turn
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, HIGH);
//  
//  // Continue turning right while checking for obstacles
//  while (millis() - startTime < 400) { // Turn for 400 milliseconds
//    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//    if (distance < 10) { // If obstacle detected, stop turning
//      stopRobot();
//      Serial.println("Obstacle detected!");
//      return;
//    }
//  }
//  stopRobot(); // Stop turning after 400 milliseconds
//  swivelNeck(90); // Adjust the angle to make the direction straight
//}
//
//void turnAround() {
//  unsigned long startTime = millis(); // Record the start time of the turn
//  digitalWrite(motorAPin1, HIGH);
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW);
//  digitalWrite(motorBPin2, HIGH);
//  
//  // Continue turning right while checking for obstacles
//  while (millis() - startTime < 1500) { // Turn for 1500 milliseconds
//    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//    if (distance < 10) { // If obstacle detected, stop turning
//      stopRobot();
//      Serial.println("Obstacle detected!");
//      return;
//    }
//  }
//  stopRobot(); // Stop turning after 1500 milliseconds
//  swivelNeck(90); // Adjust the angle to make the direction straight
//}
//
//void swivelNeck(int angle) {
//  neckSwiveler.write(angle); // Set the angle for the neck swiveler
//  delay(1000); // Wait for the neck swiveler to reach the desired position
//}
//
//void loop() {
//  long distanceFront = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//  Serial.print("Distance Front: ");
//  Serial.print(distanceFront);
//  Serial.println(" cm");
//
//  if (distanceFront > 15) {
//    moveForward(); // Drive forward if distance is greater than 15 cm
//  } else {
//    stopRobot(); // Stop if obstacle detected
//    Serial.println("Obstacle detected!");
//
//    // Swivel the neck to the left
//    swivelNeck(0); // Adjust the angle as needed
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceLeft = ultrasonic.read();
//    Serial.print("Distance Left: ");
//    Serial.print(distanceLeft);
//    Serial.println(" cm");
//
//    // Swivel the neck to the right
//    swivelNeck(180); // Adjust the angle as needed
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceRight = ultrasonic.read();
//    Serial.print("Distance Right: ");
//    Serial.print(distanceRight);
//    Serial.println(" cm");
//
//    // Reset the neck swiveler to its center position
//    swivelNeck(90); // Adjust the angle as needed
//
//    // Determine the direction with more free space
//    if (distanceLeft > distanceRight) {
//      // Turn left if more space on the left
//      turnLeft();
//    } else if (distanceRight > distanceLeft) {
//      // Turn right if more space on the right
//      turnRight();
//    } else {
//      // If both sides have obstacles, turn right fully (make a half circle)
//      turnAround();
//    }
//  }
//
//  delay(100); // Add a short delay for stability
//}


#include <Ultrasonic.h>
#include <Servo.h>

const int neckServoPin = 2; // Neck swiveler servo signal pin
#define trigPin 12
#define echoPin 13

Ultrasonic ultrasonic(trigPin, echoPin);
Servo neckSwiveler;

const int motorRightPWM = 100; // Adjust as needed
const int motorLeftPWM = 25; // Adjust as needed
const int motorRightSpeedPin = 8; // PWM pin for controlling motor A speed
const int motorLeftSpeedPin = 9; // PWM pin for controlling motor B speed

const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

void setup() {
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  neckSwiveler.attach(neckServoPin);
  Serial.begin(9600);
  analogWrite(motorRightSpeedPin, motorRightPWM);
  analogWrite(motorLeftSpeedPin, motorLeftPWM);
}

void moveForward() {
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  analogWrite(motorRightSpeedPin, motorRightPWM);

  digitalWrite(motorBPin1, HIGH);
  digitalWrite(motorBPin2, LOW);
  analogWrite(motorLeftSpeedPin, motorLeftPWM);
}

void stopRobot() {
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
}

void turnLeft() {
  unsigned long startTime = millis(); // Record the start time of the turn
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  digitalWrite(motorBPin1, HIGH);
  digitalWrite(motorBPin2, LOW);
  
  // Continue turning left while checking for obstacles
  while (millis() - startTime < 400) { // Turn for 400 milliseconds
    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
    if (distance < 10) { // If obstacle detected, stop turning
      stopRobot();
      Serial.println("Obstacle detected!");
      return;
    }
  }
  stopRobot(); // Stop turning after 400 milliseconds
  swivelNeck(90); // Adjust the angle to make the direction straight
}

void turnRight() {
  unsigned long startTime = millis(); // Record the start time of the turn
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
  
  // Continue turning right while checking for obstacles
  while (millis() - startTime < 400) { // Turn for 400 milliseconds
    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
    if (distance < 10) { // If obstacle detected, stop turning
      stopRobot();
      Serial.println("Obstacle detected!");
      return;
    }
  }
  stopRobot(); // Stop turning after 400 milliseconds
  swivelNeck(90); // Adjust the angle to make the direction straight
}

void turnAround() {
  unsigned long startTime = millis(); // Record the start time of the turn
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
  
  // Continue turning right while checking for obstacles
  while (millis() - startTime < 1500) { // Turn for 1500 milliseconds
    long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
    if (distance < 10) { // If obstacle detected, stop turning
      stopRobot();
      Serial.println("Obstacle detected!");
      return;
    }
  }
  stopRobot(); // Stop turning after 1500 milliseconds
  swivelNeck(90); // Adjust the angle to make the direction straight
}

void swivelNeck(int angle) {
  neckSwiveler.write(angle); // Set the angle for the neck swiveler
  delay(1000); // Wait for the neck swiveler to reach the desired position
}

void loop() {
  long distanceFront = ultrasonic.read(); // Read the distance from the ultrasonic sensor
  Serial.print("Distance Front: ");
  Serial.print(distanceFront);
  Serial.println(" cm");

  if (distanceFront > 15) {
    moveForward(); // Drive forward if distance is greater than 15 cm
  } else {
    stopRobot(); // Stop if obstacle detected
    Serial.println("Obstacle detected!");

    // Swivel the neck to the left
    swivelNeck(0); // Adjust the angle as needed

    // Read the distance again from the ultrasonic sensor after swiveling
    long distanceLeft = ultrasonic.read();
    Serial.print("Distance Left: ");
    Serial.print(distanceLeft);
    Serial.println(" cm");

    // Swivel the neck to the right
    swivelNeck(180); // Adjust the angle as needed

    // Read the distance again from the ultrasonic sensor after swiveling
    long distanceRight = ultrasonic.read();
    Serial.print("Distance Right: ");
    Serial.print(distanceRight);
    Serial.println(" cm");

    // Reset the neck swiveler to its center position
    swivelNeck(90); // Adjust the angle as needed

    // Determine the direction with more free space
    if (distanceLeft > distanceRight) {
      // Turn left if more space on the left
      turnLeft();
    } else if (distanceRight > distanceLeft) {
      // Turn right if more space on the right
      turnRight();
    } else {
      // If both sides have obstacles, turn right fully (make a half circle)
      turnAround();
    }
  }

  delay(100); // Add a short delay for stability
}


//// Ultrasonic class definition
//class Ultrasonic {
//  int trigPin;
//  int echoPin;
//public:
//  Ultrasonic(int trigPin, int echoPin) : trigPin(trigPin), echoPin(echoPin) {}
//  long distanceRead() {
//    digitalWrite(trigPin, LOW);
//    delayMicroseconds(2);
//    digitalWrite(trigPin, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(trigPin, LOW);
//    long duration = pulseIn(echoPin, HIGH);
//    long distance = duration * 0.034 / 2;
//    return distance;
//  }
//};
//
//// Servo class definition
//class Servo {
//  int pin;
//public:
//  void attach(int pin) {
//    this->pin = pin;
//    pinMode(pin, OUTPUT);
//  }
//  void write(int angle) {
//    angle = constrain(angle, 0, 180);
//    int pulseWidth = map(angle, 0, 180, 544, 2400);
//    digitalWrite(pin, HIGH);
//    delayMicroseconds(pulseWidth);
//    digitalWrite(pin, LOW);
//  }
//};
//
//// Define pins for the ultrasonic sensor
//#define trigPin 12
//#define echoPin 13
//
//// Motor A (Drive Motor)
//const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
//
//// Motor B (Steering Motor)
//const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4
//
//// Define pins for the neck swiveler servo
//const int neckServoPin = 2; // Neck swiveler servo signal pin
//
//// Create objects for Ultrasonic sensor and neck swiveler servo
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//void setup() {
//  // Set the motor control pins to outputs
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//
//  // Attach the neck swiveler servo to its pin
//  neckSwiveler.attach(neckServoPin);
//
//  // Set up the serial communication
//  Serial.begin(9600);
//}
//
//// Function to move the robot forward
//void moveForward() {
//  digitalWrite(motorAPin1, HIGH); // Motor A forward
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, HIGH); // Motor B forward
//  digitalWrite(motorBPin2, LOW);
//}
//
//// Function to stop the robot
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW); // Motor A stop
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW); // Motor B stop
//  digitalWrite(motorBPin2, LOW);
//}
//
//void loop() {
//  // Read the distance from the ultrasonic sensor
//  long distanceFront = ultrasonic.distanceRead();
//
//  // Print the distance to the serial monitor
//  Serial.print("Distance Front: ");
//  Serial.println(distanceFront);
//
//  // Make decision based on the distance readings
//  if (distanceFront > 10) {
//    // Move forward if no obstacle ahead
//    moveForward();
//  } else {
//    // Stop if obstacle detected
//    stopRobot();
//
//    // Look left and right only if an obstacle is detected in front
//
//    // Swivel the neck to the left
//    neckSwiveler.write(0); // Look to the left
//
//    // Wait for the neck swiveler to reach the desired position
//    delay(1000);
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceLeft = ultrasonic.distanceRead();
//
//    // Swivel the neck to the right
//    neckSwiveler.write(180); // Look to the right
//
//    // Wait for the neck swiveler to reach the desired position
//    delay(1000);
//
//    // Read the distance again from the ultrasonic sensor after swiveling
//    long distanceRight = ultrasonic.distanceRead();
//
//    // Reset the neck swiveler to its center position
//    neckSwiveler.write(90); // Look forward again
//
//    // Decide the next move based on the distance readings
//    if (distanceLeft > 10 || distanceRight > 10) {
//      // If there's space on either side, turn in that direction
//      if (distanceLeft > distanceRight) {
//        // Turn left if more space on the left
//        // Add code to turn left
//      } else {
//        // Turn right if more space on the right
//        // Add code to turn right
//      }
//    }
//  }
//
//  delay(100); // Add a short delay for stability
//}

//#include <Ultrasonic.h>
//#include <Servo.h>
//
//// Define pins for the ultrasonic sensor
//#define trigPin 12
//#define echoPin 13
//
//// Motor A (Drive Motor)
//const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
//const int motorASpeedPin = 9; // Motor A speed control pin
//
//// Motor B (Steering Motor)
//const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4
//const int motorBSpeedPin = 10; // Motor B speed control pin
//
//// Define pins for the neck swiveler servo
//const int neckServoPin = 2; // Neck swiveler servo signal pin
//
//// Create objects for Ultrasonic sensor and neck swiveler servo
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//void setup() {
//  // Set the motor control pins to outputs
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorASpeedPin, OUTPUT);
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//  pinMode(motorBSpeedPin, OUTPUT);
//
//  // Attach the neck swiveler servo to its pin
//  neckSwiveler.attach(neckServoPin);
//
//  // Set up the serial communication
//  Serial.begin(9600);
//}
//
//// Function to move the robot forward
//void moveForward(int speed) {
//  digitalWrite(motorAPin1, HIGH); // Motor A forward
//  digitalWrite(motorAPin2, LOW);
//  analogWrite(motorASpeedPin, speed); // Set speed for Motor A
//  digitalWrite(motorBPin1, HIGH); // Motor B forward
//  digitalWrite(motorBPin2, LOW);
//  analogWrite(motorBSpeedPin, speed); // Set speed for Motor B
//}
//
//// Function to stop the robot
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW); // Motor A stop
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW); // Motor B stop
//  digitalWrite(motorBPin2, LOW);
//}
//
//void loop() {
//  // Read the distance from the ultrasonic sensor
//  long distanceFront = ultrasonic.read();
//
//  // Print the distance to the serial monitor
//  Serial.print("Distance Front: ");
//  Serial.println(distanceFront);
//
//  // Make decision based on the distance readings
//  if (distanceFront > 10) {
//    // Move forward if no obstacle ahead
//    moveForward(255); // Full speed
//  } else {
//    // Stop if obstacle detected
//    stopRobot();
//    Serial.println("Obstacle detected!");
//
//    // Look left
//    neckSwiveler.write(0); // Adjust the angle as needed
//    delay(1000);
//    long distanceLeft = ultrasonic.read();
//
//    // Look right
//    neckSwiveler.write(180); // Adjust the angle as needed
//    delay(1000);
//    long distanceRight = ultrasonic.read();
//
//    // Reset the neck swiveler to its center position
//    neckSwiveler.write(90); // Adjust the angle as needed
//
//    // Determine the direction with more free space
//    if (distanceLeft > distanceRight) {
//      // Turn left
//      Serial.println("Turning left");
//      analogWrite(motorASpeedPin, 200); // Adjust speed for Motor A
//      analogWrite(motorBSpeedPin, 255); // Full speed for Motor B
//      digitalWrite(motorAPin1, LOW); // Motor A backward
//      digitalWrite(motorAPin2, HIGH); 
//      digitalWrite(motorBPin1, HIGH); // Motor B forward
//      digitalWrite(motorBPin2, LOW);
//    } else {
//      // Turn right
//      Serial.println("Turning right");
//      analogWrite(motorASpeedPin, 255); // Full speed for Motor A
//      analogWrite(motorBSpeedPin, 200); // Adjust speed for Motor B
//      digitalWrite(motorAPin1, HIGH); // Motor A forward
//      digitalWrite(motorAPin2, LOW); 
//      digitalWrite(motorBPin1, LOW); // Motor B backward
//      digitalWrite(motorBPin2, HIGH); 
//    }
//    // Delay to allow the robot to turn
//    delay(500);
//
//    // Resume forward movement
//    moveForward(255); // Full speed
//  }
//
//  delay(100); // Add a short delay for stability
//}

// we need to use rotation sensors , R1 = right, R2 = left

//#include <Ultrasonic.h>
//#include <Servo.h>
//
//// Define pins for the ultrasonic sensor
//#define trigPin 12
//#define echoPin 13
//
//// Motor A (Drive Motor)
//const int motorAPin1 = 7;      // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5;      // Motor A pin 2 connected to Arduino pin 5
//const int motorASpeedPin = 9;  // Motor A speed control pin
//const int motorAEncoderPin = 2; // Motor A encoder pin
//
//// Motor B (Steering Motor)
//const int motorBPin1 = 6;      // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4;      // Motor B pin 2 connected to Arduino pin 4
//const int motorBSpeedPin = 10; // Motor B speed control pin
//const int motorBEncoderPin = 3; // Motor B encoder pin
//
//// Rotation sensor variables
//volatile int motorAPulses = 0;
//volatile int motorBPulses = 0;
//
//// Define pins for the neck swiveler servo
//const int neckServoPin = A0; // Neck swiveler servo signal pin
//
//// Create objects for Ultrasonic sensor and neck swiveler servo
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//void setup() {
//  // Set the motor control pins to outputs
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorASpeedPin, OUTPUT);
//  pinMode(motorAEncoderPin, INPUT_PULLUP);
//
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//  pinMode(motorBSpeedPin, OUTPUT);
//  pinMode(motorBEncoderPin, INPUT_PULLUP);
//
//  // Attach interrupts for rotation sensors
//  attachInterrupt(digitalPinToInterrupt(motorAEncoderPin), countMotorAPulses, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(motorBEncoderPin), countMotorBPulses, CHANGE);
//
//  // Attach the neck swiveler servo to its pin
//  neckSwiveler.attach(neckServoPin);
//
//  // Set up the serial communication
//  Serial.begin(9600);
//}
//
//// Function to count pulses for Motor A
//void countMotorAPulses() {
//  motorAPulses++;
//}
//
//// Function to count pulses for Motor B
//void countMotorBPulses() {
//  motorBPulses++;
//}
//
//// Function to move the robot forward
//void moveForward(int speed) {
//  digitalWrite(motorAPin1, HIGH); // Motor A forward
//  digitalWrite(motorAPin2, LOW);
//  analogWrite(motorASpeedPin, speed); // Set speed for Motor A
//  digitalWrite(motorBPin1, HIGH); // Motor B forward
//  digitalWrite(motorBPin2, LOW);
//  analogWrite(motorBSpeedPin, speed); // Set speed for Motor B
//}
//
//// Function to stop the robot
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW); // Motor A stop
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW); // Motor B stop
//  digitalWrite(motorBPin2, LOW);
//}
//
//void loop() {
//  // Read the distance from the ultrasonic sensor
//  long distanceFront = ultrasonic.read();
//
//  // Print the distance to the serial monitor
//  Serial.print("Distance Front: ");
//  Serial.println(distanceFront);
//
//  // Make decision based on the distance readings
//  if (distanceFront > 10) {
//    // Move forward if no obstacle ahead
//    moveForward(255); // Full speed
//  } else {
//    // Stop if obstacle detected
//    stopRobot();
//    Serial.println("Obstacle detected!");
//
//    // Look left
//    neckSwiveler.write(0); // Adjust the angle as needed
//    delay(1000);
//    long distanceLeft = ultrasonic.read();
//
//    // Look right
//    neckSwiveler.write(180); // Adjust the angle as needed
//    delay(1000);
//    long distanceRight = ultrasonic.read();
//
//    // Reset the neck swiveler to its center position
//    neckSwiveler.write(90); // Adjust the angle as needed
//
//    // Determine the direction with more free space
//    if (distanceLeft > distanceRight) {
//      // Turn left
//      Serial.println("Turning left");
//      while (motorAPulses < 20 && motorBPulses < 20) {
//        digitalWrite(motorAPin1, HIGH); // Motor A forward
//        digitalWrite(motorAPin2, LOW);
//        digitalWrite(motorBPin1, HIGH); // Motor B forward
//        digitalWrite(motorBPin2, LOW);
//      }
//    } else {
//      // Turn right
//      Serial.println("Turning right");
//      while (motorAPulses < 20 && motorBPulses < 20) {
//        digitalWrite(motorAPin1, HIGH); // Motor A forward
//        digitalWrite(motorAPin2, LOW);
//        digitalWrite(motorBPin1, HIGH); // Motor B forward
//        digitalWrite(motorBPin2, LOW);
//      }
//    }
//    // Reset pulse counts
//    motorAPulses = 0;
//    motorBPulses = 0;
//
//    // Resume forward movement
//    moveForward(255); // Full speed
//  }
//
//  delay(100); // Add a short delay for stability
//}


//move with pulses and rrotations and nothung else 

//#include <Ultrasonic.h>
//#include <Servo.h>
//
//// Define pins for the ultrasonic sensor
//#define trigPin 12
//#define echoPin 13
//
//// Motor A (Drive Motor)
//const int motorAPin1 = 7;      // Motor A pin 1 connected to Arduino pin 7
//const int motorAPin2 = 5;      // Motor A pin 2 connected to Arduino pin 5
//const int motorASpeedPin = 9;  // Motor A speed control pin
//const int motorAEncoderPin = 2; // Motor A encoder pin
//const int motorAPulsesPerRotation = 20; // Adjust as needed
//
//// Motor B (Steering Motor)
//const int motorBPin1 = 6;      // Motor B pin 1 connected to Arduino pin 6
//const int motorBPin2 = 4;      // Motor B pin 2 connected to Arduino pin 4
//const int motorBSpeedPin = 10; // Motor B speed control pin
//const int motorBEncoderPin = 3; // Motor B encoder pin
//const int motorBPulsesPerRotation = 20; // Adjust as needed
//
//// Rotation sensor variables
//volatile int motorAPulses = 0;
//volatile int motorBPulses = 0;
//
//// Define pins for the neck swiveler servo
//const int neckServoPin = A0; // Neck swiveler servo signal pin
//
//// Create objects for Ultrasonic sensor and neck swiveler servo
//Ultrasonic ultrasonic(trigPin, echoPin);
//Servo neckSwiveler;
//
//void setup() {
//  // Set the motor control pins to outputs
//  pinMode(motorAPin1, OUTPUT);
//  pinMode(motorAPin2, OUTPUT);
//  pinMode(motorASpeedPin, OUTPUT);
//  pinMode(motorAEncoderPin, INPUT_PULLUP);
//
//  pinMode(motorBPin1, OUTPUT);
//  pinMode(motorBPin2, OUTPUT);
//  pinMode(motorBSpeedPin, OUTPUT);
//  pinMode(motorBEncoderPin, INPUT_PULLUP);
//
//  // Attach interrupts for rotation sensors
//  attachInterrupt(digitalPinToInterrupt(motorAEncoderPin), countMotorAPulses, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(motorBEncoderPin), countMotorBPulses, CHANGE);
//
//  // Attach the neck swiveler servo to its pin
//  neckSwiveler.attach(neckServoPin);
//
//  // Set up the serial communication
//  Serial.begin(9600);
//}
//
//// Function to count pulses for Motor A
//void countMotorAPulses() {
//  motorAPulses++;
//}
//
//// Function to count pulses for Motor B
//void countMotorBPulses() {
//  motorBPulses++;
//}
//
//// Function to move the robot forward
//void moveForward(int speed) {
//  digitalWrite(motorAPin1, HIGH); // Motor A forward
//  digitalWrite(motorAPin2, LOW);
//  analogWrite(motorASpeedPin, speed); // Set speed for Motor A
//  digitalWrite(motorBPin1, HIGH); // Motor B forward
//  digitalWrite(motorBPin2, LOW);
//  analogWrite(motorBSpeedPin, speed); // Set speed for Motor B
//}
//
//// Function to stop the robot
//void stopRobot() {
//  digitalWrite(motorAPin1, LOW); // Motor A stop
//  digitalWrite(motorAPin2, LOW);
//  digitalWrite(motorBPin1, LOW); // Motor B stop
//  digitalWrite(motorBPin2, LOW);
//}
//
//void loop() {
//  // Read the distance from the ultrasonic sensor
//  long distanceFront = ultrasonic.read();
//
//  // Print the distance to the serial monitor
//  Serial.print("Distance Front: ");
//  Serial.println(distanceFront);
//
//  // Make decision based on the distance readings
//  if (distanceFront > 10) {
//    // Move forward if no obstacle ahead
//    moveForward(255); // Full speed
//  } else {
//    // Stop if obstacle detected
//    stopRobot();
//    Serial.println("Obstacle detected!");
//
//    // Look left
//    neckSwiveler.write(0); // Adjust the angle as needed
//    while (motorAPulses < motorAPulsesPerRotation && motorBPulses < motorBPulsesPerRotation) {
//      // Wait for motors to rotate
//    }
//    // Reset pulse counts
//    motorAPulses = 0;
//    motorBPulses = 0;
//
//    // Look right
//    neckSwiveler.write(180); // Adjust the angle as needed
//    while (motorAPulses < motorAPulsesPerRotation && motorBPulses < motorBPulsesPerRotation) {
//      // Wait for motors to rotate
//    }
//    // Reset pulse counts
//    motorAPulses = 0;
//    motorBPulses = 0;
//
//    // Reset the neck swiveler to its center position
//    neckSwiveler.write(90); // Adjust the angle as needed
//
//    // Resume forward movement
//    moveForward(255); // Full speed
//  }
//}
