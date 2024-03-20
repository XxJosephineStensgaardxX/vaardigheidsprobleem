// #include <Ultrasonic.h>
// #include <Servo.h>

// const int neckServoPin = 2;       // Neck swiveler servo signal pin
// #define trigPin 12
// #define echoPin 13

// Ultrasonic ultrasonic(trigPin, echoPin);
// Servo neckSwiveler;

// const int motorRightPWM = 100;     // Adjust as needed
// const int motorLeftPWM = 25;       // Adjust as needed
// const int motorRightSpeedPin = 8;  // PWM pin for controlling motor A speed
// const int motorLeftSpeedPin = 9;   // PWM pin for controlling motor B speed

// const int motorAPin1 = 7;          // Motor A pin 1 connected to Arduino pin 7
// const int motorAPin2 = 5;          // Motor A pin 2 connected to Arduino pin 5
// const int motorBPin1 = 6;          // Motor B pin 1 connected to Arduino pin 6
// const int motorBPin2 = 4;          // Motor B pin 2 connected to Arduino pin 4

// void setup() {
//   pinMode(motorAPin1, OUTPUT);
//   pinMode(motorAPin2, OUTPUT);
//   pinMode(motorBPin1, OUTPUT);
//   pinMode(motorBPin2, OUTPUT);
//   neckSwiveler.attach(neckServoPin);
//   Serial.begin(9600);
//   analogWrite(motorRightSpeedPin, motorRightPWM);
//   analogWrite(motorLeftSpeedPin, motorLeftPWM);
// }

// void moveForward() {
//   digitalWrite(motorAPin1, HIGH);
//   digitalWrite(motorAPin2, LOW);
//   analogWrite(motorRightSpeedPin, motorRightPWM);

//   digitalWrite(motorBPin1, HIGH);
//   digitalWrite(motorBPin2, LOW);
//   analogWrite(motorLeftSpeedPin, motorLeftPWM);
// }

// void stopRobot() {
//   digitalWrite(motorAPin1, LOW);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, LOW);
// }

// void turnLeft() {
//   unsigned long startTime = millis(); // Record the start time of the turn
//   digitalWrite(motorAPin1, LOW);
//   digitalWrite(motorAPin2, HIGH);
//   digitalWrite(motorBPin1, HIGH);
//   digitalWrite(motorBPin2, LOW);
  
//   // Continue turning left while checking for obstacles
//   while (millis() - startTime < 400) { // Turn for 400 milliseconds
//     long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//     if (distance < 10) { // If obstacle detected, stop turning
//       stopRobot();
//       Serial.println("Obstacle detected!");
//       return;
//     }
//   }
//   stopRobot(); // Stop turning after 400 milliseconds
//   swivelNeck(90); // Adjust the angle to make the direction straight
// }

// void turnRight() {
//   unsigned long startTime = millis(); // Record the start time of the turn
//   digitalWrite(motorAPin1, HIGH);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, HIGH);
  
//   // Continue turning right while checking for obstacles
//   while (millis() - startTime < 400) { // Turn for 400 milliseconds
//     long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//     if (distance < 10) { // If obstacle detected, stop turning
//       stopRobot();
//       Serial.println("Obstacle detected!");
//       return;
//     }
//   }
//   stopRobot(); // Stop turning after 400 milliseconds
//   swivelNeck(90); // Adjust the angle to make the direction straight
// }

// void turnAround() {
//   unsigned long startTime = millis(); // Record the start time of the turn
//   digitalWrite(motorAPin1, HIGH);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, HIGH);
  
//   // Continue turning right while checking for obstacles
//   while (millis() - startTime < 1500) { // Turn for 1500 milliseconds
//     long distance = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//     if (distance < 10) { // If obstacle detected, stop turning
//       stopRobot();
//       Serial.println("Obstacle detected!");
//       return;
//     }
//   }
//   stopRobot(); // Stop turning after 1500 milliseconds
//   swivelNeck(90); // Adjust the angle to make the direction straight
// }

// void swivelNeck(int angle) {
//   neckSwiveler.write(angle); // Set the angle for the neck swiveler
//   delay(1000); // Wait for the neck swiveler to reach the desired position
// }

// void loop() {
//   long distanceFront = ultrasonic.read(); // Read the distance from the ultrasonic sensor
//   Serial.print("Distance Front: ");
//   Serial.print(distanceFront);
//   Serial.println(" cm");

//   if (distanceFront > 15) {
//     moveForward(); // Drive forward if distance is greater than 15 cm
//   } else {
//     stopRobot(); // Stop if obstacle detected
//     Serial.println("Obstacle detected!");

//     // Swivel the neck to the left
//     swivelNeck(0); // Adjust the angle as needed

//     // Read the distance again from the ultrasonic sensor after swiveling
//     long distanceLeft = ultrasonic.read();
//     Serial.print("Distance Left: ");
//     Serial.print(distanceLeft);
//     Serial.println(" cm");

//     // Swivel the neck to the right
//     swivelNeck(180); // Adjust the angle as needed

//     // Read the distance again from the ultrasonic sensor after swiveling
//     long distanceRight = ultrasonic.read();
//     Serial.print("Distance Right: ");
//     Serial.print(distanceRight);
//     Serial.println(" cm");

//     // Reset the neck swiveler to its center position
//     swivelNeck(90); // Adjust the angle as needed

//     // Determine the direction with more free space
//     if (distanceLeft > distanceRight) {
//       // Turn left if more space on the left
//       turnLeft();
//     } else if (distanceRight > distanceLeft) {
//       // Turn right if more space on the right
//       turnRight();
//     } else {
//       // If both sides have obstacles, turn right fully (make a half circle)
//       turnAround();
//     }
//   }

//   delay(100); // Add a short delay for stability
// }

// remove swivelneck function coz its looping that being i confused, instead: make it look right and sense pulse width (distnce) and gio if not free then do the same with left

#include <Ultrasonic.h>
#include <Servo.h>

const int neckServoPin = 8;
#define trigPin 12
#define echoPin 13

Ultrasonic ultrasonic(trigPin, echoPin);
Servo neckSwiveler;

const int motorRightPWM = 102;
const int motorLeftPWM = 25;
const int motorRightSpeedPin = 3;
const int motorLeftSpeedPin = 2;

const int motorAPin1 = 7;
const int motorAPin2 = 5;
const int motorBPin1 = 6;
const int motorBPin2 = 4;

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
//motor B is for left motor
void moveForward() {
  analogWrite(motorAPin1, 255);
  digitalWrite(motorAPin2, LOW);
  analogWrite(motorRightSpeedPin, motorRightPWM);
  analogWrite(motorBPin1, 250);
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
  unsigned long startTime = millis();
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  digitalWrite(motorBPin1, HIGH);
  digitalWrite(motorBPin2, LOW);
  while (millis() - startTime < 345) {
    long distance = ultrasonic.read();
    if (distance < 10) {
      stopRobot();
      return;
    }
  }
  stopRobot();
  swivelNeck(90);
}

void turnRight() {
  unsigned long startTime = millis();
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
  while (millis() - startTime < 345) {
    long distance = ultrasonic.read();
    if (distance < 10) {
      stopRobot();
      return;
    }
  }
  stopRobot();
  swivelNeck(90);
}

void turnAround() {
  unsigned long startTime = millis();
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
  while (millis() - startTime < 1500) {
    long distance = ultrasonic.read();
    if (distance < 10) {
      stopRobot();
      return;
    }
  }
  stopRobot();
  swivelNeck(90);
}

void swivelNeck(int angle) {
  neckSwiveler.write(angle);
  delay(1000);
}

void loop() {
  long distanceFront = ultrasonic.read();
  if (distanceFront > 15) {
    moveForward();
  } else {
    stopRobot();
    swivelNeck(0);
    long distanceLeft = ultrasonic.read();
    swivelNeck(180);
    long distanceRight = ultrasonic.read();
    swivelNeck(90);
    if (distanceLeft > distanceRight) {
      turnLeft();
    } else if (distanceRight > distanceLeft) {
      turnRight();
    } else if(distanceRight = distanceLeft) {
      turnRight();
    } else {
      turnAround();
    }
  }
  delay(100);
}
