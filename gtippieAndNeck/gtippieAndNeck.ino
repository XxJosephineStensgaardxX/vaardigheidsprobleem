// gripper
// #include <Servo.h>

// // Define the pin for the servo motor
// const int servoPin = 3;

// // Create a servo object
// Servo gripperServo;

// void setup() {
//   // Attach the servo to its pin
//   gripperServo.attach(servoPin);
  
//   // Initialize the servo position to the neutral position
//   gripperServo.writeMicroseconds(1500); // Neutral position: 90 degrees
//   delay(1000); // Delay for stability
// }

// void loop() {
//   // Open the gripper
//   openGripper();
//   delay(1000); // Adjust the delay as needed
  
//   // Close the gripper
//   closeGripper();
//   delay(1000); // Adjust the delay as needed
// }

// void openGripper() {
//   gripperServo.writeMicroseconds(2000); // Adjust the value according to the pulse width for opening
// }

// void closeGripper() {
//   gripperServo.writeMicroseconds(1000); // Adjust the value according to the pulse width for closing
// }


//grip and move
// #include <Servo.h>

// // Define the pin for the servo motor
// const int servoPin = 3;

// // Motor A (Drive Motor)
// const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
// const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5

// // Motor B (Steering Motor)
// const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
// const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

// // Create a servo object
// Servo gripperServo;

// void setup() {
//   // Set the motor control pins to outputs
//   pinMode(motorAPin1, OUTPUT);
//   pinMode(motorAPin2, OUTPUT);
//   pinMode(motorBPin1, OUTPUT);
//   pinMode(motorBPin2, OUTPUT);

//   // Attach the servo to its pin
//   gripperServo.attach(servoPin);
  
//   // Initialize the servo position to the neutral position
//   gripperServo.writeMicroseconds(1500); // Neutral position: 90 degrees
//   delay(1000); // Delay for stability
// }

// // Function to move the robot forward
// void moveForward() {
//   digitalWrite(motorAPin1, HIGH); // Motor A forward
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, HIGH); // Motor B forward
//   digitalWrite(motorBPin2, LOW);
// }

// // Function to turn the robot right
// void turnRight() {
//   digitalWrite(motorAPin1, HIGH); // Motor A forward
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW); // Motor B backward
//   digitalWrite(motorBPin2, HIGH);
// }

// // Function to open the gripper
// void openGripper() {
//   gripperServo.writeMicroseconds(2000); // Adjust the value according to the pulse width for opening
// }

// // Function to close the gripper
// void closeGripper() {
//   gripperServo.writeMicroseconds(1000); // Adjust the value according to the pulse width for closing
// }

// void loop() {
//   // Grab an object
//   closeGripper();
//   delay(1000); // Adjust the delay as needed
  
//   // Move forward for 2 seconds
//   moveForward();
//   delay(2000);

//   // Stop for 1 second
//   digitalWrite(motorAPin1, LOW);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, LOW);
//   delay(1000);

//   // Turn right for 1 second
//   turnRight();
//   delay(500);

//   // Stop for 1 second
//   digitalWrite(motorAPin1, LOW);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, LOW);
//   delay(1000);

//   // Turn right again for 1 second
//   turnRight();
//   delay(500);

//   // Stop for 1 second
//   digitalWrite(motorAPin1, LOW);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, LOW);
//   delay(1000);

//   // Release the gripper
//   openGripper();
//   delay(1000); // Adjust the delay as needed
// }

#include <Ultrasonic.h>
#include <Servo.h>

// Define pins for the ultrasonic sensor
#define trigPin 12
#define echoPin 13

// Motor A (Drive Motor)
const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5

// Motor B (Steering Motor)
const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

// Define pins for the neck swiveler servo
const int neckServoPin = 2; // Neck swiveler servo signal pin

// Create objects for Ultrasonic sensor and neck swiveler servo
Ultrasonic ultrasonic(trigPin, echoPin);
Servo neckSwiveler;

void setup() {
  // Set the motor control pins to outputs
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);

  // Attach the neck swiveler servo to its pin
  neckSwiveler.attach(neckServoPin);

  // Set up the serial communication
  Serial.begin(9600);
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(motorAPin1, HIGH); // Motor A forward
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, HIGH); // Motor B forward
  digitalWrite(motorBPin2, LOW);
}

// Function to stop the robot
void stopRobot() {
  digitalWrite(motorAPin1, LOW); // Motor A stop
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW); // Motor B stop
  digitalWrite(motorBPin2, LOW);
}

void loop() {
  // Read the distance from the ultrasonic sensor
  long distanceFront = ultrasonic.distanceRead();

  // Print the distance to the serial monitor
  Serial.print("Distance Front: ");
  Serial.println(distanceFront);

  // Make decision based on the distance readings
  if (distanceFront > 10) {
    // Move forward if no obstacle ahead
    moveForward();
  } else {
    // Stop if obstacle detected
    stopRobot();

    // Swivel the neck to the left
    neckSwiveler.write(0); // Adjust the angle as needed

    // Wait for the neck swiveler to reach the desired position
    delay(1000);

    // Read the distance again from the ultrasonic sensor after swiveling
    long distanceLeft = ultrasonic.distanceRead();

    // Swivel the neck to the right
    neckSwiveler.write(180); // Adjust the angle as needed

    // Wait for the neck swiveler to reach the desired position
    delay(1000);

    // Read the distance again from the ultrasonic sensor after swiveling
    long distanceRight = ultrasonic.distanceRead();

    // Reset the neck swiveler to its center position
    neckSwiveler.write(90); // Adjust the angle as needed

    // Decide the next move based on the distance readings
    if (distanceLeft > distanceRight) {
      // Turn left if more space on the left
      // Add code to turn left
    } else {
      // Turn right if more space on the right
      // Add code to turn right
    }
  }

  delay(100); // Add a short delay for stability
}

