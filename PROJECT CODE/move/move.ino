// // Motor A
// const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
// const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5

// // Motor B
// const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
// const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

// void setup() {
//   // Set the motor control pins to outputs
//   pinMode(motorAPin1, OUTPUT);
//   pinMode(motorAPin2, OUTPUT);

//   pinMode(motorBPin1, OUTPUT);
//   pinMode(motorBPin2, OUTPUT);
// }

// void loop() {
//   // Example: Move both motors forward

//   // Motor A forward
//   digitalWrite(motorAPin1, HIGH);
//   digitalWrite(motorAPin2, LOW);

//   // Motor B forward
//   digitalWrite(motorBPin1, HIGH);
//   digitalWrite(motorBPin2, LOW);

//   delay(2000); // Wait for 2 seconds

//   // Example: Stop both motors
//   digitalWrite(motorAPin1, LOW);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, LOW);

//   delay(2000); // Wait for 2 seconds before repeating
// }

// tuning 180 and moving forward
// Motor A (Drive Motor)
// const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
// const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5

// // Motor B (Steering Motor)
// const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
// const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

// void setup() {
//   // Set the motor control pins to outputs
//   pinMode(motorAPin1, OUTPUT);
//   pinMode(motorAPin2, OUTPUT);
//   pinMode(motorBPin1, OUTPUT);
//   pinMode(motorBPin2, OUTPUT);
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

// void stopMotors() {
//   digitalWrite(motorAPin1, LOW);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, LOW);
// }

// void loop() {
//   // Move forward for 2 seconds
//   moveForward();
//   delay(2000);

//   // Stop for 1 second
//   stopMotors();
//   delay(1000);

//   // Turn right for 1 second
//   turnRight();
//   delay(1000);

//   // Stop for 1 second
//   stopMotors();
//   delay(1000);
// }


// turning right normally
// const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
// const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5

// // Motor B (Steering Motor)
// const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
// const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

// void setup() {
//   // Set the motor control pins to outputs
//   pinMode(motorAPin1, OUTPUT);
//   pinMode(motorAPin2, OUTPUT);
//   pinMode(motorBPin1, OUTPUT);
//   pinMode(motorBPin2, OUTPUT);
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

// void stopMotors() {
//   digitalWrite(motorAPin1, LOW);
//   digitalWrite(motorAPin2, LOW);
//   digitalWrite(motorBPin1, LOW);
//   digitalWrite(motorBPin2, LOW);
// }

// void loop() {
//   // Move forward for 2 seconds
//   moveForward();
//   delay(2000);

//   // Stop for 1 second
//   stopMotors();
//   delay(1000);

//   // Turn right for 0.5 second
//   turnRight();
//   delay(500);

//   // Stop for 1 second
//   stopMotors();
//   delay(1000);
// }
