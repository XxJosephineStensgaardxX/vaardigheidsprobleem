const int MOTOR_RIGHT_FORWARD = 6;  // right forward
const int MOTOR_RIGHT_BACKWARD = 11;   // right back
const int MOTOR_LEFT_FORWARD = 5;   // left forward
const int MOTOR_LEFT_BACKWARD = 10;  // left backward



void setup() {
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);

  moveForward();
//  delay(5000);  // 5000 milliseconds = 5 seconds
//  stopRobot();
}

void loop() {
}

void moveForward() {
  // Set motor directions for forward movement
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);

  // Set motor speeds
  analogWrite(MOTOR_RIGHT_FORWARD, 230); 
  analogWrite(MOTOR_LEFT_FORWARD, 205);// Adjust speed as needed// Adjust speed as needed

  //const int rightSpeed = 230;
// const int leftSpeed = 225;
}
