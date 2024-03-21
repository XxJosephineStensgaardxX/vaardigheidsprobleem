#define SERVO_NECK_PIN 8
#define GRIPPER_PIN 9
#define FRONT_TRIG_PIN 12
#define FRONT_ECHO_PIN 13
#define SIDE_TRIG_PIN A1 //analogPin
#define SIDE_ECHO_PIN A0

#define MOTOR_RIGHT_ROTATION_SENSOR 3
#define MOTOR_LEFT_ROTATION_SENSOR 2

#define MOTOR_RIGHT_FORWARD 10 // used to be 7
#define MOTOR_RIGHT_BACKWARD 5
#define MOTOR_LEFT_FORWARD 6
#define MOTOR_LEFT_BACKWARD 11 //used to be 4

const int rightSpeed = 230;
const int leftSpeed = 225;

volatile int rightPulseCount = 0;
volatile int leftPulseCount = 0;

int targetRotations = 20;
bool movingForward = false;

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
}

void loop() {

    moveForwardInRotations(targetRotations);
}

void centerRobot() {
    long sideDistance = getDistanceSide();

    if (sideDistance > 9.2) { // Far from the side obstacle
        analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
        analogWrite(MOTOR_LEFT_FORWARD, 250);
        analogWrite(MOTOR_LEFT_BACKWARD, LOW);
        analogWrite(MOTOR_RIGHT_BACKWARD, LOW);
        
    } else if (sideDistance < 7.2) { // Close to the side obstacle
        analogWrite(MOTOR_RIGHT_FORWARD, 240);
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed); // Adjust motor speeds for centering
        analogWrite(MOTOR_LEFT_BACKWARD, LOW);
        analogWrite(MOTOR_RIGHT_BACKWARD, LOW);
        
    } else {
        // Robot is centered, continue moving forward with default speeds
        analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
        analogWrite(MOTOR_LEFT_BACKWARD, LOW);
        analogWrite(MOTOR_RIGHT_BACKWARD, LOW);
    }
}

void moveForwardInRotations(int rotations) {
    if (!movingForward) {
        resetRotations();
        movingForward = true;
    }

    centerRobot(); // Call the centerRobot function to center the robot while moving

    while (rightPulseCount < rotations && leftPulseCount < rotations) {
        // Continuously sense distance while moving forward
        long distance = getDistanceForward();
        if (distance < 10) {
            stopRobot();
            determineTurn(); // Call determineTurn when an obstacle is detected
            return; // Exit the function to stop moving forward
        }

        // Continue moving forward with default speeds
        analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
        analogWrite(MOTOR_RIGHT_BACKWARD, LOW);
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
        analogWrite(MOTOR_LEFT_BACKWARD, LOW);
    }

    stopRobot();
    movingForward = false;
}

void determineTurn() {
    long distanceForward = getDistanceForward();
    long sideDistance = getDistanceSide();

    if (distanceForward < 20) { // If obstacle detected in front
        // Perform a swivel to check for space on each side
        swivelNeck(90);
        wait(1000);
        long rightDistance = getDistanceForward();
        Serial.print("Right distance: ");
        Serial.println(rightDistance);
        swivelNeck(-90);
        wait(1000);
        long leftDistance = getDistanceForward();
        Serial.print("Left distance: ");
        Serial.println(leftDistance);
        swivelNeck(0);

        if (leftDistance > rightDistance) {
            // Turn right
            turnRight(4); 
        } else if (rightDistance > leftDistance){
            // Turn left
            turnLeft(4); 
        } else if(leftDistance == rightDistance) {
            turnRight(4); // Or you can turn left or right as per your preference
        } else {
            turnRight(8); // Arbitrary turn if distance comparison fails
        }
    }  
}

    

    //side sensor
    //else if (sideDistance > 20) { 
    //      
    //        if (getDistanceSide() > getDistanceSide()) {
    //            // Turn right
    //            TurnRight(6); 
    //        } else {
    //            // Turn left
    //            TurnLeft(6); 
    //        }
    //    } 
    
//    else {
//            // Continue moving forward
//             moveForwardInRotations(targetRotations);
//        }
//}

void stopRobot() {
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
}

void turnLeft(int rotations) {
    stopRobot();
    wait(150);

    resetRotations();
    while(rightPulseCount < rotations) {
        analogWrite(MOTOR_LEFT_FORWARD, LOW);
        analogWrite(MOTOR_RIGHT_FORWARD, leftSpeed);
        analogWrite(MOTOR_RIGHT_BACKWARD, LOW);
        analogWrite(MOTOR_LEFT_BACKWARD, rightSpeed);
    }

    wait(150);

    // Continue moving forward if obstacle is cleared
    long distance = getDistanceForward();
    if (distance > 15) {
        wait(100);
        moveForwardInRotations(5);
    }
    
    wait(2000);
}

void turnRight(int rotations) {
    stopRobot();
    wait(150);
    
    resetRotations();
    while (leftPulseCount < rotations) {
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
        analogWrite(MOTOR_LEFT_BACKWARD, LOW);
        analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
        analogWrite(MOTOR_RIGHT_FORWARD, LOW);
    }

   wait(150);
   
    // Continue moving forward if obstacle is cleared
    long distance = getDistanceForward();
    if (distance > 15) {
        wait(100);
        moveForwardInRotations(5);
    }

    wait(2000);
}

void resetRotations() {
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

float getDistanceForward() {
  return round(pulse(FRONT_TRIG_PIN, FRONT_ECHO_PIN) * 100.0) / 100.0;
}

float getDistanceSide() {
  return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;
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

void swivelNeck(int angle) {
    for (int i = 0; i < 10; i++) {
        int pulseWidth = map(angle, -90, 90, 600, 2400); // Map the angle to pulse width
        digitalWrite(SERVO_NECK_PIN, HIGH);             // Set the pin high
        delayMicroseconds(pulseWidth);                   // Delay for the calculated pulse width
        digitalWrite(SERVO_NECK_PIN, LOW);              // Set the pin low
        wait(20);                                       // Add a small delay to ensure the servo has enough time to respond
    }
}

void wait(int timeToWait) {
    unsigned long startTime = millis(); // Get the current time

    // Loop until the desired time has passed
    while (millis() - startTime < timeToWait) {
        // Do nothing, just keep looping
    }
}
