#define SERVO_NECK_PIN 8
#define FRONT_TRIG_PIN 12
#define FRONT_ECHO_PIN 13

const int thresholdDistance = 10; // Threshold distance to detect obstacles

void setup() {
  pinMode(SERVO_NECK_PIN, OUTPUT);
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  long distance = getDistanceForward();
  if (distance < thresholdDistance) {
    // Obstacle detected, stop and turn the neck
    Serial.println("Obstacle detected!");
    // Rotate the neck to the right side
    swivelNeck(90); 
    delay(1000); // Wait for 1 second
    // Rotate the neck to the left side
    swivelNeck(-90);
    delay(1000); // Wait for 1 second
  } else {
    // No obstacle detected, don't turn the neck
    Serial.println("No obstacle detected.");
  }
  delay(500); // Wait for a short time before the next iteration
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

void swivelNeck(int angle) {
  int pulseWidth = map(angle, -90, 90, 600, 2400); // Map the angle to pulse width
  digitalWrite(SERVO_NECK_PIN, HIGH);             // Set the pin high
  delayMicroseconds(pulseWidth);                    // Delay for the calculated pulse width
  digitalWrite(SERVO_NECK_PIN, LOW);              // Set the pin low
  delay(20);                                       // Add a small delay to ensure the servo has enough time to respond
}


//void checkRightLeft() {
//  // Measure forward distance
//  long distanceForward = getDistanceForward();
//  
//  // Check if distance forward is less than 15
//  if (distanceForward < 15) {
//    // If forward distance is less than 15, stop the robot and make a decision based on side distances
//    stopRobot();
//    
//    // Look right
//    swivelNeck(90);
//    delay(500); // Adjust the delay time as needed
//    long distanceRight = getDistanceSide();
//    
//    // Look left
//    swivelNeck(-90);
//    delay(500); // Adjust the delay time as needed
//    long distanceLeft = getDistanceSide();
//    
//    // Determine which side is free and turn
//    if (distanceLeft > distanceRight) {
//      turnLeft();
//    } else if (distanceRight > distanceLeft) {
//      turnRight();
//    } else {
//      turnAround();
//    }
//  } else {
//    // If forward distance is greater than or equal to 15, continue moving forward
//    moveForwardInRotations(20);
//  }
//}
