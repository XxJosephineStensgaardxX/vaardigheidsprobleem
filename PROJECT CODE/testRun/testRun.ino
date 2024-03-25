#define SIDE_TRIG_PIN A1 // Analog pin for side sensor trigger
#define SIDE_ECHO_PIN A0 // Analog pin for side sensor echO

#define MOTOR_RIGHT_FORWARD 10 // used to be 7
#define MOTOR_RIGHT_BACKWARD 5
#define MOTOR_LEFT_FORWARD 6
#define MOTOR_LEFT_BACKWARD 11

const int rightSpeed = 230; // Speed for the right wheel
const int leftSpeed = 225;  // Speed for the left wheel

void setup() {
    Serial.begin(9600);
    pinMode(SIDE_TRIG_PIN, OUTPUT);
    pinMode(SIDE_ECHO_PIN, INPUT);

    // Motor pins setup
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
}

void loop() {
    // Move forward with specified speeds
    moveForward();

    // Center the robot based on the side sensor reading
    centerRobot();
    
    delay(1000); // Wait for 1 second before reading again
}

void moveForward() {
    // Set motor speeds for forward movement
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
}

void centerRobot() {
    float sideDistance = getDistanceSide();
    Serial.print("Side Distance: ");
    Serial.println(sideDistance);
    
    // Adjust motor speeds based on side distance
    if (sideDistance > 9.2) { // Far from the side obstacle
        Serial.println("sideDistance > 9.2");
        analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
        analogWrite(MOTOR_RIGHT_BACKWARD, 0);
        analogWrite(MOTOR_LEFT_FORWARD, 235);
        analogWrite(MOTOR_LEFT_BACKWARD, 0);
    } else if (sideDistance < 7.4) { // Close to the side obstacle
        Serial.println("sideDistance < 7.4");
        analogWrite(MOTOR_RIGHT_FORWARD, 255); //260
        analogWrite(MOTOR_RIGHT_BACKWARD, 0);
        analogWrite(MOTOR_LEFT_FORWARD, 215); //220
        analogWrite(MOTOR_LEFT_BACKWARD, 0);
    } else {
        // Robot is centered, continue moving forward with default speeds
        Serial.println("sideDistance ELSE");
        analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
        analogWrite(MOTOR_RIGHT_BACKWARD, 0);
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
        analogWrite(MOTOR_LEFT_BACKWARD, 0);
    }
}

float pulse(int TRIG_PIN, int ECHO_PIN) {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    float duration_us = pulseIn(ECHO_PIN, HIGH);
    return duration_us * 0.017; // Convert microseconds to centimeters
}

float getDistanceSide() {
    return round(pulse(SIDE_TRIG_PIN, SIDE_ECHO_PIN) * 100.0) / 100.0;
}
