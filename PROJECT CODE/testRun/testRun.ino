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
    pinMode(MOTOR_RIGHT_ROTATION_SENSOR, INPUT);
    pinMode(MOTOR_LEFT_ROTATION_SENSOR, INPUT);

    Serial.begin(9600);
}

void loop() {
    // Test the TurnRight function with 6 rotations
    TurnRight(6);
}

void TurnRight(int rotations) {

    delay(150);

    resetRotations();
    while (leftPulseCount < rotations) {
        analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
        analogWrite(MOTOR_RIGHT_BACKWARD, rightSpeed);
    }

    delay(150);

    // Continue moving forward if obstacle is cleared
    // Replace this section with your obstacle checking logic
  
    // moveForwardInRotations(5); // This line is commented out because moveForwardInRotations function is not defined
}

void stopRobot() {
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
}

void resetRotations() {
    rightPulseCount = 0;
    leftPulseCount = 0;
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
