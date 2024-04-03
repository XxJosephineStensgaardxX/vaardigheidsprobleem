//#include <Arduino.h>
//
//// Function prototypes
//boolean blackDetected();
//void queryIRSensors();
//
//// Mock sensorValues array
//int sensorValues[6] = {0, 0, 0, 0, 0, 0}; // Initialize all sensors as not detecting black
//
//void setup() {
//  Serial.begin(9600);
//}
//
//void loop() {
//  // Debugging and testing code
//  Serial.println("Testing black detection...");
//  boolean result = blackDetected();
//  Serial.print("Black detected: ");
//  Serial.println(result ? "Yes" : "No");
//  Serial.println("------------------------");
//  delay(2000); // Delay between tests
//}
//
//boolean blackDetected() {
//  short sum = 0;
//
//  Serial.println("black function");
//
//  queryIRSensors();
//  for (int i = 0; i < 6; i++) {
//    if (sensorValues[i]) {
//      Serial.println("black if function");
//      sum++;
//    }
//    else {
//      Serial.print("Sensor ");
//      Serial.print(i);
//      Serial.println(" did not detect black");
//    }
//  }
//
//  Serial.print("Sum: ");
//  Serial.println(sum);
//
//  return sum == 6; // Expecting sum to be 6 when all sensors detect black
//}
//
//void queryIRSensors() {
//  // Mock sensor readings
//  for (int i = 0; i < 6; i++) {
//    // Simulate some sensors detecting black
//    sensorValues[i] = (i % 2 == 0) ? 1 : 0; // Every other sensor detects black
//    Serial.print("Sensor ");
//    Serial.print(i);
//    Serial.print(" value: ");
//    Serial.println(sensorValues[i]);
//  }
//}

#include <Arduino.h>

const int sensorCount = 6; // Number of sensors in your analog line sensor
const int sensorPins[sensorCount] = {A1, A2, A3, A4, A5, A6}; // Analog sensor pins (removed pins: A0 and A7)

int sensorValues[sensorCount]; // Array to store sensor values

// Function prototypes
boolean blackDetected();
void queryIRSensors();

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Debugging and testing code
  Serial.println("Testing black detection...");
  queryIRSensors(); // Update sensor values
  boolean result = blackDetected();
  Serial.print("Black detected: ");
  Serial.println(result ? "Yes" : "No");
  Serial.println("------------------------");
  delay(2000); // Delay between tests
}

boolean blackDetected() {
  short sum = 0;

  Serial.println("black function");

  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i]) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" detected black");
      sum++;
    }
    else {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" did not detect black");
    }
  }

  Serial.print("Sum: ");
  Serial.println(sum);

  return sum == sensorCount; // Expecting sum to be sensorCount when all sensors detect black
}

void queryIRSensors() {
  // Simulate some sensors detecting black
  for (int i = 0; i < sensorCount; i++) {
    // Assume sensors 0, 2, and 4 detect black
    sensorValues[i] = (i % 2 == 0) ? 1 : 0; // Every other sensor detects black
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" value: ");
    Serial.println(sensorValues[i]);
  }
}
