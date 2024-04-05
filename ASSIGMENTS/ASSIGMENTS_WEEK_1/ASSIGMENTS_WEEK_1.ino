
//Assigment 1

// Define pin numbers for each LED
const int redPin = 13;     
const int yellowPin = 12;  
const int greenPin = 11;  

void setup() {
  // initialize digital pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
}

void loop() {
  // Red light for 3 seconds
  digitalWrite(redPin, HIGH);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW);
  delay(3000);

  // Yellow light for 1 second
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, HIGH);
  digitalWrite(greenPin, LOW);
  delay(1000);

  // Green light for 4 seconds
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, HIGH);
  delay(4000);
}

// Assigment 2

int ledRed = 13;
int brightness = 0;
int fadeAmount = 4;  // The amount to fade by

void setup() {
  pinMode(ledRed, OUTPUT);
}

void loop() {
  // Fade in
  for (brightness = 0; brightness <= 255; brightness += fadeAmount) {
    analogWrite(ledRed, brightness);
    delay(40);
  }

  // Fade out
  for (brightness = 255; brightness >= 0; brightness -= fadeAmount) {
    analogWrite(ledRed, brightness);
    delay(40);
  }
}

//Assignment 3 with delay

const int buttonPin = 10;     // Pin connected to the pushbutton
const int redPin = 13;       // Pin connected to the red LED
const int yellowPin = 12;    // Pin connected to the yellow LED
const int greenPin = 11;     // Pin connected to the green LED

int buttonState = 0;         // Variable to store the state of the pushbutton

void setup() {
  pinMode(buttonPin, INPUT);    // Initialize the pushbutton pin as an input
  pinMode(redPin, OUTPUT);      // Initialize the red LED pin as an output
  pinMode(yellowPin, OUTPUT);   // Initialize the yellow LED pin as an output
  pinMode(greenPin, OUTPUT);    // Initialize the green LED pin as an output
}

void loop() {
  buttonState = digitalRead(buttonPin);   // Read the state of the pushbutton
  
  if (buttonState == LOW) {  // If pushbutton is pressed (LOW when active)
    // Activate traffic light sequence
    digitalWrite(redPin, LOW);    // Red light
    delay(3000);                    // Delay for 3 seconds
    digitalWrite(redPin, HIGH);     // Turn off red light

    digitalWrite(greenPin, LOW);  // Green light
    delay(4000);                    // Delay for 4 seconds
    digitalWrite(greenPin, HIGH);   // Turn off green light

    digitalWrite(yellowPin, LOW);  // Yellow light
    delay(1000);                    // Delay for 1 second
    digitalWrite(yellowPin, HIGH);   // Turn off yellow light
  } else {
    // If pushbutton is not pressed, turn off all lights
    digitalWrite(redPin, HIGH);
    digitalWrite(yellowPin, HIGH);
    digitalWrite(greenPin, HIGH);
  }
}


// Assigment 3 a different way

const int buttonPin = 10;     // Pin connected to the pushbutton
const int redPin = 13;        // Pin connected to the red LED
const int yellowPin = 12;     // Pin connected to the yellow LED
const int greenPin = 11;      // Pin connected to the green LED

int buttonState = 0;          // Variable to store the state of the pushbutton
bool trafficLightActive = false;  // Variable to track if the traffic light sequence is active

void setup() {
  pinMode(buttonPin, INPUT);     // Initialize the pushbutton pin as an input
  pinMode(redPin, OUTPUT);       // Initialize the red LED pin as an output
  pinMode(yellowPin, OUTPUT);    // Initialize the yellow LED pin as an output
  pinMode(greenPin, OUTPUT);     // Initialize the green LED pin as an output
}

void loop() {
  buttonState = digitalRead(buttonPin);   // Read the state of the pushbutton

  if (buttonState == LOW) {
    // Button is pressed, toggle traffic light activation
    trafficLightActive = !trafficLightActive;
    
    if (trafficLightActive) {
      // Traffic light sequence starts
      digitalWrite(redPin, HIGH);
      delay(3000);   // Red light
      digitalWrite(redPin, LOW);

      digitalWrite(greenPin, HIGH);
      delay(4000);   // Green light
      digitalWrite(greenPin, LOW);

      digitalWrite(yellowPin, HIGH);
      delay(1000);   // Yellow light
      digitalWrite(yellowPin, LOW);
    } else {
      // Turn off all lights if traffic light sequence is stopped
      digitalWrite(redPin, LOW);
      digitalWrite(yellowPin, LOW);
      digitalWrite(greenPin, LOW);
    }
    
    // Wait until button is released to prevent rapid toggling
    while (digitalRead(buttonPin) == LOW) {
      delay(10);
    }
  }
}


//Assigment 3 with millis 

const int buttonPin = 10;     // Pin connected to the pushbutton
const int redPin = 13;        // Pin connected to the red LED
const int yellowPin = 12;     // Pin connected to the yellow LED
const int greenPin = 11;      // Pin connected to the green LED

int buttonState = 0;          // Variable to store the state of the pushbutton
bool trafficLightActive = false;  // Variable to track if the traffic light sequence is active

unsigned long previousMillis = 0;  // Variable to store the previous time
const long redInterval = 3000;     // Duration for red light (in milliseconds)
const long greenInterval = 4000;   // Duration for green light (in milliseconds)
const long yellowInterval = 1000;  // Duration for yellow light (in milliseconds)

void setup() {
  pinMode(buttonPin, INPUT);     // Initialize the pushbutton pin as an input
  pinMode(redPin, OUTPUT);       // Initialize the red LED pin as an output
  pinMode(yellowPin, OUTPUT);    // Initialize the yellow LED pin as an output
  pinMode(greenPin, OUTPUT);     // Initialize the green LED pin as an output
  
  // Turn off all LEDs initially (LEDs are active LOW)
  digitalWrite(redPin, HIGH);
  digitalWrite(yellowPin, HIGH);
  digitalWrite(greenPin, HIGH);
}

void loop() {
  buttonState = digitalRead(buttonPin);   // Read the state of the pushbutton

  if (buttonState == LOW) {
    // Button is pressed
    if (!trafficLightActive) {
      // Traffic light sequence is not active, start it
      trafficLightActive = true;
      previousMillis = millis();  // Record the start time of the sequence
    } else {
      // Traffic light sequence is already active, stop it
      trafficLightActive = false;
      // Turn off all LEDs
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(greenPin, HIGH);
    }
    // Wait until button is released to prevent rapid toggling
    while (digitalRead(buttonPin) == LOW) {
      delay(10);
    }
  }

  if (trafficLightActive) {
    // Traffic light sequence is active, execute the sequence
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis < redInterval) {
      // Red light
      digitalWrite(redPin, LOW);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(greenPin, HIGH);
    } else if (currentMillis - previousMillis < redInterval + greenInterval) {
      // Green light
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(greenPin, LOW);
    } else if (currentMillis - previousMillis < redInterval + greenInterval + yellowInterval) {
      // Yellow light
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, LOW);
      digitalWrite(greenPin, HIGH);
    } else {
      // Reset the timer and start the sequence again
      previousMillis = currentMillis;
    }
  }
}


// Assigment 4

const int redPin = 13;      // Pin connected to the red LED
const int greenPin = 11;    // Pin connected to the green LED
const int buttonPin1 = 10;  // Pin connected to the first button
const int buttonPin2 = 9;   // Pin connected to the second button
const int buttonPin3 = 8;   // Pin connected to the third button

bool greenLedState = false;  // Initial state of the green LED (OFF)
bool allButtonsPressed = false; // Flag to track if all buttons are pressed
bool anyButtonPressed = false; // Flag to track if any button is pressed
unsigned long previousMillis = 0;  // Variable to store the previous time
const long interval = 1000;   // Interval for red LED flashing (1 second)

void setup() {
  pinMode(redPin, OUTPUT);       // Initialize the red LED pin as an output
  pinMode(greenPin, OUTPUT);     // Initialize the green LED pin as an output
  pinMode(buttonPin1, INPUT_PULLUP);    // Initialize the first button pin as an input with internal pull-up resistor
  pinMode(buttonPin2, INPUT_PULLUP);    // Initialize the second button pin as an input with internal pull-up resistor
  pinMode(buttonPin3, INPUT_PULLUP);    // Initialize the third button pin as an input with internal pull-up resistor
}

void loop() {
  // Flashing red LED
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    digitalWrite(redPin, !digitalRead(redPin));  // Toggle red LED state
    previousMillis = currentMillis;  // Update previous time
  }
  
  // Checking button states
  bool button1Pressed = digitalRead(buttonPin1) == LOW;
  bool button2Pressed = digitalRead(buttonPin2) == LOW;
  bool button3Pressed = digitalRead(buttonPin3) == LOW;
  
  if (button1Pressed || button2Pressed || button3Pressed) {
    // At least one button is pressed
    anyButtonPressed = true;
    // Turn off green LED
    digitalWrite(greenPin, HIGH);
    greenLedState = false;
  } else {
    // No buttons are pressed
    anyButtonPressed = false;
  }

  if (button1Pressed && button2Pressed && button3Pressed && !greenLedState) {
    // All three buttons are pressed simultaneously and green LED is currently off
    digitalWrite(greenPin, LOW);
    greenLedState = false;
  }
}
