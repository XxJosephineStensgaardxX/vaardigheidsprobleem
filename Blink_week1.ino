/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

// //Assigment 1
// // Define pin numbers for each LED
// const int redPin = 13;     
// const int yellowPin = 12;  
// const int greenPin = 11;  

// void setup() {
//   // initialize digital pins as outputs
//   pinMode(redPin, OUTPUT);
//   pinMode(yellowPin, OUTPUT);
//   pinMode(greenPin, OUTPUT);
// }

// void loop() {
//   // Red light for 3 seconds
//   digitalWrite(redPin, HIGH);
//   digitalWrite(yellowPin, LOW);
//   digitalWrite(greenPin, LOW);
//   delay(3000);

//   // Yellow light for 1 second
//   digitalWrite(redPin, LOW);
//   digitalWrite(yellowPin, HIGH);
//   digitalWrite(greenPin, LOW);
//   delay(1000);

//   // Green light for 4 seconds
//   digitalWrite(redPin, LOW);
//   digitalWrite(yellowPin, LOW);
//   digitalWrite(greenPin, HIGH);
//   delay(4000);
// }

//Assigment 2

// int ledRed = 13;
// int brightness = 0;
// int fadeAmount = 4;  // The amount to fade by

// void setup() {
//   pinMode(ledRed, OUTPUT);
// }

// void loop() {
//   // Fade in
//   for (brightness = 0; brightness <= 255; brightness += fadeAmount) {
//     analogWrite(ledRed, brightness);
//     delay(40);
//   }

//   // Fade out
//   for (brightness = 255; brightness >= 0; brightness -= fadeAmount) {
//     analogWrite(ledRed, brightness);
//     delay(40);
//   }
// }

// //11 constants won't change. They're used here to set pin numbers:
// const int buttonPin = 1;
// //1 the number of the pushbutton pin
// const int ledPin = 13;
// //the number of the LED pin
// // variables will change:
// int buttonState = 0;
// //1 variable for reading the pushbutton

// void setup() {
// // initialize the LED pin as an output:
// pinMode(ledPin, OUTPUT) ;
// // initialize the pushbutton pin as an input:
// pinMode (buttonPin, INPUT) ;
// }
// void loop() {
// // read the state of the pushbutton value:
// buttonState = digitalRead (buttonPin) ;
// // check if the pushbutton is pressed,
// if (buttonState == LOW) {
//   digitalWrite (ledPin, LOW) ;
// // turn LED on:
// }
// else {
//   digitalWrite(ledPin, HIGH); // turn LED off:

// }
// }

// //Assignment 3 without milli
// const int buttonPin = 10;     // Pin connected to the pushbutton
// const int redPin = 13;       // Pin connected to the red LED
// const int yellowPin = 12;    // Pin connected to the yellow LED
// const int greenPin = 11;     // Pin connected to the green LED

// int buttonState = 0;         // Variable to store the state of the pushbutton

// void setup() {
//   pinMode(buttonPin, INPUT);    // Initialize the pushbutton pin as an input
//   pinMode(redPin, OUTPUT);      // Initialize the red LED pin as an output
//   pinMode(yellowPin, OUTPUT);   // Initialize the yellow LED pin as an output
//   pinMode(greenPin, OUTPUT);    // Initialize the green LED pin as an output
// }

// void loop() {
//   buttonState = digitalRead(buttonPin);   // Read the state of the pushbutton
  
//   if (buttonState == LOW) {  // If pushbutton is pressed (LOW when active)
//     // Activate traffic light sequence
//     digitalWrite(redPin, LOW);    // Red light
//     delay(3000);                    // Delay for 3 seconds
//     digitalWrite(redPin, HIGH);     // Turn off red light

//     digitalWrite(greenPin, LOW);  // Green light
//     delay(4000);                    // Delay for 4 seconds
//     digitalWrite(greenPin, HIGH);   // Turn off green light

//     digitalWrite(yellowPin, LOW);  // Yellow light
//     delay(1000);                    // Delay for 1 second
//     digitalWrite(yellowPin, HIGH);   // Turn off yellow light
//   } else {
//     // If pushbutton is not pressed, turn off all lights
//     digitalWrite(redPin, HIGH);
//     digitalWrite(yellowPin, HIGH);
//     digitalWrite(greenPin, HIGH);
//   }
// }

// const int buttonPin = 10;     // Pin connected to the pushbutton
// const int redPin = 13;        // Pin connected to the red LED
// const int yellowPin = 12;     // Pin connected to the yellow LED
// const int greenPin = 11;      // Pin connected to the green LED

// int buttonState = 0;          // Variable to store the state of the pushbutton
// bool trafficLightActive = false;  // Variable to track if the traffic light sequence is active

// void setup() {
//   pinMode(buttonPin, INPUT);     // Initialize the pushbutton pin as an input
//   pinMode(redPin, OUTPUT);       // Initialize the red LED pin as an output
//   pinMode(yellowPin, OUTPUT);    // Initialize the yellow LED pin as an output
//   pinMode(greenPin, OUTPUT);     // Initialize the green LED pin as an output
// }

// void loop() {
//   buttonState = digitalRead(buttonPin);   // Read the state of the pushbutton

//   if (buttonState == LOW) {
//     // Button is pressed, toggle traffic light activation
//     trafficLightActive = !trafficLightActive;
    
//     if (trafficLightActive) {
//       // Traffic light sequence starts
//       digitalWrite(redPin, HIGH);
//       delay(3000);   // Red light
//       digitalWrite(redPin, LOW);

//       digitalWrite(greenPin, HIGH);
//       delay(4000);   // Green light
//       digitalWrite(greenPin, LOW);

//       digitalWrite(yellowPin, HIGH);
//       delay(1000);   // Yellow light
//       digitalWrite(yellowPin, LOW);
//     } else {
//       // Turn off all lights if traffic light sequence is stopped
//       digitalWrite(redPin, LOW);
//       digitalWrite(yellowPin, LOW);
//       digitalWrite(greenPin, LOW);
//     }
    
//     // Wait until button is released to prevent rapid toggling
//     while (digitalRead(buttonPin) == LOW) {
//       delay(10);
//     }
//   }
// }

//Assigment 3 but all lights on withoiut proper delay

// const int buttonPin = 10;     // Pin connected to the pushbutton
// const int redPin = 13;        // Pin connected to the red LED
// const int yellowPin = 12;     // Pin connected to the yellow LED
// const int greenPin = 11;      // Pin connected to the green LED

// int buttonState = 0;          // Variable to store the state of the pushbutton
// bool trafficLightActive = false;  // Variable to track if the traffic light sequence is active

// unsigned long previousMillis = 0;  // Variable to store the previous time
// const long interval = 500;   // Interval for traffic light sequence (in milliseconds)

// void setup() {
//   pinMode(buttonPin, INPUT);     // Initialize the pushbutton pin as an input
//   pinMode(redPin, OUTPUT);       // Initialize the red LED pin as an output
//   pinMode(yellowPin, OUTPUT);    // Initialize the yellow LED pin as an output
//   pinMode(greenPin, OUTPUT);     // Initialize the green LED pin as an output
  
//   // Turn off all LEDs initially (LEDs are active LOW)
//   digitalWrite(redPin, HIGH);
//   digitalWrite(yellowPin, HIGH);
//   digitalWrite(greenPin, HIGH);
// }

// void loop() {
//   buttonState = digitalRead(buttonPin);   // Read the state of the pushbutton

//   if (buttonState == LOW) {
//     // Button is pressed
//     if (!trafficLightActive) {
//       // Traffic light sequence is not active, start it
//       trafficLightActive = true;
//       previousMillis = millis();  // Record the start time of the sequence
//     } else {
//       // Traffic light sequence is already active, stop it
//       trafficLightActive = false;
//       // Turn off all LEDs
//       digitalWrite(redPin, HIGH);
//       digitalWrite(yellowPin, HIGH);
//       digitalWrite(greenPin, HIGH);
//     }
//     // Wait until button is released to prevent rapid toggling
//     while (digitalRead(buttonPin) == LOW) {
//       delay(10);
//     }
//   }

//   if (trafficLightActive) {
//     // Traffic light sequence is active, execute the sequence
//     unsigned long currentMillis = millis();
//     if (currentMillis - previousMillis >= interval) {
//       // Toggle LED states (LEDs are active LOW)
//       digitalWrite(redPin, !digitalRead(redPin));
//       previousMillis = currentMillis;  // Update previous time
      
//       // Wait for a short duration before toggling the next LED
//       delay(100);  // Adjust this delay as needed
      
//       digitalWrite(yellowPin, !digitalRead(yellowPin));
      
//       // Wait for a short duration before toggling the next LED
//       delay(3000);  // Adjust this delay as needed
      
//       digitalWrite(greenPin, !digitalRead(greenPin));
//     }
//   }
// }


// //Assigment 3 proper

// const int buttonPin = 10;     // Pin connected to the pushbutton
// const int redPin = 13;        // Pin connected to the red LED
// const int yellowPin = 12;     // Pin connected to the yellow LED
// const int greenPin = 11;      // Pin connected to the green LED

// int buttonState = 0;          // Variable to store the state of the pushbutton
// bool trafficLightActive = false;  // Variable to track if the traffic light sequence is active

// unsigned long previousMillis = 0;  // Variable to store the previous time
// const long redInterval = 3000;     // Duration for red light (in milliseconds)
// const long greenInterval = 4000;   // Duration for green light (in milliseconds)
// const long yellowInterval = 1000;  // Duration for yellow light (in milliseconds)

// void setup() {
//   pinMode(buttonPin, INPUT);     // Initialize the pushbutton pin as an input
//   pinMode(redPin, OUTPUT);       // Initialize the red LED pin as an output
//   pinMode(yellowPin, OUTPUT);    // Initialize the yellow LED pin as an output
//   pinMode(greenPin, OUTPUT);     // Initialize the green LED pin as an output
  
//   // Turn off all LEDs initially (LEDs are active LOW)
//   digitalWrite(redPin, HIGH);
//   digitalWrite(yellowPin, HIGH);
//   digitalWrite(greenPin, HIGH);
// }

// void loop() {
//   buttonState = digitalRead(buttonPin);   // Read the state of the pushbutton

//   if (buttonState == LOW) {
//     // Button is pressed
//     if (!trafficLightActive) {
//       // Traffic light sequence is not active, start it
//       trafficLightActive = true;
//       previousMillis = millis();  // Record the start time of the sequence
//     } else {
//       // Traffic light sequence is already active, stop it
//       trafficLightActive = false;
//       // Turn off all LEDs
//       digitalWrite(redPin, HIGH);
//       digitalWrite(yellowPin, HIGH);
//       digitalWrite(greenPin, HIGH);
//     }
//     // Wait until button is released to prevent rapid toggling
//     while (digitalRead(buttonPin) == LOW) {
//       delay(10);
//     }
//   }

//   if (trafficLightActive) {
//     // Traffic light sequence is active, execute the sequence
//     unsigned long currentMillis = millis();
//     if (currentMillis - previousMillis < redInterval) {
//       // Red light
//       digitalWrite(redPin, LOW);
//       digitalWrite(yellowPin, HIGH);
//       digitalWrite(greenPin, HIGH);
//     } else if (currentMillis - previousMillis < redInterval + greenInterval) {
//       // Green light
//       digitalWrite(redPin, HIGH);
//       digitalWrite(yellowPin, HIGH);
//       digitalWrite(greenPin, LOW);
//     } else if (currentMillis - previousMillis < redInterval + greenInterval + yellowInterval) {
//       // Yellow light
//       digitalWrite(redPin, HIGH);
//       digitalWrite(yellowPin, LOW);
//       digitalWrite(greenPin, HIGH);
//     } else {
//       // Reset the timer and start the sequence again
//       previousMillis = currentMillis;
//     }
//   }
// }

// // Assigment 4
// const int redPin = 13;      // Pin connected to the red LED
// const int greenPin = 11;    // Pin connected to the green LED
// const int buttonPin1 = 10;  // Pin connected to the first button
// const int buttonPin2 = 9;   // Pin connected to the second button
// const int buttonPin3 = 8;   // Pin connected to the third button

// bool greenLedState = false;  // Initial state of the green LED (OFF)
// bool allButtonsPressed = false; // Flag to track if all buttons are pressed
// bool anyButtonPressed = false; // Flag to track if any button is pressed
// unsigned long previousMillis = 0;  // Variable to store the previous time
// const long interval = 1000;   // Interval for red LED flashing (1 second)

// void setup() {
//   pinMode(redPin, OUTPUT);       // Initialize the red LED pin as an output
//   pinMode(greenPin, OUTPUT);     // Initialize the green LED pin as an output
//   pinMode(buttonPin1, INPUT_PULLUP);    // Initialize the first button pin as an input with internal pull-up resistor
//   pinMode(buttonPin2, INPUT_PULLUP);    // Initialize the second button pin as an input with internal pull-up resistor
//   pinMode(buttonPin3, INPUT_PULLUP);    // Initialize the third button pin as an input with internal pull-up resistor
// }

// void loop() {
//   // Flashing red LED
//   unsigned long currentMillis = millis();
//   if (currentMillis - previousMillis >= interval) {
//     digitalWrite(redPin, !digitalRead(redPin));  // Toggle red LED state
//     previousMillis = currentMillis;  // Update previous time
//   }
  
//   // Checking button states
//   bool button1Pressed = digitalRead(buttonPin1) == LOW;
//   bool button2Pressed = digitalRead(buttonPin2) == LOW;
//   bool button3Pressed = digitalRead(buttonPin3) == LOW;
  
//   if (button1Pressed || button2Pressed || button3Pressed) {
//     // At least one button is pressed
//     anyButtonPressed = true;
//     // Turn off green LED
//     digitalWrite(greenPin, HIGH);
//     greenLedState = false;
//   } else {
//     // No buttons are pressed
//     anyButtonPressed = false;
//   }

//   if (button1Pressed && button2Pressed && button3Pressed && !greenLedState) {
//     // All three buttons are pressed simultaneously and green LED is currently off
//     digitalWrite(greenPin, LOW);
//     greenLedState = false;
//   }
// }











