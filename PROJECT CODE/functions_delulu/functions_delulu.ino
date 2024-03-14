  
  //MOVE FUNCTIONS
  //SETTING THE PINS FOR EACH MOTOR
  // Motor A (Drive Motor)
  // const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
  // const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5

  // // Motor B (SteeWring Motor)
  // const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
  // const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

  // void setup() {
  //   // Set the motor control pins to outputs
  //   pinMode(motorAPin1, OUTPUT);
  //   pinMode(motorAPin2, OUTPUT);
  //   pinMode(motorBPin1, OUTPUT);
  //   pinMode(motorBPin2, OUTPUT);

  // }

  // // MOVE FORWARD FUNCTION
  // void moveForward() {
  //   digitalWrite(motorAPin1, HIGH); // Motor A forward
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, HIGH); // Motor B forward
  //   digitalWrite(motorBPin2, LOW);
  //   delay(1000); 
     
  // // // Stop both motors after moving backward
  //   digitalWrite(motorAPin1, LOW);
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, LOW);
  //   digitalWrite(motorBPin2, LOW);
  // }

  // // MOVE BACKWARDS FUNCTION
  //   void goBackwards() {
  //   // Set both motors to move backward
  //   digitalWrite(motorAPin1, LOW); // Motor A backward
  //   digitalWrite(motorAPin2, HIGH);
  //   digitalWrite(motorBPin1, LOW); // Motor B backward
  //   digitalWrite(motorBPin2, HIGH);
  //   delay(1000); // Adjust this value to control the duration of backward movement

  //   // Stop both motors after moving backward
  //   digitalWrite(motorAPin1, LOW);
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, LOW);
  //   digitalWrite(motorBPin2, LOW);
  // }

  // // TURN RIGHT FUNCTION
  // void turnRight() {
  //   // Move forward for a short duration
  //   digitalWrite(motorAPin1, HIGH); // Motor A forward
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, HIGH); // Motor B forward
  //   digitalWrite(motorBPin2, LOW);
  //   delay(500); // Adjust this value to control the duration of forward movement
    
  //   // Then, stop both motors to allow the turn
  //   digitalWrite(motorAPin1, LOW);
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, LOW);
  //   digitalWrite(motorBPin2, LOW);
  //   delay(500); // Adjust this value to control the duration of the stop
    
  //   // Turn right for a short duration
  //   digitalWrite(motorAPin1, LOW); // Motor A backward
  //   digitalWrite(motorAPin2, HIGH);
  //   digitalWrite(motorBPin1, HIGH); // Motor B forward
  //   digitalWrite(motorBPin2, LOW);
  //   delay(500); // Adjust this value to control the duration of the turn

  //   // Stop both motors after the turn
  //   digitalWrite(motorAPin1, LOW);
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, LOW);
  //   digitalWrite(motorBPin2, LOW);
  // }

  // TURN LEFT FUNCTION
  // void turnLeft() {
  // //   // Move forward for a short duration
  //   digitalWrite(motorAPin1, HIGH); // Motor A forward
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, HIGH); // Motor B forward
  //   digitalWrite(motorBPin2, LOW);
  //   delay(500); // Adjust this value to control the duration of forward movement
    
  //   // Then, stop both motors to allow the turn
  //   digitalWrite(motorAPin1, LOW);
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, LOW);
  //   digitalWrite(motorBPin2, LOW);
  //   delay(500); // Adjust this value to control the duration of the stop
    
  //   // Turn left for a short duration
  //   digitalWrite(motorAPin1, HIGH); // Motor A forward
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, LOW); // Motor B backward
  //   digitalWrite(motorBPin2, HIGH);
  //   delay(500); // Adjust this value to control the duration of the turn

  //   // Stop both motors after the turn
  //   digitalWrite(motorAPin1, LOW);
  //   digitalWrite(motorAPin2, LOW);
  //   digitalWrite(motorBPin1, LOW);
  //   digitalWrite(motorBPin2, LOW);
  // }

  // void loop() {
  //   // CALL THE FUNCTIONS HERE
  //   moveForward();
  //   turnRight();
  //   turnLeft();
  //   goBackwards();
  // }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //GRIPPER FUNCTION

    // #include <Servo.h>

    // const int gripperPin = 3; 

    // Servo gripperServo; // Create a servo object

    // void setup() {
    //   // Attach the servo to the pin
    //   gripperServo.attach(gripperPin);
    // }

    // void loop() {
    //   // Open the gripper
    //   openGripper();
    //   delay(2000); // Wait for 2 seconds
      
    //   // Close the gripper
    //   closeGripper();
    //   delay(2000); // Wait for 2 seconds
    // }

    // // Function to open the gripper
    // void openGripper() {
    //   gripperServo.write(90); // Adjust the angle as needed for opening
    // }

    // // Function to close the gripper
    // void closeGripper() {
    //   gripperServo.write(0); // Adjust the angle as needed for closing
    // }


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //LINE SENSOR



  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //NEO PIXELS
  /////NEO PIXEL RAINBOW COLOR /////
    // #include <Adafruit_NeoPixel.h>

    // class Strip
    // {
    // public:
    //   uint8_t   effect;
    //   uint8_t   effects;
    //   uint16_t  effStep;
    //   unsigned long effStart;
    //   Adafruit_NeoPixel strip;
    //   Strip(uint16_t leds, uint8_t pin, uint8_t toteffects, uint16_t striptype) : strip(leds, pin, striptype) {
    //     effect = -1;
    //     effects = toteffects;
    //     Reset();
    //   }
    //   void Reset(){
    //     effStep = 0;
    //     effect = (effect + 1) % effects;
    //     effStart = millis();
    //   }
    // };

    // struct Loop
    // {
    //   uint8_t currentChild;
    //   uint8_t childs;
    //   bool timeBased;
    //   uint16_t cycles;
    //   uint16_t currentTime;
    //   Loop(uint8_t totchilds, bool timebased, uint16_t tottime) {currentTime=0;currentChild=0;childs=totchilds;timeBased=timebased;cycles=tottime;}
    // };

    // Strip strip_0(60, 9, 60, NEO_GRB + NEO_KHZ800);
    // struct Loop strip0loop0(1, false, 1);

    // //[GLOBAL_VARIABLES]

    // void setup() {

    //   //Your setup here:

    //   strip_0.strip.begin();
    // }

    // void loop() {

    //   //Your code here:

    //   // Read input from pin 10
    //   int inputState = digitalRead(10);

    //   strips_loop();
    // }

    // void strips_loop() {
    //   if(strip0_loop0() & 0x01)
    //     strip_0.strip.show();
    // }

    // uint8_t strip0_loop0() {
    //   uint8_t ret = 0x00;
    //   switch(strip0loop0.currentChild) {
    //     case 0: 
    //           ret = strip0_loop0_eff0();break;
    //   }
    //   if(ret & 0x02) {
    //     ret &= 0xfd;
    //     if(strip0loop0.currentChild + 1 >= strip0loop0.childs) {
    //       strip0loop0.currentChild = 0;
    //       if(++strip0loop0.currentTime >= strip0loop0.cycles) {strip0loop0.currentTime = 0; ret |= 0x02;}
    //     }
    //     else {
    //       strip0loop0.currentChild++;
    //     }
    //   };
    //   return ret;
    // }

    // uint8_t strip0_loop0_eff0() {
    //     // Strip ID: 0 - Effect: Rainbow - LEDS: 60
    //     // Steps: 60 - Delay: 20
    //     // Colors: 3 (255.0.0, 0.255.0, 0.0.255)
    //     // Options: rainbowlen=60, toLeft=true, 
    //   if(millis() - strip_0.effStart < 20 * (strip_0.effStep)) return 0x00;
    //   float factor1, factor2;
    //   uint16_t ind;
    //   for(uint16_t j=0;j<60;j++) {
    //     ind = strip_0.effStep + j * 1;
    //     switch((int)((ind % 60) / 20)) {
    //       case 0: factor1 = 1.0 - ((float)(ind % 60 - 0 * 20) / 20);
    //               factor2 = (float)((int)(ind - 0) % 60) / 20;
    //               strip_0.strip.setPixelColor(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
    //               break;
    //       case 1: factor1 = 1.0 - ((float)(ind % 60 - 1 * 20) / 20);
    //               factor2 = (float)((int)(ind - 20) % 60) / 20;
    //               strip_0.strip.setPixelColor(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
    //               break;
    //       case 2: factor1 = 1.0 - ((float)(ind % 60 - 2 * 20) / 20);
    //               factor2 = (float)((int)(ind - 40) % 60) / 20;
    //               strip_0.strip.setPixelColor(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
    //               break;
    //     }
    //   }
    //   if(strip_0.effStep >= 60) {strip_0.Reset(); return 0x03; }
    //   else strip_0.effStep++;
    //   return 0x01;
    // }

    /////////////// TURN LEFT FUNCTION NEOPIXEL //////////////
    // #include <Adafruit_NeoPixel.h>

    // const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
    // const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
    // const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
    // const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

    // class Strip {
    // public:
    //   Adafruit_NeoPixel strip;
    //   Strip(uint16_t leds, uint8_t pin) : strip(leds, pin, NEO_GRB + NEO_KHZ800) {
    //     strip.begin();
    //   }

    //   void turnLeft() {
      // strip.setPixelColor(0, 0, 0, 255); // Left indicator - Blue
      // strip.setPixelColor(1, 0, 0, 0); // Right indicator - Blue
      // strip.setPixelColor(2, 0, 0, 0);   // Right indicator - Off
      // strip.setPixelColor(3, 0, 0, 255);   // Left indicator - Off
      // strip.show();

    //   // Motor movements for turning left
    //   digitalWrite(motorAPin1, HIGH); // Motor A forward
    //   digitalWrite(motorAPin2, LOW);
    //   digitalWrite(motorBPin1, LOW);  // Motor B backward
    //   digitalWrite(motorBPin2, HIGH);
    //   delay(500); // Adjust this value to control the duration of movement

    //   // Stop motors after turning
    //   digitalWrite(motorAPin1, LOW);
    //   digitalWrite(motorAPin2, LOW);
    //   digitalWrite(motorBPin1, LOW);
    //   digitalWrite(motorBPin2, LOW);

    //   // Reset NeoPixel strip after the turn
    //   resetStrip();
    // }

    // private:
    //   void resetStrip() {
    //     strip.clear();
    //     strip.show();
    //   }
    // };

    // Strip strip_0(4, 9);

    // void setup() {
    //   pinMode(motorAPin1, OUTPUT);
    //   pinMode(motorAPin2, OUTPUT);
    //   pinMode(motorBPin1, OUTPUT);
    //   pinMode(motorBPin2, OUTPUT);
    // }

    // void loop() {
    //   strip_0.turnLeft();
    //   delay(1000); // Delay before next operation
    // }

     /////////////// TURN RIGHT FUNCTION NEOPIXEL //////////////
    // #include <Adafruit_NeoPixel.h>

    // const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
    // const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
    // const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
    // const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

    // // Class to control NeoPixel strip and motor movements
    // class Strip {
    // public:
    //   Adafruit_NeoPixel strip;
    //   Strip(uint16_t leds, uint8_t pin) : strip(leds, pin, NEO_GRB + NEO_KHZ800) {
    //     strip.begin();
    //   }

    //   // Method to turn right with specific colors
    //   void turnRight() {
    //     strip.setPixelColor(0, 0, 0, 0); // Right indicator - Red
    //     strip.setPixelColor(1, 0, 0, 255);   // Left indicator - Off
    //     strip.setPixelColor(2, 0, 0, 255);   // Left indicator - Off
    //     strip.setPixelColor(3, 0, 0, 0); // Right indicator - Red
    //     strip.show();

    //     // Motor movements for turning right
    //     digitalWrite(motorAPin1, LOW);  // Motor A backward
    //     digitalWrite(motorAPin2, HIGH);
    //     digitalWrite(motorBPin1, HIGH); // Motor B forward
    //     digitalWrite(motorBPin2, LOW);
    //     delay(500); // Adjust this value to control the duration of movement

    //     // Stop motors after turning
    //     digitalWrite(motorAPin1, LOW);
    //     digitalWrite(motorAPin2, LOW);
    //     digitalWrite(motorBPin1, LOW);
    //     digitalWrite(motorBPin2, LOW);

    //     // Reset NeoPixel strip after the turn
    //     resetStrip();
    //   }

    // private:
    //   void resetStrip() {
    //     strip.clear();
    //     strip.show();
    //   }
    // };

    // Strip strip_0(4, 9);

    // void setup() {
    //   pinMode(motorAPin1, OUTPUT);
    //   pinMode(motorAPin2, OUTPUT);
    //   pinMode(motorBPin1, OUTPUT);
    //   pinMode(motorBPin2, OUTPUT);
    // }

    // void loop() {
    //   strip_0.turnRight();
    //   delay(1000); // Delay before next operation
    // }

    /////////////// GOING STRAIGHT FUNCTION NEOPIXEL //////////////

    // #include <Adafruit_NeoPixel.h>

    // const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
    // const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
    // const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
    // const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

    // // Class to control NeoPixel strip and motor movements
    // class Strip {
    // public:
    //   Adafruit_NeoPixel strip;
    //   Strip(uint16_t leds, uint8_t pin) : strip(leds, pin, NEO_GRB + NEO_KHZ800) {
    //     strip.begin();
    //   }

    //   // Method to turn right with specific colors
    //   void turnStraight() {
    //     strip.setPixelColor(0, 0, 0, 0); // Right indicator - Red
    //     strip.setPixelColor(1, 0, 0, 0);   // Left indicator - Off
    //     strip.setPixelColor(2, 0, 0, 255);   // Left indicator - Off
    //     strip.setPixelColor(3, 0, 0, 255); // Right indicator - Red
    //     strip.show();

          
    //       digitalWrite(motorAPin1, HIGH); // Motor A forward
    //       digitalWrite(motorAPin2, LOW);
    //       digitalWrite(motorBPin1, HIGH); // Motor B forward
    //       digitalWrite(motorBPin2, LOW);
    //       delay(500); 

    //     // Stop motors after turning
    //     digitalWrite(motorAPin1, LOW);
    //     digitalWrite(motorAPin2, LOW);
    //     digitalWrite(motorBPin1, LOW);
    //     digitalWrite(motorBPin2, LOW);

    //     // Reset NeoPixel strip after the turn
    //     resetStrip();
    //   }

    //   // Method to move straight with blinking front lights
    //   void moveStraight() {
    //     // Blink the two front lights
    //     strip.setPixelColor(0, 255, 255, 255); // Front light - White
    //     strip.setPixelColor(1, 255, 255, 255); // Front light - White
    //     strip.setPixelColor(2, 0, 0, 0);       // Left indicator - Off
    //     strip.setPixelColor(3, 0, 0, 0);       // Right indicator - Off
    //     strip.show();

    //     // Additional actions for moving straight if needed

    //     // Reset NeoPixel strip after moving straight
    //     resetStrip();
    //   }

    // private:
    //   void resetStrip() {
    //     strip.clear();
    //     strip.show();
    //   }
    // };

    // Strip strip_0(4, 9);

    // void setup() {
    //   pinMode(motorAPin1, OUTPUT);
    //   pinMode(motorAPin2, OUTPUT);
    //   pinMode(motorBPin1, OUTPUT);
    //   pinMode(motorBPin2, OUTPUT);
    // }

    // void loop() {
    //   // Move straight for some time
    //   strip_0.moveStraight();
    //   delay(1000); // Delay before turning right
    //   // Turn right after moving straight
    //   strip_0.turnStraight();
    //   delay(1000); // Delay before next operation
    // }


    /////////////// GOING BACK FUNCTION NEOPIXEL //////////////
    // #include <Adafruit_NeoPixel.h>

    // const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
    // const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
    // const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
    // const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

    // // Class to control NeoPixel strip and motor movements
    // class Strip {
    // public:
    //   Adafruit_NeoPixel strip;
    //   Strip(uint16_t leds, uint8_t pin) : strip(leds, pin, NEO_GRB + NEO_KHZ800) {
    //     strip.begin();
    //   }

    //   // Method to go backwards with specific colors
    //   void goBackwards() {
    //     strip.setPixelColor(0, 0, 255, 0); // Right indicator - Red
    //     strip.setPixelColor(1, 0, 255, 0); // Left indicator - Red
    //     strip.setPixelColor(2, 0, 0, 0);   // Right indicator - Off
    //     strip.setPixelColor(3, 0, 0, 0);   // Left indicator - Off
    //     strip.show();

    //     // Motor movements for going backwards
    //     digitalWrite(motorAPin1, HIGH); // Motor A forward
    //     digitalWrite(motorAPin2, LOW);
    //     digitalWrite(motorBPin1, LOW);  // Motor B backward
    //     digitalWrite(motorBPin2, HIGH);
    //     delay(500); // Adjust this value to control the duration of movement

    //     // Stop motors after going backwards
    //     stopMotors();

    //     // Reset NeoPixel strip after going backwards
    //     resetStrip();
    //   }

    // private:
    //   void resetStrip() {
    //     strip.clear();
    //     strip.show();
    //   }

    //   void stopMotors() {
    //     digitalWrite(motorAPin1, LOW);
    //     digitalWrite(motorAPin2, LOW);
    //     digitalWrite(motorBPin1, LOW);
    //     digitalWrite(motorBPin2, LOW);
    //   }
    // };

    // Strip strip_0(4, 9);

    // void setup() {
    //   pinMode(motorAPin1, OUTPUT);
    //   pinMode(motorAPin2, OUTPUT);
    //   pinMode(motorBPin1, OUTPUT);
    //   pinMode(motorBPin2, OUTPUT);
    // }

    // void loop() {
    //   // Go backwards
    //   strip_0.goBackwards();
    //   delay(1000); // Delay before next operation
    // }


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //ULTRA SONIC DISTANCE SENSOR
    // #include <Servo.h>
    // #include <Ultrasonic.h>

    // const int neckServoPin = 2;
    // Servo neckSwiveler;

    // const int trigPin = 12;
    // const int echoPin = 13;

    // Ultrasonic ultrasonic(trigPin, echoPin);

    // void setup() {
    //   neckSwiveler.attach(neckServoPin);
    //   Serial.begin(9600);
    // }

    // void loop() {
    //   // Sweep the servo from left to right
    //   for (int angle = 0; angle <= 180; angle ++) {
    //     neckSwiveler.write(angle);
    //     delay(20);
        
    //     // Check if an obstacle is detected
    //     long distance = ultrasonic.distanceRead();
    //     if (distance < 20) { // Adjust threshold as needed
    //       Serial.println("Obstacle detected! Stopping.");
    //       while (distance < 20) {
    //         // Keep checking for obstacle until it's removed
    //         delay(100);
    //         distance = ultrasonic.distanceRead();
    //       }
    //       break; // Exit the loop if obstacle detected
    //     }
    //   }
    // }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //COMBINATION EXPERIMENT #1
    #include <Ultrasonic.h>
    #include <Servo.h>

    const int neckServoPin = 2; // Neck swiveler servo signal pin
    #define trigPin 12
    #define echoPin 13

    Ultrasonic ultrasonic(trigPin, echoPin);
    Servo neckSwiveler;

    const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
    const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
    const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
    const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

    void setup() {
      pinMode(motorAPin1, OUTPUT);
      pinMode(motorAPin2, OUTPUT);
      pinMode(motorBPin1, OUTPUT);
      pinMode(motorBPin2, OUTPUT);
      neckSwiveler.attach(neckServoPin);
      Serial.begin(9600);
    }

    void moveForward() {
      digitalWrite(motorAPin1, HIGH);
      digitalWrite(motorAPin2, LOW);
      digitalWrite(motorBPin1, HIGH);
      digitalWrite(motorBPin2, LOW);
    }

    void stopRobot() {
      digitalWrite(motorAPin1, LOW);
      digitalWrite(motorAPin2, LOW);
      digitalWrite(motorBPin1, LOW);
      digitalWrite(motorBPin2, LOW);
    }

    void turnLeft() {
      digitalWrite(motorAPin1, LOW);
      digitalWrite(motorAPin2, HIGH);
      digitalWrite(motorBPin1, HIGH);
      digitalWrite(motorBPin2, LOW);
      delay(1000); // Adjust duration as needed
    }

    void turnRight() {
      digitalWrite(motorAPin1, HIGH);
      digitalWrite(motorAPin2, LOW);
      digitalWrite(motorBPin1, LOW);
      digitalWrite(motorBPin2, HIGH);
      delay(1000); // Adjust duration as needed
    }

    void loop() {
      long distanceFront = ultrasonic.distanceRead();
      Serial.print("Distance Front: ");
      Serial.println(distanceFront);

      if (distanceFront > 10) {
        moveForward(); // Drive forward if no obstacle ahead
      } else {
        stopRobot(); // Stop if obstacle detected
        Serial.println("Obstacle detected!");

        // Determine which direction to turn
        if (random(0, 2) == 0) { // 50% chance to turn left
          Serial.println("Turning left");
          turnLeft();
        } else { // 50% chance to turn right
          Serial.println("Turning right");
          turnRight();
        }

        // Resume forward movement
        moveForward();
      }
    }



  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //COMBINATION EXPERIMENT #2

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



    ///////////////////////////////////////////////////////////////////////////////////////////////
    // SERVO LIBRARY CODE GRIPPER WITHOUT LIBRARY 
    // #include <Servo.h>

    // const int gripperPin = 3; 

    // Servo gripperServo; // Create a servo object

    // void setup() {
    //   // Attach the servo to the pin
    //   gripperServo.attach(gripperPin);
    // }

    // void loop() {
    //   // Open the gripper
    //   openGripper();
    //   delay(2000); // Wait for 2 seconds
      
    //   // Close the gripper
    //   closeGripper();
    //   delay(2000); // Wait for 2 seconds
    // }

    // // Function to open the gripper
    // void openGripper() {
    //   gripperServo.write(90); // Adjust the angle as needed for opening
    // }

    // // Function to close the gripper
    // void closeGripper() {
    //   gripperServo.write(0); // Adjust the angle as needed for closing
    // }

    // /////////////////////////////////////////////////////////////////////////////////////////////
    // SONIC LIBARAY CODE WITHOUT INCLUDE
   




    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // COMBINATION EXPERIMENT #3

    //   #include <Adafruit_NeoPixel.h>

    //   const int motorAPin1 = 7; // Motor A pin 1 connected to Arduino pin 7
    //   const int motorAPin2 = 5; // Motor A pin 2 connected to Arduino pin 5
    //   const int motorBPin1 = 6; // Motor B pin 1 connected to Arduino pin 6
    //   const int motorBPin2 = 4; // Motor B pin 2 connected to Arduino pin 4

    //   // Class to control NeoPixel strip and motor movements
    //   class Strip {
    //   public:
    //     Adafruit_NeoPixel strip;
    //     Strip(uint16_t leds, uint8_t pin) : strip(leds, pin, NEO_GRB + NEO_KHZ800) {
    //       strip.begin();
    //     }

    //   // Method to turn right with specific colors
    //   void turnRight() {
    //     strip.setPixelColor(0, 0, 0, 0); // Right indicator - Red
    //     strip.setPixelColor(1, 0, 0, 255);   // Left indicator - Off
    //     strip.setPixelColor(2, 0, 0, 255);   // Left indicator - Off
    //     strip.setPixelColor(3, 0, 0, 0); // Right indicator - Red
    //     strip.show();

    //     // Motor movements for turning right
    //     digitalWrite(motorAPin1, LOW);  // Motor A backward
    //     digitalWrite(motorAPin2, HIGH);
    //     digitalWrite(motorBPin1, HIGH); // Motor B forward
    //     digitalWrite(motorBPin2, LOW);
    //     delay(500); // Adjust this value to control the duration of movement

    //     // Stop motors after turning
    //     digitalWrite(motorAPin1, LOW);
    //     digitalWrite(motorAPin2, LOW);
    //     digitalWrite(motorBPin1, LOW);
    //     digitalWrite(motorBPin2, LOW);

    //     // Reset NeoPixel strip after the turn
    //     resetStrip();
    //   }


    //     void moveStraight() {
    //   strip.setPixelColor(0, 0, 0, 0); // Right indicator - Red
    //   strip.setPixelColor(1, 0, 0, 0);   // Left indicator - Off
    //   strip.setPixelColor(2, 0, 0, 255);   // Left indicator - Off
    //   strip.setPixelColor(3, 0, 0, 255); // Right indicator - Red
    //   strip.show();

        
    //     digitalWrite(motorAPin1, HIGH); // Motor A forward
    //     digitalWrite(motorAPin2, LOW);
    //     digitalWrite(motorBPin1, HIGH); // Motor B forward
    //     digitalWrite(motorBPin2, LOW);
    //     delay(500); 

    //   // Stop motors after turning
    //   digitalWrite(motorAPin1, LOW);
    //   digitalWrite(motorAPin2, LOW);
    //   digitalWrite(motorBPin1, LOW);
    //   digitalWrite(motorBPin2, LOW);

    //   // Reset NeoPixel strip after the turn
    //   resetStrip();
    // }


    // void goBackwards() {
    //      strip.setPixelColor(0, 0, 255, 0); // Right indicator - Red
    //      strip.setPixelColor(1, 0, 255, 0); // Left indicator - Red
    //      strip.setPixelColor(2, 0, 0, 0);   // Right indicator - Off
    //      strip.setPixelColor(3, 0, 0, 0);   // Left indicator - Off
    //      strip.show();

    //      // Motor movements for going backwards
    //      digitalWrite(motorAPin1, LOW); // Motor A forward
    //      digitalWrite(motorAPin2, HIGH);
    //      digitalWrite(motorBPin1, LOW);  // Motor B backward
    //      digitalWrite(motorBPin2, HIGH);
    //      delay(500); // Adjust this value to control the duration of movement


    //      // Reset NeoPixel strip after going backwards
    //      resetStrip();
    //    }


    //   void turnLeft() {
    //     strip.setPixelColor(0, 0, 0, 255); // Left indicator - Blue
    //     strip.setPixelColor(1, 0, 0, 0); // Right indicator - Blue
    //     strip.setPixelColor(2, 0, 0, 0);   // Right indicator - Off
    //     strip.setPixelColor(3, 0, 0, 255);   // Left indicator - Off
    //     strip.show();

    //     // Motor movements for turning left
    //     digitalWrite(motorAPin1, HIGH); // Motor A forward
    //     digitalWrite(motorAPin2, LOW);
    //     digitalWrite(motorBPin1, LOW);  // Motor B backward
    //     digitalWrite(motorBPin2, HIGH);
    //     delay(500); // Adjust this value to control the duration of movement

    //     // Stop motors after turning
    //     digitalWrite(motorAPin1, LOW);
    //     digitalWrite(motorAPin2, LOW);
    //     digitalWrite(motorBPin1, LOW);
    //     digitalWrite(motorBPin2, LOW);

    //     // Reset NeoPixel strip after the turn
    //     resetStrip();
    //   }


    // private:
    //   void resetStrip() {
    //     strip.clear();
    //     strip.show();
    //   }
    // };

    // Strip strip_0(4, 9);

    // void setup() {
    //   pinMode(motorAPin1, OUTPUT);
    //   pinMode(motorAPin2, OUTPUT);
    //   pinMode(motorBPin1, OUTPUT);
    //   pinMode(motorBPin2, OUTPUT);
    // }

    // void loop() {
    //   strip_0.turnRight();
    //   delay(1000); // Delay before next operation
    //   strip_0.moveStraight();
    //   delay(1000);
    //   strip_0.moveStraight();
    //   delay(1000);
    //   strip_0.turnLeft();
    //   delay(1000);
    //   strip_0.moveStraight();
    //   delay(500);
    //   strip_0.turnRight();
    //   delay(1000);
    //   strip_0.goBackwards();
    // }
    










