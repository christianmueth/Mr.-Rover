Roving Robot, equipped with an ultrasonic sensor and Sentry mode

![image](https://github.com/user-attachments/assets/28c07ff6-66b4-4fad-9910-0809f3f346ad)


Circuit:

![image](https://github.com/user-attachments/assets/de6c9d2f-b153-49ec-9e51-1e860796aa68)


Code:

#include <ESP32Servo.h>

// Uses ESP32 DEVKIT V1. Drivers for board are here: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads

#define SERVO_PIN1 13 // GPIO pin connected to servo1 (180 degree servo)

#define SERVO_PIN2 12 // GPIO pin connected to servo2, back right wheel

#define SERVO_PIN3 14 // GPIO pin connected to servo3, back left wheel

#include "BluetoothSerial.h"  // Include the Bluetooth Serial library

BluetoothSerial ESP_BT;       // Create a BluetoothSerial object

#define trigPin 27

#define echoPin 26

// Note: pins 26 and 27. Echo appears to be on pin 26

// Note: may need to reopen the rover, in order to reconfigure the sensor vcc properly

#define SERVO_MIN_PULSE_WIDTH       500     // the shortest pulse sent to a servo  

#define SERVO_MAX_PULSE_WIDTH      2500     // the longest pulse sent to a servo 

#define SERVO_MAX_ANGLE 180 // Maximum angle in degrees

Servo servo1; // Create servo objects

Servo servo2; // Create servo objects

Servo servo3; // Create servo objects

int currentPosition = 90;   // Current position of the servo

void setup() {

  Serial.begin(115200); // declare serial baud rate // Initialize serial communication

  servo1.attach(SERVO_PIN1); // Attach servo1 to GPIO pin

  servo2.attach(SERVO_PIN2); // Attach servo2 to GPIO pin

  servo3.attach(SERVO_PIN3); // Attach servo3 to GPIO pin

  // Bluetooth initialization:
  ESP_BT.begin("ESP32_BT");   // Initialize Bluetooth with a name for the device

  Serial.println("Bluetooth Device is Ready to Pair");

  // Sonar sensor pins:
  pinMode(trigPin, OUTPUT);

  pinMode(echoPin, INPUT);

  // Servo initialization:
  servo1.write(47.5);

  servo2.write(90); // Initialize to be inert. 180 for full-speed reverse, 0 for full-speed forward 

  servo3.write(90); // Initialize to be inert. 0 for full-speed reverse, 180 for full-speed forward 
}

// Need to read in ASCII characters 50 through 55
void loop() {

  if (ESP_BT.available()) {           // Check if there's data available

    int incomingByte = ESP_BT.read();  // Read the byte as an integer

    Serial.print("Received Byte: ");

    Serial.println(incomingByte);

    // Process the received byte
    if (incomingByte == 55) {

      // stationary_func();//change to a scout function, with a while condition
      scout_func();

    } else if (incomingByte == 50) {

      forward_func();

    } else if (incomingByte == 52) {

      reverse_func();

    } else if (incomingByte == 51) {

      stationary_func();

    } else if (incomingByte == 53) {

      left_func();

    } else if (incomingByte == 54) {

      right_func();

    }
  }

  // Send a pulse to trigger the sonar sensor
  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  // Measure the time it takes for the echo to return
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in centimeters (speed of sound is ~343 m/s)
  int distance = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");

  Serial.print(distance);

  Serial.println(" cm");

  delay(500);  // Wait a bit before the next reading
}

void scout_func() {

  int bluetooth_num = 55;  // Initialize to the "active" button number

  unsigned long last_check_time = 0;

  while (bluetooth_num == 55) {  // Stay in the loop while `bluetooth_num` is 55

    // Check Bluetooth more frequently
    if (millis() - last_check_time > 50) { // Check every 50ms

      last_check_time = millis();

      // Check if there's new data over Bluetooth
      if (ESP_BT.available()) {

        bluetooth_num = ESP_BT.read();  // Update `bluetooth_num` if new data is available

        if (bluetooth_num != 55) {  // Exit the loop if the received number is not 55

          break;

        }
      }
    }

    // Trigger the sonar sensor
    digitalWrite(trigPin, LOW);

    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);

    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    // Measure the time for the echo to return
    long pulse_duration = pulseIn(echoPin, HIGH, 30000); // Add a timeout of 30ms

    int pulse_distance = pulse_duration * 0.034 / 2;

    Serial.print("Pulse distance: ");

    Serial.println(pulse_distance);

    // Check the pulse distance and execute movement functions accordingly
    // consider using a sequence where the bot reverses and then turns around, instead of simple turns
    if (pulse_distance <= 9) {

      int rand_num = random(0, 2);

      if (rand_num == 1) {

        left_func();

      } else {

        right_func();

      }

      delay(500); // Short delay to allow for quick re-checking
    } else {

      int rand_walk_num = generateRandomNumber();

      if (rand_walk_num == 1) {

        forward_func();

      } else if (rand_walk_num == 2) {

        right_func();

      } else if (rand_walk_num == 3) {

        left_func();

      }

      delay(500); // Short delay to allow for quick re-checking
    }
  }
}

void left_func() {

  servo1.write(17.5);

  delay(50);

  servo2.write(0); // Initialize to be inert. 180 for full-speed reverse, 0 for full-speed forward 

  delay(50);

  servo3.write(90); // Initialize to be inert. 0 for full-speed reverse, 180 for full-speed forward 
}

void right_func() {

  servo1.write(77.5);

  delay(50);

  servo2.write(90); // Initialize to be inert. 180 for full-speed reverse, 0 for full-speed forward 

  delay(50);

  servo3.write(180); // Initialize to be inert. 0 for full-speed reverse, 180 for full-speed forward
}

void forward_func() {

  servo1.write(47.5);

  delay(50);

  servo2.write(0); // Initialize to be inert. 180 for full-speed reverse, 0 for full-speed forward 

  delay(50);

  servo3.write(180); // Initialize to be inert. 0 for full-speed reverse, 180 for full-speed forward 
}

void reverse_func() {

  servo1.write(47.5);

  delay(50);

  servo2.write(180); // Initialize to be inert. 180 for full-speed reverse, 0 for full-speed forward 

  delay(50);

  servo3.write(0); // Initialize to be inert. 0 for full-speed reverse, 180 for full-speed forward 
}

void stationary_func() {

  servo1.write(47.5);

  delay(50);

  servo2.write(90); // Initialize to be inert. 180 for full-speed reverse, 0 for full-speed forward 

  delay(50);

  servo3.write(90); // Initialize to be inert. 0 for full-speed reverse, 180 for full-speed forward 
}

int generateRandomNumber() {

  int randValue = random(1, 101); // Generate a random number from 1 to 100

  if (randValue <= 70) {

    return 1; // 70% chance

  } else if (randValue <= 85) {

    return 2; // 15% chance (from 71 to 85)

  } else {

    return 3; // 15% chance (from 86 to 100)
  }
}
