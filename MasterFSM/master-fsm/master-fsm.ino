#include <TMCStepper.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include "shared_defines.hpp"
// Define UART communication pins
#define RX_PIN_R 0 // TMC2209 TX (PDN)RIGHT MOTOR
#define TX_PIN_R 1 // TMC2209 RX (PDN)
#define RX_PIN_L 7 // TMC2209 TX (PDN)LEFT MOTOR
#define TX_PIN_L 8 // TMC2209 RX (PDN)

// Define Stepper Motor Driver Pins
#define STEP_PIN_R 4
#define DIR_PIN_R 3
#define ENABLE_PIN_R 2 

#define STEP_PIN_L 11
#define DIR_PIN_L 6
#define ENABLE_PIN_L 5

// Pin for signal from sensor arduino.
#define ARDUINO_COMMUNICATION_PIN_OUT 12
#define ARDUINO_COMMUNICATION_PIN_IN  13

// TODO: figure out how many ms corresponds to a 90 deg turn
#define DEGREES_90 100

#define MOVE_FORWARD(x) \
startTime = millis();\
stepper_R.setSpeed(stepperMaxSpeed);\
stepper_L.setSpeed(-1*stepperMaxSpeed);\
while (millis() - startTime < (x)) {\
  stepper_L.runSpeed();\
  stepper_R.runSpeed();\
}

#define TURN_CW(x) \
startTime = millis();\
stepper_R.setSpeed(stepperTurnSpeed);\
stepper_L.setSpeed(stepperTurnSpeed);\
while (millis() - startTime < (x)) {\
  stepper_L.runSpeed();\
  stepper_R.runSpeed();\
}

#define TURN_CCW(x) \
startTime = millis();\
stepper_R.setSpeed(-1*stepperTurnSpeed);\
stepper_L.setSpeed(-1*stepperTurnSpeed);\
while (millis() - startTime < (x)) {\
  stepper_L.runSpeed();\
  stepper_R.runSpeed();\
}

// *****************************

#define DRIVER_ADDRESS 0b00 // Default TMC2209 address
#define RSENSE 0.11f // R_SENSE resistor value (check your hardware)

// Create SoftwareSerial instance for TMC2209 UART communication
SoftwareSerial SERIAL_PORT_RIGHT(RX_PIN_R, TX_PIN_R);
SoftwareSerial SERIAL_PORT_LEFT(RX_PIN_L, TX_PIN_L);
// *****************************

// Create TMC2209 driver object (MUST use SoftwareSerial pointer)
TMC2209Stepper driver_L(&SERIAL_PORT_LEFT, RSENSE, DRIVER_ADDRESS);
TMC2209Stepper driver_R(&SERIAL_PORT_RIGHT, RSENSE, DRIVER_ADDRESS);
// *****************************

// Stepper Motor (Step/Dir Control)
AccelStepper stepper_R(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper stepper_L(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);
const float stepperMaxSpeed = 500;
const float stepperTurnSpeed = 1000;
// *****************************
unsigned long startTime = 0;
void setup() {
  // put your setup code here, to run once:
  
  // setup pins

  // setup ultrasonic interrupts

  // setup motors
  driver_R.beginSerial(115200);
  driver_L.beginSerial(115200);

  pinMode(ENABLE_PIN_L, OUTPUT);
  digitalWrite(ENABLE_PIN_L, LOW); 
  pinMode(STEP_PIN_L, OUTPUT);
  pinMode(DIR_PIN_L, OUTPUT);

  pinMode(ENABLE_PIN_R, OUTPUT);
  digitalWrite(ENABLE_PIN_R, LOW);
  pinMode(STEP_PIN_R, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);

  pinMode(ARDUINO_COMMUNICATION_PIN_IN, INPUT);  
  pinMode(ARDUINO_COMMUNICATION_PIN_OUT, OUTPUT);
  digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, LOW);

  delay(100); // Wait for the driver to initialize

  // Initialize TMC2209 -- see if this initializes both??
  driver_L.begin();
  driver_L.toff(4); // Enable motor
  driver_L.rms_current(1500); // Set motor current (800mA)
  driver_L.microsteps(16); // Set microstepping to 1/16
  driver_L.pwm_autoscale(true); // Enable StealthChop2

  driver_R.begin();
  driver_R.toff(4); // Enable motor
  driver_R.rms_current(1500); // Set motor current (800mA)
  driver_R.microsteps(16); // Set microstepping to 1/16
  driver_R.pwm_autoscale(true); // Enable StealthChop2

  // Configure Stepper Motor
  stepper_L.setMaxSpeed(10000); //do not change this
  // stepper_L.setSpeed(-4500); //fastest possible speed
  stepper_R.setMaxSpeed(10000); //do not change this
  // stepper_R.setSpeed(4500); //fastest possible speed

  Serial.begin(9600);
  Serial.println("TMC2209 Initialized. Motor should start moving...");
}

void loop2() {
  MOVE_FORWARD(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Start game (start buzzer)

  // Orient
  {
    while (1) {
      // Turn a little
      TURN_CW(500);

      // Tell sensors we are ready.
      // TODO: add a short delay in the sensor arduino to ensure
      // pulseIn is called while still reading LOW.
      digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, HIGH);

      // Wait for sensors to read.
      unsigned long duration = pulseIn(ARDUINO_COMMUNICATION_PIN_IN, HIGH);
      Serial.print("Duration seen: ");
      Serial.println(duration);
      digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, LOW);

      // if we are in the right direction, continue with the program.
      if (duration >= LONG_PULSE) 
        break;
    }
  }

  Serial.println("Stage 1");
  MOVE_FORWARD(1000);

  // Turn 90 deg clockwise
  // TODO: define a turn speed separate from max speed.
  Serial.println("Stage 2");
  TURN_CW(DEGREES_90);

  // Move forward
  Serial.println("Stage 3");
  MOVE_FORWARD(1000);

  // Turn 90 deg counterclockwise
  Serial.println("Stage 4");
  TURN_CCW(DEGREES_90);

  // Move forward
  MOVE_FORWARD(1000);

  // Turn 90 deg counterclockwise
  TURN_CCW(DEGREES_90);

  // Move forward, pushing the pot
  MOVE_FORWARD(1000);

  // Turn 90 deg counterclockwise
  TURN_CCW(DEGREES_90);

  // Move forward to go around the pot handle
  MOVE_FORWARD(500);

  // Turn 90 deg clockwise
  TURN_CW(DEGREES_90);

  // Move forward a short bit to get between handles
  MOVE_FORWARD(200);

  // Turn 90 deg clockwise to face pot
  TURN_CW(DEGREES_90);

  // Move forward to hit pot
  MOVE_FORWARD(100);

  // Release ball

  // End game (shut off buzzer)
}
