#include <TMCStepper.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define LONG_PULSE 500
#define SHORT_PULSE 50

#define PIN_TRIG_1 13
#define PIN_TRIG_2 2
#define PIN_ECHO_1 12
#define PIN_ECHO_2 5

// Define UART communication pins
#define RX_PIN_R 0 // TMC2209 TX (PDN)RIGHT MOTOR
#define TX_PIN_R 1 // TMC2209 RX (PDN)
#define RX_PIN_L 7 // TMC2209 TX (PDN)LEFT MOTOR
#define TX_PIN_L 8 // TMC2209 RX (PDN)

// Define Stepper Motor Driver Pins
#define STEP_PIN_R 4
#define DIR_PIN_R 3

#define STEP_PIN_L 11
#define DIR_PIN_L 6

// TODO: figure out how many ms corresponds to a 90 deg turn
#define DEGREES_90 700

Servo ignition;
Servo latch;

#define MOVE_FORWARD(x) \
startTime = millis();\
stepper_R.setSpeed(stepperMaxSpeed);\
stepper_L.setSpeed(-1*stepperMaxSpeed);\
while (millis() - startTime < (x)) {\
  stepper_L.runSpeed();\
  stepper_R.runSpeed();\
}\
delay(500);

#define MOVE_BACKWARD(x) \
startTime = millis();\
stepper_R.setSpeed(-1*stepperMaxSpeed);\
stepper_L.setSpeed(stepperMaxSpeed);\
while (millis() - startTime < (x)) {\
 stepper_L.runSpeed();\
 stepper_R.runSpeed();\
}\
delay(500);


#define TURN_CW(x) \
startTime = millis();\
stepper_R.setSpeed(stepperTurnSpeed);\
stepper_L.setSpeed(stepperTurnSpeed);\
while (millis() - startTime < (x)) {\
  stepper_L.runSpeed();\
  stepper_R.runSpeed();\
}\

#define TURN_CCW(x) \
startTime = millis();\
stepper_R.setSpeed(-1*stepperTurnSpeed);\
stepper_L.setSpeed(-1*stepperTurnSpeed);\
while (millis() - startTime < (x)) {\
  stepper_L.runSpeed();\
  stepper_R.runSpeed();\
}\

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
float stepperMaxSpeed = 4500;
const float stepperTurnSpeed = 2000;

// *****************************
unsigned long startTime = 0;

float distanceRead(int sensorNum = 0) {
  long avg = 0;
  uint8_t echoPin = sensorNum == 0 ? PIN_ECHO_1 : PIN_ECHO_2;
  uint8_t trigPin = sensorNum == 0 ? PIN_TRIG_1 : PIN_TRIG_2;
  for (int i = 0; i < 8; ++i) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    avg += pulseIn(echoPin, HIGH) >> 3;
  }
  return (avg*.0343)/2;  
}

void setup() {
  // put your setup code here, to run once:
  
  // setup pins

  // setup ultrasonic interrupts

  // setup motors
  driver_R.beginSerial(115200);
  driver_L.beginSerial(115200);

  pinMode(STEP_PIN_L, OUTPUT);
  pinMode(DIR_PIN_L, OUTPUT);

  pinMode(STEP_PIN_R, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);

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

  ignition.attach(9);
  latch.attach(10);
  latch.write(0);
  ignition.write(90);

  pinMode(PIN_TRIG_1, OUTPUT);
  pinMode(PIN_TRIG_2, OUTPUT);
  pinMode(PIN_ECHO_1, INPUT);
  pinMode(PIN_ECHO_2, INPUT);
  digitalWrite(PIN_TRIG_1, LOW);
  digitalWrite(PIN_TRIG_2, LOW);

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
    Serial.println("Orientation start.");
    while (1) {
      Serial.println("Reading...");
      // Take reading.
      float dist0 = distanceRead(0);
      delay(50); // Delay to ensure no cross-contamination between the ultrasonic sensors.
      float dist1 = distanceRead(1);
      Serial.print("Dist0: ");
      Serial.print(dist0);
      Serial.print(", Dist1: ");
      Serial.println(dist1);
      float diff = dist1 - dist0;
      float delta = (diff < 0) ? -1*diff : diff; // delta = abs(diff)
      bool isFacingNorth = ((delta < 5) && (dist0 > 50));
      Serial.print("isFacingNorth: ");
      Serial.println(isFacingNorth);


      if (isFacingNorth) break;

      // Turn a little
      TURN_CW(200);
    }
  }

  // Turn to face north
  TURN_CW(DEGREES_90);
  delay(500);
  stepperMaxSpeed = 4000;
  MOVE_BACKWARD(500);
  stepperMaxSpeed = 4500;

  Serial.println("Stage 1");
  //igition servo
  ignition.write(0);
  delay(200);
  MOVE_FORWARD(1000);

  // Serial.println("Stage 2");
  TURN_CW(DEGREES_90);

  //hit ignition
  stepperMaxSpeed = 3000;
  MOVE_FORWARD(300);
  stepperMaxSpeed = 4500;

  // // Move forward
  // Serial.println("Stage 3");
  // MOVE_BACKWARD(4000);

  // // Turn 90 deg counterclockwise
  // Serial.println("Stage 4");
  // TURN_CCW(DEGREES_90);

  //   // Move forward
  // MOVE_FORWARD(600);

  // MOVE_BACKWARD(200);

  // // Turn 90 deg counterclockwise
  // TURN_CW(600);

  // // Move forward, pushing the pot
  // MOVE_FORWARD(4000);

  // MOVE_BACKWARD(300);

  // // Turn 90 deg counterclockwise
  // TURN_CW(DEGREES_90/2);

  // // Move forward to go around the pot handle
  // MOVE_FORWARD(700);

  // // Turn 90 deg clockwise
  // TURN_CCW(900);

  // // Move forward a short bit to get between handles\
  // //ball release
  // MOVE_FORWARD(1000);
  // latch.write(180);
  // MOVE_FORWARD(2000);
  // MOVE_BACKWARD(1000);
  while (1);
 }

