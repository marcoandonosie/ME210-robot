#include <TMCStepper.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>

// Define UART communication pins
#define RX_PIN_R 0 // TMC2209 TX (PDN)RIGHT MOTOR
#define TX_PIN_R 1 // TMC2209 RX (PDN)
#define RX_PIN_L 7 // TMC2209 TX (PDN)LEFT MOTOR
#define TX_PIN_L 8 //TMC2209 RX (PDN)
// *****************************

#define DRIVER_ADDRESS 0b00 // Default TMC2209 address
#define RSENSE 0.11f // R_SENSE resistor value (check your hardware)

// Ultrasonic sensor functions and pin defines
#define PIN_TRIG_1 10
#define PIN_TRIG_2 11
#define PIN_ECHO_1 2  // MUST be an interrupt pin
#define PIN_ECHO_2 3  // MUST be an interrupt pin

void ultraEmit(int sensorNum);
void ultraEchoCallback();
void ultraAvgRead(int sensorNum);
// *****************************

// Utility macro
#define ABS(x) (((x) > 0) ? (x) : (-1 * (x)))
// *****************************

// Create SoftwareSerial instance for TMC2209 UART communication
SoftwareSerial SERIAL_PORT_RIGHT(RX_PIN_R, TX_PIN_R);
SoftwareSerial SERIAL_PORT_LEFT(RX_PIN_L, TX_PIN_L);
// *****************************

// Create TMC2209 driver object (MUST use SoftwareSerial pointer)
TMC2209Stepper driver_L(&SERIAL_PORT_LEFT, RSENSE, DRIVER_ADDRESS);
TMC2209Stepper driver_R(&SERIAL_PORT_RIGHT, RSENSE, DRIVER_ADDRESS);
// *****************************

// Define Stepper Motor Driver Pins
#define STEP_PIN_R 4
#define DIR_PIN_R 3
#define ENABLE_PIN_R 2

#define STEP_PIN_L 9
#define DIR_PIN_L 6
#define ENABLE_PIN_L 10
// *****************************

// Stepper Motor (Step/Dir Control)
AccelStepper stepper_R(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper stepper_L(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);
// *****************************

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
  stepper_L.setSpeed(-4500); //fastest possible speed
  stepper_R.setMaxSpeed(10000); //do not change this
  stepper_R.setSpeed(4500); //fastest possible speed

  Serial.println("TMC2209 Initialized. Motor should start moving...");
}

void loop() {
  // put your main code here, to run repeatedly:

  // Start game (start buzzer)

  // Orient
  {
    while (1) {
      // Turn a little

      // Stop turning

      // read from sensors
      int read0 = ultraAvgRead(0);
      int read1 = ultraAvgRead(1);
      // If delta between sensors is small and 
      // measured distance is about 30-35", we are facing north
      int diff = read0 - read1;
      int delta = ABS(diff);

      const int thresh = 0;
      if (delta < thresh && 30 < read0 && read0 < 35) {
        // we are perpendicular
        break;
      }
    }
  }

  // Move forward

  // Turn 90 deg clockwise

  // Move forward

  // Turn 90 deg counterclockwise

  // Move forward

  // Turn 90 deg counterclockwise

  // Move forward, pushing the pot

  // Turn 90 deg counterclockwise

  // Move forward to go around the pot handle

  // Turn 90 deg clockwise

  // Move forward a short bit to get between handles

  // Turn 90 deg clockwise to face pot

  // Move forward to hit pot

  // Release ball

  // End game (shut off buzzer)
}
