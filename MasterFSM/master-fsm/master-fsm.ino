
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


// Define Stepper Motor Driver Pins
#define STEP_PIN_R 4
#define DIR_PIN_R 3

#define DIR_PIN_L 6
#define RX_PIN_L 7 // TMC2209 TX (PDN)LEFT MOTOR
#define TX_PIN_L 8 // TMC2209 RX (PDN)
#define STEP_PIN_L 11

#define DEGREES_90 700
#define DEGREES_90_SPEED_3000 DEGREES_90 + 55

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

#define MOVE_BACKWARD(x) \
startTime = millis();\
stepper_R.setSpeed(-1*stepperMaxSpeed);\
stepper_L.setSpeed(stepperMaxSpeed);\
while (millis() - startTime < (x)) {\
 stepper_L.runSpeed();\
 stepper_R.runSpeed();\
}\


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

#define DELAY(x) \
startTime = millis();\
while (millis() - startTime < (x));

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
const int numPantryVisits = 0; // TODO: Increase this to the most visits that can happen in 2min 10sec

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

void selectionSort(long arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        int minIndex = i;  // Assume the current index is the minimum

        // Find the index of the smallest element in the remaining unsorted part
        for (int j = i + 1; j < n; j++) {
            if (arr[j] < arr[minIndex]) {
                minIndex = j;  // Update minIndex if a smaller element is found
            }
        }

        // Swap the found minimum element with the element at the current index
        if (minIndex != i) {
            long temp = arr[i];
            arr[i] = arr[minIndex];
            arr[minIndex] = temp;
        }
    }
}

float distanceReadMedian(int sensorNum = 0) {
  long avg = 0;
  const int n_reads = 8;
  uint8_t echoPin = sensorNum == 0 ? PIN_ECHO_1 : PIN_ECHO_2;
  uint8_t trigPin = sensorNum == 0 ? PIN_TRIG_1 : PIN_TRIG_2;\
  long reads[8] = {0,0,0,0,0,0,0,0};
  for (int i = 0; i < n_reads; ++i) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    reads[i] = pulseIn(echoPin, HIGH);
  }

  selectionSort(reads, n_reads);
  long median = (reads[3]+reads[4]) >> 1; // divided by 2
  return median * .0343 / 2;
}

// All instructions from aligning with north wall,
// pushing the pot, to dropping a ball.
void endRunSequence() {
  Serial.println("endRunSequence beginning");
  // START end-run sequence
  // Move forward, aligning with the north wall.
  stepperMaxSpeed = 3000;

  DELAY(300);
  // Turn 90 deg counterclockwise
  TURN_CW(900);

  // Move forward, pushing the pot
  MOVE_FORWARD(6000);

  stepperMaxSpeed = 2000;
  DELAY(300);

  MOVE_BACKWARD(800);

  // Turn 90 deg counterclockwise
  TURN_CW(600);

  // Move forward to go around the pot handle
  MOVE_FORWARD(2000);
  DELAY(500);
  TURN_CCW(1000);
  DELAY(500);

  // Move forward a short bit to get between handles
  MOVE_FORWARD(2000);

  // ball release
  latch.write(90);

  // Wait for balls to drop.
  DELAY(2000);

  latch.write(0);

  // END end-run sequence
  stepperMaxSpeed = 3000;
  // Bookending with setting stepperMaxSpeed
  // to ensure speed appears unchanged to code outside this function.
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
  stepper_R.setMaxSpeed(10000); //do not change this

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

  void loop() {
  // Start game (start buzzer)

  // // Orient
  // while (1) {
  //   // Take reading.
  //   float dist0 = distanceReadMedian(0);
  //   delay(50); // Delay to ensure no cross-contamination between the ultrasonic sensors.
  //   float dist1 = distanceReadMedian(1);
  //   float diff = dist1 - dist0;
  //   float delta = (diff < 0) ? -1*diff : diff; // delta = abs(diff)
  //   Serial.print("Delta ");
  //   Serial.print(delta);
  //   Serial.print(", Dist ");
  //   Serial.println(dist0);
  //   bool isFacingNorth = ((delta < 5) && (dist0 > 50));


  //   if (isFacingNorth) break;

  //   // Turn a little
  //   TURN_CW(200);
  // }

  // // Turn to face north
  // TURN_CW(DEGREES_90);

  // // Align with south wall.
  // stepperMaxSpeed = 2000;
  // MOVE_BACKWARD(800);
  // stepperMaxSpeed = 4500;

  // Serial.println("Stage 1");
  // //igition servo
  // ignition.write(0);
  // MOVE_FORWARD(450);
  // // Serial.println("Stage 2");
  // TURN_CW(DEGREES_90+80);

  // //DELAY(300);
  // //hit ignition
  // stepperMaxSpeed = 3000;

  // MOVE_FORWARD(1000);
  // //stepperMaxSpeed = 4500;

  // DELAY(1000);
  // // Move forward
  // // Serial.println("Stage 3");

  // // Move across from the west wall to the east wall
  // MOVE_BACKWARD(6000);
  // // Turn 90 deg counterclockwise
  // // Serial.println("Stage 4");
  // TURN_CCW(DEGREES_90+80);

  // MOVE_FORWARD(1000);

  // endRunSequence();

  // // TODO: change numPantryVisits to a nonzero number.
  // for (int i = 0; i < numPantryVisits; ++i) {
  //   // Move out of the pot.
  //   MOVE_BACKWARD(1000);

  //   // Realign with the west wall.
  //   TURN_CW(DEGREES_90);
  //   MOVE_BACKWARD(500);

  //   // Move to the east wall.
  //   MOVE_FORWARD(6000);

  //   // Turn to face north.
  //   TURN_CCW(DEGREES_90);

  //   // Move backward into the pantry.
  //   MOVE_BACKWARD(3000);

  //   // Move forward to the north wall.
  //   MOVE_FORWARD(3000);

  //   // Move to the pot and drop the balls in again.
  //   endRunSequence();
  // }

  // START THE END OF THE GAME
  stepperMaxSpeed = 3000;
  // Move back out of the pot.
  MOVE_BACKWARD(1000);

  DELAY(500); // to ensure stop before turning. possibly can remove.
  // Hit the ignition.
  stepperMaxSpeed = 3000; // This is here to recreate the ignition code from earlier here.

  // Now turn left, and turn OFF ignition
  TURN_CW(DEGREES_90_SPEED_3000); // NOTE: From here on out, CW = CCW and CCW = CW. :()
  MOVE_FORWARD(500);

  DELAY(500);

  // Re-enter the pot
  MOVE_BACKWARD(500);
  TURN_CCW(DEGREES_90_SPEED_3000);
  MOVE_FORWARD(500);

  // Push to the customer window.
  TURN_CCW(DEGREES_90_SPEED_3000);
  MOVE_FORWARD(8000);

  // Celebrate.
  TURN_CW(DEGREES_90_SPEED_3000 << 2); // do a 360 degree rotation.

  // TODO: Turn off buzzer. Requires a digital pin.
  // Could connect the buzzer to the same pin used for 
  // a sensor's TRIG pin, since readings are no longer important here.
  while (1);
 }

void loop2() {
  stepperMaxSpeed = 3000;
  delay(1000);
  TURN_CW(DEGREES_90 + 55);
}
