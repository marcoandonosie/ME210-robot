#include <TMCStepper.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>
//#include "ServoControl.ino" //add this if servocontrol is not in the same file in IDE / in an open tab atm 


/*––––––––––––TIMER STUFF––––––––––––––*/
// Select the timers you're using, ITimer1 is alr connected to USE_TIMER_1
#define USE_TIMER_1 true 
#define USE_TIMER_2 true
#include <TimerInterrupt.hpp>         //https://github.com/khoih-prog/TimerInterrupt
#include <ISR_Timer.hpp>
#include "TimerInterrupt.h"           //https://github.com/khoih-prog/TimerInterrupt
#include "ISR_Timer.h"


volatile bool timerStarted = false; 
volatile bool timerTriggered = false; // Set to true when interrupt fires


// Define UART communication pins
#define RX_PIN_R 0 // TMC2209 TX (PDN)RIGHT MOTOR
#define TX_PIN_R 1 // TMC2209 RX (PDN)
#define RX_PIN_L 7 // TMC2209 TX (PDN)LEFT MOTOR
#define TX_PIN_L 8 //TMC2209 RX (PDN)

#define DRIVER_ADDRESS 0b00 // Default TMC2209 address
#define RSENSE 0.11f // R_SENSE resistor value (check your hardware)

// Create SoftwareSerial instance for TMC2209 UART communication
SoftwareSerial SERIAL_PORT_RIGHT(RX_PIN_R, TX_PIN_R);
SoftwareSerial SERIAL_PORT_LEFT(RX_PIN_L, TX_PIN_L);


// Create TMC2209 driver object (MUST use SoftwareSerial pointer)
TMC2209Stepper driver_L(&SERIAL_PORT_LEFT, RSENSE, DRIVER_ADDRESS);
TMC2209Stepper driver_R(&SERIAL_PORT_RIGHT, RSENSE, DRIVER_ADDRESS);

// Define Stepper Motor Driver Pins
//right motors
#define STEP_PIN_R 4
#define DIR_PIN_R 3
#define ENABLE_PIN_R 2

//left motors 
#define STEP_PIN_L 9
#define DIR_PIN_L 6
#define ENABLE_PIN_L 10

// Stepper Motor (Step/Dir Control)
AccelStepper stepper_R(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper stepper_L(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);

//Servo Motors 
//TODO: change the pin configs so the two servos have pins to attach to the board 
#define SERVO_IGNITION_PIN //TODO: reconfigure these pins and assign SERVO_IGNITION something 
#define SERVO_RAMP_PIN //TODO: reconfigure these pins and assign SERVO_IGNITION something 


/*---------------CONSTANTS FOR MOVEMENT ----------------------------*/

//TODO: write constants here for our timer logic for moving the robot





/*---------------STATE MACHINE SETUP----------------------------*/
typedef enum {
 STATE_ORIENTATION, // state 1
 STATE_IGNITION,  // state 2
 STATE_ENTER_PANTRY, //state 3
 STATE_LOAD_RAMP
} States_t;

States_t state; //create state variable 

volatile int step = 0; // Tracks movement phase
//these track if a state is done or no
volatile bool orientationDone = false;
volatile bool ignitionDone = false;
volatile bool enterPantryDone = false;

typedef enum {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT, 
  STOP
} Directions_t;

Directions_t direction; //create a direction variable 
/*--------------––––––––––––––––––––––----------------------------*/
/*               MAIN CODE BEGINS HERE                            */
/*--------------––––––––––––––––––––––----------------------------*/

void setup() {
Serial.begin(115200);
SERIAL_PORT_RIGHT.begin(115200); // Corrected baud rate for UART
SERIAL_PORT_LEFT.begin(115200);
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

pinMode(IGNITION_SERVO_PIN, OUTPUT);
pinMode(RAMP_SERVO_PIN, OUTPUT);
//TODO: decide if I need to pinmode servos to low OR high in the beginning 


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
//TODO: declare the acceleration speed, and set the acceleration speed lower so our robot doesnt go off course when going stop-> forward

//configure servos
IgnitionServo.attach(SERVO_IGNITION);
RampServo.attach (SERVO_RAMP);

//timer logic 
Timer1.initialize(2000000);  // Set timer to 2,000,000 µs (2 sec)
  timerTriggered = false; // Set to true when interrupt fires

//state variables here
state = STATE_ORIENTATION; //state 1
direction = STOP;



Serial.println("TMC2209 Initialized. Motor should start moving...");
}


void loop() {
  checkGlobalEvents();
  switch (state) {
    case STATE_ORIENTATION:
      Serial.println("We are now in state 1: orientation");
      HandleOrientation();
      break;
    case STATE_IGNITION:
      Serial.println("We are now in state 2: ignition");
      HandleIgnition();
      break;
    case STATE_ENTER_PANTRY:
      Serial.println("We are now in state 3: entering the pantry");
      HandleEnterPantry();
      break;
    case STATE_LOAD_RAMP:
      Serial.println("We are now in state 4: loading the ramp");
      HandleLoadRamp();
      break;
    default: // Should never get into an unhandled state
      Serial.println("bruh we hit default");
      break;
  }
}














/*---------------Checking for state transitions----------------*/
//declare all functions for state transitions below
void checkGlobalEvents(void) {
 //acivates the response / state changes code ONCE, when the test conditions are triggered
 if (OrientationIsFound()) RespOrientationIsFound(); 
 if (IgnitionIsHit()) RespIgnitionIsHit();
 if (PantryIsEntered()) RespPantryIsEntered();
}




bool OrientationIsFound() {
  //if micha's ultrasonic sensing is DONE and robot is in the right position, orientationDone will be the code for that actually
  return (orientationDone);
}

void RespOrientationIsFound() {
  //state transition here
 if (state == STATE_ORIENTATION) {
   state = STATE_IGNITION;
   step = 0;
 }
}


bool IgnitionIsHit() {
  return (ignitionDone);
}

void RespIgnitionIsHit() {
  if (state == STATE_IGNITION) {
    state = STATE_ENTER_PANTRY;
    step = 0;
  }
}

bool PantryIsEntered() {
  return (enterPantryDone); // Placeholder: Replace with real sensor check
}

void RespPantryIsEntered() {
  if (state == STATE_ENTER_PANTRY) { 
    //TODO: implement future states
    state = STATE_LOAD_RAMP; //amend this state transition to state 5 once I implement future states
    step = 0;
  }
}


/*---------------STATE FUNCTIONS----------------*/
//these we call in our states to get the robot to do what we want it to


void HandleOrientation(void) {
  //micha's code 
  orientationDone = true;
}

void HandleIgnition (void) {
  //here the robot is facing forwards already 
  if (step == 0) { // Step 1: Turn Right
    if (!timerStarted) {
      timerTriggered = false;
      ITimer1.attachInterruptInterval(2000000, setMoveStopTimer); //only happens once 
      //TODO: define the time above, and everywhere else, instead of just writing a number 
      timerStarted = true;
      Serial.println("Ignition 1: Move forward towards ignition!!");
    }
    SetMoveForward();  //only runs once
    //timerTriggered is set true eventually; sets timerTriggered true and increments 'step' by 1
  } 
   else if (step == 1 && timerTriggered) { // Step 2: Move Forward
    if (!timerStarted) {
      timerTriggered = false;
      ITimer1.attachInterruptInterval(2000000, setMoveStopTimer); //only happens once 
      //TODO: define the time above, and everywhere else, instead of just writing a number 
      timerStarted = true;
      Serial.println("Ignitionn 2: Ignition Servo ignites!!");
    }
    ActivateIgnitionServo();
    SetMoveStop();
  } 
  else if (step == 2 && timerTriggered) { // Step 5: Stop Movement
    timerTriggered = false;
    Serial.println("Ignition sequence complete.");
    SetMoveStop();
    step = 0; //Reset the sequence counter
    IgnitionDone = true;
  }

  move();
}

void HandleEnterPantry() {
  if (step == 0) { // Step 1: Turn Right
    if (!timerStarted) {
      ITimer1.attachInterruptInterval(2000000, setMoveStopTimer); //only happens once 
      //TODO: define the time above, and everywhere else, instead of just writing a number 
      timerStarted = true;
      Serial.println("Pantry 1: turn towards far wall!!");
    }
    SetMoveRight();  //only runs once
    //timerTriggered is set true eventually; sets timerTriggered true and increments 'step' by 1
  } 
  else if (step == 1 && timerTriggered) { // Step 2: Move Forward
    if (!timerStarted) {
      timerTriggered = false;
      ITimer1.attachInterruptInterval(2000000, setMoveStopTimer); //only happens once 
      //TODO: define the time above, and everywhere else, instead of just writing a number 
      timerStarted = true;
      Serial.println("Pantry 2: drive forwards towards far wall!!");
    }
    SetMoveForward();  //only runs once
  } 
  else if (step == 2 && timerTriggered) { // Step 3: Turn Left
    if (!timerStarted) {
      timerTriggered = false;
      ITimer1.attachInterruptInterval(2000000, setMoveStopTimer); //only happens once 
      //TODO: define the time above, and everywhere else, instead of just writing a number 
      timerStarted = true;
      Serial.println("Pantry 3: turn left to face customer wall!");
    }
    SetMoveLeft();  //only runs once
  } 
  else if (step == 3 && timerTriggered) { // Step 4: Move Backward
    if (!timerStarted) {
      timerTriggered = false;
      ITimer1.attachInterruptInterval(2000000, setMoveStopTimer); //only happens once 
      //TODO: define the time above, and everywhere else, instead of just writing a number 
      timerStarted = true;
      Serial.println("Pantry 4: turn left to face customer wall!");
    }
    SetMoveLeft();  //only runs once
  } 
  else if (step == 4 && timerTriggered) { // Step 5: Stop Movement
    timerTriggered = false;
    Serial.println("Pantry sequence complete.");
    SetMoveStop();
    step = 0; //Reset the sequence counter
    enterPantryDone = true;
    //TODO: this should trigger the next stage: rampDeployed, in global variables, which we will make after this initial code works 
  }

  move(); // Ensure stepper motor movement continues smoothly
}

void HandleLoadRamp() {
  Serial.println("YASSSSSS we're 30% there");
}


/*---------------HELPER FUNCTIONS----------------*/
//the bottommost level of functions that we call in the state functions


//setting direction 
void SetMoveForward(void) {
  if (direction != FORWARD) {
  stepper_R.setSpeed(4500);
  stepper_L.setSpeed(-4500);
  direction = FORWARD;
  }
}

void SetMoveBackward(void) {
  if (direction != BACKWARD) {
  stepper_R.setSpeed(-4500);
  stepper_L.setSpeed(-4500);
  direction = BACKWARD;
  }
}
 //TODO: check if left and right directions are swapped
void SetMoveLeft(void) {
  if (direction != LEFT) {
  stepper_R.setSpeed(1000);
  stepper_L.setSpeed(1000);
  direction = LEFT;
  }
}

void SetMoveRight(void) {
  if (direction != RIGHT) {
  stepper_R.setSpeed(-1000);
  stepper_L.setSpeed(-1000);
  direction = RIGHT;
  }
}

void SetMoveStop(void) {
  if (direction != STOP) {
  stepper_R.setSpeed(-0);
  stepper_L.setSpeed(-0);
  direction = STOP;
  //TODO: make sure we turn the motors OFF so they dont draw current / hold torque when we're in the pantry and stopping fr
  }
}

//this version is the brief stop between motions; includes timer logic 
void SetMoveStopTimer(void) {
  if (direction != STOP) {
  stepper_R.setSpeed(-0);
  stepper_L.setSpeed(-0);
  direction = STOP;
  timerTriggered = true;
  timerStarted = false;
  step++;
  ITimer1.detachInterrupt(); //means that timer detaches itself once the interrupt hits once 
  }
}


//MOVING
void move(void) {
  stepper_L.runSpeed();
  stepper_R.runSpeed();
  switch (direction) {
    case FORWARD:
      Serial.println("Moving forward...");
      break;
    case BACKWARD:
      Serial.println("Moving backward...");
      break;
    case RIGHT:
      Serial.println("Turning right...");
      break;
    case LEFT:
      Serial.println("Turning left...");
      break;
    case STOP:
      Serial.println("Stopped...");
      break;
    default: // Should never get into an unhandled state
      Serial.println("bruh we hit default direction");
      break;
  }
}

