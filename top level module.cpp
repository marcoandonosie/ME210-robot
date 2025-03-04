/*---------------Pins Defines-----------------------------*/
//define all our pins here
#define OUTPUT_PIN 8  
#define POT_PIN 14 
#define PIN_SIGNAL_IN 3




/*---------------Timer Defines-----------------------------*/
// Select the timers you're using, ITimer1 is alr connected to USE_TIMER_1
#define USE_TIMER_1 true


#include <TimerInterrupt.hpp>         //https://github.com/khoih-prog/TimerInterrupt
#include <ISR_Timer.hpp>
#include "TimerInterrupt.h"           //https://github.com/khoih-prog/TimerInterrupt
#include "ISR_Timer.h"




/*---------------Module Defines-----------------------------*/
//define all of our values, specific timer interrupt values here 
//PLEASE use these constants since it will make tweaking values for precise timing way easier 

//the sensor constants (ultrasonic, IR detection will go here later)
#define LIGHT_THRESHOLD         300  //around 450 in the day
#define LINE_THRESHOLD          300   // *Choose your own thresholds*


//the timer constants 
#define TIME_TO_DIAGONAL       1000
#define TIME_IN_PANTRY         2000

//motor control speeds; to be tweaked alter 
#define HALF_SPEED              50
#define VAR_SPEED1              70
#define VAR_SPEED2              40
#define NO_SPEED                0


#define TIMER_0                 0
/*---------------Module Function Prototypes-----------------*/
void checkGlobalEvents(void);

//sub these for movement helper functions
void handleInitialMovement(void);
void handleMoveForward(void);
void handleMoveBackward(void);


//sub these for sensing helper functions 




//sub these for state transitions (comes in pairs; one to trigger one to respond)
//e.g:
unsigned char TestForLightOn(void);
void RespToLightOn(void);
unsigned char TestForLightOff(void);
void RespToLightOff(void);
unsigned char TestForFence(void);
void RespToFence(void);


/*---------------State Definitions--------------------------*/
//oh we'll have HELLA states here
typedef enum {
 STATE_MOVE_FORWARD, 
 STATE_MOVE_BACKWARD, 
 STATE_LIGHT_OFF
} States_t;


/*---------------Module Variables---------------------------*/
States_t state;
static Metro metTimer0 = Metro(LED_TIME_INTERVAL);
static Metro metTimer1 = Metro(MOTOR_TIME_INTERVAL); //new timer for the motor; replaces the delay
uint8_t isLEDOn;

//PWM parameters
//depending on how Jay implements we can change this later 
const unsigned long PERIOD = 1000000.0 / TIMER_FREQ_HZ; // Convert frequency to microseconds
volatile unsigned long high_time;
volatile unsigned long low_time;
volatile unsigned long new_high_time;
volatile unsigned long new_low_time;
volatile bool pwmState = true; //state tracking for PWM toggling;

/*---------------Raptor Main Functions----------------*/


void setup() {
//pinmodes and digitalwrites go here 


while (!Serial);  // Done so we don't run the serial prints by accident before serial terminal is set up
ITimer1.init();

}




void loop() {
 //  put your main code here, to run repeatedly:
 //Serial.println("line value is:" + String(raptor.EdgeRight()));
 //Serial.println("light value is:" + String(raptor.LightLevel()));
 checkGlobalEvents();
 switch (state) {
//make this code for each state; we can decompose each state into a helper function
   case STATE_1:
     //Serial.println("we are now in state 1");
     helper_1();
     break;
case STATE_2:
     //Serial.println("we are now in state 2");
     helper_2();
     break;
   default:    // Should never get into an unhandled state
     Serial.println("bruh we hit default");
 }
}




/*---------------Checking for state transitions----------------*/
//declare all functions for state transitions below
void checkGlobalEvents(void) {
 //acivates the response / state changes code ONCE, when the test conditions are triggered
 if (TestLedTimerExpired()) RespLedTimerExpired();
 if (TestForKey()) RespToKey();
 if (TestForLightOn()) RespToLightOn(); //new addition
 if (TestForLightOff()) RespToLightOff(); //new addition
 if (TestForFence()) RespToFence(); //new addition
}

//below here write down all the helper functions (trigger and response) we need to do state transitions 
