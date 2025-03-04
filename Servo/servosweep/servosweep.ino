#include <Servo.h>


//12 servos max on most boards
Servo IgnitionServo;  // create servo object to control a servo
Servo RampServo;


void ActivateIgnitionServo();  // Function to start servo movement
void ReverseIgnitionServo();   // Function to reverse servo movement
void DisableIgnitionServo();   // Function to stop servo movement

void ActivateRampServo();  // Function to start servo movement
void ReverseRampServo();   // Function to reverse servo movement
void DisableRampServo();   // Function to stop servo movement

int pos = 0;    // variable to store the servo position


void ActivateIgnitionServo() {
  Serial.println("Ignition Servo is opening...")
    for (pos = 0; pos <= 180; pos += 2) {
      //made increments 2 so it would go faster
        Ignitionservo.write(pos);
        delay(15);
    }
}

void ReverseIgnitionServo() {
  Serial.println("Ignition Servo is closing...")
    for (pos = 0; pos >= 180; pos -= 2) {
      //made increments 2 so it would go faster
        Ignitionservo.write(pos);
        delay(15);
    }
}

void DisableIgnitionServo() {
    Ignitionservo.detach(); // Detaches the servo to stop it from moving
}



void ActivateRampServo() {
  Serial.println("Ramp Servo is opening...")
    for (pos = 0; pos <= 180; pos += 2) {
      //made increments 2 so it would go faster
        Rampservo.write(pos);
        delay(15);
    }
}

void ReverseRampServo() {
  Serial.println("Ramp Servo is closing...")
    for (pos = 0; pos >= 180; pos -= 2) {
      //made increments 2 so it would go faster
        Rampservo.write(pos);
        delay(15);
    }
}

void DisableRampServo() {
    Rampservo.detach(); // Detaches the servo to stop it from moving
}
