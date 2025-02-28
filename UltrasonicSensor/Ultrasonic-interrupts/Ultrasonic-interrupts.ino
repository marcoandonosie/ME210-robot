#define PIN_TRIG 6
#define PIN_ECHO 3

#define USE_TIMER_1 true
#include "TimerInterrupt.h"
#include "ISR_Timer.h"

float distance, duration;
long avgLast4 = 0;
long last2Avgs[2] = {0,0};
long last4[4] = {0,0,0,0};
uint32_t n = 0; 
uint32_t m = 0;
long pulseBeginTimeMicros = 0;
void ultrasonicCallback() {
  long duration = micros() - pulseBeginTimeMicros;
  // Serial.print("Sensor read ");
  // Serial.println(duration);
  avgLast4 += duration >> 2;
  if (n == 3) {
    avgLast4 >>= 1; // >> 2 to average, then another >> 1 for consistency.
    // Serial.print(avgLast4);
    // Serial.print(",");
    // Serial.print(last2Avgs[1]);
    // Serial.print(",");
    // Serial.println(last2Avgs[0]);

    if ((avgLast4 > (last2Avgs[1] + 1)) && (last2Avgs[0] > last2Avgs[1] + 1))
      {Serial.print(m++); Serial.println("min found!");
      Serial.print(avgLast4);
      Serial.print(",");
      Serial.print(last2Avgs[1]);
      Serial.print(",");
      Serial.println(last2Avgs[0]);}

    last2Avgs[0] = last2Avgs[1];
    last2Avgs[1] = avgLast4;
  }
  n = (n < 3) ? n + 1 : 0;
  
}

void ultraStartCallback() {
  // Generate an ultrasonic wave.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  pulseBeginTimeMicros = micros();
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
}

void setup() {
  ITimer1.init();
  ITimer1.attachInterrupt(100.0, ultraStartCallback); // 10 times per second
  // put your setup code here, to run once:
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(9600);
  while (!Serial);

  attachInterrupt(digitalPinToInterrupt(PIN_ECHO),
    ultrasonicCallback,
    FALLING);

  // Generate first ultrasonic wave.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  pulseBeginTimeMicros = micros();
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
}

void loop() {
  // ensure ultrasonic sensor has had a read before next emission.
  // delay(100);
}
