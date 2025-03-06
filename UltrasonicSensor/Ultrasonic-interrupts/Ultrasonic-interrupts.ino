#define PIN_TRIG_1 8
#define PIN_TRIG_2 9
#define PIN_ECHO_1 2
#define PIN_ECHO_2 3
#define ARDUINO_COMMUNICATION_PIN_OUT 13
#define ARDUINO_COMMUNICATION_PIN_IN 12

#include "shared_defines.hpp"

void ultraStartCallback(int);

long avgRead(int sensorNum = 0) {
  long avg = 0;
  uint8_t echoPin = sensorNum == 0 ? PIN_ECHO_1 : PIN_ECHO_2;
  for (int i = 0; i < 8; ++i) {
    ultraStartCallback(sensorNum); // Send out an ultrasonic wave.
    avg += pulseIn(echoPin, HIGH) >> 3;
  }
  return avg;
}

float distanceRead(int sensorNum = 0) {
  long avg = 0;
  uint8_t echoPin = sensorNum == 0 ? PIN_ECHO_1 : PIN_ECHO_2;
  for (int i = 0; i < 8; ++i) {
    ultraStartCallback(sensorNum); // Send out an ultrasonic wave.
    avg += pulseIn(echoPin, HIGH) >> 3;
  }
  return (avg*.0343)/2;  
}

void ultraStartCallback(int sensorNum = 0) {
  // Generate an ultrasonic wave.
  uint8_t trigPin = sensorNum == 0 ? PIN_TRIG_1 : PIN_TRIG_2;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // pulseBeginTimeMicros = micros();
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_TRIG_1, OUTPUT);
  pinMode(PIN_TRIG_2, OUTPUT);
  pinMode(PIN_ECHO_1, INPUT);
  pinMode(PIN_ECHO_2, INPUT);
  pinMode(ARDUINO_COMMUNICATION_PIN_IN, INPUT);
  pinMode(ARDUINO_COMMUNICATION_PIN_OUT, OUTPUT);

  digitalWrite(PIN_TRIG_1, LOW);
  digitalWrite(PIN_TRIG_2, LOW);
  digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, LOW);

  Serial.begin(9600);
  while (!Serial);
}

void loop() {
  // Wait for a high on ARDUINO_COMMUNICATION_PIN_IN.
  while (!digitalRead(ARDUINO_COMMUNICATION_PIN_IN));

  // Take reading.
  float dist0 = distanceRead(0);
  delay(100); // Delay to ensure no cross-contamination between the ultrasonic sensors.
  float dist1 = distanceRead(1);
  float diff = dist1 - dist0;
  float delta = (diff < 0) ? -1*diff : diff; // delta = abs(diff)
  bool isFacingNorth = ((delta < 5) && (dist0 > 50));

  // Send info back to other board.
  if (isFacingNorth) {
    // long pulse.
    digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, HIGH);
    delay(LONG_PULSE);
    digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, LOW);
  } else {
    // short pulse.
    digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, HIGH);
    delay(SHORT_PULSE);
    digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, LOW);
  }
}

// The following function is used for calibration.
// Send a byte to the Arduino (just hit ENTER in the serial monitor)
// and you will see readings from both sensors in the monitor
// after a short delay.
void loop2() {
  if (Serial.read() != -1) {
    // int read0 = avgRead(0);
    float dist0 = distanceRead(0);
    delay(100);
    float dist1 = distanceRead(1);
    // int read1 = avgRead(1);
    // int delta = read0 - read1;
    
    Serial.print("Sensor 0: ");
    // Serial.print(read0);
    Serial.print(dist0);
    Serial.print(" Sensor 1: ");
    // Serial.println(read1);
    Serial.print(dist1);
    Serial.print(" Delta: ");
    Serial.println(dist0 - dist1);
    // Serial.println(delta);
  }
}
