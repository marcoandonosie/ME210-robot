#define PIN_TRIG_1 8
#define PIN_TRIG_2 9
#define PIN_ECHO_1 2
#define PIN_ECHO_2 3

long avgLast4 = 0;
// long last2Avgs[2] = {0,0};
// long last4[4] = {0,0,0,0};
uint32_t readings[32];
uint32_t n = 0; 
uint32_t m = 0;
long pulseBeginTimeMicros = 0;

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
  pulseBeginTimeMicros = micros();
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
  digitalWrite(PIN_TRIG_1, LOW);
  digitalWrite(PIN_TRIG_2, LOW);

  Serial.begin(9600);
  while (!Serial);
}

void loop() {
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
