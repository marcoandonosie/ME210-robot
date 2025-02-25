#define PIN_TRIG 6
#define PIN_ECHO 3


float distance, duration;
long pulseBeginTimeMicros = 0;
void ultrasonicCallback() {
  long duration = micros() - pulseBeginTimeMicros;
  Serial.print("Sensor read ");
  Serial.println(duration);

  // Generate an ultrasonic wave.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  pulseBeginTimeMicros = micros();
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
}

void setup() {
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
