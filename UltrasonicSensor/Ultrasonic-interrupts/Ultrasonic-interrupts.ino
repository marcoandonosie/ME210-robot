#define PIN_TRIG_1 13
#define PIN_TRIG_2 2
#define PIN_ECHO_1 12
#define PIN_ECHO_2 5
#define ARDUINO_COMMUNICATION_PIN_OUT 13
#define ARDUINO_COMMUNICATION_PIN_IN 12

// ensure following are defined equally in this and master-fsm.
#define LONG_PULSE 500
#define SHORT_PULSE 50

void ultraStartCallback(int);

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
  pinMode(PIN_TRIG_1, OUTPUT);
  pinMode(PIN_TRIG_2, OUTPUT);
  pinMode(PIN_ECHO_1, INPUT);
  pinMode(PIN_ECHO_2, INPUT);

  digitalWrite(PIN_TRIG_1, LOW);
  digitalWrite(PIN_TRIG_2, LOW);
  digitalWrite(ARDUINO_COMMUNICATION_PIN_OUT, LOW);

  Serial.begin(9600);
  while (!Serial);
}

void loop3() {
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

// The following function is used for calibration.
// Send a byte to the Arduino (just hit ENTER in the serial monitor)
// and you will see readings from both sensors in the monitor
// after a short delay.
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
    reads[i] = pulseIn(echoPin, HIGH) >> 3;
  }

  selectionSort(reads, n_reads);
  
  return (reads[3] + reads[4]) * .0343 / 2;
}

void loop() {
  if (Serial.read() != -1) {
    float dist0 = distanceReadMedian(0);
    delay(100);
    float dist1 = distanceReadMedian(1);
    
    Serial.print("Sensor 0: ");
    Serial.print(dist0);
    Serial.print(" Sensor 1: ");
    Serial.print(dist1);
    Serial.print(" Delta: ");
    Serial.println(dist0 - dist1);
  }
}

