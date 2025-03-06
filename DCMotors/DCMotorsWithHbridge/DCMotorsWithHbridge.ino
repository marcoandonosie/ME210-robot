#define enA 9
#define in1 6
#define in2 7
#define in3 10
#define in4 11
#define enB 5

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void loop() {
  int pwmOutput = 255;
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutput);
  // if (pressed == true  & rotDirection == 0) {
    // digitalWrite(in1, HIGH);
    // digitalWrite(in2, LOW);
    // rotDirection = 1;
    // delay(20);
  // }
  // If button is pressed - change rotation direction
  // if (pressed == false & rotDirection == 1) {
  //   digitalWrite(in1, LOW);
  //   digitalWrite(in2, HIGH);
  //   rotDirection = 0;
  //   delay(20);
  // }
}