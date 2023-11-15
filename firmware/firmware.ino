const int TRIG_1 = 16;
const int ECHO_1 = 10;
const int TRIG_2 = 9;
const int ECHO_2 = 8;

double duration = 0;
double distance = 0;

double measureDistance(int TRIG, int ECHO) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);

  if (duration > 0) {
    distance = duration / 2 / 1000000 * 340 * 100;
    return distance;
  }
  return -1;
}

void setup() {
  pinMode(TRIG_1, OUTPUT);
  pinMode(TRIG_2, OUTPUT);
  pinMode(ECHO_1, INPUT);
  pinMode(ECHO_2, INPUT);
}

void loop() {
  double distance_1 = measureDistance(TRIG_1, ECHO_1);
  double distance_2 = measureDistance(TRIG_2, ECHO_2);
  Serial.print(distance_1);
  Serial.print(", ");
  Serial.println(distance_2);

  delay(200);
}
