#include <Adafruit_MMC56x3.h>

typedef struct {
  float L;
  float C;
  float h;
} LCh_t;
typedef struct {
  float r;
  float g;
  float b;
} RGB_t;

Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
sensors_event_t mag_event;

float min_x, max_x, mid_x;
float min_y, max_y, mid_y;

const int TRIG_1 = 16;
const int ECHO_1 = 10;
const int TRIG_2 = 9;
const int ECHO_2 = 8;

double duration = 0;
double distance = 0;

double measure_distance(int TRIG, int ECHO) {
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

void calibrate_mag(sensors_event_t* mag_event, float* x, float* y) {
  float raw_x = mag_event->magnetic.x;
  float raw_y = mag_event->magnetic.y;

  min_x = min(min_x, raw_x);
  min_y = min(min_y, raw_y);

  max_x = max(max_x, raw_x);
  max_y = max(max_y, raw_y);

  mid_x = (max_x + min_x) / 2;
  mid_y = (max_y + min_y) / 2;

  *x = raw_x - mid_x;
  *y = raw_y - mid_y;

  return;
}

float get_heading(const float x, const float y) {
  float heading = -1 * atan2(x, y);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  return heading;
}

float gamma_correction(float c) {
  if (c <= 0.0031308) {
    return c * 12.92;
  }
  return 1.055 * pow(c, 1 / 2.4) - 0.055;
}

RGB_t oklch_to_rgb(LCh_t* lch) {
  float a = lch->C * cos(lch->h);
  float b = lch->C * sin(lch->h);

  float l_ = lch->L + 0.3963377774f * a + 0.2158037573f * b;
  float m_ = lch->L - 0.1055613458f * a - 0.0638541728f * b;
  float s_ = lch->L - 0.0894841775f * a - 1.2914855480f * b;

  float l = l_*l_*l_;
  float m = m_*m_*m_;
  float s = s_*s_*s_;

  return {
    .r = gamma_correction(+4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s),
    .g = gamma_correction(-1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s),
    .b = gamma_correction(-0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s)
  };
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_1, OUTPUT);
  pinMode(TRIG_2, OUTPUT);
  pinMode(ECHO_1, INPUT);
  pinMode(ECHO_2, INPUT);

  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
    Serial.println(F("Could not find a magnetometer, check wiring!"));
    while(1) delay(10);
  }
  delay(100);

  mag.getEvent(&mag_event);
  min_x = max_x = mag_event.magnetic.x;
  min_y = max_y = mag_event.magnetic.y;
  
  delay(10);
}

void loop() {
  mag.getEvent(&mag_event);
  float mag_x = 0;
  float mag_y = 0;
  calibrate_mag(&mag_event, &mag_x, &mag_y);

  float heading = get_heading(mag_x, mag_y);

  double distance_1 = measure_distance(TRIG_1, ECHO_1);
  double distance_2 = measure_distance(TRIG_2, ECHO_2);

  Serial.print(F("Heading: "));
  Serial.print(heading);

  Serial.print(F(" Field: ("));
  Serial.print((max_x - min_x)/2); Serial.print(", ");
  Serial.print((max_y - min_y)/2); Serial.print(")");

  Serial.print(F(" Dist: "));
  Serial.print(distance_1); Serial.print(", ");
  Serial.print(distance_2); Serial.println(")");

  delay(200);
}
