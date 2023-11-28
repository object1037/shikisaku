#include <Adafruit_MMC56x3.h>

const int TRIG_H = 16;
const int ECHO_H = 10;
const int TRIG_V = 9;
const int ECHO_V = 8;
const int LED_R = 6;
const int LED_G = 9;
const int LED_B = 10;

const int DIST_H_MIN = 15;
const int DIST_H_MAX = 27;
const int DIST_V_MIN = 18;
const int DIST_V_MAX = 43;

typedef struct {
  double L;
  double C;
  double h;
} LCh_t;
typedef struct {
  double r;
  double g;
  double b;
} RGB_t;

const double hard_iron[3] = {32.51, -21.84, 54.53};
const double soft_iron[3][3] = {
  {1.006, 0.018, 0.001},
  {0.018, 0.956, -0.009},
  {0.001, 0.009, 1.041}
};

Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
sensors_event_t mag_event;

double dist_h_lp = 0;
double dist_v_lp = 0;

double lowpass(const double sample, const double prev, const double rc) {
  return sample - rc * (sample - prev);
}

double measure_distance(const int TRIG, const int ECHO, const double temp) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  double duration = pulseIn(ECHO, HIGH);
  double velocity = 331.45 + 0.61 * temp;

  if (duration > 0) {
    double distance = duration / 2 / 1000000 * velocity * 100;
    return distance;
  }
  return -1;
}

double dist_percentage(const double dist, const int d_min, const int d_max) {
  int range = d_max - d_min;
  return (dist - d_min) / range;
}

void calibrate_mag(sensors_event_t* mag_event, double* x, double* y) {
  double mag_data[] = {
    mag_event->magnetic.x,
    mag_event->magnetic.y,
    mag_event->magnetic.z
  };
  double hi_cal[3];

  for (int i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }
  for (int i = 0; i < 3; i++) {
    mag_data[i] = soft_iron[i][0] * hi_cal[0] + soft_iron[i][1] * hi_cal[1] + soft_iron[i][2] * hi_cal[2];
  }

  *x = mag_data[0];
  *y = mag_data[1];

  return;
}

double get_heading(const double x, const double y) {
  double heading = -1 * atan2(x, y);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  return heading;
}

double gamma_correction(const double c) {
  double abs = abs(c);
  int sign = 1;
  if (c < 0) {
    sign = -1;
  }
  if (abs <= 0.0031308) {
    return c * 12.92;
  }

  return sign * (1.055 * pow(abs, 1.0 / 2.4) - 0.055);
}

RGB_t oklch_to_rgb(const LCh_t* lch) {
  double a = lch->C * cos(lch->h);
  double b = lch->C * sin(lch->h);

  double l_ = lch->L + 0.3963377774f * a + 0.2158037573f * b;
  double m_ = lch->L - 0.1055613458f * a - 0.0638541728f * b;
  double s_ = lch->L - 0.0894841775f * a - 1.2914855480f * b;

  double l = l_*l_*l_;
  double m = m_*m_*m_;
  double s = s_*s_*s_;

  return {
    .r = gamma_correction(+4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s),
    .g = gamma_correction(-1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s),
    .b = gamma_correction(-0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s)
  };
}

int to_valid(const double c) {
  return constrain(int(255 * c), 0, 255);
}

void light_LED(const RGB_t *rgb) {
  analogWrite(LED_R, (int)(255 * to_valid(rgb->r)));
  analogWrite(LED_G, (int)(255 * to_valid(rgb->g)));
  analogWrite(LED_B, (int)(255 * to_valid(rgb->b)));
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_H, OUTPUT);
  pinMode(TRIG_V, OUTPUT);
  pinMode(ECHO_H, INPUT);
  pinMode(ECHO_V, INPUT);

  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
    Serial.println(F("Could not find a magnetometer, check wiring!"));
    while(1) delay(10);
  }
  delay(100);

  mag.getEvent(&mag_event);
  
  delay(10);
}

void loop() {
  mag.getEvent(&mag_event);
  double mag_x = 0;
  double mag_y = 0;
  calibrate_mag(&mag_event, &mag_x, &mag_y);

  double heading_rad = get_heading(mag_x, mag_y);
  double heading_deg = heading_rad * 180 / M_PI;

  double temp = mag.readTemperature();
  double dist_h_raw = measure_distance(TRIG_H, ECHO_H, temp);
  double dist_v_raw = measure_distance(TRIG_V, ECHO_V, temp);
  dist_h_lp = lowpass(constrain(dist_h_raw, DIST_H_MIN, DIST_H_MAX), dist_h_lp, 0.5);
  dist_v_lp = lowpass(constrain(dist_v_raw, DIST_V_MIN, DIST_V_MAX), dist_v_lp, 0.5);
  double h_perc = dist_percentage(dist_h_lp, DIST_H_MIN, DIST_H_MAX);
  double v_perc = dist_percentage(dist_v_lp, DIST_V_MIN, DIST_V_MAX);

  LCh_t lch = {
    .L = v_perc,
    .C = 0.3 * h_perc,
    .h = heading_rad
  };
  RGB_t rgb = oklch_to_rgb(&lch);

  // Serial.print(F("Heading: "));
  Serial.print(heading_deg); Serial.print(", ");

  // Serial.print(F(" Dist: "));
  Serial.print(dist_h_lp); Serial.print(", ");
  Serial.print(h_perc); Serial.print(", ");
  Serial.print(dist_v_lp); Serial.print(", ");
  Serial.print(v_perc); Serial.print(", ");

  Serial.print("lch(");
  Serial.print(lch.L, 5); Serial.print(", ");
  Serial.print(lch.C, 5); Serial.print(", ");
  Serial.print(lch.h * 180 / M_PI); Serial.print(") -> ");

  Serial.print("rgb(");
  Serial.print(255 * rgb.r); Serial.print(", ");
  Serial.print(255 * rgb.g); Serial.print(", ");
  Serial.print(255 * rgb.b); Serial.print(")");
  Serial.println();

  light_LED(&rgb);

  delay(200);
}
