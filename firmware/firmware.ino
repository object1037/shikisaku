#include <Adafruit_MMC56x3.h>
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <Keyboard.h>
#include <EEPROM.h>

// #define DEBUG

const int TRIG_H = 18;
const int ECHO_H = 15;
const int TRIG_V = 14;
const int ECHO_V = 16;
const int LED = 1;
const int NUMLEDS = 8;
const int KEY = 19;

const int DIST_H_MIN = 15;
const int DIST_H_MAX = 25;
const int DIST_V_MIN = 20;
const int DIST_V_MAX = 40;

const int HOLD_ADDR = 0;
const int COLOR_ADDR = HOLD_ADDR + sizeof(bool);

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

const double hard_iron[3] = {28.38, -42.28, 71.21};
const double soft_iron[3][3] = {
  {0.975, 0.01, 0.003},
  {0.01, 1.015, -0.039},
  {0.003, -0.039, 1.012}
};

Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
sensors_event_t mag_event;

Adafruit_NeoPixel pixels(NUMLEDS, LED, NEO_GRB + NEO_KHZ800);

double dist_h_lp = 0;
double dist_v_lp = 0;

RGB_t prev_color = {.r = 0, .g = 0, .b = 0};
RGB_t cur_color = {.r = 0, .g = 0, .b = 0};
int grad_idx = 0;

unsigned long prev_time = 0;
unsigned long prev_micros = 0;
unsigned long loop_interval = 200;

bool is_holding;
RGB_t hold_color;

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
    .r = +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s,
    .g = -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s,
    .b = -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s
  };
}

bool is_valid(const RGB_t *rgb) {
  bool ans = true;
  if (rgb->r < 0 || rgb->r > 1) ans = false;
  if (rgb->g < 0 || rgb->g > 1) ans = false;
  if (rgb->b < 0 || rgb->b > 1) ans = false;
  return ans;
}

int to_valid(const double c) {
  return constrain(int(255 * c), 0, 255);
}

double nth_div(const double prev, const double cur, const int idx, const int n) {
  double ans = prev + (cur - prev) / (n-1) * idx;
  return ans;
}

void light_LED(const RGB_t *prev, RGB_t *cur, const int idx) {
  for (int j = 0; j < NUMLEDS; j++) {
    pixels.setPixelColor(j, pixels.Color(
      to_valid(nth_div(prev->r, cur->r, idx, 10)),
      to_valid(nth_div(prev->g, cur->g, idx, 10)),
      to_valid(nth_div(prev->b, cur->b, idx, 10))
    ));
  }
  pixels.show();
}

void light_LED_delay(const RGB_t *prev, const RGB_t *cur) {
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < NUMLEDS; j++) {
      pixels.setPixelColor(j, pixels.Color(
        to_valid(nth_div(prev->r, cur->r, i, 10)),
        to_valid(nth_div(prev->g, cur->g, i, 10)),
        to_valid(nth_div(prev->b, cur->b, i, 10))
      ));
    }
    pixels.show();
    delay(15);
  }
}

void print_hex(const RGB_t *rgb) {
  long R = to_valid(gamma_correction(rgb->r));
  long G = to_valid(gamma_correction(rgb->g));
  long B = to_valid(gamma_correction(rgb->b));
  long RGB = (R << 16L) | (G << 8L) | B;
  String hex = String("#" + String(RGB, HEX));
  Keyboard.print(hex);
}

void setup() {
  pinMode(TRIG_H, OUTPUT);
  pinMode(TRIG_V, OUTPUT);
  pinMode(ECHO_H, INPUT);
  pinMode(ECHO_V, INPUT);
  pinMode(KEY, INPUT_PULLUP);

  Keyboard.begin();

  EEPROM.get(HOLD_ADDR, is_holding);
  EEPROM.get(COLOR_ADDR, hold_color);

  if (is_holding) {
    prev_color = hold_color;
    cur_color = hold_color;
  }

  pixels.begin();
  pixels.show();
  // pixels.setBrightness(75);

#ifdef DEBUG
  Serial.begin(9600);
#endif
  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
#ifdef DEBUG
    Serial.println(F("Could not find a magnetometer, check wiring!"));
#endif
    while(1) delay(10);
  }
  delay(100);

  mag.getEvent(&mag_event);
  
  delay(10);
}

void loop() {
  unsigned long cur_time = millis();
  if (is_holding || cur_time - prev_time < loop_interval) {
    if (is_holding) {
      light_LED(&hold_color, &hold_color, 9);
    } else {
      unsigned long cur_micros = micros();
      if (cur_micros - prev_micros >= 10000) {
        light_LED(&prev_color, &cur_color, grad_idx);
        grad_idx++;
        if (grad_idx >= 10) grad_idx = 9;
        prev_micros = cur_micros;
      }
    }
    if (digitalRead(KEY) == LOW) {
      bool holded = false;
      while (digitalRead(KEY) == LOW) {
        unsigned long next_time = millis();
        if (!holded && next_time - cur_time > 250) {
          holded = true;
          is_holding = !is_holding;
          hold_color = cur_color;
          EEPROM.put(HOLD_ADDR, is_holding);
          EEPROM.put(COLOR_ADDR, hold_color);
          const RGB_t black = {.r = 0, .g = 0, .b = 0};
          light_LED_delay(&hold_color, &black);
          light_LED_delay(&black, &hold_color);
        }
      }
      if (!holded) {
        print_hex(&cur_color);
      }
    }
    return;
  }
  prev_color = cur_color;

  mag.getEvent(&mag_event);
  double mag_x = 0;
  double mag_y = 0;
  calibrate_mag(&mag_event, &mag_x, &mag_y);

  double heading_rad = get_heading(mag_x, mag_y);
  
  double temp = mag.readTemperature();
  double dist_v_raw = measure_distance(TRIG_V, ECHO_V, temp);
  double dist_h_raw = measure_distance(TRIG_H, ECHO_H, temp);
  dist_h_lp = lowpass(constrain(dist_h_raw, DIST_H_MIN, DIST_H_MAX), dist_h_lp, 0.5);
  dist_v_lp = lowpass(constrain(dist_v_raw, DIST_V_MIN, DIST_V_MAX), dist_v_lp, 0.5);
  double h_perc = dist_percentage(dist_h_lp, DIST_H_MIN, DIST_H_MAX);
  double v_perc = dist_percentage(dist_v_lp, DIST_V_MIN, DIST_V_MAX);

  LCh_t lch = {
    .L = v_perc,
    .C = 0.3 * h_perc,
    .h = heading_rad
  };
  cur_color = oklch_to_rgb(&lch);

  if (!is_valid(&cur_color)) {
    cur_color.r = 0;
    cur_color.g = 0;
    cur_color.b = 0;
  }

#ifdef DEBUG
  // Serial.print(F("Heading: "));
  Serial.print(prev_time); Serial.print("->");
  Serial.print(cur_time); Serial.print(" ");
  double heading_deg = heading_rad * 180 / M_PI;
  Serial.print(heading_deg); Serial.print(", ");
  Serial.print(temp); Serial.print(", ");

  // Serial.print(F(" Dist: "));
  Serial.print(dist_h_raw); Serial.print(", ");
  Serial.print(dist_h_lp); Serial.print(", ");
  Serial.print(dist_v_raw); Serial.print(", ");
  Serial.print(dist_v_lp); Serial.print(", ");

  Serial.print("lch(");
  Serial.print(lch.L, 5); Serial.print(", ");
  Serial.print(lch.C, 5); Serial.print(", ");
  Serial.print(lch.h * 180 / M_PI); Serial.print(") -> ");

  Serial.print("rgb(");
  Serial.print(255 * cur_color.r); Serial.print(", ");
  Serial.print(255 * cur_color.g); Serial.print(", ");
  Serial.print(255 * cur_color.b); Serial.print("), ");

  Serial.print("prev(");
  Serial.print(255 * prev_color.r); Serial.print(", ");
  Serial.print(255 * prev_color.g); Serial.print(", ");
  Serial.print(255 * prev_color.b); Serial.print(")");
  Serial.println();
#endif

  prev_time = cur_time;
  grad_idx = 0;
}
