#include <Adafruit_MMC56x3.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
sensors_event_t mag_event;

sensors_event_t event;
float min_x, max_x, mid_x, cal_x;
float min_y, max_y, mid_y, cal_y;
float min_z, max_z, mid_z, cal_z;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println(F("Sensor Lab - Magnetometer Calibration!"));
  
  Serial.println(F("Looking for a magnetometer"));
  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
    Serial.println(F("Could not find a magnetometer, check wiring!"));
    while(1) delay(10);
  }
  mag.printSensorDetails();
  delay(100);

  mag.getEvent(&event);
  min_x = max_x = event.magnetic.x;
  min_y = max_y = event.magnetic.y;
  min_z = max_z = event.magnetic.z;
  delay(10);
}



void loop() {
  mag.getEvent(&event);
  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;
  
  Serial.print(F("Mag: ("));
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(z); Serial.print(")");

  min_x = min(min_x, x);
  min_y = min(min_y, y);
  min_z = min(min_z, z);

  max_x = max(max_x, x);
  max_y = max(max_y, y);
  max_z = max(max_z, z);

  mid_x = (max_x + min_x) / 2;
  mid_y = (max_y + min_y) / 2;
  mid_z = (max_z + min_z) / 2;

  cal_x = x - mid_x;
  cal_y = y - mid_y;
  cal_z = z - mid_z;
  Serial.print(F(" Hard offset: ("));
  Serial.print(mid_x); Serial.print(", ");
  Serial.print(mid_y); Serial.print(", ");
  Serial.print(mid_z); Serial.print(")");  

  Serial.print(F(" Field: ("));
  Serial.print((max_x - min_x)/2); Serial.print(", ");
  Serial.print((max_y - min_y)/2); Serial.print(", ");
  Serial.print((max_z - min_z)/2); Serial.println(")");

  Serial.print(F(" calibrated: ("));
  Serial.print(cal_x); Serial.print(", ");
  Serial.print(cal_y); Serial.print(", ");
  Serial.print(cal_z); Serial.println(")");

  float Pi = 3.14159;
  float heading = (atan2(cal_y, cal_x) * 180) / Pi;
  if (heading < 0) {
    heading = 360 + heading;
  }

  Serial.println(" heading: ");
  Serial.println(heading);
  
  delay(10); 
}