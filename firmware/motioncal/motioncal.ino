#include <Arduino.h>

#include <Adafruit_MMC56x3.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);

int loopcount = 0;

// Hard-iron calibration settings
const float hard_iron[3] = {
    0, 0, 0
};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 }
};

void setup(void)
{
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit_MMC5603 Magnetometer Test");
    Serial.println("");

    /* Initialise the sensor */
    if (!mmc.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) { // I2C mode
        /* There was a problem detecting the MMC5603 ... check your connections */
        Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
        while (1)
            delay(10);
    }

    /* Display some basic information on this sensor */
    mmc.printSensorDetails();
}

void loop(void)
{
    static float hi_cal[3];
    // Get a new sensor event
    sensors_event_t event;
    mmc.getEvent(&event);

    // Put raw magnetometer readings into an array
    float mag_data[] = { event.magnetic.x,
        event.magnetic.y,
        event.magnetic.z };

    // Apply hard-iron offsets
    for (uint8_t i = 0; i < 3; i++) {
        hi_cal[i] = mag_data[i] - hard_iron[i];
    }

    // Apply soft-iron scaling
    for (uint8_t i = 0; i < 3; i++) {
        mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
    }

    // 'Raw' values to match expectation of MOtionCal
    // Serial.print("Raw:");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(int(mag_data[0] * 10));
    // Serial.print(",");
    // Serial.print(int(mag_data[1] * 10));
    // Serial.print(",");
    // Serial.print(int(mag_data[2] * 10));
    // Serial.println("");

    // unified data
    // Serial.print("Uni:");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    Serial.print(mag_data[0]);
    Serial.print(",");
    Serial.print(mag_data[1]);
    Serial.print(",");
    Serial.print(mag_data[2]);
    Serial.println("");

    delay(10);
}