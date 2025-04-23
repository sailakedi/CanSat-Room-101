#include "Adafruit_VEML7700.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_BMP280 bmp; // I2C interface

float initial_pressure; // just declare it here, assign it later

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (!veml.begin()) {
    Serial.println("VEML not found");
    while (1);
  }
  Serial.println("VEML found");

  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  } 

  delay(200); // Let sensor stabilize

  initial_pressure = bmp.readPressure() / 100.0F;

  // Optional: validate that it's a sane value
  if (isnan(initial_pressure) || initial_pressure < 300 || initial_pressure > 1100) {
    Serial.println("Initial pressure invalid, using default 1013.25");
    initial_pressure = 1013.25;
  }

  Serial.print("Initial pressure: ");
  Serial.println(initial_pressure);

  Serial.println("Lumin(lux), Temp(c), pres(hPa), altitude(m)"); // units
}

void loop() {
  float lux = veml.readLux(VEML_LUX_AUTO);
  Serial.print(lux); Serial.print(", ");

  Serial.print(bmp.readTemperature()); Serial.print(", ");

  Serial.print(bmp.readPressure() / 100.0F); Serial.print(", ");

  Serial.print(bmp.readAltitude(initial_pressure)); Serial.println();

  delay(1000);
}
