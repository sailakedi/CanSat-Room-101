#include <SD.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_VEML7700.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPSMinus.h>

#define RXPIN 14
#define TXPIN -1
#define GPSBAUD 9600

#define BYPASS_LORA false  // Set true for Serial output, false for LoRa

TinyGPSMinus gps;
SoftwareSerial ss(RXPIN, TXPIN);
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_BMP280 bmp;

float initial_pressure;

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

#if BYPASS_LORA
  #define PRINT(x)    Serial.print(x)
  #define PRINTLN(x)  Serial.println(x)
  #define PRINTLN_()  Serial.println()
  #define BEGIN_PKT()
  #define END_PKT()
#else
  #define PRINT(x)    LoRa.print(x)
  #define PRINTLN(x)  LoRa.println(x)
  #define PRINTLN_()  LoRa.println()
  #define BEGIN_PKT() LoRa.beginPacket()
  #define END_PKT()   LoRa.endPacket()
#endif

void setup() {
  #if BYPASS_LORA
    Serial.begin(115200);
    while (!Serial) delay(10);
  #else
    LoRa.begin(915E6);                // Set LoRa frequency
    LoRa.setSyncWord(0xF3);           // Custom sync word
  #endif

  ss.begin(GPSBAUD);

  if (!veml.begin()) {
    PRINTLN("VEML not found");
    while (1);
  }
  PRINTLN("VEML found");

  if (!bmp.begin(0x76)) {
    PRINTLN("Could not find BMP280!");
    while (1);
  }

  if (!mpu.begin()) {
    PRINTLN("MPU6050 not found!");
    while (1);
  }
  PRINTLN("MPU6050 Found");

  delay(200);  // Sensor settle time

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  initial_pressure = bmp.readPressure() / 100.0F;
  PRINT("Initial pressure: ");
  PRINTLN(initial_pressure);

  PRINTLN("lat, long, Lumin(lux), Temp(c), pres(hPa), altitude(m)");
}

void loop() {
  bool newData = false;

  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (ss.available()) {
      char c = ss.read();
      if (gps.encode(c))
        newData = true;
    }
  }

  BEGIN_PKT();

  if (newData) {
    char lat[9], lon[10];
    unsigned long age;
    strcpy(lat, gps.get_latitude());
    strcpy(lon, gps.get_longitude());
    gps.get_pos_age(&age);

    PRINT(lat); PRINT(", ");
    PRINT(lon); PRINT(", ");
  }

  float lux = veml.readLux(VEML_LUX_AUTO);
  PRINT(lux); PRINT(", ");

  PRINT(bmp.readTemperature()); PRINT(", ");
  PRINT(bmp.readPressure() / 100.0F); PRINT(", ");
  PRINT(bmp.readAltitude(initial_pressure));

  sensors_event_t accel;
  mpu_accel->getEvent(&accel);
  PRINT(accel.acceleration.x); PRINT(", ");
  PRINT(accel.acceleration.y); PRINT(", ");
  PRINTLN(accel.acceleration.z); PRINT(", ");

  END_PKT();

  delay(1000);
}
