// HMC5883L.cpp
#include <Wire.h>
#include "HMC5883L.h"

void initHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x01);
  Wire.write(0xA0);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

MagnetometerCalibration calibrateHMC5883L(unsigned long calibrationTime) {
  int16_t x, y, z;
  int16_t xMin = 32767, xMax = -32768;
  int16_t yMin = 32767, yMax = -32768;

  unsigned long startTime = millis();
  Serial.println("Rotate the sensor in all directions...");

  while (millis() - startTime < calibrationTime) {
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(HMC5883L_ADDRESS, 6);

    if (Wire.available() == 6) {
      x = Wire.read() << 8 | Wire.read();
      z = Wire.read() << 8 | Wire.read();
      y = Wire.read() << 8 | Wire.read();

      if (x < xMin) xMin = x;
      if (x > xMax) xMax = x;
      if (y < yMin) yMin = y;
      if (y > yMax) yMax = y;

      Serial.print(".");
    }
    delay(100);
  }

  Serial.println("\nCalibration done.");
  MagnetometerCalibration calib;
  calib.xOffset = (xMin + xMax) / 2;
  calib.yOffset = (yMin + yMax) / 2;

  Serial.print("X offset: "); Serial.println(calib.xOffset);
  Serial.print("Y offset: "); Serial.println(calib.yOffset);

  return calib;
}

void readCalibratedHMC5883L(const MagnetometerCalibration& calib, int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDRESS, 6);

  if (Wire.available() == 6) {
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();

    x -= calib.xOffset;
    y -= calib.yOffset;
  }
}