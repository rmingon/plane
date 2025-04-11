// HMC5883L.h
#ifndef HMC5883L_H
#define HMC5883L_H

#include <Arduino.h>

#define HMC5883L_ADDRESS 0x1E

struct MagnetometerCalibration {
  int16_t xOffset;
  int16_t yOffset;
};

void initHMC5883L();
MagnetometerCalibration calibrateHMC5883L(unsigned long calibrationTime = 10000);
void readCalibratedHMC5883L(const MagnetometerCalibration& calib, int16_t &x, int16_t &y, int16_t &z);

#endif // HMC5883L_H