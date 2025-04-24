#include <Arduino.h>
#define LED 23

#include "Wire.h"
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

#include <SPI.h>
#include <LoRa.h>
SPIClass LoRaSPI;

#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

#include <HMC5883L.h>
MagnetometerCalibration calib;

int oESC = 100;

#include <MPU6050_light.h>
MPU6050 mpu(Wire);
long timer = 0;

struct Data {
  float lat;
  float lon;
  float kmph;
  float deg;
  double x;
  double y;
  float alt;
  int16_t magX;
  int16_t magY;
  int16_t magZ;
  int speed;
  byte cmd;
};
Data dataToSend;

struct DataReceive {
  byte command;
  int roll;
  int pitch;
  int throttle;
  double targetAltitude;
};
DataReceive dataReceive;

double currentAltitude= 0;
double currentRoll = 0;
double throttleOutput;
double rollOutput;
double targetRoll = 0;

double currentPitch = 0;
double targetPitch = 0;
double outputPitch = 0;

#include <PID_v1.h>
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
double rollKp = 1.5, rollKi = 0.3, rollKd = 0.8;
double pitchKp = 1.5, pitchKi = 0.3, pitchKd = 0.8;
PID altitudePID(&currentAltitude, &throttleOutput, &dataReceive.targetAltitude, Kp, Ki, Kd, DIRECT);
PID rollPID(&currentRoll, &rollOutput, &targetRoll, rollKp, rollKi, rollKd, DIRECT);
PID pitchPID(&currentPitch, &outputPitch, &targetPitch, pitchKp, pitchKi, pitchKd, DIRECT);

struct HmcData {
  int16_t x;
  int16_t y;
  int16_t z;
};

#include <ESP32Servo.h>
Servo left;
Servo right;
Servo esc;

void loraSend(byte*, size_t);
void displayInfo();
void autoPilot();
void calibEsc();
void testEsc();
void roll(double);
void yaw(int);

void onReceive(int packetSize) {
  if (packetSize == sizeof(dataReceive)) {
    digitalWrite(LED, !digitalRead(LED));

    for (int i = 0; i < sizeof(dataReceive); i++) {
      *((uint8_t*)&dataReceive + i) = LoRa.read();
    }
    const int cPosition = 0;
    const int zPosition = 1;

    bool cIsPressed = (dataReceive.command & (1 << cPosition)) != 0;
    bool zIsPressed = (dataReceive.command & (1 << zPosition)) != 0;

    // Serial.println("pitch: " + String(dataReceive.pitch));
    // Serial.println("roll: " + String(dataReceive.roll));
    targetRoll = dataReceive.roll;
  }
}

volatile bool sendEnd = false;

void calibEsc() {
    Serial.println("Starting ESC calibration...");
    
    // Step 1: Send max throttle
    esc.writeMicroseconds(2000);
    Serial.println("Sending MAX throttle...");
    delay(5000);  // Wait for ESC beep sequence

    // Step 2: Send min throttle
    esc.writeMicroseconds(1000);
    Serial.println("Sending MIN throttle...");
    delay(5000);  // Wait for ESC confirmation

    Serial.println("ESC Calibration complete!");
}

void testEsc () {
    esc.write(30);  // Slow speed
    delay(3000);
    
    esc.write(60);  // Medium speed
    delay(3000);
    
    esc.write(80);  // High speed
    delay(3000);
    
    esc.write(100);  // High speed
    delay(3000);
    
    esc.write(60);  // High speed
    delay(3000);
    
    esc.write(20);  // High speed
    delay(3000);
    
    esc.write(0);  // Stop
    delay(3000);
}

int leftServoOffset = 0;
int rightServoOffset = 0;

void setup() {
  setCpuFrequencyMhz(240);
  pinMode(LED, OUTPUT);
  Wire.begin();
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 4, 5);

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  esc.setPeriodHertz(50);
  esc.attach(33, 1000, 2000);

  calibEsc();

  left.setPeriodHertz(50);
  left.attach(26, 1000, 2000);
  right.setPeriodHertz(50);
  right.attach(25, 1000, 2000);

  left.write(40);
  
  right.write(80);
  
  LoRaSPI.begin(14, 12, 13);
  LoRa.setSPIFrequency(12000000);
  LoRa.setSPI(LoRaSPI);
  LoRa.setPins(15, 16, 17);
  if (!LoRa.begin(868E6)) {
      Serial.println("LoRa init failed. Check your connections.");
      while (true);
  }
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(8);
  LoRa.setSpreadingFactor(7);
  LoRa.setTxPower(20);
  LoRa.setSyncWord(0xF3);
  LoRa.enableCrc();

  if (!bmp.begin()) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
      while (true);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
  Adafruit_BMP280::SAMPLING_X2,  
  Adafruit_BMP280::SAMPLING_X16,   
  Adafruit_BMP280::FILTER_X16,     
  Adafruit_BMP280::STANDBY_MS_500); 

  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  mpu.calcOffsets(true,true); // gyro and accelero

  currentAltitude = bmp.readAltitude(1016);
  dataReceive.targetAltitude = currentAltitude;

  altitudePID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-50, 50);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-50, 50);

  initHMC5883L();
  calib = calibrateHMC5883L();
  Serial.print("GO");
}

void roll(double roll) {
  left.write(40 - (roll));
  right.write(80 - (roll));
}

void yaw(unsigned int yaw) {
  // backRight.write(100 - yaw);
}

void loop() {
  onReceive(LoRa.parsePacket());

  altitudePID.Compute();
  rollPID.Compute();
  pitchPID.Compute();

  roll(rollOutput);
  esc.write(throttleOutput);

  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read())) {
      dataToSend.lat = gps.location.isValid() ? gps.location.lat() : 0.0;
      dataToSend.lon = gps.location.isValid() ? gps.location.lng() : 0.0;
      dataToSend.kmph = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
      dataToSend.deg = gps.course.isValid() ? gps.course.deg() : 0.0;
    }

  mpu.update();
  dataToSend.x = mpu.getAngleX();
  dataToSend.y = mpu.getAngleY();
  currentRoll = mpu.getAngleY();

  if (bmp.takeForcedMeasurement()) {
    dataToSend.alt = bmp.readAltitude(1016);
    currentAltitude = dataToSend.alt;
  }
    
  readCalibratedHMC5883L(calib, dataToSend.magX, dataToSend.magY, dataToSend.magZ);
  if(millis() - timer > 500) {
    dataToSend.speed = oESC;

    loraSend((uint8_t*)&dataToSend, sizeof(dataToSend));
    timer = millis();
  }
}

void autoPilot() {
  mpu.update();
  roll(int(trunc(mpu.getAccAngleY()*2)));
}

void loraSend(byte* data, size_t length) {
  LoRa.beginPacket();
  LoRa.write(data, length);
  LoRa.endPacket();
  LoRa.receive();
}