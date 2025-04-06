#include <Arduino.h>
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

const int PWM_FREQ    = 50;
const int PWM_RES_BITS = 16;
int oESC = 100;

#include <MPU6050_light.h>
MPU6050 mpu(Wire);
long timer = 0;

#include <ESP32Servo.h>
Servo left;
Servo right;
Servo backLeft;
Servo backRight;
Servo esc;

#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;
char packetBuffer[255];
unsigned int localPort = 9999;

const char* ssid     = "plane";
const char* password = "0987654321";

IPAddress local_ip(192,168,0,1);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);

void loraSend(byte*, size_t);

void onReceive(int packetSize) {
  // received a packet
  Serial.print("Received packet '");

  // read packet
  for (int i = 0; i < packetSize; i++) {
    Serial.println(LoRa.read(), BIN);
  }

  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
}

bool buttonC, buttonZ = false;
int joyX, joyY, rollAngle, pitchAngle, accX, accY, accZ = 0;
 
void displayInfo();
void autoPilot();
void handleUDP();
void calibEsc();
void testEsc();
void roll(unsigned int);

void yaw(unsigned int);

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

void setup(){
  Wire.begin();
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 4, 5);

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  delay(3000);

  esc.setPeriodHertz(50);
  esc.attach(33, 1000, 2000);

  calibEsc();

  delay(1000);

  testEsc();

  left.setPeriodHertz(50);
  left.attach(25, 1000, 2000);
  right.setPeriodHertz(50);
  right.attach(26, 1000, 2000);
  backLeft.setPeriodHertz(50);
  backLeft.attach(32, 1000, 2000);
  backRight.setPeriodHertz(50);
  backRight.attach(27, 1000, 2000);
  
  left.write(90);
  right.write(90);
  backLeft.write(90);
  backRight.write(90);

  yaw(0);
  roll(0);
  
  LoRaSPI.begin(14, 12, 13);
  LoRa.setSPI(LoRaSPI);
  LoRa.setPins(15, 16, 17);
  if (!LoRa.begin(868E6)) {
      Serial.println("LoRa init failed. Check your connections.");
      while (true);                       // if failed, do nothing
  }

  // if (!bmp.begin()) {
  //     Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
  //                     "try a different address!"));
  //     while (1) delay(10);
  // }
  // bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
  // Adafruit_BMP280::SAMPLING_X2,  
  // Adafruit_BMP280::SAMPLING_X16,   
  // Adafruit_BMP280::FILTER_X16,     
  // Adafruit_BMP280::STANDBY_MS_500); 

  
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());

  udp.begin(localPort);

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setTxPower(20);
  LoRa.setSyncWord(0xF3);

  LoRa.onReceive(onReceive);
  LoRa.receive();
}

void roll(unsigned int roll) {
  left.write(106 - roll);
  right.write(85 - roll);
}

void yaw(unsigned int yaw) {
  backRight.write(100 - yaw);
}

void handleUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0; // Null-terminate
    }
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, packetBuffer);
    if (!error) {
      JsonObject data = doc["data"];
        if (data.containsKey("joyX"))   joyX   = data["joyX"].as<int>();
        if (data.containsKey("joyY"))   joyY   = data["joyY"].as<int>();
        if (data.containsKey("rollAngle"))   rollAngle   = data["rollAngle"].as<int>();
        if (data.containsKey("pitchAngle"))   pitchAngle   = data["pitchAngle"].as<int>();
        if (data.containsKey("accX"))   accX   = data["accX"].as<int>();
        if (data.containsKey("accY"))   accY   = data["accY"].as<int>();
        if (data.containsKey("accZ"))   accY   = data["accZ"].as<int>();
        if (data.containsKey("buttonC"))   buttonC   = data["buttonC"].as<bool>();
        if (data.containsKey("buttonZ"))   buttonZ   = data["buttonZ"].as<bool>();
        roll(joyX);
        if (buttonC) {
          if (oESC < 180) {
            oESC += 1;
          }
          esc.write(oESC);
        }
        if (buttonZ) {
          if (oESC > 0) {
            oESC -= 1;
          }
          esc.write(oESC);
        }
    } else {
      Serial.println("[UDP] JSON parse failed.");
    }
  }
}
 
void loop(){
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  
  // handleUDP();
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

  if(millis() - timer > 1000) { // print data every second
  
    mpu.update();
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAccAngleY());
    Serial.print("\troll: ");Serial.println(int(trunc(mpu.getAccAngleY()*2)));

    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();

    // if (bmp.takeForcedMeasurement()) {
    //   Serial.print(F("Temperature = "));
    //   Serial.print(bmp.readTemperature());
    //   Serial.println(" *C");

    //   Serial.print(F("Pressure = "));
    //   Serial.print(bmp.readPressure());
    //   Serial.println(" Pa");

    //   Serial.print(F("Approx altitude = "));
    //   Serial.print(bmp.readAltitude(1013.25));
    //   Serial.println(" m");

    //   Serial.println();
    // } else {
    //   Serial.println("Forced measurement failed!");
    // }

  }
}

void autoPilot() {
  mpu.update();
  roll(int(trunc(mpu.getAccAngleY()*2)));
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.println(gps.location.lng(), 6);
  }
  else
  {
    Serial.println(F("INVALID"));
  }
  Serial.print(F("Speed: ")); 
  if (gps.speed.isValid())
  {
    
    Serial.println(gps.speed.kmph());
  }
  else
  {
    Serial.println(F("INVALID"));
  }

  Serial.print(F("  Course: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.course.deg());
    Serial.println(TinyGPSPlus::cardinal(gps.course.deg()));
  }
  else
  {
    Serial.println(F("INVALID"));
  }
}


void loraSend(byte* data, size_t length) {
  LoRa.beginPacket();
  LoRa.write(data, length);
  LoRa.endPacket();
  LoRa.receive();
}