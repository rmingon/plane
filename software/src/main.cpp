#include <Arduino.h>
#include "Wire.h"

#include <SPI.h>
#include <LoRa.h>
SPIClass LoRaSPI;

#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

#include <MPU6050_light.h>
MPU6050 mpu(Wire);
long timer = 0;

#include <ESP32Servo.h>
Servo left;
Servo right;
Servo backLeft;
Servo backRight;

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

void onReceive(int packetSize) {
  // received a packet
  Serial.print("Received packet '");

  // read packet
  for (int i = 0; i < packetSize; i++) {
    Serial.print((char)LoRa.read());
  }

  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
}

void roll(unsigned int);

void yaw(unsigned int);

void setup(){
  Wire.begin();    
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
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

  for(unsigned int i = 0; i < 60; i++) {
    yaw(i);
    roll(i);
    delay(50);
  }
  for(unsigned int i = 0; i < 60; i++) {
    yaw(-i);
    roll(-i);
    delay(50);
  }

  yaw(0);
  roll(0);
  
  LoRaSPI.begin(14, 12, 13);
  LoRa.setSPI(LoRaSPI);
  LoRa.setPins(15, 16, 17);
  if (!LoRa.begin(868E6)) {
      Serial.println("LoRa init failed. Check your connections.");
      while (true);                       // if failed, do nothing
  }
  if (!bmp.begin()) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
      while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
              Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
              Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
              Adafruit_BMP280::FILTER_X16,      /* Filtering. */
              Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

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
 
void loop(){

  int packetSize = udp.parsePacket();
  Serial.print(" Received packet from : "); Serial.println(udp.remoteIP());
  Serial.print(" Size : "); Serial.println(packetSize);
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len - 1] = 0;
    Serial.printf("Data : %s\n", packetBuffer);
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.printf("UDP packet was received OK\r\n");
    udp.endPacket();
  }
  Serial.println("\n");
  delay(500);
  Serial.print("[Server Connected] ");
  Serial.println (WiFi.localIP());

  mpu.update();

  if(millis() - timer > 50) { // print data every second
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAccAngleY());
    roll(int(trunc(mpu.getAccAngleY()*2)));
    Serial.print("\troll: ");Serial.println(int(trunc(mpu.getAccAngleY()*2)));

    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();

    if (bmp.takeForcedMeasurement()) {
      // can now print out the new measurements
      Serial.print(F("Temperature = "));
      Serial.print(bmp.readTemperature());
      Serial.println(" *C");

      Serial.print(F("Pressure = "));
      Serial.print(bmp.readPressure());
      Serial.println(" Pa");

      Serial.print(F("Approx altitude = "));
      Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
      Serial.println(" m");

      Serial.println();
    } else {
      Serial.println("Forced measurement failed!");
    }
  }


}