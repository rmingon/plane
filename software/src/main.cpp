#include <Arduino.h>
#include "Wire.h"

#include <SPI.h>              // include libraries
#include <LoRa.h>
SPIClass LoRaSPI;

#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

#include <ESP32Servo.h>
Servo myservo;

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

void setup(){
    Wire.begin();    
    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
    myservo.setPeriodHertz(50);
    myservo.attach(32, 1000, 2000);

    myservo.write(0);
    delay(1000);
    myservo.write(180);
    
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
    LoRa.onReceive(onReceive);
    LoRa.receive();
} 
 
void loop(){
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
    delay(2000);
  } else {
    Serial.println("Forced measurement failed!");
  }
}