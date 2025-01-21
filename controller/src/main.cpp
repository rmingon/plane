#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;

#include <WiiChuck.h>

#include "config.h"
#include <ArduinoJson.h>

char packetBuffer[2000];

int led = 12;

Accessory nunchuck;

bool buttonC, buttonZ = false;
int joyX, joyY, rollAngle, pitchAngle, accX, accY, accZ = 0;

String uniq = "";
void getMac() {
  byte mac[6];
  WiFi.macAddress(mac);
  uniq = String(mac[0],HEX) +String(mac[1],HEX) +String(mac[2],HEX) +String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX);
}

void sendDataToServer(String type, JsonDocument data) {
  JsonDocument doc;
  doc["uniq"] = uniq;
  doc["type"] = type;
  doc["data"] = data;
  udp.beginPacket(SERVER_HOST_NAME, UDP_PORT);
  serializeJson(doc, udp);
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  nunchuck.begin();
	if (nunchuck.type == Unknown) {
		nunchuck.type = NUNCHUCK;
	}
  getMac();

  WiFi.begin(SSID, PASSWORD);
  for(int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
      Serial.print(".");
      delay(1000);
  }

  udp.begin(UDP_PORT);
  JsonDocument data;
  sendDataToServer("controller", data);
  digitalWrite(led, HIGH);


  nunchuck.readData();
  joyX = nunchuck.values[0];
  joyY = nunchuck.values[1];
  rollAngle = nunchuck.values[2];
  pitchAngle = nunchuck.values[3];
  accX = nunchuck.values[4];
  accY = nunchuck.values[5];
  accZ = nunchuck.values[6];
}

void loop() {
  nunchuck.readData();
  JsonDocument motion;
  
  if(nunchuck.values[0] != joyX) {
    motion["joyX"] = nunchuck.values[0] - 127;
    joyX = nunchuck.values[0];
  }
  
  if(nunchuck.values[1] != joyY) {
    motion["joyY"] = nunchuck.values[1] - 127;
    joyY = nunchuck.values[1];
  }
  
  if(nunchuck.values[2] != rollAngle) {
    motion["rollAngle"] = nunchuck.values[2] - 127;
    rollAngle = nunchuck.values[2];
  }
  
  if(nunchuck.values[3] != pitchAngle) {
    motion["pitchAngle"] = nunchuck.values[3] - 127;
    pitchAngle = nunchuck.values[3];
  }

  if(nunchuck.values[4] != accX) {
    motion["accX"] = nunchuck.values[4] - 127;
    accX = nunchuck.values[4];
  }
  
  if(nunchuck.values[5] != accY) {
    motion["accY"] = nunchuck.values[5] - 127;
    accY = nunchuck.values[5];
  }
  
  if(nunchuck.values[6] != accZ) {
    motion["accZ"] = nunchuck.values[6] - 127;
    accZ = nunchuck.values[6];
  }

  if (nunchuck.values[10] > 20 != buttonZ) {
    motion["buttonZ"] = nunchuck.values[10] > 20 ? true : false;
    buttonZ = nunchuck.values[10] > 20 ? true : false;
  }
  if (nunchuck.values[11] > 20 != buttonC) {
    motion["buttonC"] = nunchuck.values[11] > 20 ? true : false;
    buttonC = nunchuck.values[11] > 20 ? true : false;
  }

  if (motion.size() > 0) {
    sendDataToServer("motion", motion);
  }

  Serial.print("JoyX = ");Serial.println(nunchuck.values[0]);//Joystick axe X
  Serial.print("JoyY = ");Serial.println(nunchuck.values[1]);//Joystick axe Y
  Serial.print("RollAngle = ");Serial.println(nunchuck.values[2]);//Angle de roulis
  Serial.print("PitchAngle = ");Serial.println(nunchuck.values[3]);//Angle d'inclinaison
  Serial.print("AccX = ");Serial.println(nunchuck.values[4]);//Acceleration axe X
  Serial.print("AccY = ");Serial.println(nunchuck.values[5]);//Acceleration axe Y
  Serial.print("AccZ = ");Serial.println(nunchuck.values[6]);//Acceleration axe Z
  Serial.print("BoutonZ = ");Serial.println(nunchuck.values[10] > 20 ? true : false);//Bouton Z
  Serial.print("BoutonC = ");Serial.println(nunchuck.values[11] > 20 ? true : false);//Bouton C
}