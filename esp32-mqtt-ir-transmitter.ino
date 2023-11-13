#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include "constants.h"
#include "secrets.h"


IRsend irsend(irLedPin);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Adafruit_AHTX0 aht; // Temp/humid sensor
Adafruit_BMP280 bmp; // barometric pressure sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X(); // Time-of-flight sensor (ir laser)


void callback(char* topic, byte* payload, unsigned int length) {
  // announce message in terminal
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // transmit topic code
  String receivedTopic = String(topic);
  const int numTopicIRCodes = sizeof(topicToIRCode) / sizeof(topicToIRCode[0]);
  for (int i = 0; i < numTopicIRCodes; i++) {
    if (receivedTopic.equals(topicToIRCode[i].topic)) {
      irsend.sendNEC(topicToIRCode[i].irCode);
      Serial.print(topicToIRCode[i].topic);
      Serial.println(" command sent");
      return;
    }
  }

  Serial.println("Unknown topic, no command sent");
}
void subscribeToAllTopics() {
  const int numTopics = sizeof(subscribeTopics) / sizeof(subscribeTopics[0]);
  for (int i = 0; i < numTopics; i++) {
    mqttClient.subscribe(subscribeTopics[i]);
  }
}
void connectMqtt() {
  Serial.print("Attempting MQTT connection...");
  String clientId = MQTT_CLIENT_ID + WiFi.macAddress();

  // connect, setting a LastWillandTestament that lets the network know this device is offline
  if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword, LWT, 0, true, "0")) {
    Serial.println("connected MQTT");
    // let the network know this device is online
    mqttClient.publish(LWT,"1");
    subscribeToAllTopics();
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
  }
}

void ipHandler(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WiFi connected. IP address: ");
  Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
}
void connectedHandler(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print("CONNECTED to SSID: ");
  Serial.println(reinterpret_cast<char*>(info.wifi_sta_connected.ssid));
}
void disconnectHandler(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Lost connection.  Reconnecting");
}
void connectWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    Serial.print("Connected to Wifi?");
    Serial.println(WiFi.status());
    WiFi.onEvent(ipHandler, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(connectedHandler, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent(disconnectHandler, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  }
}
void tofSensorLoop() {
  if (lox.isRangeComplete()) {
    uint16_t range = lox.readRange();
    bool rangeIsClose = range < 60;
    // set to rangeIsClose so that the PWR command is never sent at startup
    static bool lastRangeWasClose = rangeIsClose;
    // if state has changed, send PWR command
    if (rangeIsClose != lastRangeWasClose) {
      lastRangeWasClose = rangeIsClose;
      irsend.sendNEC(PWR);
      Serial.print("Range is close: ");
      Serial.print(rangeIsClose);
      Serial.println(" | PWR command sent");
    }
    Serial.print("Range: ");
    Serial.println(range);
  }
}

void bmp280Loop() {
  static float prevPressure;
  float pressure = bmp.readPressure() / 100.0F;
  if (abs(pressure - prevPressure) > .1) {
    prevPressure = pressure;
    char pressureStr[10];
    dtostrf(pressure, 4, 1, pressureStr);
    mqttClient.publish(PRESSURE_TOPIC, pressureStr);
    Serial.print("Pressure: ");
    Serial.println(pressure);
  }
}
void aht20Loop() {
  static float prevTempC;
  static float prevHumidity;
  sensors_event_t humidity, tempC;
  aht.getEvent(&humidity, &tempC);
  if (abs(tempC.temperature - prevTempC) > .1) {
    float tempF = tempC.temperature * 1.8 + 32;
    prevTempC = tempC.temperature;
    char tempFStr[10];
    dtostrf(tempF, 4, 1, tempFStr);
    mqttClient.publish(TEMPERATURE_TOPIC, tempFStr);
    Serial.print("Temperature F: ");
    Serial.println(tempFStr);
  }
  if (abs(humidity.relative_humidity - prevHumidity) > .1) {
    prevHumidity = humidity.relative_humidity;
    char humidityStr[10];
    dtostrf(humidity.relative_humidity, 4, 1, humidityStr);
    mqttClient.publish(HUMIDITY_TOPIC, humidityStr);
    Serial.print("Humidity: ");
    Serial.println(humidityStr);
  }
}


void setup() {
  Serial.begin(115200);
  while (! Serial) {
    delay(10);
  }
  irsend.begin();

  Wire.begin(21,22);
  unsigned status;
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor. Check wiring!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BMp 280.\n");
    Serial.print("        ID of 0x61 represents a BMp 680.\n");
  } else {
    Serial.println("BMP280 found.");
  }

  if (!aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
  } else {
    Serial.println("AHT10 or AHT20 found");
  }

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X rangefinder"));
  } else {
    // 1000 ms between readings
    lox.startRangeContinuous(1000);
    Serial.println("Found VL53L0X rangefinder");
  }

  connectWifi();

  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);
  connectMqtt();

}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  if (!mqttClient.connected()) {
    connectMqtt();
  } else {
    mqttClient.loop();
  }
  tofSensorLoop();
  bmp280Loop();
  aht20Loop();
  delay(500);
}
