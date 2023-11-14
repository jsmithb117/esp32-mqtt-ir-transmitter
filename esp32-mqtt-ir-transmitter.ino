#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include <assert.h>
#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRtext.h>
#include <IRutils.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include "constants.h"
#include "secrets.h"


IRsend irsend(irLedPin); // IR transmitter
WiFiClient wifiClient; // Wifi
PubSubClient mqttClient(wifiClient); // MQTT
Adafruit_AHTX0 aht; // Temp/humid sensor
Adafruit_BMP280 bmp; // barometric pressure sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X(); // Time-of-flight sensor (ir laser)
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true); // IR receiver

void callback(char* topic, byte* payload, unsigned int length) {
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
  // TODO: Use a map instead of a loop
  for (int i = 0; i < numTopicIRCodes; i++) {
    if (receivedTopic.equals(topicToIRCode[i].topic)) {
      // Pause IR receiver
      irrecv.disableIRIn();
      // Transmit
      irsend.sendNEC(topicToIRCode[i].irCode);
      // Resume IR receiver
      irrecv.enableIRIn();
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
    Serial.print("Connected to Wifi.  Status: ");
    Serial.println(WiFi.status());
    WiFi.onEvent(ipHandler, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(connectedHandler, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent(disconnectHandler, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  }
}
void tofSensorLoop() {
  VL53L0X_RangingMeasurementData_t measurement;
  // Pause IR receiver
  irrecv.disableIRIn();
  // take measurement
  lox.getSingleRangingMeasurement(&measurement);
  // Resume IR receiver
  irrecv.enableIRIn();

  bool rangeIsClose = measurement.RangeMilliMeter < 50;
  // set to rangeIsClose so that the PWR command is never sent at startup
  static bool lastRangeWasClose = rangeIsClose;
  // if state has changed, send PWR command
  if (rangeIsClose != lastRangeWasClose) {
    lastRangeWasClose = rangeIsClose;

    // Pause IR receiver
    irrecv.disableIRIn();
    // Transmit
    irsend.sendNEC(PWR);
    // Resume IR receiver
    irrecv.enableIRIn();

    Serial.print("range: ");
    Serial.println(measurement.RangeMilliMeter);
    Serial.print("Range is close: ");
    Serial.print(rangeIsClose);
    Serial.println(" | PWR command sent");
  }
}

void bmp280Loop(bool force = false) {
  static float prevPressure;
  float pressure = bmp.readPressure() / 100.0F;
  if (abs(pressure - prevPressure) > .1 || force) {
    prevPressure = pressure;
    char pressureStr[10];
    dtostrf(pressure, 4, 1, pressureStr);
    mqttClient.publish(PRESSURE_TOPIC, pressureStr);
    Serial.print("Pressure: ");
    Serial.println(pressure);
  }
}
void aht20Loop(bool force = false) {
  static float prevTempC;
  static float prevHumidity;
  sensors_event_t humidity, tempC;
  aht.getEvent(&humidity, &tempC);
  if (abs(tempC.temperature - prevTempC) > .1 || force) {
    float tempF = tempC.temperature * 1.8 + 32;
    prevTempC = tempC.temperature;
    char tempFStr[10];
    dtostrf(tempF, 4, 1, tempFStr);
    mqttClient.publish(TEMPERATURE_TOPIC, tempFStr);
    Serial.print("Temperature F: ");
    Serial.println(tempFStr);
  }
  if (abs(humidity.relative_humidity - prevHumidity) > .3) {
    prevHumidity = humidity.relative_humidity;
    char humidityStr[10];
    dtostrf(humidity.relative_humidity, 4, 1, humidityStr);
    mqttClient.publish(HUMIDITY_TOPIC, humidityStr);
    Serial.print("Humidity: ");
    Serial.println(humidityStr);
  }
}

void irReceiverLoop() {
  decode_results results;
  if (irrecv.decode(&results)) {
    String humanResult = resultToHumanReadableBasic(&results);
    Serial.print("IR Receiver: ");
    Serial.println(humanResult);
    mqttClient.publish(IR_TOPIC, humanResult.c_str());
  }
}
void wifiLoop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }
  // get RSSI and report
  long rssi = WiFi.RSSI();
  static long prevRssi;
  if (abs(rssi - prevRssi) > 2) {
    prevRssi = rssi;
    mqttClient.publish(RSSI_TOPIC, String(rssi).c_str());
    Serial.print("RSSI: ");
    Serial.println(rssi);
  }
}
void setup() {
  Serial.begin(115200);
  while (! Serial) {
    delay(10);
  }
  // IR Transmitter
  irsend.begin();

  // IR Receiver
  assert(irutils::lowLevelSanityCheck() == 0);
  irrecv.enableIRIn();

  // Sensors
  Wire.begin(21,22);
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor. Check wiring."));
  } else {
    Serial.println("BMP280 found.");
  }

  if (!aht.begin()) {
    Serial.println("Could not find AHT. Check wiring");
  } else {
    Serial.println("AHT10 or AHT20 found");
  }

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X rangefinder"));
  } else {
    Serial.println("Found VL53L0X rangefinder");
  }

  connectWifi();

  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);
  connectMqtt();

  // force sensors to report on startup
  bmp280Loop(true);
  aht20Loop(true);
}

int count = 0;
void loop() {
  wifiLoop();

  if (!mqttClient.connected()) {
    connectMqtt();
  } else {
    mqttClient.loop();
  }
  if (count == 5) {
    tofSensorLoop();
    count = 0;
  }
  count++;
  bmp280Loop();
  aht20Loop();
  irReceiverLoop();
  delay(100);
}
