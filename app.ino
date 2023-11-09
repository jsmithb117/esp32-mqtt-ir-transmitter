#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "constants.h"
#include "secrets.h"


IRsend irsend(kIrLed);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

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
  for (int i = 0; i < numTopics; i++) {
    mqttClient.subscribe(subscribeTopics[i]);
  }
}
void connectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = MQTT_CLIENT_ID + WiFi.macAddress();

    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("connected");

      mqttClient.publish("home/outTopic","hello world");

      const int numTopics = sizeof(topics) / sizeof(topics[0]);
      subscribeToAllTopics();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
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
void setup() {
  irsend.begin();
  Serial.begin(115200, SERIAL_8N1);

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
}
