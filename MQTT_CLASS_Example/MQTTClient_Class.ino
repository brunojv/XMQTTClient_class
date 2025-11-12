#include "MQTTClient.h"

const char* ssid = "YourSSID";
const char* password = "YourPassword";
const char* mqttServer = "broker.hivemq.com";
const uint16_t mqttPort = 1883;
const char* clientId = "ESP32Client";
const char* willTopic = "status/ESP32Client";
const char* willMessage = "offline";

MQTTClient mqtt(mqttServer, mqttPort, clientId, willTopic, willMessage);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  mqtt.begin(ssid, password);
  mqtt.setCallback(callback);
  mqtt.subscribe("test/topic");
  mqtt.publish("status/ESP32Client", "online");
}

void loop() {
  mqtt.loop();

  // Example: publish every 10 seconds
  static unsigned long lastPublish = 0;
  if (millis() - lastPublish > 10000) {
    mqtt.publish("test/topic", "Hello from ESP32!");
    lastPublish = millis();
  }
}
