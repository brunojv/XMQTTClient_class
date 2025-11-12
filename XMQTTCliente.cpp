#include "XMQTTCliente.h"

XMQTTCliente::XMQTTCliente(const char* server, uint16_t port, const char* clientId,
                       const char* willTopic, const char* willMessage,
                       uint8_t willQoS, bool willRetain) : mqttClient(wifiClient), mqttServer(server), mqttPort(port), mqttClientId(clientId),
    willTopic(willTopic), willMessage(willMessage), willQoS(willQoS), willRetain(willRetain) {
  mqttClient.setServer(mqttServer, mqttPort);
}

void XMQTTCliente::begin(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

bool XMQTTCliente::wifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

void XMQTTCliente::loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
}

void XMQTTCliente::setCallback(MQTT_CALLBACK_SIGNATURE) {
  mqttClient.setCallback(callback);
}

void XMQTTCliente::subscribe(const char* topic) {
  mqttClient.subscribe(topic);
}

void XMQTTCliente::publish(const char* topic, const char* payload) {
  mqttClient.publish(topic, payload);
}

bool XMQTTCliente::isConnected() {
  return mqttClient.connected();
}

bool XMQTTCliente::connect() {
  return mqttClient.connect(mqttClientId, willTopic, willQoS, willRetain, willMessage);
}

void XMQTTCliente::reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(mqttClientId, willTopic, willQoS, willRetain, willMessage)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
