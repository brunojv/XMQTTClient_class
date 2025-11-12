#ifndef XMQTTCLIENT_H
#define XMQTTCLIENT_H

#include <WiFi.h>
#include <PubSubClient.h>

class XMQTTClient {
public:
  XMQTTClient(const char* server, uint16_t port, const char* clientId,const char* willTopic, const char* willMessage, uint8_t willQoS = 1, bool willRetain = true);
  void begin(const char* ssid, const char* password);
  void loop();
  void setCallback(MQTT_CALLBACK_SIGNATURE);
  void subscribe(const char* topic);
  void publish(const char* topic, const char* payload);
  bool isConnected();
  bool connect();
  bool wifiConnected(); 


private:
  WiFiClient wifiClient;
  PubSubClient mqttClient;
  const char* mqttServer;
  uint16_t mqttPort;
  const char* mqttClientId;
  const char* willTopic;
  const char* willMessage;
  uint8_t willQoS;
  bool willRetain;

  void reconnect();
};

#endif
