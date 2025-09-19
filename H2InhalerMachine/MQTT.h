#ifndef _MQTT_H_
#define _MQTT_H_

#include <Arduino.h>
#include <ArduinoJson.h>
#include <H2InhallerMachine.h>
#include "OTA_MQTT.h"

typedef void (*MQTTMessageHandler)(const JsonDocument &doc);
extern OTA_MQTT _ota;


class MQTT {
private:
  static void reconnectMQTT();
  static void mqttCallback(char *topic, byte *payload, unsigned int length);
  static void handleOTAUpdate(String payload);
  static void publishOTAStatus(void);
  static void handleParameterUpdate(String payload, unsigned int length);
  static TaskHandle_t checkMQTTConnection;
  
  static void check_mqtt_connection(void *param);
  // Con trỏ hàm do người dùng gán để xử lý nội dung JSON
  static MQTTMessageHandler userMessageHandler;

public:
  static void MQTT_init(const char* broker, const int port, const char* username, const char* password);
  // Gán callback xử lý nội dung JSON
  static void setMessageHandler(MQTTMessageHandler handler);
  void publishMessage(const char* publish_topic, const char* message);
  void subscribeTopic(const char* subscribe_topic);
};
extern MQTT _mqtt;
#endif //_MQTT_H_