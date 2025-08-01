#include "WiFiType.h"
#include "WiFi.h"
#include "freertos/projdefs.h"
#include "MQTT.h"
#include <PubSubClient.h>
#include "WiFiClientSecure.h"
#include "WIFI_Class.h"

// ===== MQTT Broker =====
const char* mqtt_broker;
int mqtt_port;
const char* mqtt_sub_topic;  // Use to receive message from App/MQTT broker
const char* mqtt_pub_topic;  // Use to send message to App/MQTT broker

WiFiClientSecure secureClient;
PubSubClient client(secureClient);

MQTTMessageHandler MQTT::userMessageHandler = nullptr;

TaskHandle_t MQTT::checkMQTTConnection = NULL;

void MQTT::MQTT_init(const char* broker, const int port, const char* username, const char* password) {
  mqtt_broker = broker;
  mqtt_port = port;
  secureClient.setInsecure();
  client.setServer(mqtt_broker, mqtt_port);
  // client.setCredentials(username, password);
  if (!client.connected()) reconnectMQTT();
  client.setCallback(mqttCallback);

  xTaskCreate(MQTT::check_mqtt_connection, "Check MQTT connection", 4096, NULL, 3, &checkMQTTConnection);
}

void MQTT::check_mqtt_connection(void *param) {
  for (;;) {
    if (!client.connected() && WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconnecting MQTT...");
      reconnectMQTT();
    }
    client.loop();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void MQTT::reconnectMQTT(void) {
  String clientId = "esp32-client-" + String(random(0xffff), HEX);
  int entry = 0;
  while (!client.connected() && entry < 1) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(". Going to try again");
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    entry++;
  }
}

void MQTT::mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  char body[512];
  memcpy(body, payload, length);
  body[length] = '\0';
  Serial.print("Message: ");
  Serial.println(body);
  Serial.println("-----------------------");

  StaticJsonDocument<512> jsonDocument;
  DeserializationError error = deserializeJson(jsonDocument, body);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (userMessageHandler) {  // Gọi callback xử lý nội dung
    userMessageHandler(jsonDocument);
  }
}

void MQTT::setMessageHandler(MQTTMessageHandler handler) {
  userMessageHandler = handler;
}

void MQTT::publishMessage(const char* publish_topic, const char* message) {
  client.publish(publish_topic, message);
}

void MQTT::subscribeTopic(const char* subscribe_topic) {
  client.subscribe(subscribe_topic);
}