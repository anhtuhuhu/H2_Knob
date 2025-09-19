#include "WString.h"
#include "WiFi.h"
#include "freertos/projdefs.h"
#include "MQTT.h"
#include <PubSubClient.h>
#include "WiFiClientSecure.h"
#include "WIFI_Class.h"
#include "OTA_MQTT.h"

// ===== MQTT Broker =====
const char *mqtt_broker;
int mqtt_port;
const char *mqtt_sub_topic; // Use to receive message from App/MQTT broker
const char *mqtt_pub_topic; // Use to send message to App/MQTT broker

WiFiClientSecure secureClient;
PubSubClient client(secureClient);

MQTTMessageHandler MQTT::userMessageHandler = nullptr;

TaskHandle_t MQTT::checkMQTTConnection = NULL;
String currentVersion = VERSION;
MQTT _mqtt;
OTA_MQTT _ota;
void MQTT::MQTT_init(const char *broker, const int port, const char *username, const char *password)
{
  mqtt_broker = broker;
  mqtt_port = port;
  secureClient.setInsecure();
  client.setServer(mqtt_broker, mqtt_port);
  client.setKeepAlive(120);
  client.setSocketTimeout(120);
  if (client.setBufferSize(256))
  {
    uint16_t buf_size = client.getBufferSize();
    Serial.println("MQTT MAX SIZE after set: " + String(buf_size));
  }
  // client.setCredentials(username, password);
  if (!client.connected())
    reconnectMQTT();
  client.setCallback(mqttCallback);

  xTaskCreatePinnedToCore(MQTT::check_mqtt_connection, "Check MQTT connection", 8192, NULL, 3, &checkMQTTConnection, 0);
}

void MQTT::check_mqtt_connection(void *param)
{
  for (;;)
  {
    client.loop();
    if (!client.connected() && WiFi.status() == WL_CONNECTED)
    {
      Serial.println("Reconnecting MQTT...");
      reconnectMQTT();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void MQTT::reconnectMQTT(void)
{
  String mac = WiFi.macAddress(); // Ví dụ: "B0:81:84:85:2D:C8"
  
  // Tách 2 byte cuối (sau dấu ':')
  int lastColon = mac.lastIndexOf(':');
  String last2Bytes = mac.substring(lastColon + 1); // "C8"
  
  int secondLastColon = mac.lastIndexOf(':', lastColon - 1);
  String secondLastByte = mac.substring(secondLastColon + 1, lastColon); // "2D"
  
  // Ghép lại thành chuỗi thường
  String clientId = "esp-client-" + secondLastByte + last2Bytes; // "esp-client-2DC8"
  clientId.toLowerCase(); // thành "esp-client-2dc8" nếu bạn muốn toàn chữ thường

  Serial.print("MQTT Client ID: ");
  Serial.println(clientId);
  int entry = 0;
  while (!client.connected() && entry < 1)
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientId.c_str()))
    {
      Serial.println("MQTT connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(". Going to try again");
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    entry++;
  }
}

void MQTT::mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String topicStr = String(topic);
  String payloadStr = "";
  for (unsigned int i = 0; i < length; i++)
  {
    payloadStr += (char)payload[i];
  }

  if (strstr(topic, "/ota/") != NULL)
  {
    Serial.println("Handling OTA update message");
    handleOTAUpdate(payloadStr);
  }
  else
  {
    handleParameterUpdate(payloadStr, length);
  }
}

void MQTT::handleOTAUpdate(String payload)
{
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error)
  {
    Serial.print("OTA JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (!doc.containsKey("version") || !doc.containsKey("url") || !doc.containsKey("target"))
  {
    Serial.println("OTA command missing keys (need version, url, target)");
    return;
  }

  String newVersion = doc["version"];
  String url = doc["url"];
  String targetStr = doc["target"];

  OTA_MQTT::Target_Device target =
      (targetStr == "Knob") ? OTA_MQTT::KNOB : OTA_MQTT::CONTROLLER;

  Serial.printf("[OTA] New Version: %s, Current: %s\n", newVersion.c_str(), currentVersion.c_str());
  Serial.printf("[OTA] Target: %s\n", targetStr.c_str());
  Serial.printf("[OTA] URL: %s\n", url.c_str());

  // So sánh version
  if (newVersion == currentVersion)
  {
    Serial.println("[OTA] Already up-to-date");
    return;
  }

  // Bắt đầu OTA
  if (_ota.getStatus().receivedSize <= 0)
  {
    _ota.reset();
  }
  
  // _ota.reset();
  if (!_ota.handleOTAPacket(url, target))
  {
    Serial.println("[OTA] Failed during OTA process");
    publishOTAStatus(); // báo lỗi
  }
  else
  {
    Serial.println("[OTA] OTA process finished successfully");
    publishOTAStatus(); // báo thành công
  }
}

void MQTT::handleParameterUpdate(String payload, unsigned int length)
{
  Serial.print("Message arrived in topic for parameter update");
  char body[512];
  memcpy(body, (char *)payload.c_str(), length);
  body[length] = '\0';
  Serial.print("Message: ");
  Serial.println(body);
  Serial.println("-----------------------");

  StaticJsonDocument<512> jsonDocument;
  DeserializationError error = deserializeJson(jsonDocument, body);
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (userMessageHandler)
  { // Gọi callback xử lý nội dung
    userMessageHandler(jsonDocument);
  }
}

void MQTT::setMessageHandler(MQTTMessageHandler handler)
{
  userMessageHandler = handler;
}

void MQTT::publishMessage(const char *publish_topic, const char *message)
{
  client.publish(publish_topic, message);
}

void MQTT::subscribeTopic(const char *subscribe_topic)
{
  client.subscribe(subscribe_topic);
}

void MQTT::publishOTAStatus(void)
{
  StaticJsonDocument<256> doc;
  OTA_MQTT::OTA_Status status = _ota.getStatus();

  doc["state"] = status.state == OTA_MQTT::OTA_SENDING ? "SENDING" : status.state == OTA_MQTT::OTA_WAIT_READY ? "WAIT_READY"
                                                                 : status.state == OTA_MQTT::OTA_END          ? "END"
                                                                                                              : "IDLE";
  doc["target"] = status.target == OTA_MQTT::CONTROLLER ? "Controller" : "Knob";
  doc["received_size"] = status.receivedSize;
  doc["total_size"] = status.totalSize;
  if (status.error.length() > 0)
  {
    doc["error"] = status.error;
  }

  String MAC_id = WIFI::getMacAddress();
  String statusJson;
  serializeJson(doc, statusJson);
  char topic[MQTT_TOPIC_MAX_LEN];

  snprintf(topic, sizeof(topic), "device/%s/%s/%s", MAC_id.c_str(), mqtt_topics[OTA_STATUS].service, mqtt_topics[OTA_STATUS].action);
  _mqtt.publishMessage(topic, statusJson.c_str());
}