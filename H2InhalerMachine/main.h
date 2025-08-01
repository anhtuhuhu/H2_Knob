// #include <stdint.h>
#ifndef _MAIN_h
#define _MAIN_h


#include <Arduino.h>
#include "ADS1X15.h"
#include "driver/ledc.h"
#include "H2InhallerMachine.h"
#include "OTA.h"
#include "WIFI_Class.h"
#include "MQTT.h"

// ADS channels enumeration
typedef enum {
  PRESSURE_ADS_CHANNEL,
  TDS_ADS_CHANNEL,
  H2_SENSOR_ADS_CHANNEL,
  POTEN_ADS_CHANNEL
} ads_channel_t;

// Data structure for sensor readings
typedef struct
{
  uint32_t pressure_data;
  uint32_t tds_data;
  uint32_t h2_data;
  uint32_t poten_data;
} ads_data_t;

typedef enum {
  HEPA_FILTER_MISSING = 0,
  WATER_LEAK_DETECTED,
  FLOAT_DETECTED,
  POOR_WATER_QUALITY,
  H2_LEAK_DETECTED
} ErrorCode;

typedef struct {
  uint8_t hepaFilterMissing:1;
  uint8_t waterLeakDetected:1;
  uint8_t floatDetected:1;
  uint8_t poorWaterQuality:1;
  uint8_t h2LeakDetected:1;
} Error_status_t;

typedef struct {
  uint8_t state;
  char timer[10];
  float percent;
  float typeMachine;
} Knob_data_t;

typedef enum {
  PARAMETER_UPDATE,
  PARAMETER_CURRENT,
  PARAMETER_REQUEST,
  ERROR_UPDATE,
  ERROR_CLEAR,
  TOPIC_COUNT
} mqtt_topic_id_t;

typedef struct {
  const char *service; // "parameter", "error"
  const char *action;  // "update", "current", etc.
  uint8_t direction;   // 0: cloud->device, 1: device->cloud
} mqtt_topic_info_t;

void Setup(void);
void Main(void);

void send_error_to_Knob(ErrorCode error);
void ads_setting(void);
#ifndef COCAMBIEN
bool readH2Sensor(void);
bool readTDS(void);
#else
bool readH2Sensor(int h2_val);
bool readTDS(int tds_val);
#endif
void handleSerialCommands(void);
void handleKnobCommand(void);
void connectedWiFi_cb(void);

void IRAM_ATTR limitsw_handleInterrupt();
void IRAM_ATTR levelfloat_handleInterrupt();
void IRAM_ATTR waterleak_handleInterrupt();
void updatePWMVoltage(float voltage);

void Serial_Handle_task(void *param);
void warning_Handle_Task(void *param);
void PumpControlTask(void *param);
void SendMQTTDataToKnobTask(void *param);

void handleMQTTReceivedMessage(const JsonDocument &doc);
void parseKnobData(const char* input);
void mqtt_publish_message(int topic_id, const char *mac, String message);
void mqtt_subscribe(int topic_id, const char *mac_id);
String buildKnobDataJson(const Knob_data_t& data);
String buildErrorJson(const Error_status_t& errorStatus);

#endif