// #include <stdint.h>
#ifndef _MAIN_h
#define _MAIN_h


#include <Arduino.h>
#include "ADS1X15.h"
#include "driver/ledc.h"
#include "H2InhallerMachine.h"
// #include "OTA.h"
#include "OTA_MQTT.h"
#include "WIFI_Class.h"
#include "MQTT.h"


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
uint8_t check_temperature(void);
int voltageToStep(float Vout, float Vref, int maxSteps);
void setWiper(uint8_t value);
float calculateVoltage(int h2_percent);

void handleSerialCommands(void);
void handleKnobCommand(void);
void connectedWiFi_cb(void);

void IRAM_ATTR limitsw_handleInterrupt();
void IRAM_ATTR level_float_1_ISR();
void IRAM_ATTR level_float_2_ISR();
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