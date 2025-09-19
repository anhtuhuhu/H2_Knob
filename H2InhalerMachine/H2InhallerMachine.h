#include "WString.h"
#include <stdint.h>
#ifndef _H2_INHALLER_MACHINE_H
#define _H2_INHALLER_MACHINE_H

#define VERSION "v3.9.1"

#define UART2_RX_PIN GPIO_NUM_1
#define UART2_TX_PIN GPIO_NUM_2

#define TMP36_SHUTDOWN_PIN GPIO_NUM_4
#define TEMP_HUMI_PIN GPIO_NUM_8

#define AIR_PUMP_PIN GPIO_NUM_5
#define WATER_PUMP_PIN GPIO_NUM_45

#define LIMIT_SW_PIN GPIO_NUM_6
#define FLOW_PIN GPIO_NUM_9

#define LED_G_PIN GPIO_NUM_10
#define LED_R_PIN GPIO_NUM_11

#define EN_LOAD_PUMP_PIN GPIO_NUM_12
#define EN_LOAD_FAN_PIN GPIO_NUM_16
#define ENABLE_FAN_PIN GPIO_NUM_21
#define ENB_POWER_PIN GPIO_NUM_46

#define LATCH_LOAD_FAN_PIN GPIO_NUM_13
#define LATCH_LOAD_PUMP_PIN GPIO_NUM_7

#define LOAD_FAN_PIN GPIO_NUM_14
#define LOAD_PUMP_PIN GPIO_NUM_42

#define H2_LEAK_PIN GPIO_NUM_3
#define WATER_POOR_PIN GPIO_NUM_15

#define LEVEL_FLOAT1_PIN GPIO_NUM_47
#define LEVEL_FLOAT2_PIN GPIO_NUM_48

#define I2C_SDA_PIN GPIO_NUM_18
#define I2C_SCL_PIN GPIO_NUM_17


#define DEBOUNCE_TIME_MS 10

#define SENSOR_READ_TIME 1000  // 1s

#define MQTT_TOPIC_MAX_LEN 64

#define VT_THRESHOLD 0.4
#define TPL0401B_ADDR 0x3E  // Địa chỉ I2C của TPL0401B
#define CMD_WIPER 0x00      // Thanh ghi Wiper Register

#define TEMPERATURE_UPPER_THRESHOLD 50
#define TEMPERATURE_LOWER_THRESHOLD 15

#define VREF 5.0
#define MAX_STEPS 128

// #define DEBUG

// #define NO_SENSOR

// ADS channels enumeration
typedef enum {
  TDS_ADS_CHANNEL = 1,
  H2_SENSOR_1_ADS_CHANNEL,
  H2_SENSOR_2_ADS_CHANNEL
} ads_channel_t;

// Data structure for sensor readings
typedef struct
{
  uint32_t tds_data;
  uint32_t h2_data_1;
  uint32_t h2_data_2;
} ads_data_t;

typedef enum {
  HEPA_FILTER_MISSING = 0,
  POOR_WATER_QUALITY,
  H2_LEAK_DETECTED,
  WATER_LEVEL_LOW,
  TEMPERATURE_TOO_HIGH,
  TEMPERATURE_TOO_LOW
} ErrorCode;

typedef enum {
  NORMAL_TEMPERATURE = 0,
  HIGH_TEMPERATURE,
  LOW_TEMPERATURE
} Temperature_return_val;

typedef struct {
  uint8_t hepaFilterMissing:1;
  uint8_t waterLevelLow:1;
  uint8_t temperatureHigh:1;
  uint8_t temperatureLow:1;
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
  OTA_UPDATE,
  OTA_STATUS,
  TOPIC_COUNT
} mqtt_topic_id_t;

typedef struct {
  const char *service; // "parameter", "error"
  const char *action;  // "update", "current", etc.
  uint8_t direction;   // 0: cloud->device, 1: device->cloud
} mqtt_topic_info_t;

const mqtt_topic_info_t mqtt_topics[TOPIC_COUNT] = {
    [PARAMETER_UPDATE] = {"parameter", "update", 1},
    [PARAMETER_CURRENT] = {"parameter", "current", 0},
    [PARAMETER_REQUEST] = {"parameter", "request", 1},

    [ERROR_UPDATE] = {"error", "update", 1},
    [ERROR_CLEAR] = {"error", "clear", 1},

    [OTA_UPDATE] = {"ota", "update", 0},
    [OTA_STATUS] = {"ota", "current", 1},
};

#endif