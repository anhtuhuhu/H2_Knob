#include "WString.h"
#include <stdint.h>
#ifndef _H2_INHALLER_MACHINE_H
#define _H2_INHALLER_MACHINE_H

#define VERSION "v3.9.0"

#define UART2_RX_PIN GPIO_NUM_44
#define UART2_TX_PIN GPIO_NUM_43

#define WATER_PUMP_PIN GPIO_NUM_4
#define AIR_PUMP_PIN GPIO_NUM_5

#define LIMIT_SW_PIN GPIO_NUM_6
#define WATER_LEAK_PIN GPIO_NUM_15
#define LEVEL_FLOAT_PIN GPIO_NUM_7

#define I2C_SDA_PIN GPIO_NUM_17
#define I2C_SCL_PIN GPIO_NUM_18
#define DEBOUNCE_TIME_MS 10

#define SENSOR_READ_TIME 1000  // 1s

#define MQTT_TOPIC_MAX_LEN 64

// #define DEBUG

#define NO_SENSOR





#endif