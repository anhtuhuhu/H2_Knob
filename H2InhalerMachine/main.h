#ifndef _MAIN_h
#define _MAIN_h


#include <Arduino.h>
#include "ADS1X15.h"
#include "driver/ledc.h"
#include <Preferences.h>
#include "POSTGET.h"
#include "H2InhallerMachine.h"
#include "OTA.h"

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
void handleConfigWifi(void);

#endif