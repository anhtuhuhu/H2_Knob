#include <Arduino.h>
#include <Wire.h>
#include "ADS1X15.h"

#define I2C_SDA_PIN GPIO_NUM_18
#define I2C_SCL_PIN GPIO_NUM_17

ADS1115 ADS(0x48);
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

ads_data_t _ads_data;

void ads_setting(void) {
  ADS.begin();
  ADS.setGain(0);      //  6.144 volt
  ADS.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
  ADS.setMode(0);      //  continuous mode
  ADS.readADC(0);      //  first read to trigger
}

void test_H2(void) {
  ADS.readADC(H2_SENSOR_1_ADS_CHANNEL);
  _ads_data.h2_data_1 = ADS.getValue();
  Serial.println("H2 sensor 1 value: " + String(_ads_data.h2_data_1));

  ADS.readADC(H2_SENSOR_2_ADS_CHANNEL);
  _ads_data.h2_data_2 = ADS.getValue();
  Serial.println("H2 sensor 2 value: " + String(_ads_data.h2_data_2));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Khởi tạo I2C
  ads_setting();
}

void loop() {
  test_H2();
  delay(1000);
}
