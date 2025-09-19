#include <Arduino.h>
#include <Wire.h>
#include "ADS1X15.h"

#define TMP36_PIN GPIO_NUM_4
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

#define VT_THRESHOLD 0.4
#define TPL0401B_ADDR 0x3E  // Địa chỉ I2C của TPL0401B
#define CMD_WIPER 0x00      // Thanh ghi Wiper Register

#define VREF 5.0
#define MAX_STEPS 128

#define DEBOUNCE_TIME_MS 10

HardwareSerial UART2(1);

#define TEST_TMP36
#define TEST_PUMP
#define TEST_FAN
#define TEST_FLOW_SENSOR
#define TEST_CURRENT_CONTROL
#define TEST_TDS
#define TEST_LIMIT_SW
#define TEST_FLOAT_SENSOR
#define TEST_H2_SENSOR
#define TEST_KNOB

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

volatile bool limitsw_interruptFlag = false;
volatile bool float1_interruptFlag = false;
volatile bool float2_interruptFlag = false;

void test_TMP36(void) {
  int TMP36_RawValue = 0;
  float Voltage_mV;
  float Temperature_C;
  float Temperature_F;

  // Read TMP36_Sensor ADC Pin
  TMP36_RawValue = analogRead(TMP36_PIN);
  Voltage_mV = ((TMP36_RawValue * 5.0) / 1023.0) * 1000;
  // TempC = Voltage(mV) / 10
  Temperature_C = Voltage_mV / 10;
  Temperature_F = (Temperature_C * 1.8) + 32;

  // Print The Readings
  Serial.print("Temperature = ");
  Serial.print(Temperature_C);
  Serial.print(" °C , ");
  Serial.print("Temperature = ");
  Serial.print(Temperature_F);
  Serial.println(" °F");
}

void test_Pumps(void) {
  Serial.println("Pumps ON");
  digitalWrite(AIR_PUMP_PIN, HIGH);
  digitalWrite(WATER_PUMP_PIN, HIGH);
  delay(5000);
  Serial.println("Pumps OFF");
  digitalWrite(AIR_PUMP_PIN, LOW);
  digitalWrite(WATER_PUMP_PIN, LOW);
  delay(5000);
}

void test_Fan(void) {
  Serial.println("Fan ON");
  digitalWrite(ENABLE_FAN_PIN, HIGH);
  delay(5000);
  Serial.println("Fan OFF");
  digitalWrite(ENABLE_FAN_PIN, LOW);
  delay(5000);
}

int readMedianADC(int pin) {
  const int N = 5;
  int values[N];

  // Đọc 5 mẫu liên tiếp
  for (int i = 0; i < N; i++) {
    values[i] = analogRead(pin);
    delayMicroseconds(200);  // nghỉ 0.2ms để ADC ổn định
  }

  // Sắp xếp 5 mẫu nhỏ -> lớn
  for (int i = 0; i < N - 1; i++) {
    for (int j = i + 1; j < N; j++) {
      if (values[j] < values[i]) {
        int tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }

  // Trả về giá trị trung vị (mẫu ở giữa: index 2)
  return values[N / 2];
}

void test_flow_sensor(void) {
  int adcValue = readMedianADC(FLOW_PIN);

  // Chuyển sang điện áp (V), nếu cần
  float voltage = (adcValue / 4095.0) * 3.3;

  Serial.print("ADC median = ");
  Serial.print(adcValue);
  Serial.print("   Voltage = ");
  Serial.println(voltage, 3);

  // So sánh với ngưỡng VT (ví dụ 0.4V)
  if (voltage < VT_THRESHOLD) {
    Serial.println("==> HIGH WATER LEVEL");
  } else {
    Serial.println("==> LOW WATER LEVEL");
  }
}

// Hàm ghi giá trị Wiper
void setWiper(uint8_t value) {
  Wire.beginTransmission(TPL0401B_ADDR);
  Wire.write(CMD_WIPER);     // Thanh ghi Wiper = 0x00
  Wire.write(value & 0x7F);  // 7 bit (0-127)
  Wire.endTransmission();
}

void test_current_control(void) {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    Serial.println("Receive command: " + cmd);
    float recv_value = cmd.toFloat();
    recv_value = round(recv_value * 10) / 10.0;
    if (recv_value >= 0.0 && recv_value <= 4.0) {
      float voltage = 1.0933 * recv_value + 0.4321;
      int step = voltageToStep(voltage, VREF, MAX_STEPS);
      if (step >= 0 && step < 128) {
        setWiper(step);
        Serial.print("Wiper set to: ");
        Serial.println(step);
      }
    } else {
      Serial.println("Only enter value from 0.0 to 4.0 with step 0.1");
    }
  }
}

void ads_setting(void) {
  ADS.begin();
  ADS.setGain(0);      //  6.144 volt
  ADS.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
  ADS.setMode(0);      //  continuous mode
  ADS.readADC(0);      //  first read to trigger
}

void test_TDS(void) {
  ADS.readADC(TDS_ADS_CHANNEL);
  _ads_data.tds_data = ADS.getValue();
  Serial.println("TDS value: " + String(_ads_data.tds_data));

  if (_ads_data.tds_data > 5) {
    Serial.println("TDS sensor triggered: High TDS level detected.");
  } else {
    Serial.println("TDS value is normal");
  }
}

void test_H2(void) {
  ADS.readADC(H2_SENSOR_1_ADS_CHANNEL);
  _ads_data.h2_data_1 = ADS.getValue();
  Serial.println("H2 sensor 1 value: " + String(_ads_data.h2_data_1));

  ADS.readADC(H2_SENSOR_2_ADS_CHANNEL);
  _ads_data.h2_data_2 = ADS.getValue();
  Serial.println("H2 sensor 2 value: " + String(_ads_data.h2_data_2));
}

/*
  Equation y=1.0933x + 0.4321
  y: voltage
  x: %H2
*/

int voltageToStep(float Vout, float Vref, int maxSteps) {
  if (Vout < 0)
    Vout = 0;
  if (Vout > Vref)
    Vout = Vref;

  float ratio = Vout / Vref;  // phần trăm
  int step = round(ratio * maxSteps);
  return step;
}

void IRAM_ATTR limitsw_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = esp_timer_get_time() / 1000;

  if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS) {
    limitsw_interruptFlag = true;
    last_isr_time = current_time;
  }
}

void IRAM_ATTR level_float_1_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = esp_timer_get_time() / 1000;

  if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS) {
    float1_interruptFlag = true;
    last_isr_time = current_time;
  }
}

void IRAM_ATTR level_float_2_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = esp_timer_get_time() / 1000;

  if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS) {
    float2_interruptFlag = true;
    last_isr_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
#ifdef TEST_TMP36
  pinMode(TMP36_PIN, INPUT);
  pinMode(TEMP_HUMI_PIN, INPUT);
#endif

#ifdef TEST_PUMP
  pinMode(AIR_PUMP_PIN, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);
#endif

#ifdef TEST_FAN
  pinMode(ENABLE_FAN_PIN, INPUT);
#endif

#ifdef TEST_FLOW_SENSOR
  analogReadResolution(12);  // ADC Resolution = 12 bit (0-4095)
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Khởi tạo I2C

#ifdef TEST_CURRENT_CONTROL
  pinMode(ENB_POWER_PIN, INPUT);
  digitalWrite(ENB_POWER_PIN, HIGH);
#endif

#ifdef TEST_TDS || TEST_H2_SENSOR
  ads_setting();
#endif

#ifdef TEST_LIMIT_SW
  pinMode(LIMIT_SW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limitsw_handleInterrupt, CHANGE);
#endif

#ifdef TEST_FLOAT_SENSOR
  pinMode(LEVEL_FLOAT1_PIN, INPUT);
  pinMode(LEVEL_FLOAT2_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT1_PIN), level_float_1_handleInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT2_PIN), level_float_2_handleInterrupt, RISING);
#endif

#ifdef TEST_KNOB
  UART2.begin(460800, SERIAL_8N1, GPIO_NUM_1, GPIO_NUM_2);
#endif

}

void loop() {
#ifdef TEST_TMP36
  test_TMP36();
#endif

#ifdef TEST_PUMP
  test_Pumps();
#endif

#ifdef TEST_FAN
  test_Fan();
#endif

#ifdef TEST_FLOW_SENSOR
  test_flow_sensor();
#endif

#ifdef TEST_CURRENT_CONTROL
  test_current_control();
#endif

#ifdef TEST_TDS
  test_TDS();
#endif

#ifdef TEST_H2_SENSOR
  test_H2();
#endif

#ifdef TEST_LIMIT_SW
  if (limitsw_interruptFlag || digitalRead(LIMIT_SW_PIN) == LOW) {
    if (!digitalRead(LIMIT_SW_PIN)) {
      Serial.println("Limit switch is triggered");
      limitsw_interruptFlag = true;
      delay(500);
    } else {
      limitsw_interruptFlag = false;
    }
  }
#endif

#ifdef TEST_FLOAT_SENSOR
  if (!float1_interruptFlag && !float2_interruptFlag) {
    Serial.println("Water level is high (> 75%)");
  } else if (float1_interruptFlag && !float2_interruptFlag) {
    Serial.println("Water level is medium (< 75% and > 25%)");
  } else {
    Serial.println("Water level is low (< 25%)");
  }
#endif

#ifdef TEST_KNOB
  if (UART2.available()) {
    String recv = UART2.readStringUntil('\n');
    Serial.println(recv);
  }
#endif

  delay(100);
}