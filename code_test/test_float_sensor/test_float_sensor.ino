#include <Arduino.h>

#define LEVEL_FLOAT1_PIN GPIO_NUM_47
#define LEVEL_FLOAT2_PIN GPIO_NUM_48

#define DEBOUNCE_TIME_MS 10

volatile bool float1_interruptFlag = false;
volatile bool float2_interruptFlag = false;

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
  pinMode(LEVEL_FLOAT1_PIN, INPUT);
  pinMode(LEVEL_FLOAT2_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT1_PIN), level_float_1_handleInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT2_PIN), level_float_2_handleInterrupt, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!float1_interruptFlag && !float2_interruptFlag) {
    Serial.println("Water level is high (> 75%)");
  } else if (float1_interruptFlag && !float2_interruptFlag) {
    Serial.println("Water level is medium (< 75% and > 25%)");
  } else {
    Serial.println("Water level is low (< 25%)");
  }
  delay(500);
}
