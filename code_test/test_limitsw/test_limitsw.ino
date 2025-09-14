#include <Arduino.h>

#define LIMIT_SW_PIN GPIO_NUM_6

#define DEBOUNCE_TIME_MS 10

volatile bool limitsw_interruptFlag = false;

void IRAM_ATTR limitsw_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = esp_timer_get_time() / 1000;

  if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS) {
    limitsw_interruptFlag = true;
    last_isr_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SW_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limitsw_handleInterrupt, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (limitsw_interruptFlag || digitalRead(LIMIT_SW_PIN) == LOW) {
    if (!digitalRead(LIMIT_SW_PIN)) {
      Serial.println("Limit switch is triggered");
      limitsw_interruptFlag = true;
      delay(500);
    } else {
      limitsw_interruptFlag = false;
    }
  }
  delay(10);
}
