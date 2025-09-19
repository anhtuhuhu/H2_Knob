#include <Arduino.h>

#define LIMIT_SW_PIN GPIO_NUM_6
#define LEVEL_FLOAT1_PIN GPIO_NUM_47
#define LEVEL_FLOAT2_PIN GPIO_NUM_48
#define DEBOUNCE_TIME_MS 10

volatile bool limitsw_state = false;

void IRAM_ATTR limitsw_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = millis();

  if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS) {
    limitsw_state = digitalRead(LIMIT_SW_PIN);
    last_isr_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);

  limitsw_state = digitalRead(LIMIT_SW_PIN);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limitsw_handleInterrupt, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (limitsw_state == LOW) {
    Serial.println("Limit switch is LOW level");
  }
  Serial.println("SW state: " + String(digitalRead(LIMIT_SW_PIN)));

  delay(300);
}
