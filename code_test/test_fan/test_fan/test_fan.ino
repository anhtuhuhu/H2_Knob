#include <Arduino.h>

#define ENABLE_FAN_PIN GPIO_NUM_21

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_FAN_PIN, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Fan ON");
  digitalWrite(ENABLE_FAN_PIN, HIGH);
  delay(5000);
  Serial.println("Fan OFF");
  digitalWrite(ENABLE_FAN_PIN, LOW);
  delay(5000);
}
