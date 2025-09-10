#include <Arduino.h>

#define AIR_PUMP_PIN GPIO_NUM_5
#define WATER_PUMP_PIN GPIO_NUM_45

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(AIR_PUMP_PIN, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);

}

void loop() {
  Serial.println("Pumps ON");
  digitalWrite(AIR_PUMP_PIN, HIGH);
  digitalWrite(WATER_PUMP_PIN, HIGH);
  delay(5000);
  Serial.println("Pumps OFF");
  digitalWrite(AIR_PUMP_PIN, LOW);
  digitalWrite(WATER_PUMP_PIN, LOW);
  delay(5000);
}
