#include <Arduino.h>

#define TMP36_PIN GPIO_NUM_4
#define TEMP_HUMI_PIN GPIO_NUM_8

void test_TMP36(void) {
  int TMP36_RawValue = 0;
  float Voltage_mV;
  float Temperature_C;
  float Temperature_F;

  // Read TMP36_Sensor ADC Pin
  TMP36_RawValue = analogRead(TEMP_HUMI_PIN);
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(TEMP_HUMI_PIN, INPUT);
  pinMode(TMP36_PIN, INPUT);
  digitalWrite(TMP36_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  test_TMP36();
  delay(100);
}
