#include <Arduino.h>

#define FLOW_PIN GPIO_NUM_9
#define VT_THRESHOLD 0.4

int readMedianADC(int pin) {
  const int N = 5;
  int values[N];

  // Đọc 5 mẫu liên tiếp
  for (int i = 0; i < N; i++) {
    values[i] = analogRead(pin);
    delayMicroseconds(200); // nghỉ 0.2ms để ADC ổn định
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

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);   // ADC Resolution = 12 bit (0-4095)
}

void loop() {
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

  delay(200); // đọc mỗi 200ms

}
