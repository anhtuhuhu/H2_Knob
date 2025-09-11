#include <Wire.h>

#define TPL0401B_ADDR 0x3E  // Địa chỉ I2C của TPL0401B
#define CMD_WIPER     0x00  // Thanh ghi Wiper Register

#define I2C_SDA GPIO_NUM_18
#define I2C_SCL GPIO_NUM_17
#define ENB_POWER_PIN GPIO_NUM_46

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);  // Khởi tạo I2C
  pinMode(ENB_POWER_PIN, INPUT);
  digitalWrite(ENB_POWER_PIN, HIGH);
  Serial.println("TPL0401B Test Start!");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    Serial.println("Receive command: " + cmd);
    int recv_value = cmd.toInt();
    if (recv_value >= 0 && recv_value < 128)
    {
      setWiper(recv_value);
      Serial.print("Wiper set to: ");
      Serial.println(recv_value);
    }
  }

  delay(1000);
}

// Hàm ghi giá trị Wiper
void setWiper(uint8_t value) {
  Wire.beginTransmission(TPL0401B_ADDR);
  Wire.write(CMD_WIPER); // Thanh ghi Wiper = 0x00
  Wire.write(value & 0x7F); // 7 bit (0-127)
  Wire.endTransmission();
}