#include <Wire.h>

#define TPL0401B_ADDR 0x3E  // Địa chỉ I2C của TPL0401B
#define CMD_WIPER     0x00  // Thanh ghi Wiper Register

#define I2C_SDA GPIO_NUM_18
#define I2C_SCL GPIO_NUM_17
#define ENB_POWER_PIN GPIO_NUM_46

#define VREF 5.0
#define MAX_STEPS 128

/*
  Equation y=1.0933x + 0.4321
  y: voltage
  x: %H2
*/

int voltageToStep(float Vout, float Vref, int maxSteps)
{
    if (Vout < 0)
        Vout = 0;
    if (Vout > Vref)
        Vout = Vref;

    float ratio = Vout / Vref; // phần trăm
    int step = round(ratio * maxSteps);
    return step;
}

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
    float recv_value = cmd.toFloat();
    recv_value = round(recv_value * 10) / 10.0;
    if (recv_value >= 0.0 && recv_value <= 4.0) {
      float voltage = 1.0933 * recv_value + 0.4321;
      int step = voltageToStep(voltage, VREF, MAX_STEPS);
      if (step >= 0 && step < 128)
      {
        setWiper(step);
        Serial.print("Wiper set to: ");
        Serial.println(step);
      }
    }
    else {
      Serial.println("Only enter value from 0.0 to 4.0 with step 0.1");
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