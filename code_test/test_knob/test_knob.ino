HardwareSerial UART2(1);

void setup() {
  Serial.begin(115200);
  UART2.begin(460800, SERIAL_8N1, GPIO_NUM_1, GPIO_NUM_2);
}

void loop() {
  if (UART2.available()) {
    String recv = UART2.readStringUntil('\n');
    Serial.println(recv);
  }
}
