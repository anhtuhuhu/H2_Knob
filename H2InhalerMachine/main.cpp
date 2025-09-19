#include "projdefs.h"
#include "main.h"
#include "esp_system.h"
// ============================================================CLASS OBJECT============================================================
// OTA _ota;   // Old OTA object (HTTP)
// OTA_MQTT _ota; // New OTA object (MQTT)
WIFI _wifi; // WIFI object
// MQTT _mqtt;    // MQTT object
// ====================================================================================================================================

// =============================================================GLOBAL VAR=============================================================
// WebServer instance - commented out as we use MQTT for OTA now
// WebServer server(80);
// UART for Knob communication
HardwareSerial UART2(1);

ADS1115 ADS(0x48);
Knob_data_t _board2Knob;
Knob_data_t _knob2Board;
Error_status_t _errorStatus;
String MAC_id;

typedef void (*WiFiReconnectFunc)();

ads_data_t _ads_data;

// Interrupt flags
volatile bool limitsw_state = false;
volatile bool float1_state = false;
volatile bool float2_state = false;

const int pwmPin = 16;        // PWM pin
const int pwmFreq = 4000;     // PWM frequency
const int pwmResolution = 13; // 13 bit
const int pwmChannel = 1;     // PWM channel

const char *H2_MQTT_BROKER = "broker.emqx.io";
const int H2_MQTT_PORT = 8883;
const char *H2_MQTT_USERNAME = "inhaler-h2";
const char *H2_MQTT_PASSWORD = "inhaler-h2";

uint32_t lastReadTimeSensor = 0;
volatile unsigned long lastDebounceTime = 0;
String h2_percent;
String tds_data;
bool h2;
bool tds;
uint8_t temp_return_val;
bool haveMessageViaMQTT;
bool clearErrorFlag = false;
// Note : If H2 = 0 meaning Pause/Timeout wait until Resume/Start
// float _percentH2 = 0;
int flow;
String timer;
float airFlowRate;
bool _run = false;

// 0: cloud->device, 1: device->cloud

TaskHandle_t pumpControlTaskHandle = NULL;
TaskHandle_t sendMQTTDataToKnobTaskHandle = NULL;

// ====================================================================================================================================

// ===============================================================SETUP================================================================
void Setup()
{

  Serial.begin(115200);
  UART2.begin(460800, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  Serial.print("VERSION: ");
  Serial.println(VERSION);

  delay(200);

  //=================================I2C INIT==================================
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  //=====================================PEM====================================
#if BOARD == OLD
  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  turnOffPEM();
#elif BOARD == NEW
  turnOffPEM();
#endif
  //=================================DEFINE PINS=================================
  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(AIR_PUMP_PIN, OUTPUT);

  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  pinMode(LEVEL_FLOAT1_PIN, INPUT_PULLUP);
  pinMode(LEVEL_FLOAT2_PIN, INPUT_PULLUP);

  limitsw_state = digitalRead(LIMIT_SW_PIN); // bth = true. trigger = false;
  float1_state = digitalRead(LEVEL_FLOAT1_PIN);
  float2_state = digitalRead(LEVEL_FLOAT2_PIN); // có nước -> thông -> xuống GND. Bình thường đèn sáng. Hết nước đèn tắt -> bth: false. trigger = true; 

  pinMode(ENB_PWR_PEM_PIN, OUTPUT);
  digitalWrite(ENB_PWR_PEM_PIN, HIGH); // always enable power for PEM

  digitalWrite(WATER_PUMP_PIN, 0); // off water pump
  digitalWrite(AIR_PUMP_PIN, 0);   // off air pump

  pinMode(EN_FAN_PIN, OUTPUT);
  digitalWrite(EN_FAN_PIN, 0); // on fan for air circulation

  pinMode(TMP36_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(TMP36_SHUTDOWN_PIN, HIGH); // turn on TMP36 sensor

  //=================================INTERRUPT=================================
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limitsw_handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT1_PIN), level_float_1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT2_PIN), level_float_2_ISR, CHANGE);

  //=================================ADS INIT==================================
  ads_setting();
  //====================================WI-FI====================================
  _wifi.init();
  MAC_id = _wifi.getMacAddress();
  Serial.println("MAC ID: " + MAC_id);

  for (int i = 0; i < TOPIC_COUNT; i++)
  {
    if (mqtt_topics[i].direction == 1)
    {
      Serial.print("Topic for publish: ");
      mqtt_publish_message(i, MAC_id.c_str(), "");
    }
    else
    {
      Serial.print("Topic for subscribe: ");
      mqtt_subscribe(i, MAC_id.c_str());
    }
  }

  WiFiReconnectFunc reconnect_handler = connectedWiFi_cb;
  _wifi.setConnectedCallback(reconnect_handler);

  UART2.print("\n");
  delay(1000);
  UART2.println(VERSION);
  delay(1000);

  if (WiFi.status() == WL_CONNECTED)
  {
    //=================================Send VERSION & IP to KNOB=================================
    UART2.print("\n");
    delay(1000);
    UART2.println(VERSION);
    delay(1000);
    char ip_address[20];
    snprintf(ip_address, sizeof(ip_address), "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

    Serial.print("ip_address: ");
    Serial.println(ip_address);
    UART2.print("IP:" + String(ip_address) + "\n");

    //=========================================MQTT=========================================
    _mqtt.MQTT_init(H2_MQTT_BROKER, H2_MQTT_PORT, H2_MQTT_USERNAME, H2_MQTT_PASSWORD);
    mqtt_subscribe(PARAMETER_CURRENT, MAC_id.c_str());
    mqtt_publish_message(PARAMETER_REQUEST, MAC_id.c_str(), "{}");
  }

  _mqtt.setMessageHandler(handleMQTTReceivedMessage);

  //====================================Tasks Creation====================================
  xTaskCreatePinnedToCore(Serial_Handle_task, "Serial_Handle_task", 4096, NULL, 6, NULL, 0);
  xTaskCreatePinnedToCore(warning_Handle_Task, "warning_Handle_Task", 4096, NULL, 6, NULL, 0);
  xTaskCreatePinnedToCore(PumpControlTask, "PumpControlTask", 4096, NULL, 6, &pumpControlTaskHandle, 0);
  // xTaskCreatePinnedToCore(SendMQTTDataToKnobTask, "SendMQTTDataToKnobTask", 4096, NULL, 6, &sendMQTTDataToKnobTaskHandle, 0);
  Serial.print("Free heap: ");
  Serial.println(esp_get_free_heap_size());
}

// ===============================================================MAIN================================================================
void Main()
{
  vTaskDelay(1);

  // Comment out HTTP server handling as we use MQTT for OTA now
  // server.handleClient();

  // Handle OTA state for Knob via UART responses
  // if (_ota.otaState == _ota.OTA_WAIT_READY || _ota.otaState == _ota.OTA_SENDING || _ota.otaState == _ota.OTA_END)

  // if (_ota.getStatus().state == OTA_MQTT::OTA_WAIT_READY || _ota.getStatus().state == OTA_MQTT::OTA_SENDING || _ota.getStatus().state == OTA_MQTT::OTA_END) {
  //   if (UART2.available()) {
  //     String response = UART2.readStringUntil('\n');
  //     response.trim();
  //     // if (response == "READY" && _ota.otaState == _ota.OTA_WAIT_READY)
  //     if (response == "READY" && _ota.getStatus().state == OTA_MQTT::OTA_WAIT_READY) {
  //       // _ota.otaState = _ota.OTA_SENDING;
  //       _ota.getStatus().state = OTA_MQTT::OTA_SENDING;
  //       Serial.println("Knob is ready for OTA");
  //     }
  //     // else if (response == "OTA_SUCCESS" && _ota.otaState == _ota.OTA_END)
  //     else if (response == "OTA_SUCCESS" && _ota.getStatus().state == OTA_MQTT::OTA_END) {
  //       // _ota.otaState = _ota.OTA_IDLE;
  //       _ota.getStatus().state = OTA_MQTT::OTA_IDLE;
  //       Serial.println("Knob OTA successful!");
  //       delay(4000);
  //       Serial.println("Send version and IP");
  //       UART2.println(VERSION);
  //       delay(1000);
  //       char ip_address[20];
  //       snprintf(ip_address, sizeof(ip_address), "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  //       UART2.println("IP:" + String(ip_address));
  //       delay(1000);
  //     } else if (response.startsWith("ERR")) {
  //       Serial.println("Knob OTA failed: " + response);
  //       // _ota.otaState = _ota.OTA_IDLE;
  //       _ota.getStatus().state = OTA_MQTT::OTA_IDLE;
  //     }
  //   }
  // }
}

// =============================================================INTERRUPT=============================================================
void IRAM_ATTR limitsw_handleInterrupt()
{
  static uint32_t last_isr_time = 0;
  uint32_t current_time = millis();

  if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS)
  {
    limitsw_state = digitalRead(LIMIT_SW_PIN);
    last_isr_time = current_time;
  }
}

void IRAM_ATTR level_float_1_ISR()
{
  static uint32_t last_isr_time = 0;
  uint32_t now = millis();
  if (now - last_isr_time > DEBOUNCE_TIME_MS)
  {
    float1_state = digitalRead(LEVEL_FLOAT1_PIN); // lưu trạng thái hiện tại
    last_isr_time = now;
  }
}

void IRAM_ATTR level_float_2_ISR()
{
  static uint32_t last_isr_time = 0;
  uint32_t now = millis();
  if (now - last_isr_time > DEBOUNCE_TIME_MS)
  {
    float2_state = digitalRead(LEVEL_FLOAT2_PIN);
    last_isr_time = now;
  }
}

void ads_setting(void)
{
  ADS.begin();
  ADS.setGain(0);     //  6.144 volt
  ADS.setDataRate(7); //  0 = slow   4 = medium   7 = fast
  ADS.setMode(0);     //  continuous mode
  ADS.readADC(0);     //  first read to trigger
}

void turnOffPEM()
{
#if BOARD == OLD
  ledcWrite(pwmPin, 0);
#elif BOARD == NEW
  setWiper(0);
#endif
}
// =============================================================SENSORS=============================================================
#if BOARD == OLD
bool readH2Sensor(int h2_val)
{
  ADS.readADC(H2_SENSOR_ADS_CHANNEL);
  _ads_data.h2_data = ADS.getValue();
  // Serial.print("H2 Value: ");
  // Serial.println(h2_val);

  if (h2_val > 11000)
  {
    turnOffPEM();
    return false;
  }

  return true;
}

bool readTDS(int tds_val)
{
  ADS.readADC(TDS_ADS_CHANNEL);
  _ads_data.tds_data = ADS.getValue();

  // float f = ADS.toVoltage(1);
  // uint32_t tdsValue = (float)((_ads_data.tds_data * f) / _Volfor1000ppmTDS) * 1000;  // Convert _ads_data.tds_data -> tdsValue
  uint32_t tdsValue = tds_val;
  // Serial.print("TDS Value: ");
  // Serial.print(tdsValue);
  // Serial.println("ppm");

  if (tdsValue > 5)
  {
    // Serial.println("TDS sensor triggered: High TDS level detected.");
    // return false;
  }

  return true;
}
#elif BOARD == NEW
bool readH2Sensor(void)
{
  ADS.readADC(H2_SENSOR_1_ADS_CHANNEL);
  _ads_data.h2_data_1 = ADS.getValue();
  Serial.print("H2 sensor 1 Value: ");
  Serial.println(_ads_data.h2_data_1);

  ADS.readADC(H2_SENSOR_2_ADS_CHANNEL);
  _ads_data.h2_data_2 = ADS.getValue();
  Serial.print("H2 sensor 2 Value: ");
  Serial.println(_ads_data.h2_data_2);

  if (_ads_data.h2_data_1 > H2_SENSOR_THRESHOLD || _ads_data.h2_data_2 > H2_SENSOR_THRESHOLD)
  {
    turnOffPEM();
    return false;
  }

  return true;
}

bool readTDS(void)
{
  ADS.readADC(TDS_ADS_CHANNEL);
  _ads_data.tds_data = ADS.getValue();

  // float f = ADS.toVoltage(1);
  // uint32_t tdsValue = (float)((_ads_data.tds_data * f) / _Volfor1000ppmTDS) * 1000;  // Convert _ads_data.tds_data -> tdsValue

  Serial.print("TDS Value: ");
  Serial.print(_ads_data.tds_data);
  Serial.println("ppm");

  if (_ads_data.tds_data > TDS_THRESHOLD)
  {
    Serial.println("TDS sensor triggered: High TDS level detected.");
    return false;
  }

  return true;
}
#endif

uint8_t check_temperature(void)
{
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

  if (Temperature_C > TEMPERATURE_UPPER_THRESHOLD || Temperature_C < TEMPERATURE_LOWER_THRESHOLD)
  {
    if (Temperature_C > TEMPERATURE_UPPER_THRESHOLD)
    {
      Serial.println("Temperature is too high");
      return HIGH_TEMPERATURE;
    }
    else
    {
      Serial.println("Temperature is too low");
      return LOW_TEMPERATURE;
    }
  }
  return NORMAL_TEMPERATURE;
}

// =============================================================CURRENT CONTROL=============================================================
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

void setWiper(uint8_t value)
{
  Wire.beginTransmission(TPL0401B_ADDR);
  Wire.write(CMD_WIPER);    // Thanh ghi Wiper = 0x00
  Wire.write(value & 0x7F); // 7 bit (0-127)
  Wire.endTransmission();
}

float calculateVoltage(int h2_percent)
{
  return 1.0933 * (float)(h2_percent / 100) + 0.4321;
}

// =============================================================KNOB COMMUNICATION=============================================================
void connectedWiFi_cb(void)
{
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  MAC_id = _wifi.getMacAddress();

  Serial.println("Base MAC: " + MAC_id);

  char ip_address[20];
  snprintf(ip_address, sizeof(ip_address), "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

  Serial.print("ip_address: ");
  Serial.println(ip_address);
  UART2.print("IP:" + String(ip_address) + "\n");

  // server.on("/", std::bind(&OTA::handleRoot, &_ota)); // need to use std::bind(&Class::method, &object) for calling a method of a class in
  // server.on("/update", HTTP_POST, std::bind(&OTA::handleUpdate, &_ota), std::bind(&OTA::handleUpload, &_ota));
  // server.begin();
  // Serial.println("HTTP server started");

  _mqtt.MQTT_init(H2_MQTT_BROKER, H2_MQTT_PORT, H2_MQTT_USERNAME, H2_MQTT_PASSWORD);
  mqtt_publish_message(PARAMETER_REQUEST, MAC_id.c_str(), "{}");
  mqtt_subscribe(PARAMETER_CURRENT, MAC_id.c_str());
  mqtt_subscribe(OTA_UPDATE, MAC_id.c_str());
}

void handleKnobCommand(void)
{
  // Keep reading data until a value greater than 0 is received
  do
  {
    // if (_ota.otaState != _ota.OTA_IDLE)
    if (_ota.getStatus().state != OTA_MQTT::OTA_IDLE)
    {
      Serial.println("Is OTA");
      break;
    }

    if (UART2.available())
    {
      String receivedData = UART2.readStringUntil('\n'); // Read data from UART2
      Serial.println("Received data from Knob: ");
      Serial.println(receivedData);

      parseKnobData(receivedData.c_str());
      flow = _knob2Board.percent * 100;
      Serial.println("Flow nhận được : " + String(flow));
      // String jsonToSend = buildKnobDataJson(_knob2Board);
      // Serial.println("JSON to send: " + jsonToSend);
      // mqtt_publish_message(PARAMETER_UPDATE, MAC_id.c_str(), jsonToSend);

      _run = true;
    }
    int dutyValue;
    if (flow == 0)
    {
      turnOffPEM();
      // Serial.println("Flow is 0. Stopping PWM.");
      _run = false;
      if (h2 && tds && !limitsw_state && float2_state)
      {
         // Khi hết lỗi -> knob yêu cầu pwm trở về 0 -> cần gửi lại lệnh reset
        UART2.println("X");
        if (!clearErrorFlag)
        {
          mqtt_publish_message(ERROR_CLEAR, MAC_id.c_str(), "{}");
          _errorStatus.hepaFilterMissing = 0;
          _errorStatus.waterLevelLow = 0;
          _errorStatus.waterPurityIssue = 0;
          _errorStatus.h2LeakDetected = 0;
          clearErrorFlag = true;
        }
      }
    }
#if BOARD == NEW
    if(flow == 0)
    {
      turnOffPEM();
      delay(400);
      continue;
    }
    float voltage = calculateVoltage(flow); // nếu flow = 0 -> voltage = 0.4321
    int step = voltageToStep(voltage, VREF, MAX_STEPS);
    if (step >= 0 && step < MAX_STEPS)
    {
      setWiper(step);
      Serial.print("Wiper set to: ");
      Serial.println(step);
    }
#elif BOARD == OLD
    dutyValue = map(flow, 10, 400, 550, 7976);
    ledcWrite(pwmPin, dutyValue);
#endif 
    delay(400);
  } while (flow == 0); // Continue waiting if flow is 0
}

void send_error_to_Knob(ErrorCode error)
{
  switch (error)
  {
  case HEPA_FILTER_MISSING:
    _errorStatus.hepaFilterMissing = 1;
    clearErrorFlag = false;
    UART2.println("HEPA filter missing");
#ifdef DEBUG
    Serial.println("HEPA filter missing-");
#endif
    break;
  case TEMPERATURE_TOO_HIGH:
    _errorStatus.temperatureHigh = 1;
    clearErrorFlag = false;
    UART2.println("Temperature Sensor");
#ifdef DEBUG
    Serial.println("Temperature too high-");
#endif
    break;
  case TEMPERATURE_TOO_LOW:
    _errorStatus.temperatureLow = 1;
    clearErrorFlag = false;
    UART2.println("Temperature Sensor");
#ifdef DEBUG
    Serial.println("Temperature too low-");
#endif
    break;
  case WATER_PURITY_ISSUE:
    _errorStatus.waterPurityIssue = 1;
    clearErrorFlag = false;
    UART2.println("Water Purity Issue");
#ifdef DEBUG
    Serial.println("Water Purity Issue-");
#endif
    break;
  case H2_LEAK_DETECTED:
    _errorStatus.h2LeakDetected = 1;
    clearErrorFlag = false;
    UART2.println("Hydrogen leak found");
#ifdef DEBUG
    Serial.println("Hydrogen leak found-");
#endif
    break;
  }
}

// =============================================================TASKS=============================================================
void Serial_Handle_task(void *param)
{
  for (;;)
  {
    handleKnobCommand();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void warning_Handle_Task(void *param)
{
  for (;;)
  {
    if (limitsw_state == LOW)
    {
        if (!digitalRead(LIMIT_SW_PIN))
        {
          turnOffPEM();
          send_error_to_Knob(HEPA_FILTER_MISSING);
          delay(500);
        }
        else
        {
          _errorStatus.hepaFilterMissing = 0;
        }
    }
#if BOARD == NEW
      if (float1_state == LOW && float2_state == LOW)
      {
        Serial.println("Water level is HIGH (> 75%)");
        _errorStatus.waterLevelLow = 0;
      }
      else if (float1_state == HIGH && float2_state == LOW)
      {
        Serial.println("Water level is MEDIUM (25% - 75%)");
        _errorStatus.waterLevelLow = 0;
      }
      else if (float2_state == HIGH)
      {
        Serial.println("Water level is LOW (< 25%)");
        turnOffPEM();
        send_error_to_Knob(WATER_LEVEL_LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
#elif BOARD == OLD
      if (float2_state == LOW) { // board cũ bth có nước (không thể nhấn nút mãi) => đảo trạng thái thành bth = true, trigger = false
        turnOffPEM();
        send_error_to_Knob(WATER_LEVEL_LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      else {
        _errorStatus.waterLevelLow = 0;
      }
#endif
      if (millis() - lastReadTimeSensor > SENSOR_READ_TIME)
      {
        lastReadTimeSensor = millis();
// #if BOARD == OLD
//         h2 = readH2Sensor(h2_percent.toInt());
//         tds = readTDS(tds_data.toInt());
// #elif BOARD == NEW
//         h2 = readH2Sensor();
//         tds = readTDS();
// #endif
        h2 = true;
        tds = true;
        if (!tds)
        {
          send_error_to_Knob(WATER_PURITY_ISSUE);
          delay(500);
        }
        else
        {
          _errorStatus.waterPurityIssue = 0;
        }
        if (!h2)
        {
          send_error_to_Knob(H2_LEAK_DETECTED);
          delay(500);
        }
        else
        {
          _errorStatus.h2LeakDetected = 0;
        }

#if BOARD == NEW
        // temp_return_val = check_temperature();
        // if (temp_return_val == NORMAL_TEMPERATURE)
        // {
        //   _errorStatus.temperatureHigh = 0;
        //   _errorStatus.temperatureLow = 0;
        // }
        // else
        // {
        //   if (temp_return_val == HIGH_TEMPERATURE)
        //   {
        //     send_error_to_Knob(TEMPERATURE_TOO_HIGH);
        //     delay(500);
        //   }
        //   else if (temp_return_val == LOW_TEMPERATURE)
        //   {
        //     send_error_to_Knob(TEMPERATURE_TOO_LOW);
        //     delay(500);
        //   }
        // }
#endif
        // if (_errorStatus.hepaFilterMissing || _errorStatus.waterLevelLow || _errorStatus.waterPurityIssue || _errorStatus.h2LeakDetected || _errorStatus.temperatureHigh || _errorStatus.temperatureLow)
        // {
        //   String errorJSON = buildErrorJson(_errorStatus);
        //   mqtt_publish_message(ERROR_UPDATE, MAC_id.c_str(), errorJSON);
        // }
      }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void PumpControlTask(void *param)
{
  for (;;)
  {
    if (_ota.getStatus().state == OTA_MQTT::OTA_IDLE)
    {
#if BOARD == OLD
      if (h2 && tds && float2_state && limitsw_state && _run && _knob2Board.state) {
        digitalWrite(WATER_PUMP_PIN, 1);
        digitalWrite(AIR_PUMP_PIN, 1);
        // maybe call POST function here
      } else {
        digitalWrite(WATER_PUMP_PIN, 0);
        digitalWrite(AIR_PUMP_PIN, 0);
        // maybe call POST function here
      }
#elif BOARD == NEW
      // Điều khiển bơm nước và bơm không khí
      if (h2 && tds && limitsw_state && float2_state && _run && _knob2Board.state)
      {
        digitalWrite(WATER_PUMP_PIN, 1);
        digitalWrite(AIR_PUMP_PIN, 1);
        // maybe call POST function here
      }
      else
      {
        digitalWrite(WATER_PUMP_PIN, 0);
        digitalWrite(AIR_PUMP_PIN, 0);
        // maybe call POST function here
      }
#endif
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void SendMQTTDataToKnobTask(void *param)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("SendMQTTDataToKnobTask takes notify successfully");

    // while (UART2.available()) {
    //   UART2.read();
    // }

    String knobDataToSend = buildKnobDataJson(_board2Knob);
    UART2.println(knobDataToSend);
  }
}

// =============================================================MQTT HANDLE=============================================================
void handleMQTTReceivedMessage(const JsonDocument &doc)
{
  haveMessageViaMQTT = false;

  _board2Knob.state = doc["state"];

  const char *t = doc["timer"];
  if (t != nullptr)
  {
    strncpy(_board2Knob.timer, t, sizeof(_board2Knob.timer) - 1);
    _board2Knob.timer[sizeof(_board2Knob.timer) - 1] = '\0';
  }

  _board2Knob.percent = doc["percent"] | _board2Knob.percent;
  _board2Knob.typeMachine = doc["type_machine"] | _board2Knob.typeMachine;

  Serial.println("================== KNOB MQTT DATA ==================");
  Serial.println("KNOB state: " + String(_board2Knob.state));
  Serial.println("KNOB timer: " + String(_board2Knob.timer));
  Serial.println("KNOB percent: " + String(_board2Knob.percent));
  Serial.println("KNOB type_machine: " + String(_board2Knob.typeMachine));

  haveMessageViaMQTT = true;
  if (sendMQTTDataToKnobTaskHandle != NULL)
  {
    xTaskNotifyGive(sendMQTTDataToKnobTaskHandle);
  }
}

void parseKnobData(const char *input)
{
  // Find the ':' character to skip the prefix "Data From Knob:"
  const char *dataStart = strchr(input, ':');
  if (dataStart == NULL)
  {
    Serial.println("Invalid format: no ':' found");
    return;
  }

  dataStart++; // Move past the ':' character

  // Copy the substring after ':' into a local buffer (strtok modifies the string)
  char buffer[64];
  strncpy(buffer, dataStart, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-terminated string

  // Use strtok to split the string by the '|' delimiter
  char *token = strtok(buffer, "|");
  int index = 0;

  // Parse each token into the appropriate field in _knobData
  while (token != NULL)
  {
    switch (index)
    {
    case 0:
      _knob2Board.state = atoi(token); // Convert string to integer
      break;
    case 1:
      strncpy(_knob2Board.timer, token, sizeof(_knob2Board.timer) - 1); // Copy timer string
      _knob2Board.timer[sizeof(_knob2Board.timer) - 1] = '\0';
      break;
    case 2:
      _knob2Board.percent = atof(token); // Convert string to float
      break;
    case 3:
      _knob2Board.typeMachine = atof(token); // Convert string to float
      break;
    }
    token = strtok(NULL, "|"); // Move to the next token
    index++;
  }

  // Ensure all 4 fields were parsed
  if (index < 4)
  {
    Serial.println("Invalid data: not enough fields");
    return;
  }

  // Debug output
  Serial.println("===========================================");
  Serial.println("Parsed Knob Data:");
  Serial.print("State: ");
  Serial.println(_knob2Board.state);
  Serial.print("Timer: ");
  Serial.println(_knob2Board.timer);
  Serial.print("Percent: ");
  Serial.println(_knob2Board.percent);
  Serial.print("Type Machine: ");
  Serial.println(_knob2Board.typeMachine);
  Serial.println("===========================================");
}

void mqtt_publish_message(int topic_id, const char *mac_id, String message)
{
  char topic[MQTT_TOPIC_MAX_LEN];

  snprintf(topic, sizeof(topic), "device/%s/%s/%s", mac_id, mqtt_topics[topic_id].service, mqtt_topics[topic_id].action);

  if (message != "")
  {
    _mqtt.publishMessage(topic, message.c_str());
  }

  Serial.println(String(topic));
}

void mqtt_subscribe(int topic_id, const char *mac_id)
{
  char topic[MQTT_TOPIC_MAX_LEN];

  snprintf(topic, sizeof(topic), "device/%s/%s/%s", mac_id, mqtt_topics[topic_id].service, mqtt_topics[topic_id].action);

  _mqtt.subscribeTopic(topic);
  Serial.println(String(topic));
}

String buildKnobDataJson(const Knob_data_t &data)
{
  StaticJsonDocument<128> doc;

  doc["state"] = data.state;
  doc["timer"] = data.timer;
  doc["percent"] = data.percent;
  doc["type_machine"] = data.typeMachine;

  String jsonString;
  serializeJson(doc, jsonString); // Convert to String
  return jsonString;
}

String buildErrorJson(const Error_status_t &errorStatus)
{
  StaticJsonDocument<256> doc;
  doc["HEPA_FILTER_MISSING"] = errorStatus.hepaFilterMissing ? 1 : 0;
  doc["WATER_PURITY_ISSUE"] = errorStatus.waterPurityIssue ? 1 : 0;
  doc["H2_ABOVE_1_PERCENT"] = errorStatus.h2LeakDetected ? 1 : 0;
  doc["WATER_LEVEL_LOW"] = errorStatus.waterLevelLow ? 1 : 0;
  doc["TEMPERATURE_SENSOR"] = (errorStatus.temperatureHigh || errorStatus.temperatureLow) ? 1 : 0;

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}