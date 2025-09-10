// #include <stdio.h>
// #include "freertos/portmacro.h"
#include "main.h"
// ============================================================CLASS OBJECT============================================================
OTA _ota;                                     // OTA object
WIFI _wifi;                                   // WIFI object
MQTT _mqtt;                                   // MQTT object
// ====================================================================================================================================

// =============================================================GLOBAL VAR=============================================================
// WebServer instance
WebServer server(80);
// UART for Knob communication
HardwareSerial UART2(1);

ADS1115 ADS(0x48);
Knob_data_t _board2Knob;
Knob_data_t _knob2Board;
Error_status_t _errorStatus;
String MAC_id;

typedef void (*WiFiReconnectFunc) ();

ads_data_t _ads_data;

// Interrupt flags
volatile bool limitsw_interruptFlag = false;
volatile bool levelfloat_interruptFlag = false;
volatile bool waterleak_interruptFlag = false;

const int pwmPin = 16;        // PWM pin
const int pwmFreq = 4000;     // PWM frequency
const int pwmResolution = 13; // 13 bit
const int pwmChannel = 1;     // PWM channel

const char* H2_MQTT_BROKER = "broker.emqx.io";
const int H2_MQTT_PORT     = 8883;
const char* H2_MQTT_USERNAME = "inhaler-h2";
const char* H2_MQTT_PASSWORD = "inhaler-h2";

float _Volfor100PercentH2 = 0;
float _Volfor1000ppmTDS = 0;  // max sensor is 1000ppm

uint32_t lastReadTimeSensor = 0;
volatile unsigned long lastDebounceTime = 0;
String h2_percent;
String tds_data;
bool h2;
bool tds;
bool haveMessageViaMQTT;
bool clearErrorFlag = false;
// Note : If H2 = 0 meaning Pause/Timeout wait until Resume/Start
// float _percentH2 = 0;
int flow;
String timer;
float airFlowRate;
bool _run = false;

const mqtt_topic_info_t mqtt_topics[TOPIC_COUNT] = {
  [PARAMETER_UPDATE] = {"parameter", "update", 1},
  [PARAMETER_CURRENT] = {"parameter", "current", 0},
  [PARAMETER_REQUEST] = {"parameter", "request", 1},

  [ERROR_UPDATE] = {"error", "update", 1},
  [ERROR_CLEAR] = {"error", "clear", 0},
};

TaskHandle_t pumpControlTaskHandle = NULL;
TaskHandle_t sendMQTTDataToKnobTaskHandle = NULL;

// ====================================================================================================================================

// ===============================================================SETUP================================================================
void Setup() {
  Serial.begin(115200);
  UART2.begin(460800, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

  delay(200);  
  //=====================================LEDC====================================
  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  updatePWMVoltage(0);
  //=================================DEFINE PINS=================================
  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(AIR_PUMP_PIN, OUTPUT);

  pinMode(LIMIT_SW_PIN, INPUT);
  pinMode(WATER_LEAK_PIN, INPUT_PULLDOWN);
  pinMode(LEVEL_FLOAT_PIN, INPUT);

  digitalWrite(WATER_PUMP_PIN, 0);
  digitalWrite(AIR_PUMP_PIN, 0);
  //=================================INTERRUPT=================================
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limitsw_handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT_PIN), levelfloat_handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WATER_LEAK_PIN), waterleak_handleInterrupt, CHANGE);
  //=================================I2C INIT==================================
  Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
  //=================================ADS INIT==================================
  ads_setting();
  _Volfor100PercentH2 = ADS.getMaxVoltage();  // Need to change if max voltage is not 100% H2
  _Volfor1000ppmTDS = ADS.getMaxVoltage();    // Need to change if max voltage is not 1000ppm TDS
  //====================================WI-FI====================================
  // _wifi.wifi_scan_handle();
  // _wifi.ConnectWifi();
  _wifi.init();
  MAC_id = _wifi.getMacAddress();
  Serial.println("MAC ID: " + MAC_id);

  for (int i = 0; i < TOPIC_COUNT; i++) {
    if (mqtt_topics[i].direction == 1) {
      Serial.print("Topic for publish: ");
      mqtt_publish_message(i, MAC_id.c_str(), "");
    }
    else {
      Serial.print("Topic for subscribe: ");
      mqtt_subscribe(i, MAC_id.c_str());
    }
  }

  WiFiReconnectFunc reconnect_handler = connectedWiFi_cb;
  _wifi.setConnectedCallback(reconnect_handler);

  if (WiFi.status() == WL_CONNECTED) {
    server.on("/", std::bind(&OTA::handleRoot, &_ota)); // need to use std::bind(&Class::method, &object) for calling a method of a class in 
    server.on("/update", HTTP_POST, std::bind(&OTA::handleUpdate, &_ota), std::bind(&OTA::handleUpload, &_ota));
    server.begin();
    Serial.println("HTTP server started");

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

  // _wifi.wifiSetup();
  //====================================Tasks Creation====================================
  xTaskCreate(Serial_Handle_task, "Serial_Handle_task", 4096, NULL, 6, NULL);
  xTaskCreate(warning_Handle_Task, "warning_Handle_Task", 4096, NULL, 6, NULL);
  xTaskCreate(PumpControlTask, "PumpControlTask", 4096, NULL, 6, &pumpControlTaskHandle);
  xTaskCreate(SendMQTTDataToKnobTask, "SendMQTTDataToKnobTask", 4096, NULL, 6, &sendMQTTDataToKnobTaskHandle);
  
}

// ===============================================================MAIN================================================================
void Main() {
  vTaskDelay(1);
  server.handleClient();
  // Handle OTA state for Knob via UART responses
  if (_ota.otaState == _ota.OTA_WAIT_READY || _ota.otaState == _ota.OTA_SENDING || _ota.otaState == _ota.OTA_END) {
    if (UART2.available()) {
      String response = UART2.readStringUntil('\n');
      response.trim();
      if (response == "READY" && _ota.otaState == _ota.OTA_WAIT_READY) {
        _ota.otaState = _ota.OTA_SENDING;
        Serial.println("Knob is ready for OTA");
      } 
      else if (response == "OTA_SUCCESS" && _ota.otaState == _ota.OTA_END) {
        _ota.otaState = _ota.OTA_IDLE;
        Serial.println("Knob OTA successful!");
        delay(4000);
        Serial.println("Send version and IP");
        UART2.println(VERSION);
        delay(1000);
        char ip_address[20];
        snprintf(ip_address, sizeof(ip_address), "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
        UART2.println("IP:" + String(ip_address));
        delay(1000);
      } 
      else if (response.startsWith("ERR")) {
        Serial.println("Knob OTA failed: " + response);
        _ota.otaState = _ota.OTA_IDLE;
      }
    }
  }
}

// =============================================================INTERRUPT=============================================================
void IRAM_ATTR limitsw_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = esp_timer_get_time() / 1000;

  if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS) {
    limitsw_interruptFlag = true;
    last_isr_time = current_time;
  }
}

void IRAM_ATTR levelfloat_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = esp_timer_get_time() / 1000;

  if ((current_time - last_isr_time) > 1000) {
    levelfloat_interruptFlag = true;
    last_isr_time = current_time;
  }
}

void IRAM_ATTR waterleak_handleInterrupt() {
  if ((millis() - lastDebounceTime) > 50) {
    waterleak_interruptFlag = true;
    lastDebounceTime = millis();
  }
}

void ads_setting(void) {
  ADS.begin();
  ADS.setGain(0);      //  6.144 volt
  ADS.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
  ADS.setMode(0);      //  continuous mode
  ADS.readADC(0);      //  first read to trigger
}

void updatePWMVoltage(float voltage) {
  ledcWrite(pwmPin, voltage);
}

// =============================================================SENSORS=============================================================
#ifdef NO_SENSOR
bool readH2Sensor(int h2_val) {
  ADS.readADC(H2_SENSOR_ADS_CHANNEL);
  _ads_data.h2_data = ADS.getValue();
  // Serial.print("H2 Value: ");
  // Serial.println(h2_val);

  if(h2_val > 11000){
    updatePWMVoltage(0);
    return false;
  }
    
  return true;
}

bool readTDS(int tds_val) {
  ADS.readADC(TDS_ADS_CHANNEL);
  _ads_data.tds_data = ADS.getValue();

  // float f = ADS.toVoltage(1);
  // uint32_t tdsValue = (float)((_ads_data.tds_data * f) / _Volfor1000ppmTDS) * 1000;  // Convert _ads_data.tds_data -> tdsValue
  uint32_t tdsValue = tds_val;
  // Serial.print("TDS Value: ");
  // Serial.print(tdsValue);
  // Serial.println("ppm");

  if (tdsValue > 5) {
    // Serial.println("TDS sensor triggered: High TDS level detected.");
    // return false;
    
  }

  return true;
}
#else
bool readH2Sensor(void) {
  ADS.readADC(H2_SENSOR_ADS_CHANNEL);
  _ads_data.h2_data = ADS.getValue();
  Serial.print("H2 Value: ");
  Serial.println(_ads_data.h2_data);

  if(_ads_data.h2_data > 11000){
    updatePWMVoltage(0);
    return false;
  }
    
  return true;
}

bool readTDS(void) {
  ADS.readADC(TDS_ADS_CHANNEL);
  _ads_data.tds_data = ADS.getValue();

  // float f = ADS.toVoltage(1);
  // uint32_t tdsValue = (float)((_ads_data.tds_data * f) / _Volfor1000ppmTDS) * 1000;  // Convert _ads_data.tds_data -> tdsValue

  Serial.print("TDS Value: ");
  Serial.print(_ads_data.tds_data);
  Serial.println("ppm");

  if (_ads_data.tds_data > 5) {
    Serial.println("TDS sensor triggered: High TDS level detected.");
    return false;
    
  }

  return true;
}
#endif

#ifdef NO_SENSOR
void handleSerialCommands(void) {
  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();

    if (receivedData.startsWith("H2:")) {
      h2_percent = receivedData.substring(3);  // "H2:"
      Serial.printf("H2: %d\n", h2_percent.toInt());
    } else if (receivedData.startsWith("TDS:")) {
      tds_data = receivedData.substring(4);  // "TDS:"
      Serial.printf("TDS: %d\n", tds_data.toInt());
    }
  }
}
#endif

// =============================================================KNOB COMMUNICATION=============================================================
void connectedWiFi_cb(void) {
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

  server.on("/", std::bind(&OTA::handleRoot, &_ota)); // need to use std::bind(&Class::method, &object) for calling a method of a class in 
  server.on("/update", HTTP_POST, std::bind(&OTA::handleUpdate, &_ota), std::bind(&OTA::handleUpload, &_ota));
  server.begin();
  Serial.println("HTTP server started");

  _mqtt.MQTT_init(H2_MQTT_BROKER, H2_MQTT_PORT, H2_MQTT_USERNAME, H2_MQTT_PASSWORD);
  mqtt_publish_message(PARAMETER_REQUEST, MAC_id.c_str(), "{}");
  mqtt_subscribe(PARAMETER_CURRENT, MAC_id.c_str());
}

void handleKnobCommand(void) {
  // Keep reading data until a value greater than 0 is received
  do {
    if(_ota.otaState != _ota.OTA_IDLE)
      break;
    if (UART2.available()) {
        String receivedData = UART2.readStringUntil('\n');  // Read data from UART2
        Serial.println("Received data from Knob: ");
        Serial.println(receivedData);
        parseKnobData(receivedData.c_str());
        String jsonToSend = buildKnobDataJson(_knob2Board);
        Serial.println("JSON to send: " + jsonToSend);
        flow = _knob2Board.percent * 100;

        mqtt_publish_message(PARAMETER_UPDATE, MAC_id.c_str(), jsonToSend);

        _run = true;
      }
      int dutyValue;
      if (flow == 0)
      {
        updatePWMVoltage(0);
        // Serial.println("Flow is 0. Stopping PWM.");
        _run = false;
        if (h2 && tds && !levelfloat_interruptFlag && !limitsw_interruptFlag && !waterleak_interruptFlag) { // Khi hết lỗi -> knob yêu cầu pwm trở về 0 -> cần gửi lại lệnh reset
          UART2.println("X");
          if (!clearErrorFlag) {
            mqtt_publish_message(ERROR_CLEAR, MAC_id.c_str(), "{}");
            _errorStatus.hepaFilterMissing = 0;
            _errorStatus.waterLeakDetected = 0;
            _errorStatus.floatDetected = 0;
            _errorStatus.poorWaterQuality = 0;
            _errorStatus.h2LeakDetected = 0;
            clearErrorFlag = true;
          }
            
        }
      }
      dutyValue = map(flow, 10, 400, 550, 7976);
      ledcWrite(pwmPin, dutyValue);
    delay(400);
  } while (flow == 0 );  // Continue waiting if flow is 0
}

void send_error_to_Knob(ErrorCode error) {
  switch (error) {
    case HEPA_FILTER_MISSING:
      _errorStatus.hepaFilterMissing = 1;
      clearErrorFlag = false;
      UART2.println("HEPA filter missing");
#ifdef DEBUG
      Serial.println("HEPA filter missing-");
#endif
      break;
    case WATER_LEAK_DETECTED:
      _errorStatus.waterLeakDetected = 1;
      clearErrorFlag = false;
      UART2.println("Water leak found");
#ifdef DEBUG
      Serial.println("Water leak found-");
#endif
      break;
    case FLOAT_DETECTED:
      _errorStatus.floatDetected = 1;
      clearErrorFlag = false;
      UART2.println("Water level low");
#ifdef DEBUG
      Serial.println("Water level low-");
#endif
      break;
    case POOR_WATER_QUALITY:
      _errorStatus.poorWaterQuality = 1;
      clearErrorFlag = false;
      UART2.println("Water quality is low");
#ifdef DEBUG
      Serial.println("Water quality is low-");
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
void Serial_Handle_task(void *param) {
  for (;;) {
    if(_ota.OTA_IDLE == _ota.otaState)
    {
#ifdef NO_SENSOR
      handleSerialCommands();
#endif
      handleKnobCommand();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void warning_Handle_Task(void *param) {
  for (;;) {
    if(_ota.OTA_IDLE == _ota.otaState)
    {
      if (limitsw_interruptFlag || digitalRead(LIMIT_SW_PIN) == LOW) {
        if (!digitalRead(LIMIT_SW_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(HEPA_FILTER_MISSING);
          limitsw_interruptFlag = true;
          delay(500);
        } else {
          limitsw_interruptFlag = false;
          _errorStatus.hepaFilterMissing = 0;
        }
      }

      if(waterleak_interruptFlag || digitalRead(WATER_LEAK_PIN) == HIGH) {
        delay(50);
        if (digitalRead(WATER_LEAK_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(WATER_LEAK_DETECTED);
          waterleak_interruptFlag = true;
          delay(500);
        } else {
          waterleak_interruptFlag = false;
          _errorStatus.waterLeakDetected = 0;
        }
      }

      if (levelfloat_interruptFlag || digitalRead(LEVEL_FLOAT_PIN) == LOW) {
        if (!digitalRead(LEVEL_FLOAT_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(FLOAT_DETECTED);
          levelfloat_interruptFlag = true;
          delay(500);
        }
        else {
          levelfloat_interruptFlag = false;
          _errorStatus.floatDetected = 0;
        }
      }

      if (millis() - lastReadTimeSensor > SENSOR_READ_TIME) {
        lastReadTimeSensor = millis();
#ifdef NO_SENSOR
        h2 = readH2Sensor(h2_percent.toInt());
        tds = readTDS(tds_data.toInt());
#else
        h2 = readH2Sensor();
        tds = readTDS();
#endif
        if (!tds) {
          send_error_to_Knob(POOR_WATER_QUALITY);
          delay(500);
        }
        else {
          _errorStatus.poorWaterQuality = 0;
        }
        if (!h2) {
          send_error_to_Knob(H2_LEAK_DETECTED);
          delay(500);
        }
        else {
          _errorStatus.h2LeakDetected = 0;
        }

        if (_errorStatus.hepaFilterMissing || _errorStatus.waterLeakDetected || _errorStatus.floatDetected || _errorStatus.poorWaterQuality || _errorStatus.h2LeakDetected) {
          String errorJSON = buildErrorJson(_errorStatus);
          mqtt_publish_message(ERROR_UPDATE, MAC_id.c_str(), errorJSON);
        }
        
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void PumpControlTask(void *param) {
  for (;;) {
    if (_ota.otaState == _ota.OTA_IDLE) {
      // Điều khiển bơm nước và bơm không khí
      if (h2 && tds && !levelfloat_interruptFlag && !limitsw_interruptFlag && !waterleak_interruptFlag && _run && _knob2Board.state) {
        digitalWrite(WATER_PUMP_PIN, 1);
        digitalWrite(AIR_PUMP_PIN, 1);
        // maybe call POST function here
      } else {
        digitalWrite(WATER_PUMP_PIN, 0);
        digitalWrite(AIR_PUMP_PIN, 0);
        // maybe call POST function here
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void SendMQTTDataToKnobTask(void *param) {
  for (;;) {
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
void handleMQTTReceivedMessage(const JsonDocument &doc) {
  haveMessageViaMQTT = false;

  _board2Knob.state = doc["state"];

  const char* t = doc["timer"];
  if (t != nullptr) {
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
  if (sendMQTTDataToKnobTaskHandle != NULL) {
    xTaskNotifyGive(sendMQTTDataToKnobTaskHandle);
  }

}

void parseKnobData(const char* input) {
  // Find the ':' character to skip the prefix "Data From Knob:"
  const char* dataStart = strchr(input, ':');
  if (dataStart == NULL) {
    Serial.println("Invalid format: no ':' found");
    return;
  }

  dataStart++; // Move past the ':' character

  // Copy the substring after ':' into a local buffer (strtok modifies the string)
  char buffer[64];
  strncpy(buffer, dataStart, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-terminated string

  // Use strtok to split the string by the '|' delimiter
  char* token = strtok(buffer, "|");
  int index = 0;

  // Parse each token into the appropriate field in _knobData
  while (token != NULL) {
    switch (index) {
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
  if (index < 4) {
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

void mqtt_publish_message(int topic_id, const char *mac_id, String message) {
  char topic[MQTT_TOPIC_MAX_LEN];

  snprintf(topic, sizeof(topic), "device/%s/%s/%s", mac_id, mqtt_topics[topic_id].service, mqtt_topics[topic_id].action);

  if (message != "") {
    _mqtt.publishMessage(topic, message.c_str());
  }
  
  Serial.println(String(topic));
  
}

void mqtt_subscribe(int topic_id, const char *mac_id) {
  char topic[MQTT_TOPIC_MAX_LEN];

  snprintf(topic, sizeof(topic), "device/%s/%s/%s", mac_id, mqtt_topics[topic_id].service, mqtt_topics[topic_id].action);

  _mqtt.subscribeTopic(topic);
  Serial.println(String(topic));
}

String buildKnobDataJson(const Knob_data_t& data) {
  StaticJsonDocument<128> doc;

  doc["state"] = data.state;
  doc["timer"] = data.timer;
  doc["percent"] = data.percent;
  doc["type_machine"] = data.typeMachine;

  String jsonString;
  serializeJson(doc, jsonString);  // Convert to String
  return jsonString;
}

String buildErrorJson(const Error_status_t& errorStatus) {
  StaticJsonDocument<256> doc;
  doc["HEPA_FILTER_MISSING"] = errorStatus.hepaFilterMissing ? 1 : 0;
  doc["WATER_LEAK_DETECTED"] = errorStatus.waterLeakDetected ? 1 : 0;
  doc["FLOAT_DETECTED"] = errorStatus.floatDetected ? 1 : 0;
  doc["POOR_WATER_QUALITY"] = errorStatus.poorWaterQuality ? 1 : 0;
  doc["H2_ABOVE_1_PERCENT"] = errorStatus.h2LeakDetected ? 1 : 0;

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}