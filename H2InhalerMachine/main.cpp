#include "projdefs.h"
#include "main.h"

OTA _ota;                                     // OTA object
POSTGET _POSTGET;                             // POSTGET object
Preferences preferences;
// WebServer instance
WebServer server(80);

extern const char* ap_ssid;
extern const char* ap_password;
// UART for Knob communication
HardwareSerial UART2(1);

//==================================================================================================
ADS1115 ADS(0x48);

ads_data_t _ads_data;

// Interrupt flags
volatile bool limitsw_interruptFlag = false;
volatile bool levelfloat_interruptFlag = false;
volatile bool waterleak_interruptFlag = false;

const int pwmPin = 16;        // PWM pin
const int pwmFreq = 4000;     // PWM frequency
const int pwmResolution = 13; // 13 bit
const int pwmChannel = 1;     // PWM channel

float _Volfor100PercentH2 = 0;
float _Volfor1000ppmTDS = 0;  // max sensor is 1000ppm

// #define SENSOR_READ_TIME 1000  // 1s
uint32_t lastReadTimeSensor = 0;

String h2_percent;
String tds_data;
bool h2;
bool tds;

// Note : If H2 = 0 meaning Pause/Timeout wait until Resume/Start
// float _percentH2 = 0;
int flow;
String timer;
float airFlowRate;
bool _run = false;
static SemaphoreHandle_t sensorDataMutex = NULL; // Semaphore để bảo vệ dữ liệu cảm biến

TaskHandle_t serialTaskHandle = NULL;
TaskHandle_t warningTaskHandle = NULL;

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

volatile unsigned long lastDebounceTime = 0;
void IRAM_ATTR waterleak_handleInterrupt() {
  if ((millis() - lastDebounceTime) > 50) {
    waterleak_interruptFlag = true;
    lastDebounceTime = millis();
  }
}

void send_error_to_Knob(ErrorCode error) {
  switch (error) {
    case HEPA_FILTER_MISSING:
      UART2.println("HEPA filter missing");
#ifdef DEBUG
      Serial.println("HEPA filter missing-");
#endif
      break;
    case WATER_LEAK_DETECTED:
      UART2.println("Water leak found");
#ifdef DEBUG
      Serial.println("Water leak found-");
#endif
      break;
    case FLOAT_DETECTED:
      UART2.println("Water level low");
#ifdef DEBUG
      Serial.println("Water level low-");
#endif
      break;
    case POOR_WATER_QUALITY:
      UART2.println("Water quality is low");
#ifdef DEBUG
      Serial.println("Water quality is low-");
#endif
      break;
    case H2_LEAK_DETECTED:
      UART2.println("Hydrogen leak found");
#ifdef DEBUG
      Serial.println("Hydrogen leak found-");
#endif
      break;
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
#ifndef COCAMBIEN
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
    // Serial.println("TDS sensor triggered: High TDS level detected.");
    // return false;
    
  }

  return true;
}
#else
bool readH2Sensor(int h2_val) {
  ADS.readADC(H2_SENSOR_ADS_CHANNEL);
  _ads_data.h2_data = ADS.getValue();
  Serial.print("H2 Value: ");
  Serial.println(h2_val);

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
  Serial.print("TDS Value: ");
  Serial.print(tdsValue);
  Serial.println("ppm");

  if (tdsValue > 5) {
    // Serial.println("TDS sensor triggered: High TDS level detected.");
    // return false;
    
  }

  return true;
}
#endif
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


void handleKnobCommand(void) {
  // Keep reading data until a value greater than 0 is received
  do {
    if(_ota.otaState != _ota.OTA_IDLE)
      break;
    if (UART2.available()) {
        String receivedData = UART2.readStringUntil('\n');  // Read data from UART2
        Serial.print("Received data from Knob: ");
        Serial.println(receivedData);
        if (receivedData.indexOf("H2_Percent:") != -1) {
          int len = String("H2_Percent:").length();
          String flowData = receivedData.substring(len);
          flow = flowData.toInt();
        }
        if (receivedData.indexOf("Timer:") != -1) {
          int len = String("Timer:").length();
          String timerData = receivedData.substring(len);
          timer = String(timerData);
        }
        
        _run = true;
      }
      int dutyValue;
      if (flow == 0)
      {
        updatePWMVoltage(0);
        Serial.println("Flow is 0. Stopping PWM.");
        _run = false;
        if (h2 && tds && !levelfloat_interruptFlag && !limitsw_interruptFlag && !waterleak_interruptFlag) // Khi hết lỗi -> knob yêu cầu pwm trở về 0 -> cần gửi lại lệnh reset
          UART2.println("X");
      }
      dutyValue = map(flow, 10, 400, 550, 7976);
      ledcWrite(pwmPin, dutyValue);
    delay(1000);
  } while (flow == 0 );  // Continue waiting if flow is 0
}

void Serial_Handle_task(void *param) {
  for (;;) {
    if(_ota.OTA_IDLE == _ota.otaState)
    {
      handleSerialCommands();
      handleKnobCommand();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

  }
}

// void warning_Handle_Task(void *param) {
//   for (;;) {
//     if(_ota.OTA_IDLE == _ota.otaState)
//     {
//       if (limitsw_interruptFlag || digitalRead(LIMIT_SW_PIN) == LOW) {
//         if (!digitalRead(LIMIT_SW_PIN)) {
//           updatePWMVoltage(0);
//           send_error_to_Knob(HEPA_FILTER_MISSING);
//           limitsw_interruptFlag = true;
//           delay(1000);
//         } else {
//           limitsw_interruptFlag = false;
//         }
//       }

//       if(waterleak_interruptFlag || digitalRead(WATER_LEAK_PIN) == HIGH) {
//         delay(50);
//         if (digitalRead(WATER_LEAK_PIN)) {
//           updatePWMVoltage(0);
//           send_error_to_Knob(WATER_LEAK_DETECTED);
//           waterleak_interruptFlag = true;
//           delay(1000);
//         } else {
//           waterleak_interruptFlag = false;
//         }
//       }

//       if (levelfloat_interruptFlag || digitalRead(LEVEL_FLOAT_PIN) == LOW) {
//         if (!digitalRead(LEVEL_FLOAT_PIN)) {
//             updatePWMVoltage(0);
//             send_error_to_Knob(FLOAT_DETECTED);
//             levelfloat_interruptFlag = true;
//             delay(1000);
//         }
//         else {
//           levelfloat_interruptFlag = false;
//         }
//       }

//       if (millis() - lastReadTimeSensor > SENSOR_READ_TIME) {
//         lastReadTimeSensor = millis();
// #ifndef COCAMBIEN
//         h2 = readH2Sensor();
//         tds = readTDS();
// #else
//         h2 = readH2Sensor(h2_percent.toInt());
//         tds = readTDS(tds_data.toInt());
// #endif
//         if (!tds) {
//           send_error_to_Knob(POOR_WATER_QUALITY);
//           delay(1000);
//         }
//         if (!h2) {
//           send_error_to_Knob(H2_LEAK_DETECTED);
//           delay(1000);
//         }

//         // bật khi các cảm biến không lỗi và pwm H2 > 0 
//         if (h2 && tds && !levelfloat_interruptFlag && !limitsw_interruptFlag && !waterleak_interruptFlag && _run) {
//           digitalWrite(WATER_PUMP_PIN, 1);
//           digitalWrite(AIR_PUMP_PIN, 1);

//           // maybe call POST function here
//         } else {
//           digitalWrite(WATER_PUMP_PIN, 0);
//           digitalWrite(AIR_PUMP_PIN, 0);

//           // maybe call POST function here
//         }
//       }
//     }
//     vTaskDelay(pdMS_TO_TICKS(1));
//   }
// }

void warning_Handle_Task(void *param) {
  unsigned long lastErrorTime = 0; // Thời điểm lần cuối phát hiện lỗi
  const unsigned long errorInterval = 1000; // Khoảng thời gian giữa các lần phát hiện lỗi (1 giây)

  for (;;) {
    if (_ota.OTA_IDLE == _ota.otaState) {
      unsigned long currentTime = millis();

      // Kiểm tra lỗi HEPA_FILTER_MISSING
      if ((limitsw_interruptFlag || digitalRead(LIMIT_SW_PIN) == LOW) && (currentTime - lastErrorTime >= errorInterval)) {
        if (!digitalRead(LIMIT_SW_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(HEPA_FILTER_MISSING);
          limitsw_interruptFlag = true;
          lastErrorTime = currentTime;
        } else {
          limitsw_interruptFlag = false;
        }
      }

      // Kiểm tra lỗi WATER_LEAK_DETECTED
      if ((waterleak_interruptFlag || digitalRead(WATER_LEAK_PIN) == HIGH) && (currentTime - lastErrorTime >= errorInterval)) {
        if (digitalRead(WATER_LEAK_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(WATER_LEAK_DETECTED);
          waterleak_interruptFlag = true;
          lastErrorTime = currentTime;
        } else {
          waterleak_interruptFlag = false;
        }
      }

      // Kiểm tra lỗi FLOAT_DETECTED
      if ((levelfloat_interruptFlag || digitalRead(LEVEL_FLOAT_PIN) == LOW) && (currentTime - lastErrorTime >= errorInterval)) {
        if (!digitalRead(LEVEL_FLOAT_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(FLOAT_DETECTED);
          levelfloat_interruptFlag = true;
          lastErrorTime = currentTime;
        } else {
          levelfloat_interruptFlag = false;
        }
      }

      // Đọc cảm biến định kỳ
      if (currentTime - lastReadTimeSensor > SENSOR_READ_TIME) {
        lastReadTimeSensor = currentTime;
#ifndef COCAMBIEN
        h2 = readH2Sensor();
        tds = readTDS();
#else
        h2 = readH2Sensor(h2_percent.toInt());
        tds = readTDS(tds_data.toInt());
#endif
        if (!tds) {
          send_error_to_Knob(POOR_WATER_QUALITY);
          lastErrorTime = currentTime; // Cập nhật thời gian lỗi
        }
        if (!h2) {
          send_error_to_Knob(H2_LEAK_DETECTED);
          lastErrorTime = currentTime; // Cập nhật thời gian lỗi
        }
        xSemaphoreGive(sensorDataMutex);
        
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // Nhường CPU cho các task khác
  }
}

void PumpControlTask(void *param) {
  for (;;) {
    if (_ota.otaState == _ota.OTA_IDLE) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Điều khiển bơm nước và bơm không khí
        if (h2 && tds && !levelfloat_interruptFlag && !limitsw_interruptFlag && !waterleak_interruptFlag && _run) {
          digitalWrite(WATER_PUMP_PIN, 1);
          digitalWrite(AIR_PUMP_PIN, 1);
          // maybe call POST function here
        } else {
          digitalWrite(WATER_PUMP_PIN, 0);
          digitalWrite(AIR_PUMP_PIN, 0);
          // maybe call POST function here
        }
        xSemaphoreGive(sensorDataMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void handleConfigWifi(void) {
  const char* configPage =
  "<form action='/save' method='POST'>"
  "SSID: <input type='text' name='ssid'><br>"
  "Password: <input type='password' name='password'><br>"
  "<input type='submit' value='Save'>"
  "</form>";
  server.send(200, "text/html", configPage);
}

void Setup() {
  Serial.begin(115200);
  UART2.begin(460800, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);

  delay(200);  

  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  updatePWMVoltage(0);

  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(AIR_PUMP_PIN, OUTPUT);

  pinMode(LIMIT_SW_PIN, INPUT);
  pinMode(WATER_LEAK_PIN, INPUT_PULLDOWN);
  pinMode(LEVEL_FLOAT_PIN, INPUT);

  digitalWrite(WATER_PUMP_PIN, 0);
  digitalWrite(AIR_PUMP_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limitsw_handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT_PIN), levelfloat_handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WATER_LEAK_PIN), waterleak_handleInterrupt, CHANGE);

  Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
  ads_setting();

  _Volfor100PercentH2 = ADS.getMaxVoltage();  // Need to change if max voltage is not 100% H2
  _Volfor1000ppmTDS = ADS.getMaxVoltage();    // Need to change if max voltage is not 1000ppm TDS

  // Connect to WiFi
  // WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi...");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("\nConnected to WiFi!");
  // while (WiFi.localIP() == INADDR_NONE) {
  //   Serial.println("Waiting for IP...");
  //   delay(500);
  // }

  preferences.begin("wifi_config", false);
  String stored_ssid = preferences.getString("ssid", "");
  String stored_password = preferences.getString("password", "");

  if (stored_ssid.length() > 0) {
    WiFi.begin(stored_ssid.c_str(), stored_password.c_str());
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to WiFi!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      // server.on("/", _ota.handleRoot());
      server.on("/", std::bind(&OTA::handleRoot, &_ota)); // need to use std::bind(&Class::method, &object) for calling a method of a class in 

      char ip_address[20];
      snprintf(ip_address, sizeof(ip_address), "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
      Serial.print("Received ip_address: ");
      Serial.println(ip_address);
      UART2.print("IP:" + String(ip_address)+"\n");
      delay(1000);
    } else {
      Serial.println("\nWiFi connection failed, switching to AP mode...");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.softAP(ap_ssid, ap_password);
    Serial.println("Access Point started. Connect to 'ESP32S3_Setup' and go to 192.168.4.1");
    server.on("/", handleConfigWifi);
  }

  server.on("/save", HTTP_POST, []() {
    String new_ssid = server.arg("ssid");
    String new_password = server.arg("password");
    if (new_ssid.length() > 0 && new_password.length() > 0) {
      preferences.putString("ssid", new_ssid);
      preferences.putString("password", new_password);
      Serial.print("SSID: ");
      Serial.println(new_ssid);
      Serial.print("PW: ");
      Serial.println(new_password);

      server.send(200, "text/html", "Saved! Restarting ESP32S3...");
      delay(1000);
      ESP.restart();
    } else {
      server.send(400, "text/html", "Invalid input. Try again.");
    }
  });

  delay(1000);
  UART2.print("\n");
  delay(1000);
  UART2.println(VERSION);
  delay(1000);
  char ip_address[20];
  snprintf(ip_address, sizeof(ip_address), "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  Serial.print("ip_address: ");
  Serial.println(ip_address);
  UART2.print("IP:" + String(ip_address)+"\n");
  delay(1000);

  // Configure WebServer routes
  // server.on("/", handleRoot);
  // Lấy target từ query parameter
  // server.on("/update", HTTP_POST, _ota.handleUpdate(), _ota.handleUpload());
  server.on("/update", HTTP_POST, std::bind(&OTA::handleUpdate, &_ota), std::bind(&OTA::handleUpload, &_ota));
  server.begin();
  Serial.println("HTTP server started");

  sensorDataMutex = xSemaphoreCreateMutex();
  if (sensorDataMutex == NULL) {
    // Xử lý lỗi (ví dụ: log lỗi)
    return;
  }

  xTaskCreate(Serial_Handle_task, "Serial_Handle_task", 4096, NULL, 6, &serialTaskHandle);
  xTaskCreate(warning_Handle_Task, "warning_Handle_Task", 4096, NULL, 6, &warningTaskHandle);
  xTaskCreate(PumpControlTask, "PumpControlTask", 4096, NULL, 6, NULL);
}

void Main() {
  delay(1);
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