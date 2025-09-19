// #include <Arduino.h>
// #include <WiFi.h>
// #include <WebServer.h>
// #include <Update.h>
// #include <HardwareSerial.h>
// #include "ADS1X15.h"
// #include "driver/ledc.h"
// #include <Preferences.h>
// #include "POSTGET.h"

#include "main.h"

// #define VERSION "v12.6.1"

// WiFi credentials
const char* ap_ssid = "ESP32S3_setup";
const char* ap_password = "12345678";
Preferences preferences;

const char* configPage =
"<form action='/save' method='POST'>"
"SSID: <input type='text' name='ssid'><br>"
"Password: <input type='password' name='password'><br>"
"<input type='submit' value='Save'>"
"</form>";

// WebServer instance
WebServer server(80);

// UART for Knob communication
HardwareSerial UART2(1);
// #define UART1_RX_PIN GPIO_NUM_44
// #define UART1_TX_PIN GPIO_NUM_43

// OTA state
enum OTAState {
  OTA_IDLE,
  OTA_WAIT_READY,
  OTA_SENDING,
  OTA_END
};
OTAState otaState = OTA_IDLE;

// Global variable for target management
String currentTarget = "";

// Function prototypes
void handleRoot();
void handleUpdate();
void handleUpload();
void sendChunkToKnob(uint8_t* data, size_t len);
bool waitForKnobResponse(const char* expected, uint32_t timeout = 3000);


//==================================================================================================
// Pin definitions
// #define WATER_PUMP_PIN GPIO_NUM_4
// #define AIR_PUMP_PIN GPIO_NUM_5

// #define LIMIT_SW_PIN GPIO_NUM_6
// #define WATER_LEAK_PIN GPIO_NUM_15
// #define LEVEL_FLOAT_PIN GPIO_NUM_7

// #define I2C_SDA_PIN GPIO_NUM_17
// #define I2C_SCL_PIN GPIO_NUM_18
// #define DEBOUNCE_TIME_MS 10
// ADS1115 instance
ADS1115 ADS(0x48);

// ADS channels enumeration
typedef enum {
  PRESSURE_ADS_CHANNEL,
  TDS_ADS_CHANNEL,
  H2_SENSOR_ADS_CHANNEL,
  POTEN_ADS_CHANNEL
} ads_channel_t;

// Data structure for sensor readings
typedef struct
{
  uint32_t pressure_data;
  uint32_t tds_data;
  uint32_t h2_data;
  uint32_t poten_data;
} ads_data_t;

ads_data_t _ads_data;

typedef enum {
  HEPA_FILTER_MISSING = 0,
  WATER_LEAK_DETECTED,
  FLOAT_DETECTED,
  POOR_WATER_QUALITY,
  H2_LEAK_DETECTED
} ErrorCode;

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
      // Serial.println("HEPA filter missing-");
      break;
    case WATER_LEAK_DETECTED:
      UART2.println("Water leak found");
      // Serial.println("Water leak found-");
      break;
    case FLOAT_DETECTED:
      UART2.println("Water level low");
      // Serial.println("Water level low-");
      break;
    case POOR_WATER_QUALITY:
      UART2.println("Water quality is low");
      // Serial.println("Water quality is low-");
      break;
    case H2_LEAK_DETECTED:
      UART2.println("Hydrogen leak found");
      // Serial.println("Hydrogen leak found-");
      break;
  }
}

void ads_setting() {
  ADS.begin();
  ADS.setGain(0);      //  6.144 volt
  ADS.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
  ADS.setMode(0);      //  continuous mode
  ADS.readADC(0);      //  first read to trigger
}

void updatePWMVoltage(float voltage) {
  ledcWrite(pwmPin, voltage);
}

bool readH2Sensor() {
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

bool readTDS(uint32_t tds_val) {
  ADS.readADC(TDS_ADS_CHANNEL);
  _ads_data.tds_data = ADS.getValue();

  // float f = ADS.toVoltage(1);
  // uint32_t tdsValue = (float)((_ads_data.tds_data * f) / _Volfor1000ppmTDS) * 1000;  // Convert _ads_data.tds_data -> tdsValue
  // uint32_t tdsValue = tds_val;
  Serial.print("TDS Value: ");
  Serial.print(_ads_data.tds_data);
  Serial.println("ppm");

  if (_ads_data.tds_data > 5) {
    // Serial.println("TDS sensor triggered: High TDS level detected.");
    // return false;
    
  }

  return true;
}

void handleSerialCommands() {
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

// Note : If H2 = 0 meaning Pause/Timeout wait until Resume/Start
// float _percentH2 = 0;
int flow;
int selectedEquation;
float airFlowRate;
bool _run = false;
void handleKnobCommand() {
  // Keep reading data until a value greater than 0 is received
  do {
    if(otaState != OTA_IDLE)
      break;
    if (UART2.available()) {
        String receivedData = UART2.readStringUntil('\n');  // Read data from UART2
        Serial.print("Received data from Knob: ");
        Serial.println(receivedData);
        flow = receivedData.toInt();
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
      // else if (flow >= 10 && flow <= 20) //OK
      // {
      //   dutyValue = map(flow, 10, 20, 550, 935);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 20 && flow <= 30) //OK
      // {
      //   dutyValue = map(flow, 20, 30, 935, 1274);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 30 && flow <= 40) //OK
      // {
      //   dutyValue = map(flow, 30, 40, 1274, 1572);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 40 && flow <= 50) //OK
      // {
      //   dutyValue = map(flow, 40, 50, 1572, 1845);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 50 && flow <= 60) //OK
      // {
      //   dutyValue = map(flow, 50, 60, 1845, 2090);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 60 && flow <= 70) //OK
      // {
      //   dutyValue = map(flow, 60, 70, 2090, 2296);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 70 && flow <= 80) //OK
      // {
      //   dutyValue = map(flow, 70, 80, 2296, 2490);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 80 && flow <= 90) //OK
      // {
      //   dutyValue = map(flow, 80, 90, 2490, 2600);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 90 && flow <= 100) //OK
      // {
      //   dutyValue = map(flow, 90, 100, 2600, 2835);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 100 && flow <= 190) //OK
      // {
      //   dutyValue = map(flow, 100, 190, 2835, 3985);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 190 && flow <= 230) //OK
      // {
      //   dutyValue = map(flow, 200, 230, 4250, 5423);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 230 && flow <= 260) //OK
      // {
      //   dutyValue = map(flow, 240, 260, 5990, 7350);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 260 && flow <= 290) //OK
      // {
      //   dutyValue = map(flow, 270, 290, 7717, 7778);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 290 && flow <= 310) //OK
      // {
      //   dutyValue = map(flow, 300, 310, 7797, 7814);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow == 320) //OK
      // {
      //   dutyValue = 7827;
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow > 320 && flow <= 390) //OK
      // {
      //   dutyValue = map(flow, 330, 390, 7828, 7954);
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else if (flow == 400) //OK
      // {
      //   dutyValue = 7976;
      //   ledcWrite(pwmPin, dutyValue);
      //   Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      // }
      // else
      // {
      //   Serial.println("Invalid flow value! Enter a number between 10 and 400.");
      // }
      dutyValue = map(flow, 10, 400, 550, 7976);
      ledcWrite(pwmPin, dutyValue);
    delay(1000);
  } while (flow == 0 );  // Continue waiting if flow is 0
}

void Serial_Handle_task(void *param) {
  for (;;) {
    if(OTA_IDLE == otaState)
    {
      // Serial.println("Serial task");
      handleSerialCommands();
      handleKnobCommand();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

  }
}

void warning_Handle_Task(void *param) {
  for (;;) {
    if(OTA_IDLE == otaState)
    {
      if (limitsw_interruptFlag || digitalRead(LIMIT_SW_PIN) == LOW) {
        if (!digitalRead(LIMIT_SW_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(HEPA_FILTER_MISSING);
          limitsw_interruptFlag = true;
          delay(1000);
        } else {
          limitsw_interruptFlag = false;
        }
      }

      if(waterleak_interruptFlag || digitalRead(WATER_LEAK_PIN) == HIGH) {
        delay(50);
        if (digitalRead(WATER_LEAK_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(WATER_LEAK_DETECTED);
          waterleak_interruptFlag = true;
          delay(1000);
        } else {
          waterleak_interruptFlag = false;
        }
      }

      if (levelfloat_interruptFlag || digitalRead(LEVEL_FLOAT_PIN) == LOW) {
        if (!digitalRead(LEVEL_FLOAT_PIN)) {
            updatePWMVoltage(0);
            send_error_to_Knob(FLOAT_DETECTED);
            levelfloat_interruptFlag = true;
            delay(1000);
        }
        else {
          levelfloat_interruptFlag = false;
        }
      }

      if (millis() - lastReadTimeSensor > SENSOR_READ_TIME) {
        lastReadTimeSensor = millis();
        h2 = readH2Sensor();
        tds = readTDS(tds_data.toInt());

        if (!tds) {
          send_error_to_Knob(POOR_WATER_QUALITY);
          delay(1000);
        }
        if (!h2) {
          send_error_to_Knob(H2_LEAK_DETECTED);
          delay(1000);
        }

        // bật khi các cảm biến không lỗi và pwm H2 > 0 
        if (h2 && tds && !levelfloat_interruptFlag && !limitsw_interruptFlag && !waterleak_interruptFlag && _run) {
          digitalWrite(WATER_PUMP_PIN, 1);
          digitalWrite(AIR_PUMP_PIN, 1);

          // maybe call POST function here
        } else {
          digitalWrite(WATER_PUMP_PIN, 0);
          digitalWrite(AIR_PUMP_PIN, 0);

          // maybe call POST function here
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
//==================================================================================================

void setup() {
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
      server.on("/", handleRoot);

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
  server.on("/update", HTTP_POST, handleUpdate, handleUpload);
  server.begin();
  Serial.println("HTTP server started");

  xTaskCreate(Serial_Handle_task, "Serial_Handle_task", 4096, NULL, 6, &serialTaskHandle);
  xTaskCreate(warning_Handle_Task, "warning_Handle_Task", 8192, NULL, 6, &warningTaskHandle);
}

void loop() {
  delay(1);
  server.handleClient();
  // Handle OTA state for Knob via UART responses
  if (otaState == OTA_WAIT_READY || otaState == OTA_SENDING || otaState == OTA_END) {
    if (UART2.available()) {
      String response = UART2.readStringUntil('\n');
      response.trim();
      if (response == "READY" && otaState == OTA_WAIT_READY) {
        otaState = OTA_SENDING;
        Serial.println("Knob is ready for OTA");
      } 
      else if (response == "OTA_SUCCESS" && otaState == OTA_END) {
        otaState = OTA_IDLE;
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
        otaState = OTA_IDLE;
      }
    }
  }
}

void handleConfigWifi() {
  const char* configPage =
  "<form action='/save' method='POST'>"
  "SSID: <input type='text' name='ssid'><br>"
  "Password: <input type='password' name='password'><br>"
  "<input type='submit' value='Save'>"
  "</form>";
  server.send(200, "text/html", configPage);
}

// Handle root route: gửi HTML form
void handleRoot() {
  const char* otaPage = R"(
  <!DOCTYPE html>
  <html>
  <head>
    <title>OTA Update</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 20px; }
      h1 { color: #333; }
      form { margin-top: 20px; }
      label { display: block; margin: 10px 0 5px; }
      input[type="file"] { margin-bottom: 10px; }
      input[type="submit"] { background: #007bff; color: #fff; border: none; padding: 10px 20px; cursor: pointer; }
      input[type="submit"]:hover { background: #0056b3; }
    </style>
  </head>
  <body>
    <h1>OTA Update</h1>
    <form id="otaForm" method="POST" action="/update" enctype="multipart/form-data">
      <label for="update">Select firmware file:</label>
      <input type="file" name="update" id="update" required><br><br>
      <label>Select target:</label>
      <input type="radio" id="controller" name="target" value="Controller" checked>
      <label for="controller">Controller (ESP32)</label><br>
      <input type="radio" id="knob" name="target" value="Knob">
      <label for="knob">Knob (ESP32-C3 via UART)</label><br><br>
      <input type="submit" value="Update">
    </form>
    <script>
      document.getElementById("otaForm").onsubmit = function() {
        var radios = document.getElementsByName("target");
        var selectedValue = "";
        for (var i = 0; i < radios.length; i++){
          if (radios[i].checked) {
            selectedValue = radios[i].value;
            break;
          }
        }
        this.action = "/update?target=" + selectedValue;
      };
    </script>
  </body>
  </html>
  )";
  server.send(200, "text/html", otaPage);
}

// Callback chính sau khi upload hoàn tất: lấy target từ query parameter
void handleUpdate() {
  currentTarget = server.arg("target");
  Serial.print("Selected target (from handleUpdate): ");
  Serial.println(currentTarget);

  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");

  // Nếu target là Controller thì restart ngay, nếu là Knob thì chờ xác nhận qua UART
  if (currentTarget == "Controller") {
    UART2.println("RESET");
    delay(500);
    ESP.restart();
  }
}

String filterResponse(String response) {
  String cleaned = "";
  for (unsigned int i = 0; i < response.length(); i++) {
    char c = response.charAt(i);
    if (c >= 32 && c <= 126) { // chỉ giữ các ký tự in được
      cleaned += c;
    }
  }
  return cleaned;
}
// Callback xử lý file upload
void handleUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    currentTarget = server.arg("target");
    Serial.print("Starting upload for target: ");
    Serial.println(currentTarget);

    if (currentTarget == "Knob") {
      UART2.println("OTA_START");
      UART2.println("OTA_START");
      otaState = OTA_WAIT_READY;
      Serial.println("Waiting for Knob to be ready...");

      // Thêm phần đợi READY với timeout
      unsigned long start = millis();
      while (otaState == OTA_WAIT_READY && millis() - start < 10000) {
        if (UART2.available()) {
          String response = UART2.readStringUntil('\n');
          response = filterResponse(response);
          response.trim();
          Serial.println(response);
          if (response.indexOf("READY") >= 0) {
            otaState = OTA_SENDING;
            Serial.println("Knob is ready!");
            break;
          } else if (response.startsWith("ERR")) {
            Serial.println("Knob error: " + response);
            server.send(500, "text/plain", "Knob initialization failed");
            return;
          }
        }
        delay(10);
      }

      if (otaState != OTA_SENDING) {
        Serial.println("Knob not ready, aborting OTA");
        server.send(500, "text/plain", "Knob not responding");
        return;
      }
    } else {
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.println("Failed to start OTA on Controller!");
        server.send(500, "text/plain", "Failed to start OTA on Controller!");
        return;
      }
    }
  } 
  else if (upload.status == UPLOAD_FILE_WRITE) {
    if (currentTarget == "Knob") {
      Serial.println("Knob is sending!");
      sendChunkToKnob(upload.buf, upload.currentSize);
    } else {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Serial.println("Failed to write firmware chunk!");
        server.send(500, "text/plain", "Failed to write firmware chunk!");
        return;
      }
    }
  } 
  else if (upload.status == UPLOAD_FILE_END) {
    if (currentTarget == "Knob") {
      UART2.print("OTA_END\n");
      otaState = OTA_END;
      Serial.println("Waiting for Knob to finish OTA...");
    } else {
      if (Update.end(true)) {
        Serial.println("Controller OTA successful!");
      } else {
        Serial.println("Failed to end OTA on Controller!");
        server.send(500, "text/plain", "Failed to end OTA on Controller!");
        return;
      }
    }
  }
}

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xEDB88320;
      else
        crc = crc >> 1;
    }
  }
  return crc ^ 0xFFFFFFFF;
}

// #define MAGIC_MARKER 0xAABBCCDD

void sendChunkToKnob(uint8_t* data, size_t len) {
  uint32_t marker = MAGIC_MARKER;
  uint32_t crc = crc32(data, len);
  
  // Gửi marker, kích thước và CRC (mỗi giá trị 4 byte)
  UART2.flush(); 
  UART2.write((uint8_t*)&marker, sizeof(marker));
  UART2.write((uint8_t*)&len, sizeof(len));
  UART2.write((uint8_t*)&crc, sizeof(crc));
  UART2.flush(); // Đảm bảo marker và header đã được gửi hết
  // Gửi dữ liệu firmware
  UART2.write(data, len);
  
  if (!waitForKnobResponse("OK", 3000)) {
    Serial.println("Failed to send chunk to Knob!");
    server.send(500, "text/plain", "Failed to send chunk to Knob!");
    return;
  }
}

// Hàm chờ phản hồi từ Knob qua UART
bool waitForKnobResponse(const char* expected, uint32_t timeout) {
  uint32_t start = millis();
  while (millis() - start < timeout) {
    if (UART2.available()) {
      String response = UART2.readStringUntil('\n');
      response.trim();
      if (response == expected) {
        return true;
      }
    }
    delay(10);
  }
  return false;
}