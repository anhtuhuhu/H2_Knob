#include <Arduino.h>
#include "driver/ledc.h"
#include "ADS1X15.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>

const char* version = "v26.5.1";
Preferences preferences;
WebServer server(80);

const char* ap_ssid = "ESP32_Setup";
const char* ap_password = "12345678";

const char* configPage =
"<form action='/save' method='POST'>"
"SSID: <input type='text' name='ssid'><br>"
"Password: <input type='password' name='password'><br>"
"<input type='submit' value='Save'>"
"</form>";

const char* otaPage = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
"<input type='file' name='update'>"
"<input type='submit' value='Update'>"
"</form>"
"<div id='prg'>progress: 0%</div>"
"<script>"
"$('form').submit(function(e){"
"e.preventDefault();"
"var form = $('#upload_form')[0];"
"var data = new FormData(form);"
" $.ajax({"
"url: '/update',"
"type: 'POST',"
"data: data,"
"contentType: false,"
"processData:false,"
"xhr: function() {"
"var xhr = new window.XMLHttpRequest();"
"xhr.upload.addEventListener('progress', function(evt) {"
"if (evt.lengthComputable) {"
"var per = evt.loaded / evt.total;"
"$('#prg').html('progress: ' + Math.round(per*100) + '%');"
"}"
"}, false);"
"return xhr;"
"},"
"success:function(d, s) {"
"console.log('success!')" 
"},"
"error: function (a, b, c) {}"
"});"
"});"
"</script>";

// Pin definitions
#define WATER_PUMP_PIN GPIO_NUM_4
#define AIR_PUMP_PIN GPIO_NUM_5

#define LIMIT_SW_PIN GPIO_NUM_6
#define WATER_LEAK_PIN GPIO_NUM_15
#define LEVEL_FLOAT_PIN GPIO_NUM_7

#define I2C_SDA_PIN GPIO_NUM_17
#define I2C_SCL_PIN GPIO_NUM_18

HardwareSerial UART2(1);
#define UART1_RX_PIN GPIO_NUM_44
#define UART1_TX_PIN GPIO_NUM_43

#define DEBOUNCE_TIME_MS 10

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
  H2_ABOVE_1_PERCENT
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

#define SENSOR_READ_TIME 1000  // 1s
uint32_t lastReadTimeSensor = 0;

String h2_percent;
String tds_data;
bool h2;
bool tds;

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
//-------------------------------------------------------------
void send_error_to_Knob(ErrorCode error) {
  switch (error) {
    case HEPA_FILTER_MISSING:
      UART2.println("No HEPA filter detected");
      Serial.println("No HEPA filter detected-");
      break;
    case WATER_LEAK_DETECTED:
      UART2.println("Water Leak Detected");
      Serial.println("Water Leak Detected");
      break;
    case FLOAT_DETECTED:
      UART2.println("Float Detected");
      Serial.println("Float Detected-");
      break;
    case POOR_WATER_QUALITY:
      UART2.println("Poor Water Quality");
      Serial.println("Poor Water Quality-");
      break;
    case H2_ABOVE_1_PERCENT:
      UART2.println("H2 leak detected");
      Serial.println("H2 is above 1%-");
      break;
  }
}
//-------------------------------------------------------------
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

bool readH2Sensor(uint32_t h2_val) {
  ADS.readADC(H2_SENSOR_ADS_CHANNEL);
  _ads_data.h2_data = ADS.getValue();
  float f = ADS.toVoltage(1);
  // uint32_t H2Value = (float)((_ads_data.h2_data * f) / _Volfor100PercentH2) * 100;
  uint32_t H2Value = h2_val;
  Serial.print("H2 Value: ");
  Serial.print(H2Value);
  Serial.println("%");
  if (H2Value > 1) {
    Serial.print("H2: ");
    Serial.println(H2Value);
    return false;
  } else {
    return true;
  }
}

bool readTDS(uint32_t tds_val) {
  ADS.readADC(TDS_ADS_CHANNEL);
  _ads_data.tds_data = ADS.getValue();

  float f = ADS.toVoltage(1);
  // uint32_t tdsValue = (float)((_ads_data.tds_data * f) / _Volfor1000ppmTDS) * 1000;  // Convert _ads_data.tds_data -> tdsValue
  uint32_t tdsValue = tds_val;
  Serial.print("TDS Value: ");
  Serial.print(tdsValue);
  Serial.println("ppm");

  if (tdsValue > 5) {
    Serial.println("TDS sensor triggered: High TDS level detected.");
    return false;
  } else {
    return true;
  }
}
//------------------------------------------
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
      else if (flow >= 10 && flow <= 20) //OK
      {
        dutyValue = map(flow, 10, 20, 550, 935);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 20 && flow <= 30) //OK
      {
        dutyValue = map(flow, 20, 30, 935, 1274);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 30 && flow <= 40) //OK
      {
        dutyValue = map(flow, 30, 40, 1274, 1572);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 40 && flow <= 50) //OK
      {
        dutyValue = map(flow, 40, 50, 1572, 1845);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 50 && flow <= 60) //OK
      {
        dutyValue = map(flow, 50, 60, 1845, 2090);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 60 && flow <= 70) //OK
      {
        dutyValue = map(flow, 60, 70, 2090, 2296);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 70 && flow <= 80) //OK
      {
        dutyValue = map(flow, 70, 80, 2296, 2490);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 80 && flow <= 90) //OK
      {
        dutyValue = map(flow, 80, 90, 2490, 2600);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 90 && flow <= 100) //OK
      {
        dutyValue = map(flow, 90, 100, 2600, 2835);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 100 && flow <= 190) //OK
      {
        dutyValue = map(flow, 100, 190, 2835, 3985);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 190 && flow <= 230) //OK
      {
        dutyValue = map(flow, 200, 230, 4250, 5423);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 230 && flow <= 260) //OK
      {
        dutyValue = map(flow, 240, 260, 5990, 7350);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 260 && flow <= 290) //OK
      {
        dutyValue = map(flow, 270, 290, 7717, 7778);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 290 && flow <= 310) //OK
      {
        dutyValue = map(flow, 300, 310, 7797, 7814);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow == 320) //OK
      {
        dutyValue = 7827;
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 320 && flow <= 390) //OK
      {
        dutyValue = map(flow, 330, 390, 7828, 7954);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow == 400) //OK
      {
        dutyValue = 7976;
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else
      {
        Serial.println("Invalid flow value! Enter a number between 10 and 400.");
      }
    delay(1000);
  } while (flow == 0);  // Continue waiting if flow is 0
}


void OTA_task(void *param) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Serial_Handle_task(void *param) {
  for (;;) {
    // Serial.println("Serial task");
    handleSerialCommands();
    handleKnobCommand();

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void loop_task(void *param) {
  for (;;) {
    if (limitsw_interruptFlag) {
      if (!digitalRead(LIMIT_SW_PIN)) {
        updatePWMVoltage(0);
        send_error_to_Knob(HEPA_FILTER_MISSING);
        delay(1000);
      } else {
        limitsw_interruptFlag = false;
      }
    }

    if(waterleak_interruptFlag) {
      delay(50);
      if (digitalRead(WATER_LEAK_PIN)) {
        updatePWMVoltage(0);
        send_error_to_Knob(WATER_LEAK_DETECTED);
        delay(1000);
      } else {
        waterleak_interruptFlag = false;
      }
    }

    if (levelfloat_interruptFlag) {
      if (!digitalRead(LEVEL_FLOAT_PIN)) {
          updatePWMVoltage(0);
          send_error_to_Knob(FLOAT_DETECTED);
          delay(1000);
      }
      else {
        levelfloat_interruptFlag = false;
      }
    }

    if (millis() - lastReadTimeSensor > SENSOR_READ_TIME) {
      lastReadTimeSensor = millis();
      h2 = readH2Sensor(h2_percent.toInt());
      tds = readTDS(tds_data.toInt());

      if (!tds) {
        send_error_to_Knob(POOR_WATER_QUALITY);
        delay(1000);
      }
      if (!h2) {
        send_error_to_Knob(H2_ABOVE_1_PERCENT);
        delay(1000);
      }

      // bật khi các cảm biến không lỗi và pwm H2 > 0 
      if (h2 && tds && !levelfloat_interruptFlag && !limitsw_interruptFlag && !waterleak_interruptFlag && _run) {
        digitalWrite(WATER_PUMP_PIN, 1);
        digitalWrite(AIR_PUMP_PIN, 1);
      } else {
        digitalWrite(WATER_PUMP_PIN, 0);
        digitalWrite(AIR_PUMP_PIN, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
//------------------------------------------
void setup() {
  Serial.begin(115200);
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

  UART2.begin(9600, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
  ads_setting();
  Serial.print("Version");
  Serial.println(version);
  UART2.println(version);

  _Volfor100PercentH2 = ADS.getMaxVoltage();  // Need to change if max voltage is not 100% H2
  _Volfor1000ppmTDS = ADS.getMaxVoltage();    // Need to change if max voltage is not 1000ppm TDS

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
    } else {
      Serial.println("\nWiFi connection failed, switching to AP mode...");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.softAP(ap_ssid, ap_password);
    Serial.println("Access Point started. Connect to 'ESP32_Setup' and go to 192.168.4.1");
  }

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", WiFi.status() == WL_CONNECTED ? otaPage : configPage);
  });

  server.on("/save", HTTP_POST, []() {
    String new_ssid = server.arg("ssid");
    String new_password = server.arg("password");
    if (new_ssid.length() > 0 && new_password.length() > 0) {
      preferences.putString("ssid", new_ssid);
      preferences.putString("password", new_password);
      server.send(200, "text/html", "Saved! Restarting ESP32...");
      delay(1000);
      ESP.restart();
    } else {
      server.send(400, "text/html", "Invalid input. Try again.");
    }
  });

  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
  xTaskCreate(OTA_task, "OTA_task", 4096, NULL, 5, NULL);
  xTaskCreate(Serial_Handle_task, "Serial_Handle_task", 4096, NULL, 5, NULL);
  xTaskCreate(loop_task, "loop_task", 8192, NULL, 6, NULL);

}

void loop() {
  delay(1);
}

/*
Vừa khởi động xong: Relay tắt, PWM = 0
Khi hoạt động: PWM ứng với Flow (Từ Knob), Relay sáng 
Có lỗi: Knob hiện lỗi, Relay tắt, PWM = 0
Hết lỗi: Knob về main, Relay tắt, PWM = 0

*/
