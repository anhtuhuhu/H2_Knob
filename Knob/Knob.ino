#include <Arduino.h>
#include <ESP_Panel_Library.h>
#include <lvgl.h>
#include "lvgl_port_v8.h"
#include "driver/timer.h"
#include "nvs_flash.h"
#include "nvs.h"

//==================================================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>

const char* version = "v18.3.2";
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

//==================================================================================================
#ifdef KNOB21
#define GPIO_NUM_KNOB_PIN_A     6
#define GPIO_NUM_KNOB_PIN_B     5
#define GPIO_BUTTON_PIN         GPIO_NUM_0
#endif
#ifdef KNOB13
#define GPIO_NUM_KNOB_PIN_A     7
#define GPIO_NUM_KNOB_PIN_B     6
#define GPIO_BUTTON_PIN         GPIO_NUM_9
#endif

#ifdef KNOB
#include <ESP_Knob.h>
#include <Button.h>

ESP_Knob *knob;
#endif
#include <ui.h>
#include <HardwareSerial.h>
#define TIMER_MAX_LENGTH 10

HardwareSerial MySerial(1);

uint8_t savedDataToSend = false;
float percentH2FromKnob;
float airFlowRate = 10;   // 10L/min
char timerFromKnob[10] = "00:00:00";
bool isErrorScreenActive = false;
TaskHandle_t countdown_task_handle = NULL;
bool stop_countdown_flag = false;
uint8_t error_flag = false;
extern knob_state_t knob_state;
extern countdown_state_t countdown_state;
//=====================================================================

//=====================================================================

void onKnobLeftEventCallback(int count, void *usr_data)
{
    Serial.printf("Detect left event, count is %d\n", count);
    lvgl_port_lock(-1);
    LVGL_knob_event((void*)KNOB_LEFT);
    lvgl_port_unlock();
}

void onKnobRightEventCallback(int count, void *usr_data)
{
    Serial.printf("Detect right event, count is %d\n", count);
    lvgl_port_lock(-1);
    LVGL_knob_event((void*)KNOB_RIGHT);
    lvgl_port_unlock();
}

static void SingleClickCb(void *button_handle, void *usr_data) {
    Serial.println("Button Single Click");
    lvgl_port_lock(-1);
    LVGL_button_event((void*)BUTTON_SINGLE_CLICK);
    lvgl_port_unlock();
}

static void LongPressHoldCb(void *button_handle, void *usr_data) {
  lvgl_port_lock(-1);
  LVGL_button_event((void*)BUTTON_LONG_PRESS_HOLD);
  lvgl_port_unlock();
}

// String receivedString; // Variable that receive data via Serial

//==================================================Begin - NVS===================================================
void save_data_to_nvs(char *timer, float *_h2_percent) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  err = nvs_open("data", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    Serial.printf("Error opening NVS: %s\n", esp_err_to_name(err));
    return;
  }

  const char *timer_key = "timerKnob";
  const char *h2_percent_key = "h2Knob";

  err = nvs_set_str(nvs_handle, timer_key, timer);
  if (err == ESP_OK) {
    Serial.printf("Save timer: %s to NVS\n", timer);
  } else {
    Serial.printf("Error saving timer: %s\n", esp_err_to_name(err));
  }

  err = nvs_set_i32(nvs_handle, h2_percent_key, (int)(*_h2_percent * 100));
  if (err == ESP_OK) {
    Serial.printf("Save H2Percent: %f to NVS\n", *_h2_percent);
  } else {
    Serial.printf("Error saving H2Percent: %s\n", esp_err_to_name(err));
  }

  nvs_commit(nvs_handle);

  nvs_close(nvs_handle);

}

void get_data_from_nvs(char *timer, float *_h2_percent) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  err = nvs_open("data", NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    Serial.printf("Error opening NVS: %s\n", esp_err_to_name(err));
    return;
  }

  const char *timer_key = "timerKnob";
  const char *h2_percent_key = "h2Knob";
  size_t timer_len = TIMER_MAX_LENGTH;

  err = nvs_get_str(nvs_handle, timer_key, timer, &timer_len);
  if (err == ESP_OK) {
    Serial.printf("Read timer: %s\n", timer);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("Timer not found in NVS.");
    timer[0] = '\0'; // Set to empty if UUID not found
  } else {
    Serial.printf("Error reading timer: %s\n", esp_err_to_name(err));
    timer[0] = '\0'; // Set to empty if error occured
  }

  int32_t h2_int_value = 0;
  err = nvs_get_i32(nvs_handle, h2_percent_key, &h2_int_value);
  *_h2_percent = (float)(h2_int_value / 100.0f);

  if (err == ESP_OK) {
    Serial.printf("Read H2Percent: %f\n", *_h2_percent);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("H2Percent not found in NVS.");
    *_h2_percent = 0.0f;
  } else {
    Serial.printf("Error reading H2Percent: %s\n", esp_err_to_name(err));
    *_h2_percent = 0.0f;
  }

  nvs_close(nvs_handle);
}
//==================================================End - NVS===================================================

// Task đếm ngược thời gian
void countdown_task(void *pvParameters) {
    int hour = (timerFromKnob[0] - '0') * 10 + (timerFromKnob[1] - '0');
    int minute = (timerFromKnob[3] - '0') * 10 + (timerFromKnob[4] - '0');
    int second = (timerFromKnob[6] - '0') * 10 + (timerFromKnob[7] - '0');
    if (second != 0) second = 0;
    int total_seconds = hour * 3600 + minute * 60 + second;

    while (total_seconds > 0) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 giây
        if (stop_countdown_flag) break;
        total_seconds--;

        hour = total_seconds / 3600;
        minute = (total_seconds % 3600) / 60;
        second = total_seconds % 60;

        // Cập nhật giao diện
        lvgl_port_lock(-1);
        update_display(hour, minute, second);
        lvgl_port_unlock();
    }

    lvgl_port_lock(-1);
    delete_animation(arc1);
    delete_animation(arc2);
    lvgl_port_unlock();
    // Xóa task
    stop_countdown();
    
}

void update_display(int hour, int minute, int second) {
    // Cập nhật lại chuỗi timerFromKnob
    sprintf(timerFromKnob, "%02d:%02d:%02d", hour, minute, second);

    // Cập nhật nhãn hiển thị thời gian
    lv_label_set_text(label_time, timerFromKnob);

    // Cập nhật Arc chỉ theo giờ
    // lv_arc_set_value(arc, hour);
}

void start_countdown() {
  stop_countdown_flag = false;

  // if (countdown_task_handle == NULL) {
    xTaskCreate(countdown_task, "CountdownTask", 4096, NULL, 8, &countdown_task_handle);
    countdown_state = COUNTDOWN_START;
  // }
    
}

void pause_countdown() {
  countdown_state = COUNTDOWN_PAUSE;
  vTaskSuspend(countdown_task_handle);
}

void resume_countdown() {
  countdown_state = COUNTDOWN_RESUME;
  vTaskResume(countdown_task_handle);
}

void stop_countdown() {
  lvgl_port_lock(-1);
  percentH2FromKnob = 0.0;
  savedDataToSend = true;
  lvgl_port_unlock();
  reset_ui();
  save_data_to_nvs(timerFromKnob, &percentH2FromKnob);
  
  stop_countdown_flag = true; // Bật flag dừng
  if (countdown_task_handle != NULL) {
    countdown_state = COUNTDOWN_STOP;
    vTaskDelete(countdown_task_handle);
    countdown_task_handle = NULL;
  }
  
}

const int MAX_LENGTH = 30;  // Kích thước tối đa của mảng char
char receivedDataArray[MAX_LENGTH];  // Mảng lưu dữ liệu

void handleReceivedData(const String& data) {
  lvgl_port_lock(-1);

  // Chuyển String thành mảng char[]
  strncpy(receivedDataArray, data.c_str(), MAX_LENGTH - 1);
  receivedDataArray[MAX_LENGTH - 1] = '\0';  // Đảm bảo chuỗi kết thúc đúng

  Serial.print("Received Data as char array: ");
  Serial.println(receivedDataArray);  // In ra để kiểm tra

  // Kiểm tra xem có "v" trong chuỗi không (để cập nhật version)
  char* vPos = strchr(receivedDataArray, 'v');
  if (vPos != nullptr) {
    strncpy(versionBoard, vPos, sizeof(versionBoard) - 1);
    versionBoard[sizeof(versionBoard) - 1] = '\0';  // Đảm bảo chuỗi kết thúc

    Serial.print("Extracted version: ");
    Serial.println(versionBoard);
  }

  // Kiểm tra và xử lý dữ liệu lỗi
  if (strlen(receivedDataArray) == 0 || strcmp(receivedDataArray, "X") == 0 || containsSpecialChar(receivedDataArray)) {
    if (isErrorScreenActive) {
      isErrorScreenActive = false;
      error_flag = false;
      switch_ui(ui_main, label_notice);
      reset_ui();

      strcpy(timerFromKnob, "00:00:00");
      lv_label_set_text(label_time, timerFromKnob);
      exchange_H2Percent();
      lv_label_set_text(label_percent, percentH2_str);
    }
  } else { 
    if (!isErrorScreenActive) {
      isErrorScreenActive = true;
      switch_ui(ui_display_error, label_error_message);
      stop_countdown();
      knob_state = ERROR;
      error_flag = true;
    }
    lv_label_set_text(label_error_message, receivedDataArray);
  }

  lvgl_port_unlock();
}


void Receive_task(void *param) {
  for (;;) {
    lvgl_port_lock(-1);
    if (MySerial.available() > 0) {
      String receivedString = MySerial.readStringUntil('\n'); 
      receivedString.trim();
      Serial.println(receivedString);
      handleReceivedData(receivedString);  // Xử lý dữ liệu nhận được
    }
    lvgl_port_unlock();
    vTaskDelay(pdMS_TO_TICKS(50));  // Delay ngắn sau khi nhận dữ liệu
  } 
}

void Send_task(void *param) {
  for (;;) {
    
    
    if (savedDataToSend) {
      lvgl_port_lock(-1);
      Send_data_to_board();
      savedDataToSend = false;
      lvgl_port_unlock();
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Send_data_to_board(void) {
  if (percentH2FromKnob != -1){
    MySerial.print((percentH2FromKnob *(airFlowRate * 1000)) / 100);
  }
}

bool containsSpecialChar(const String &str) {
    for (size_t i = 0; i < str.length(); i++) {
        char c = str[i];
        if (!isalnum(c) && c != ' ' && c != '_') {  
            return true;  // Có ký tự đặc biệt
        }
    }
    return false;  // Không có ký tự đặc biệt
}

void OTA_task(void *param) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void setup()
{
    const char* title = "LVGL porting example";
#ifdef IM
    pinMode(IM1, OUTPUT);
    digitalWrite(IM1, HIGH);
  #ifdef BOARD_VIEWE_ESP_S3_Touch_LCD_35_V2
    pinMode(IM0, OUTPUT);
    digitalWrite(IM0, HIGH);
  #endif
  #ifndef BOARD_VIEWE_ESP_S3_Touch_LCD_35_V2
    pinMode(IM0, OUTPUT);
    digitalWrite(IM0, LOW);
  #endif
#endif

    Serial.begin(115200);
    MySerial.begin(9600, SERIAL_8N1, 20, 21);
    // Serial.println(title + " start");

    Serial.println("Initialize panel device");
    ESP_Panel *panel = new ESP_Panel();
    panel->init();
#if LVGL_PORT_AVOID_TEAR
    // When avoid tearing function is enabled, configure the bus according to the LVGL configuration
    ESP_PanelBus *lcd_bus = panel->getLcd()->getBus();
#if ESP_PANEL_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB
    static_cast<ESP_PanelBus_RGB *>(lcd_bus)->configRgbBounceBufferSize(LVGL_PORT_RGB_BOUNCE_BUFFER_SIZE);
    static_cast<ESP_PanelBus_RGB *>(lcd_bus)->configRgbFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#elif ESP_PANEL_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_MIPI_DSI
    static_cast<ESP_PanelBus_DSI *>(lcd_bus)->configDpiFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#endif
#endif
    panel->begin();
#ifdef KNOB
    Serial.println("Initialize Knob device");
    knob = new ESP_Knob(GPIO_NUM_KNOB_PIN_A, GPIO_NUM_KNOB_PIN_B);
    knob->begin();
    knob->attachLeftEventCallback(onKnobLeftEventCallback);
    knob->attachRightEventCallback(onKnobRightEventCallback);

    Button *btn = new Button(GPIO_BUTTON_PIN, false);
    btn->attachSingleClickEventCb(&SingleClickCb, NULL);
    btn->attachLongPressHoldEventCb(&LongPressHoldCb, NULL);
    // btn->attachDoubleClickEventCb(&DoubleClickCb, NULL);
    // btn->attachLongPressStartEventCb(&LongPressStartCb, NULL);

#endif

  Serial.println("Initialize LVGL");
  lvgl_port_init(panel->getLcd(), panel->getTouch());

  Serial.println("Create UI");
  /* Lock the mutex due to the LVGL APIs are not thread-safe */
  lvgl_port_lock(-1);

  ui_init();

  /* Release the mutex */
  lvgl_port_unlock();
  nvs_flash_init();
  // Serial.println(title + " end");

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
  Serial.print("Version Knob: ");
  Serial.println(version);


  xTaskCreate(Receive_task, "Receive_task", 10000, NULL, 6, NULL);
  xTaskCreate(Send_task, "Send_task", 8192, NULL, 5, NULL);
  xTaskCreate(OTA_task, "OTA_task", 4096, NULL, 5, NULL);
}

void loop()
{
  delay(1);
}
