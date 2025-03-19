#include <Arduino.h>
#include <ESP_Panel_Library.h>
#include <lvgl.h>
#include "lvgl_port_v8.h"
#include "driver/timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <Update.h>

#define VERSION "v19.3.2"

//==================================================================================================
#ifdef KNOB21
#define GPIO_NUM_KNOB_PIN_A 6
#define GPIO_NUM_KNOB_PIN_B 5
#define GPIO_BUTTON_PIN GPIO_NUM_0
#endif
#ifdef KNOB13
#define GPIO_NUM_KNOB_PIN_A 7
#define GPIO_NUM_KNOB_PIN_B 6
#define GPIO_BUTTON_PIN GPIO_NUM_9
#endif

#ifdef KNOB
#include <ESP_Knob.h>
#include <Button.h>

ESP_Knob *knob;
#endif
#include <ui.h>
#include <HardwareSerial.h>
#define TIMER_MAX_LENGTH 10

uint8_t savedDataToSend = false;
float percentH2FromKnob;
float airFlowRate = 10;  // 10L/min
char timerFromKnob[10] = "00:00:00";
bool isErrorScreenActive = false;
TaskHandle_t countdown_task_handle = NULL;
bool stop_countdown_flag = false;
uint8_t error_flag = false;
extern knob_state_t knob_state;
extern countdown_state_t countdown_state;
//=====================================================================

//=====================================================================

void onKnobLeftEventCallback(int count, void *usr_data) {
  Serial.printf("Detect left event, count is %d\n", count);
  lvgl_port_lock(-1);
  LVGL_knob_event((void *)KNOB_LEFT);
  lvgl_port_unlock();
}

void onKnobRightEventCallback(int count, void *usr_data) {
  Serial.printf("Detect right event, count is %d\n", count);
  lvgl_port_lock(-1);
  LVGL_knob_event((void *)KNOB_RIGHT);
  lvgl_port_unlock();
}

static void SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button Single Click");
  lvgl_port_lock(-1);
  LVGL_button_event((void *)BUTTON_SINGLE_CLICK);
  lvgl_port_unlock();
}

static void LongPressHoldCb(void *button_handle, void *usr_data) {
  lvgl_port_lock(-1);
  LVGL_button_event((void *)BUTTON_LONG_PRESS_HOLD);
  lvgl_port_unlock();
}

String receivedString;  // Variable that receive data via Serial

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
    timer[0] = '\0';  // Set to empty if UUID not found
  } else {
    Serial.printf("Error reading timer: %s\n", esp_err_to_name(err));
    timer[0] = '\0';  // Set to empty if error occured
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
    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 giây
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

  stop_countdown_flag = true;  // Bật flag dừng
  if (countdown_task_handle != NULL) {
    countdown_state = COUNTDOWN_STOP;
    vTaskDelete(countdown_task_handle);
    countdown_task_handle = NULL;
  }
}

//=====================================================================

// Cấu hình UART (điều chỉnh chân RX, TX theo board của bạn)
HardwareSerial OtaSerial(1); // RX: 20, TX: 21
#define DEBUG_SERIAL Serial   // Sử dụng Serial cho debug

// Cấu hình kích thước buffer và timeout OTA
#define OTA_TIMEOUT 10000  // 5 giây

// Các trạng thái OTA của Knob
enum KnobState {
  KNOB_IDLE,         // Không trong chế độ OTA
  KNOB_OTA_READY,    // Đã sẵn sàng nhận header cho chunk tiếp theo
  KNOB_OTA_RECEIVING // Đang nhận dữ liệu chunk
};

KnobState knobState = KNOB_IDLE;
unsigned long lastPacketTime = 0;

// Biến dùng để xử lý lệnh nhận qua UART
String commandBuffer = "";

// Biến dùng để xử lý chunk dữ liệu
uint32_t expectedSize = 0;
uint32_t currentCRC = 0;
uint8_t* fileBuffer = NULL;
size_t filePos = 0;

// Hàm tính CRC32 (chuẩn IEEE 802.3)
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

// Hàm xử lý một chunk dữ liệu nhận được
void processChunk(uint8_t* data, size_t len, uint32_t crc) {
  uint32_t calculatedCRC = crc32(data, len);
  if (calculatedCRC != crc) {
    OtaSerial.println("CRC_ERR");
    return;
  }
  if (Update.write(data, len) != len) {
    OtaSerial.println("ERR:WRITE_FAIL");
    abortOTA();
    return;
  }
  OtaSerial.flush();
  OtaSerial.println("OK");
  DEBUG_SERIAL.println("Send OK to controller");
}

// Hàm xử lý lệnh nhận qua UART (dòng văn bản kết thúc bằng '\n')
void processCommand() {
  commandBuffer.trim();
  if (commandBuffer == "OTA_START") {
    Serial.println("OTA_START");
    if (Update.begin(UPDATE_SIZE_UNKNOWN)) {
      knobState = KNOB_OTA_READY;
      while (OtaSerial.available()) {
        OtaSerial.read();
      }    
      OtaSerial.println("READY");
      DEBUG_SERIAL.println("Send READY to Knob");
    } else {
      OtaSerial.println("ERR:INIT_FAIL");
    }
  }
  else if (commandBuffer == "OTA_END") {
    if (Update.end(true)) {
      OtaSerial.println("OTA_SUCCESS");
      ESP.restart();
    } else {
      OtaSerial.println("ERR:END_FAIL");
    }
    knobState = KNOB_IDLE;
  }
  else if (commandBuffer == "OTA_ABORT") {
    Update.abort();
    knobState = KNOB_IDLE;
    OtaSerial.println("ABORTED");
  }
  // Xóa buffer lệnh sau khi xử lý
  commandBuffer = "";
}

// Hàm hủy OTA
void abortOTA() {
  Update.abort();
  knobState = KNOB_IDLE;
  OtaSerial.println("OTA_ABORTED");
}

#define MAGIC_MARKER 0xAABBCCDD
#define HEADER_TOTAL_SIZE 12  // 4 byte marker + 4 byte len + 4 byte CRC

static uint8_t headerBuffer[8];  // chỉ cần lưu phần len và CRC (8 byte)
static size_t headerBufferPos = 0;

void handleUART() {
  while (OtaSerial.available()) {
    if (knobState == KNOB_IDLE) {
      char c = OtaSerial.read();
      if (c == '\n') {
        processCommand();
      } else {
        commandBuffer += c;
      }
    }
    // Trạng thái KNOB_OTA_READY: đang chờ nhận header của chunk mới
    else if (knobState == KNOB_OTA_READY) {
      // Đảm bảo có ít nhất 4 byte để kiểm tra marker
      if (OtaSerial.available() >= 4) {
        uint32_t marker;
        // Đọc 4 byte marker
        OtaSerial.readBytes((char*)&marker, sizeof(marker));
        if (marker != MAGIC_MARKER) {
          // Nếu không khớp, coi đó là lệnh (có thể là OTA_END, v.v.)
          // Đọc lại phần còn lại của dòng lệnh cho đến khi gặp '\n'
          DEBUG_SERIAL.printf("Invalid marker: 0x%08X\n", marker); // Thêm dòng này

          commandBuffer = String((char*)&marker); 
          commandBuffer += OtaSerial.readStringUntil('\n');
          commandBuffer.trim();
          processCommand();
          continue; // bỏ qua việc xử lý header
        }
        // Marker hợp lệ, giờ đọc thêm 8 byte (len và CRC)
        headerBufferPos = 0;
        while (headerBufferPos < sizeof(headerBuffer) && OtaSerial.available()) {
          headerBuffer[headerBufferPos++] = OtaSerial.read();
        }
        if (headerBufferPos == sizeof(headerBuffer)) {
          expectedSize = *(uint32_t*)(&headerBuffer[0]);
          currentCRC = *(uint32_t*)(&headerBuffer[4]);
          if (fileBuffer) {
            free(fileBuffer);
          }
          fileBuffer = (uint8_t*) malloc(expectedSize);
          if (fileBuffer == NULL) {
            OtaSerial.println("ERR:MEM_ALLOC");
            abortOTA();
            return;
          }
          filePos = 0;
          knobState = KNOB_OTA_RECEIVING;
        }
      }
    }
    // Trạng thái nhận dữ liệu chunk
    else if (knobState == KNOB_OTA_RECEIVING) {
      while (OtaSerial.available() && filePos < expectedSize) {
        fileBuffer[filePos++] = OtaSerial.read();
      }
      if (filePos == expectedSize) {
        processChunk(fileBuffer, expectedSize, currentCRC);
        free(fileBuffer);
        fileBuffer = NULL;
        headerBufferPos = 0;
        // Sau khi xử lý xong chunk, chuyển về trạng thái nhận header cho chunk tiếp theo
        knobState = KNOB_OTA_READY;
      }
    }
    lastPacketTime = millis();
  }
}



// Hàm kiểm tra timeout OTA
void checkOTATimeout() {
  if (knobState != KNOB_IDLE && (millis() - lastPacketTime > OTA_TIMEOUT)) {
    OtaSerial.println("ERR:TIMEOUT");
    abortOTA();
  }
}

// Task xử lý OTA qua UART
void OTA_Task(void *param) {
  for (;;) {
    handleUART();
    checkOTATimeout();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {

   String title = "LVGL porting example";
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
  Serial.println(F("Initialize Knob device"));
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

  lvgl_port_init(panel->getLcd(), panel->getTouch());
  lvgl_port_lock(-1);
  ui_init();
  lvgl_port_unlock();


  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.print("VERSION");
  DEBUG_SERIAL.println(VERSION);
  // Cấu hình OtaSerial: thay đổi các chân RX, TX theo board của bạn (ví dụ: 20, 21)
  OtaSerial.begin(460800, SERIAL_8N1, 20, 21);
  nvs_flash_init();
  DEBUG_SERIAL.println("Knob OTA Receiver Starting");
  
  // Tạo task OTA
  xTaskCreate(OTA_Task, "OTA_Task", 8192, NULL, 10, NULL);
}

void loop() {
  // Không làm gì ở đây, mọi xử lý nằm trong task OTA_Task
}