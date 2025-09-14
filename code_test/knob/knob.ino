#include <ESP_Panel_Library.h>

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
HardwareSerial OtaSerial(1); // RX: 20, TX: 21
static void SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button Single Click");
  char buffer[32];

  snprintf(buffer, 32, "Knob test");
  OtaSerial.println(buffer);
  Serial.println(buffer);
}

void setup() {
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
  // knob->attachLeftEventCallback(onKnobLeftEventCallback);
  // knob->attachRightEventCallback(onKnobRightEventCallback);

  Button *btn = new Button(GPIO_BUTTON_PIN, false);
  btn->attachSingleClickEventCb(&SingleClickCb, NULL);

#endif
  // Cấu hình OtaSerial: thay đổi các chân RX, TX theo board của bạn (ví dụ: 20, 21)
  OtaSerial.begin(460800, SERIAL_8N1, 20, 21);
}

void loop() {
  // put your main code here, to run repeatedly:
}
