#include "WIFI_Class.h"

bool WIFI::isConnected = false;
wifi_credential_t WIFI::credentials = {0};
BLEHandler _ble;
TaskHandle_t WIFI::wifiTaskHandle = nullptr;
void (*WIFI::connectedCallback)() = nullptr;
WiFiScanClass wifi_scan;

void WIFI::init() {
  nvs_flash_init();

  loadFromNVS("wifi_ssid", credentials.ssid, sizeof(credentials.ssid));
  loadFromNVS("wifi_pass", credentials.pass, sizeof(credentials.pass));

  WiFi.mode(WIFI_STA);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf((char *)g_ret_mac_id, sizeof(g_ret_mac_id), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  scanAndSendAPs((char*)g_ret_list_ap);

  if (wifiTaskHandle == nullptr) {
    xTaskCreatePinnedToCore([](void*) {
      while (true) {
        handleBLECommand();
        if (!isConnected) connect();
        else _ble.end();
        vTaskDelay(pdMS_TO_TICKS(3000));
      }
    }, "wifi_loop", 8192, nullptr, 5, &wifiTaskHandle, 1);
  }

  _ble.begin();
}

void WIFI::connect() {
  WiFi.mode(WIFI_STA);
  WiFi.onEvent([](WiFiEvent_t event) {
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      if (!isConnected) {  // tránh gọi lại nhiều lần
        isConnected = true;
        Serial.println("WiFi connected with IP: " + WiFi.localIP().toString());
        _ble.pWIFIStatusChar->setValue("WIFI_CONNECTED");
        _ble.pWIFIStatusChar->notify();
        if (connectedCallback) connectedCallback();
      }
    }
  });

  if (strlen(credentials.ssid) > 0 && strlen(credentials.pass) > 0) {
    WiFi.begin(credentials.ssid, credentials.pass);
    delay(1000);
    if (WiFi.status() == WL_CONNECTED) isConnected = true;
    else Serial.println("Initial WiFi connect failed");
  }
}

void WIFI::disconnect() {
  WiFi.disconnect();
  _ble.pWIFIStatusChar->setValue("WIFI_DISCONNECTED");
  _ble.pWIFIStatusChar->notify();
  isConnected = false;
}

void WIFI::handleBLECommand() {
  char* input = _ble.ble_app_return_rx_buffer();
  if (input == nullptr) return;

  Serial.printf("BLE received: [%s]\n", input);
  if (strcmp(input, "RELOAD_WIFI\n") == 0) {
    scanAndSendAPs((char*)g_ret_list_ap);
    _ble.instance->sendSSID((char*)g_ret_list_ap);
  } else {
    char ssid[MAX_WIFI_SSID_LEN] = {0};
    char pass[MAX_WIFI_PASS_LEN] = {0};
    if (parseWiFiCommand(input, ssid, pass)) {
      strncpy(credentials.ssid, ssid, sizeof(credentials.ssid));
      strncpy(credentials.pass, pass, sizeof(credentials.pass));
      saveToNVS("wifi_ssid", credentials.ssid);
      saveToNVS("wifi_pass", credentials.pass);
      disconnect();  // reset connection
    }
  }
  memset(input, 0, strlen(input));
}

void WIFI::scanAndSendAPs(char* ap_list) {
  char* wifi_ap_list = (char*)malloc(MAX_WIFI_LIST_LEN);
  if (!wifi_ap_list) {
    Serial.println("Malloc wifi_ap_list failed");
    return;
  }
  memset(wifi_ap_list, 0, MAX_WIFI_LIST_LEN);

  if (isConnected) disconnect();

  int16_t num_of_wifi = wifi_scan.scanNetworks();
  if (num_of_wifi > 10) num_of_wifi = 10;

  if (num_of_wifi > 0 && wifi_scan.scanComplete()) {
    for (int i = 0; i < num_of_wifi; ++i) {
      String ssid = wifi_scan.SSID(i);
      if (ssid.length() == 0) continue;
      strncat(wifi_ap_list, ssid.c_str(), ssid.length());
      strcat(wifi_ap_list, ",");
      delay(10 / portTICK_PERIOD_MS);
    }
    memcpy(ap_list, wifi_ap_list, strlen(wifi_ap_list) + 1);
  }
  wifi_scan.scanDelete();
  free(wifi_ap_list);
}

// bool WIFI::parseWiFiCommand(const char* input, char* ssid, char* pass) {
//   const char* prefix = "WIFI:";
//   if (strncmp(input, prefix, strlen(prefix)) != 0) return false;

//   const char* ssidStart = input + strlen(prefix);
//   const char* colon = strchr(ssidStart, ':');
//   if (!colon) return false;

//   size_t ssidLen = colon - ssidStart;
//   size_t passLen = strlen(colon + 1);

//   if (ssidLen >= MAX_WIFI_SSID_LEN || passLen >= MAX_WIFI_PASS_LEN) return false;

//   strncpy(ssid, ssidStart, ssidLen); ssid[ssidLen] = '\0';
//   strncpy(pass, colon + 1, passLen); pass[passLen] = '\0';

//   Serial.printf("parse_wifi_info:  ssid: %s  -  pass: %s", ssid, pass);

//   return true;
// }

bool WIFI::parseWiFiCommand(const char* input, char* ssid, char* pass) {
  const char* semicolon = strchr(input, ';');
  if (!semicolon) return false;

  size_t ssidLen = semicolon - input;
  size_t passLen = strlen(semicolon + 1);
  if (ssidLen >= MAX_WIFI_SSID_LEN || passLen >= MAX_WIFI_PASS_LEN) return false;

  strncpy(ssid, input, ssidLen);
  ssid[ssidLen] = '\0';
  strncpy(pass, semicolon + 1, passLen);
  pass[passLen] = '\0';

  Serial.printf("parse_wifi_info:  ssid: %s  -  pass: %s", ssid, pass);
  return true;
}

void WIFI::saveToNVS(const char* key, const char* val) {
  nvs_handle h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_str(h, key, val);
    nvs_commit(h);
    nvs_close(h);
  }
}

bool WIFI::loadFromNVS(const char* key, char* out, size_t len) {
  nvs_handle h;
  size_t required = len;
  if (nvs_open("storage", NVS_READONLY, &h) != ESP_OK) return false;
  if (nvs_get_str(h, key, out, &required) != ESP_OK) {
    nvs_close(h);
    return false;
  }
  nvs_close(h);
  return true;
}

void WIFI::setConnectedCallback(void (*cb)()) {
  connectedCallback = cb;
}

String WIFI::getMacAddress() {
  return WiFi.macAddress();
}
