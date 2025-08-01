#ifndef _WIFI_H_
#define _WIFI_H_


#include <WiFi.h>
#include "BLE_Class.h"
#include "esp_mac.h"
#include <nvs_flash.h>

#define MAX_WIFI_SSID_LEN 32
#define MAX_WIFI_PASS_LEN 64
#define MAX_WIFI_LIST_LEN 330

struct wifi_credential_t {
  char ssid[MAX_WIFI_SSID_LEN];
  char pass[MAX_WIFI_PASS_LEN];
};

class WIFI {
public:
  static void init();
  static void connect();
  static void disconnect();

  static void setConnectedCallback(void (*cb)());
  static String getMacAddress();

private:
  static void scanAndSendAPs(char* ap_list);
  static void handleBLECommand();
  static bool parseWiFiCommand(const char* input, char* ssid, char* pass);
  static bool loadFromNVS(const char* key, char* out, size_t len);
  static void saveToNVS(const char* key, const char* val);

  static bool isConnected;
  static wifi_credential_t credentials;
  static TaskHandle_t wifiTaskHandle;
  static void (*connectedCallback)();
};


#endif //_WIFI_H_