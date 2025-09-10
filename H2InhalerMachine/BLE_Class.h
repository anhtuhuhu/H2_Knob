#ifndef _BLE_CLASS_H
#define _BLE_CLASS_H
#include "BLECharacteristic.h"
#include <Arduino.h>
#include <BLEDevice.h>
// #include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define H2_INHALER_UUID(val) ("e4f09000-" val "-4625-988c-bc09a4963c44")
#define WRITE_CHAR_UUID "7a442881-509c-47fa-ac02-b06a37d9eb76"
// const char write_char = "7a442881-509c-47fa-ac02-b06a37d9eb76";

#define MAX_MTU_SIZE 23
#define MAX_MAC_ID_LEN 18
#define MAX_LIST_AP_LEN 350
#define MAX_LEN_RX_BUFFER 150

extern uint8_t g_ret_list_ap[MAX_LIST_AP_LEN];
extern uint8_t g_ret_mac_id[MAX_MAC_ID_LEN];
extern char rxBuffer[MAX_LEN_RX_BUFFER];
extern bool isRxBufferDone;

class BLEHandler {
public:
  BLEHandler();
  void begin();
  void end();
  void sendSSID(char* ssid_list);
  char* ble_app_return_rx_buffer(void);
  static BLEHandler* instance;

  BLECharacteristic* pSSIDRequestChar;
  BLECharacteristic* pSSIDResponseChar;
  BLECharacteristic* pWIFIStatusChar;

  BLECharacteristic* pApp;
private:
  BLEServer* pServer;
  BLEService* pService;
  
  bool is_ble_start;
  uint16_t connectID;

  class BLECallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override;
  };

  
};


#endif