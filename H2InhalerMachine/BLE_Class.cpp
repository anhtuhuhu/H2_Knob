#include "BLE_Class.h"


uint8_t g_ret_mac_id[MAX_MAC_ID_LEN]    = {0x11, 0x22, 0x33, 0x44};
uint8_t g_ret_list_ap[MAX_LIST_AP_LEN]  = {0xFF, 0x22, 0x33, 0x44, 0x55, 0};
char rxBuffer[MAX_LEN_RX_BUFFER];
bool isRxBufferDone = false;
size_t rxOffSet = 0;

BLEHandler* BLEHandler::instance = nullptr;

BLEHandler::BLEHandler() {
  pServer = nullptr;
  pService = nullptr;
  pSSIDRequestChar = nullptr;
  pSSIDResponseChar = nullptr;
  pWIFIStatusChar = nullptr;
  is_ble_start = false;
  instance = this;
}

void BLEHandler::begin() {
  if (!is_ble_start) {
    Serial.println("Starting BLE......");
    String mac_id = (char*)g_ret_mac_id;
    mac_id.toUpperCase();
    String deviceName = "H2-Inhaler-BLE-";
    deviceName += mac_id[mac_id.length() - 5];
    deviceName += mac_id[mac_id.length() - 4];
    deviceName += mac_id[mac_id.length() - 2];
    deviceName += mac_id[mac_id.length() - 1];
    
    BLEDevice::init(deviceName);

    // GATT Server
    pServer = BLEDevice::createServer();

    // Service
    pService = pServer->createService(H2_INHALER_UUID("2d00"));

    // Write/Notify wifi status
    pWIFIStatusChar = pService->createCharacteristic(
      H2_INHALER_UUID("2d01"),
      BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_NOTIFY
    );
    pWIFIStatusChar->setCallbacks(new BLECallbacks());
    

    // Request char (App → ESP32)
    pSSIDRequestChar = pService->createCharacteristic(
      H2_INHALER_UUID("2d03"),
      BLECharacteristic::PROPERTY_WRITE
    );
    pSSIDRequestChar->setCallbacks(new BLECallbacks());

    // Response char (ESP32 → App)
    pSSIDResponseChar = pService->createCharacteristic(
      H2_INHALER_UUID("2d02"),
      BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY
    );
    pSSIDResponseChar->addDescriptor(new BLE2902());

    BLEDevice::setMTU(MAX_MTU_SIZE); // đề nghị MTU cao hơn mặc định
    connectID = pServer->getConnId();
    pService->start();

    // Advertising
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(H2_INHALER_UUID("2d00"));
    pAdvertising->start();

    pWIFIStatusChar->setValue((const char*)g_ret_mac_id);
    pWIFIStatusChar->notify();

    pSSIDResponseChar->setValue((const char*)g_ret_list_ap);

    is_ble_start = true;

    Serial.println("BLE service with multiple characteristics is advertising...");
  }
  
}

void BLEHandler::end() {
  if (is_ble_start) {
    Serial.println("Stopping BLE...");

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    if (pAdvertising) pAdvertising->stop();

    if (pService) pService->stop();

    BLEDevice::deinit(true);
    is_ble_start = false;
  }
}

void BLEHandler::sendSSID(char* ssid_list) {
  Serial.println("Ready to send: " + (String)ssid_list);

  if (pSSIDResponseChar) {
    pSSIDResponseChar->setValue(ssid_list);
    pSSIDResponseChar->notify();
  }
}

// Callback xử lý yêu cầu từ app
void BLEHandler::BLECallbacks::onWrite(BLECharacteristic* pChar) {
  String val = pChar->getValue();
  Serial.println("[BLE] App requested: " + String(val.c_str()) + " len: " + String(pChar->getLength()));

  for (size_t i = 0; i < pChar->getLength(); i++) {
    char c = val[i];

    if (rxOffSet < MAX_LEN_RX_BUFFER - 1) {
      rxBuffer[rxOffSet++] = c;

      if (c == '\n'){
        rxBuffer[rxOffSet-1] = '\0';
        isRxBufferDone = true;
        Serial.printf("[BLE] Full command received: %s\n", rxBuffer);
        break;
      }
    }
    else {
      rxOffSet = 0;
      Serial.println("[BLE] Error: Buffer overflow");
      break;
    }
  }
}

char* BLEHandler::ble_app_return_rx_buffer(void) {
  if (isRxBufferDone == true)
  {
    isRxBufferDone = false;
    rxOffSet = 0;
    printf("return rxBuffer: %s\n", rxBuffer);
    return rxBuffer;
  }

  return NULL;
}
