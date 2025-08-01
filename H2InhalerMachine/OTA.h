#ifndef _OTA_h
#define _OTA_h

#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
// #include "H2InhallerMachine.h"
#define MAGIC_MARKER 0xAABBCCDD

class OTA {

private:
  static inline String currentTarget = "";
  static String filterResponse(String response);
  static uint32_t crc32(const uint8_t* data, size_t length);
  static bool startKnobOTA();
  static void sendChunkToKnob(uint8_t* data, size_t len);
  static bool waitForKnobResponse(const char* expected, uint32_t timeout);
  // static const char* configPage;

public:
  // OTA state
  enum OTAState {
    OTA_IDLE,
    OTA_WAIT_READY,
    OTA_SENDING,
    OTA_END
  };
  static inline OTAState otaState = OTA_IDLE;

  void handleRoot(void);
  void handleUpdate(void);
  void handleUpload(void);

};

#endif