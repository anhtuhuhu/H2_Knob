#ifndef _OTA_h
#define _OTA_h

#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
// #include "H2InhallerMachine.h"
#define MAGIC_MARKER 0xAABBCCDD

class OTA {

private:
  String currentTarget = "";
  String filterResponse(String response);
  uint32_t crc32(const uint8_t* data, size_t length);
public:
  // OTA state
  enum OTAState {
    OTA_IDLE,
    OTA_WAIT_READY,
    OTA_SENDING,
    OTA_END
  };
  OTAState otaState = OTA_IDLE;

  void handleRoot(void);
  void handleUpdate(void);
  void handleUpload(void);
  void sendChunkToKnob(uint8_t* data, size_t len);
  bool waitForKnobResponse(const char* expected, uint32_t timeout);


};

#endif