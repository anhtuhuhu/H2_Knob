#ifndef OTA_MQTT_H
#define OTA_MQTT_H

#include <Arduino.h>
#include <Update.h>
#include <ArduinoJson.h>

class OTA_MQTT
{
public:
    enum OTA_State
    {
        OTA_IDLE,
        OTA_WAIT_READY,
        OTA_SENDING,
        OTA_END
    };

    enum Target_Device
    {
        CONTROLLER,
        KNOB
    };

    struct OTA_Status
    {
        OTA_State state;
        Target_Device target;
        size_t totalSize;
        size_t receivedSize;
        String error;
    };

    OTA_MQTT();
    void begin();
    bool handleOTAPacket(String url, Target_Device target);
    bool handleOTAPacketAtOffset(uint8_t *data, size_t len, size_t offset, Target_Device target);
    bool processChunk(uint8_t *chunk, size_t chunkSize);
    uint32_t crc32(const uint8_t *data, size_t length);
    void sendChunkToKnob(uint8_t *data, size_t len);
    bool waitForKnobResponse(const char *expected, uint32_t timeout);
    OTA_Status& getStatus() { return _status; }
    void reset();

private:
    OTA_Status _status;
    static const uint32_t MAGIC_MARKER = 0xAABBCCDD;
    size_t _written;
};

#endif // OTA_MQTT_H
