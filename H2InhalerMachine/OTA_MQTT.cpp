#include "OTA_MQTT.h"
#include "MQTT.h"
#include "HardwareSerial.h"
#include <HTTPClient.h>
#include <WiFiClient.h>

extern HardwareSerial UART2;
HTTPClient http;
OTA_MQTT::OTA_MQTT()
{
    reset();
}

void OTA_MQTT::begin()
{
    reset();
}

void OTA_MQTT::reset()
{
    _status.state = OTA_IDLE;
    _status.totalSize = 0;
    _status.receivedSize = 0;
    _status.error = "";
    _written = 0;
}

bool OTA_MQTT::handleOTAPacket(String url, Target_Device target)
{
    Serial.printf("handleOTAPacket: url=%s, target=%d, state=%d\n",
                  url.c_str(), target, _status.state);

    if (target != CONTROLLER && target != KNOB)
    {
        Serial.println("Invalid target device for OTA");
        return false;
    }
    _status.target = target;

    http.begin(url);
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK)
    {
        Serial.printf("HTTP GET failed, code: %d\n", httpCode);
        http.end();
        return false;
    }

    int contentLength = http.getSize();
    WiFiClient *stream = http.getStreamPtr();

    Serial.printf("Start OTA for target=%d, size=%d bytes\n", target, contentLength);
    _status.totalSize = contentLength;
    _status.receivedSize = 0;

    // Controller: init Update
    if (target == CONTROLLER)
    {
        if (!Update.begin(contentLength))
        {
            Serial.println("Not enough space for OTA");
            http.end();
            return false;
        }
    }
    else
    {
        // Knob: gửi lệnh OTA_START
        UART2.println("OTA_START");
        if (!waitForKnobResponse("READY", 3000))
        {
            Serial.println("Knob not ready for OTA");
            http.end();
            return false;
        }
    }

    // Đọc theo buffer
    const size_t buffSize = 2048;
    uint8_t buff[buffSize];
    while (http.connected() && (_status.receivedSize < contentLength))
    {
        size_t avail = stream->available();
        if (avail)
        {
            size_t len = stream->readBytes(buff, (avail > buffSize) ? buffSize : avail);

            if (target == CONTROLLER)
            {
                if (Update.write(buff, len) != len)
                {
                    Serial.println("Failed to write to flash");
                    http.end();
                    Update.end();
                    return false;
                }
            }
            else
            {
                sendChunkToKnob(buff, len);
            }

            _status.receivedSize += len;
            Serial.printf("OTA progress: %u/%u\n",
                          (unsigned)_status.receivedSize, (unsigned)_status.totalSize);
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // tránh watchdog
    }

    // Kết thúc
    if (target == CONTROLLER)
    {
        if (Update.end() && Update.isFinished())
        {
            Serial.println("Controller OTA complete. Rebooting...");
            ESP.restart();
        }
        else
        {
            Serial.printf("Update error: %s\n", Update.errorString());
            return false;
        }
    }
    else
    {
        UART2.println("OTA_END");
        if (!waitForKnobResponse("DONE", 5000))
        {
            Serial.println("Knob OTA did not finish correctly");
            return false;
        }
        Serial.println("Knob OTA complete!");
    }

    http.end();
    return true;
}

bool OTA_MQTT::processChunk(uint8_t *chunk, size_t chunkSize)
{
    if (_status.target == CONTROLLER)
    {
        if (Update.write(chunk, chunkSize) != chunkSize)
        {
            _status.error = "Failed to write firmware chunk";
            return false;
        }

        if (_status.state == OTA_END)
        {
            if (!Update.end(true))
            {
                _status.error = "Failed to end OTA update";
                return false;
            }
            ESP.restart();
        }
    }
    else
    {
        // For Knob, send chunk via UART
        sendChunkToKnob((uint8_t *)chunk, chunkSize);
    }

    return true;
}

uint32_t OTA_MQTT::crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc = crc >> 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

void OTA_MQTT::sendChunkToKnob(uint8_t *data, size_t len)
{
    uint32_t marker = MAGIC_MARKER;
    uint32_t crc = crc32(data, len);

    UART2.write((uint8_t *)&marker, sizeof(marker));
    UART2.write((uint8_t *)&len, sizeof(len));
    UART2.write((uint8_t *)&crc, sizeof(crc));
    UART2.write(data, len);

    if (!waitForKnobResponse("OK", 3000))
    {
        _status.error = "Failed to send chunk to Knob";
        _status.state = OTA_IDLE;
    }
}

bool OTA_MQTT::waitForKnobResponse(const char *expected, uint32_t timeout)
{
    uint32_t start = millis();
    while (millis() - start < timeout)
    {
        if (UART2.available())
        {
            String response = UART2.readStringUntil('\n');
            response.trim();
            if (response == expected)
            {
                return true;
            }
        }
        delay(10);
    }
    return false;
}