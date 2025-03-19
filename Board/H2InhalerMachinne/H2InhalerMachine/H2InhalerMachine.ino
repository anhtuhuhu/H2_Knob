#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <HardwareSerial.h>

// WiFi credentials
const char* ssid = "Anonymous";
const char* password = "0967375057abc";

// WebServer instance
WebServer server(80);

// UART for Knob communication
HardwareSerial UART2(1);
#define UART1_RX_PIN GPIO_NUM_44
#define UART1_TX_PIN GPIO_NUM_43

// OTA state
enum OTAState {
  OTA_IDLE,
  OTA_WAIT_READY,
  OTA_SENDING,
  OTA_END
};
OTAState otaState = OTA_IDLE;

// Global variable for target management
String currentTarget = "";

// Function prototypes
void handleRoot();
void handleUpdate();
void handleUpload();
void sendChunkToKnob(uint8_t* data, size_t len);
bool waitForKnobResponse(const char* expected, uint32_t timeout = 1000);

void setup() {
  Serial.begin(115200);
  UART2.begin(460800, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Configure WebServer routes
  server.on("/", handleRoot);
  // Lấy target từ query parameter
  server.on("/update", HTTP_POST, handleUpdate, handleUpload);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  // Handle OTA state for Knob via UART responses
  if (otaState == OTA_WAIT_READY || otaState == OTA_SENDING || otaState == OTA_END) {
    if (UART2.available()) {
      String response = UART2.readStringUntil('\n');
      response.trim();
      if (response == "READY" && otaState == OTA_WAIT_READY) {
        otaState = OTA_SENDING;
        Serial.println("Knob is ready for OTA");
      } else if (response == "OTA_SUCCESS" && otaState == OTA_END) {
        otaState = OTA_IDLE;
        Serial.println("Knob OTA successful!");
      } else if (response.startsWith("ERR")) {
        Serial.println("Knob OTA failed: " + response);
        otaState = OTA_IDLE;
      }
    }
  }
}

// Handle root route: gửi HTML form
void handleRoot() {
  const char* otaPage = R"(
  <!DOCTYPE html>
  <html>
  <head>
    <title>OTA Update</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 20px; }
      h1 { color: #333; }
      form { margin-top: 20px; }
      label { display: block; margin: 10px 0 5px; }
      input[type="file"] { margin-bottom: 10px; }
      input[type="submit"] { background: #007bff; color: #fff; border: none; padding: 10px 20px; cursor: pointer; }
      input[type="submit"]:hover { background: #0056b3; }
    </style>
  </head>
  <body>
    <h1>OTA Update</h1>
    <form id="otaForm" method="POST" action="/update" enctype="multipart/form-data">
      <label for="update">Select firmware file:</label>
      <input type="file" name="update" id="update" required><br><br>
      <label>Select target:</label>
      <input type="radio" id="controller" name="target" value="Controller" checked>
      <label for="controller">Controller (ESP32)</label><br>
      <input type="radio" id="knob" name="target" value="Knob">
      <label for="knob">Knob (ESP32-C3 via UART)</label><br><br>
      <input type="submit" value="Update">
    </form>
    <script>
      document.getElementById("otaForm").onsubmit = function() {
        var radios = document.getElementsByName("target");
        var selectedValue = "";
        for (var i = 0; i < radios.length; i++){
          if (radios[i].checked) {
            selectedValue = radios[i].value;
            break;
          }
        }
        this.action = "/update?target=" + selectedValue;
      };
    </script>
  </body>
  </html>
  )";
  server.send(200, "text/html", otaPage);
}

// Callback chính sau khi upload hoàn tất: lấy target từ query parameter
void handleUpdate() {
  currentTarget = server.arg("target");
  Serial.print("Selected target (from handleUpdate): ");
  Serial.println(currentTarget);

  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");

  // Nếu target là Controller thì restart ngay, nếu là Knob thì chờ xác nhận qua UART
  if (currentTarget == "Controller") {
    ESP.restart();
  }
}

String filterResponse(String response) {
  String cleaned = "";
  for (unsigned int i = 0; i < response.length(); i++) {
    char c = response.charAt(i);
    if (c >= 32 && c <= 126) { // chỉ giữ các ký tự in được
      cleaned += c;
    }
  }
  return cleaned;
}
// Callback xử lý file upload
void handleUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    currentTarget = server.arg("target");
    Serial.print("Starting upload for target: ");
    Serial.println(currentTarget);

    if (currentTarget == "Knob") {
      UART2.println("OTA_START");
      UART2.println("OTA_START");
      otaState = OTA_WAIT_READY;
      Serial.println("Waiting for Knob to be ready...");

      // Thêm phần đợi READY với timeout
      unsigned long start = millis();
      while (otaState == OTA_WAIT_READY && millis() - start < 10000) {
        if (UART2.available()) {
          String response = UART2.readStringUntil('\n');
          response = filterResponse(response);
          response.trim();
          Serial.println(response);
          if (response.indexOf("READY") >= 0) {
            otaState = OTA_SENDING;
            Serial.println("Knob is ready!");
            break;
          } else if (response.startsWith("ERR")) {
            Serial.println("Knob error: " + response);
            server.send(500, "text/plain", "Knob initialization failed");
            return;
          }
        }
        delay(10);
      }

      if (otaState != OTA_SENDING) {
        Serial.println("Knob not ready, aborting OTA");
        server.send(500, "text/plain", "Knob not responding");
        return;
      }
    } else {
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.println("Failed to start OTA on Controller!");
        server.send(500, "text/plain", "Failed to start OTA on Controller!");
        return;
      }
    }
  } 
  else if (upload.status == UPLOAD_FILE_WRITE) {
    if (currentTarget == "Knob") {
      Serial.println("Knob is sending!");
      sendChunkToKnob(upload.buf, upload.currentSize);
    } else {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Serial.println("Failed to write firmware chunk!");
        server.send(500, "text/plain", "Failed to write firmware chunk!");
        return;
      }
    }
  } 
  else if (upload.status == UPLOAD_FILE_END) {
    if (currentTarget == "Knob") {
      UART2.print("OTA_END\n");
      otaState = OTA_END;
      Serial.println("Waiting for Knob to finish OTA...");
    } else {
      if (Update.end(true)) {
        Serial.println("Controller OTA successful!");
      } else {
        Serial.println("Failed to end OTA on Controller!");
        server.send(500, "text/plain", "Failed to end OTA on Controller!");
        return;
      }
    }
  }
}

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

#define MAGIC_MARKER 0xAABBCCDD

void sendChunkToKnob(uint8_t* data, size_t len) {
  uint32_t marker = MAGIC_MARKER;
  uint32_t crc = crc32(data, len);
  
  // Gửi marker, kích thước và CRC (mỗi giá trị 4 byte)
  UART2.write((uint8_t*)&marker, sizeof(marker));
  UART2.write((uint8_t*)&len, sizeof(len));
  UART2.write((uint8_t*)&crc, sizeof(crc));
  UART2.flush(); // Đảm bảo marker và header đã được gửi hết
  // Gửi dữ liệu firmware
  UART2.write(data, len);
  
  if (!waitForKnobResponse("OK", 1000)) {
    Serial.println("Failed to send chunk to Knob!");
    server.send(500, "text/plain", "Failed to send chunk to Knob!");
    return;
  }
}

// Hàm chờ phản hồi từ Knob qua UART
bool waitForKnobResponse(const char* expected, uint32_t timeout) {
  uint32_t start = millis();
  while (millis() - start < timeout) {
    if (UART2.available()) {
      String response = UART2.readStringUntil('\n');
      response.trim();
      if (response == expected) {
        return true;
      }
    }
    delay(10);
  }
  return false;
}
