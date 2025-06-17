#include "POSTGET.h"

#include "POSTGET.h"
#include <WiFi.h>  // Thư viện WiFi của ESP32.

// #include <WiFiClientSecure.h>
// #include <HTTPClient.h>
// #include <ArduinoJson.h>

#define URL_GETLenhGuiXuongBoard "http://App.IoTVision.vn/api/EV_DuLieuGuiXuongBoard?CheDo=1&key="
#define URL_POSTDuLieuTuBoardLenServer "https://App.IoTVision.vn/api/EV_DuLieu"//https://App.IoTVision.vn/api/EV_DuLieu
#define URL_CapNhatCODE "http://App.IoTVision.vn/api/EV_DuLieuGuiXuongBoard"
#define URL_GetTienNhanDuoc  "https://oauth.casso.vn/v2/transactions?pageSize=3&sort=DESC"
#define ApiKey  "AK_CS.79089d30896311ef88510fbab66d5d63.IKWZe4pFlQBFI68hMMZuXIRo10w6hJr4SZoBidklNpMtcz4lAUcUxK3ueNE1Q7VoGFExa1Pw"


#define URL_GETParameterFromServer ""
#define URL_POSTParameterToServer ""
// Thiết lập thời gian timeout.
#define timeout 1500 //60000

//=================================================================================
// Ví dụ, DuLieu = "1089;210;13.3;20:06:13 18/08/2023"
// 1089 => K1 = 1, MODE = 0 (MAN), RSSI WiFi = 89%
// Tốc độ lắc = 210 (RPM)
// Thời gian lắc = 13.3 (giờ)
// HH:MM:SS DD/MM/YYYY = 20:06:13 18/08/2023
//=================================================================================
// bool POSTGET::POSTDuLieuBoard(String ID, String DuLieu) {
// #if defined(debug) && defined(POST_GET)  
//   if (ID.isEmpty()) {
//     Serial.printf("\n\n\t ID TRONG \n\n");
//   }
//   if (DuLieu.isEmpty()) {
//     Serial.printf("\n\n\t DuLieu TRONG \n\n");
//   }
// #endif
  
//   try {
//     if (WiFi.status() == WL_CONNECTED) {
//       this->http.setTimeout(timeout);  // Thiết lập thời gian timeout.
//       this->http.begin(URL_POSTDuLieuTuBoardLenServer);
//       this->http.addHeader("Content-Type", "application/json");
//       String data = "{\"ID\":\"" + ID + "\",\"S\":\"" + DuLieu + "\"}";
//       this->http.POST(data);  //Send the request 
//       this->http.end();
//       return 1;
//     } else
//       return 0;
//   } catch (String error) {
//     return 0;
//   }
// }

bool POSTGET::POSTParameterToServer(String ID, String DataToPOST) {
  try {
    if (WiFi.status() == WL_CONNECTED) {
      this->http.setTimeout(timeout);   // set timeout for POST
      this->http.begin(URL_POSTParameterToServer);
      this->http.addHeader("Content-Type", "application/json");
      String data = "{\"ID\":\"" + ID + "\",\"S\":\"" + DataToPOST + "\"}";
      this->http.POST(data);  // send the request
      this->http.end();
      return 1;
    }
    else
      return 0;

  }
  catch (String error) {
    return 0;
  }
}

String POSTGET::GETLenhGuiXuongBoard(String ID) {
  try {
    if (WiFi.status() == WL_CONNECTED) {
      String url = URL_GETLenhGuiXuongBoard + ID;
      this->http.setTimeout(timeout);  // Thiết lập thời gian timeout.
      this->http.begin(url);
      this->http.addHeader("Content-Type", "application/json");
      Serial.print("url: ");
      Serial.println(url);
#if defined(debug) && defined(POST_GET)
      unsigned long ms = millis();
#endif
      int httpCodeGet = this->http.GET();
      if (httpCodeGet == HTTP_CODE_OK) {  // needs a 200 to continue...
        String data = this->http.getString();
        data.remove(0, 1);                  // Loại ký tự '[' tại đầu chuỗi.
        data.remove(data.length() - 1, 1);  // Loại ký tự ']' tại cuối chuỗi.
        this->http.end();

#if defined(debug) && defined(POST_GET)
        Serial.print(millis() - ms);
        Serial.print("(ms) for GET ");
        Serial.println(data);
#endif

        return data;
        Serial.print("data: ");
        Serial.println(data);
      } else {
        Serial.println(this->http.errorToString(httpCodeGet).c_str());
#if defined(debug) && defined(POST_GET)
        Serial.println(this->http.errorToString(httpCodeGet).c_str());
#endif
        this->http.end();
        return "";
      }
    } else {
      Serial.println("Mất kết nối WIFI!");
#if defined(debug) && defined(POST_GET)
      Serial.println("Mất kết nối WIFI!");
#endif
      return "";
    }
  } catch (String error) {
    return "";
  }
}

// String POSTGET::GETSoTienGuiXuongBoard(void)
// {
//   try {
//     if (WiFi.status() == WL_CONNECTED) {
//       String url = URL_GetTienNhanDuoc;
//       this->http.setTimeout(timeout);  // Thiết lập thời gian timeout.
//       this->http.begin(url);
//       this->http.addHeader("Content-Type", "application/json");
//       this->http.addHeader("Authorization", String("Apikey ") + ApiKey);

//       int httpResponseCode = this->http.GET();

//       if (httpResponseCode == HTTP_CODE_OK) {
//         String response = this->http.getString();
//         //Serial.println(response); // In ra phản hồi từ API để kiểm tra
//         this->http.end();

//         return response;
//       } else {
//         Serial.println(this->http.errorToString(httpResponseCode).c_str());
//         this->http.end();
//         return "";
//       }
//     } else {
//         Serial.println("Mất kết nối WIFI!");
//         return "";
//     }
//   } catch (String error) {
//     return "Loi roi";
//   }
// }

// //=================================================================================
// // Ví dụ, DuLieu = "1011"
// // K1 = 1, MODE = 0 (MAN), CODE = 11
// //=================================================================================
// void POSTGET::CapNhatCODETrongDatabaseTrenServer(String ID, String DuLieu) {
//   if (WiFi.status() == WL_CONNECTED) {
//     this->http.setTimeout(timeout);  // Thiết lập thời gian timeout.
//     this->http.addHeader("Content-Type", "application/json");
//     this->http.begin(URL_CapNhatCODE);
//     this->http.addHeader("Content-Type", "application/json");

//     String data = "{\"ID\":\"" + ID + "\",\"S\":\"" + DuLieu + "\"}";
//     Serial.println("================================================");
//     Serial.println(data);
//     this->http.POST(data);  //Send the request
//     this->http.end();
//   }
// }

