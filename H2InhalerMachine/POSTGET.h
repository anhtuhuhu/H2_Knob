#ifndef _POSTGET_h
#define _POSTGET_h

#include <Arduino.h>      // Để khai báo kiểu String
#include <HTTPClient.h>   // Cho phép khai báo kiểu 

// #include <WiFiClientSecure.h>
// #include <ArduinoJson.h>

class POSTGET {
private:
  HTTPClient http;
   

public:
  // String GETSoTienGuiXuongBoard(void);
  // bool POSTDuLieuBoard(String ID, String DuLieu);
  String GETLenhGuiXuongBoard(String ID);
  // void CapNhatCODETrongDatabaseTrenServer(String ID, String DuLieu);

  bool POSTParameterToServer(String ID, String Data);
  String GETParameterFromServer(void);
};

#endif  // _POSTGET_h
