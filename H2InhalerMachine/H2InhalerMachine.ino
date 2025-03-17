#include <Arduino.h>
#include "driver/ledc.h"
#include "ADS1X15.h"

// Pin definitions
#define WATER_PUMP_PIN GPIO_NUM_4
#define AIR_PUMP_PIN GPIO_NUM_5

#define LIMIT_SW_PIN GPIO_NUM_6
#define WATER_LEAK_PIN GPIO_NUM_15
#define LEVEL_FLOAT_PIN GPIO_NUM_7

#define I2C_SDA_PIN GPIO_NUM_17
#define I2C_SCL_PIN GPIO_NUM_18

HardwareSerial UART2(1);
#define UART1_RX_PIN GPIO_NUM_44
#define UART1_TX_PIN GPIO_NUM_43

#define DEBOUNCE_TIME_MS 10

const char* ssid = "IoTVision_2.4GHz";
const char* password = "iotvision@2022";

// ADS1115 instance
ADS1115 ADS(0x48);

// ADS channels enumeration
typedef enum {
  PRESSURE_ADS_CHANNEL,
  TDS_ADS_CHANNEL,
  H2_SENSOR_ADS_CHANNEL,
  POTEN_ADS_CHANNEL
} ads_channel_t;

// Data structure for sensor readings
typedef struct
{
  uint32_t pressure_data;
  uint32_t tds_data;
  uint32_t h2_data;
  uint32_t poten_data;
} ads_data_t;

ads_data_t _ads_data;

typedef enum {
  HEPA_FILTER_MISSING = 0,
  WATER_LEAK_DETECTED,
  FLOAT_DETECTED,
  POOR_WATER_QUALITY,
  H2_ABOVE_1_PERCENT
} ErrorCode;

// Interrupt flags
volatile bool limitsw_interruptFlag = false;
volatile bool levelfloat_interruptFlag = false;
volatile bool waterleak_interruptFlag = false;

// PWM configuration
// const int pwmPin = 16;         // PWM signal output pin
// const int pwmFreq = 25000;     // PWM frequency
// const int pwmResolution = 10;  // 10-bit resolution
// const int pwmChannel = 0;      // PWM channel
const int pwmPin = 16;        // PWM pin
const int pwmFreq = 4000;     // PWM frequency
const int pwmResolution = 13; // 13 bit
const int pwmChannel = 1;     // PWM channel
float _voltage = 4.8;

float _Volfor100PercentH2 = 0;
float _Volfor1000ppmTDS = 0;  // max sensor is 1000ppm

uint32_t H2Value;

#define SENSOR_READ_TIME 1000  // 1s
uint32_t lastReadTimeSensor = 0;

String h2_percent;
String tds_data;
bool h2;
bool tds;

void IRAM_ATTR limitsw_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = esp_timer_get_time() / 1000;

  if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS) {
    limitsw_interruptFlag = true;
    last_isr_time = current_time;
  }
}

void IRAM_ATTR levelfloat_handleInterrupt() {
  static uint32_t last_isr_time = 0;
  uint32_t current_time = esp_timer_get_time() / 1000;

  if ((current_time - last_isr_time) > 1000) {
    levelfloat_interruptFlag = true;
    last_isr_time = current_time;
  }
}

volatile unsigned long lastDebounceTime = 0;
void IRAM_ATTR waterleak_handleInterrupt() {
  if ((millis() - lastDebounceTime) > 50) {
    waterleak_interruptFlag = true;
    lastDebounceTime = millis();
  }
}
//-------------------------------------------------------------
void send_error_to_Knob(ErrorCode error) {
  switch (error) {
    case HEPA_FILTER_MISSING:
      UART2.println("No HEPA filter detected");
      Serial.println("No HEPA filter detected-");
      break;
    case WATER_LEAK_DETECTED:
      UART2.println("Water Leak Detected");
      Serial.println("Water Leak Detected");
      break;
    case FLOAT_DETECTED:
      UART2.println("Float Detected");
      Serial.println("Float Detected-");
      break;
    case POOR_WATER_QUALITY:
      UART2.println("Poor Water Quality");
      Serial.println("Poor Water Quality-");
      break;
    case H2_ABOVE_1_PERCENT:
      UART2.println("H2 leak detected");
      Serial.println("H2 is above 1%-");
      break;
  }
}
//-------------------------------------------------------------
void ads_setting() {
  ADS.begin();
  ADS.setGain(0);      //  6.144 volt
  ADS.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
  ADS.setMode(0);      //  continuous mode
  ADS.readADC(0);      //  first read to trigger
}

void updatePWMVoltage(float voltage) {
  int dutyValue = map(voltage * 100, 470, 500, 1012, 1021);  // This number is adjusted for 4.8 - 5.0V because the graph is linear so use map
  // Serial.printf("Ledc write %d\n",dutyValue);
  ledcWrite(pwmPin, dutyValue);

  _voltage = voltage;

  if (voltage > 0) {
    digitalWrite(WATER_PUMP_PIN, HIGH);
    digitalWrite(AIR_PUMP_PIN, HIGH);
  } else {
    digitalWrite(WATER_PUMP_PIN, LOW);
    digitalWrite(AIR_PUMP_PIN, LOW);
  }
}

bool readH2Sensor() {
  ADS.readADC(H2_SENSOR_ADS_CHANNEL);
  _ads_data.h2_data = ADS.getValue();
  float f = ADS.toVoltage(1);
  uint32_t H2Value = (float)((_ads_data.h2_data * f) / _Volfor100PercentH2) * 100;
  // H2Value = h2_val;
  Serial.print("H2 Value: ");
  Serial.print(H2Value);
  Serial.println("%");
  if (H2Value > 1) {
    Serial.print("H2: ");
    Serial.println(H2Value);
    return false;
  } else {
    return true;
  }
}

bool readTDS() {
  ADS.readADC(TDS_ADS_CHANNEL);
  _ads_data.tds_data = ADS.getValue();

  float f = ADS.toVoltage(1);
  uint32_t tdsValue = (float)((_ads_data.tds_data * f) / _Volfor1000ppmTDS) * 1000;  // Convert _ads_data.tds_data -> tdsValue
  // uint32_t tdsValue = tds_val;
  Serial.print("TDS Value: ");
  Serial.print(tdsValue);
  Serial.println("ppm");

  if (tdsValue > 5) {
    Serial.println("TDS sensor triggered: High TDS level detected.");
    return false;
  } else {
    return true;
  }
}
//------------------------------------------
// void handleSerialCommands() {
//   if (Serial.available()) {
//     String receivedData = Serial.readStringUntil('\n');
//     receivedData.trim();

//     if (receivedData.startsWith("H2:")) {
//       h2_percent = receivedData.substring(3);  // "H2:"
//       Serial.printf("H2: %d\n", h2_percent.toInt());
//     } else if (receivedData.startsWith("TDS:")) {
//       tds_data = receivedData.substring(4);  // "TDS:"
//       Serial.printf("TDS: %d\n", tds_data.toInt());
//     }
//   }
// }


// Note : If H2 = 0 meaning Pause/Timeout wait until Resume/Start
// float _percentH2 = 0;
int flow;
int selectedEquation;
float airFlowRate;
void handleKnobCommand() {
  // Keep reading data until a value greater than 0 is received
  do {
    if (UART2.available()) {
        String receivedData = UART2.readStringUntil('\n');  // Read data from UART2
        Serial.print("Received data from Knob: ");
        Serial.println(receivedData);
        flow = receivedData.toInt();
      }
      int dutyValue;
      if (flow == 0)
      {
        updatePWMVoltage(0);
      }
      else if (flow >= 10 && flow <= 20) //OK
      {
        dutyValue = map(flow, 10, 20, 550, 935);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 20 && flow <= 30) //OK
      {
        dutyValue = map(flow, 20, 30, 935, 1274);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 30 && flow <= 40) //OK
      {
        dutyValue = map(flow, 30, 40, 1274, 1572);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 40 && flow <= 50) //OK
      {
        dutyValue = map(flow, 40, 50, 1572, 1845);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 50 && flow <= 60) //OK
      {
        dutyValue = map(flow, 50, 60, 1845, 2090);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 60 && flow <= 70) //OK
      {
        dutyValue = map(flow, 60, 70, 2090, 2296);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 70 && flow <= 80) //OK
      {
        dutyValue = map(flow, 70, 80, 2296, 2490);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 80 && flow <= 90) //OK
      {
        dutyValue = map(flow, 80, 90, 2490, 2600);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 90 && flow <= 100) //OK
      {
        dutyValue = map(flow, 90, 100, 2600, 2835);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 100 && flow <= 190) //OK
      {
        dutyValue = map(flow, 100, 190, 2835, 3985);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 190 && flow <= 230) //OK
      {
        dutyValue = map(flow, 200, 230, 4250, 5423);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 230 && flow <= 260) //OK
      {
        dutyValue = map(flow, 240, 260, 5990, 7350);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 260 && flow <= 290) //OK
      {
        dutyValue = map(flow, 270, 290, 7717, 7778);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 290 && flow <= 310) //OK
      {
        dutyValue = map(flow, 300, 310, 7797, 7814);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow == 320) //OK
      {
        dutyValue = 7827;
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow > 320 && flow <= 390) //OK
      {
        dutyValue = map(flow, 330, 390, 7828, 7954);
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else if (flow == 400) //OK
      {
        dutyValue = 7976;
        ledcWrite(pwmPin, dutyValue);
        Serial.printf("Flow: %d mL/min -> PWM Duty Cycle: %d\n", flow, dutyValue);
      }
      else
      {
        Serial.println("Invalid flow value! Enter a number between 10 and 400.");
      }
    delay(1000);
  } while (flow == 0);  // Continue waiting if flow is 0
}
//------------------------------------------
void setup() {
  Serial.begin(115200);
  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  updatePWMVoltage(_voltage);

  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(AIR_PUMP_PIN, OUTPUT);

  pinMode(LIMIT_SW_PIN, INPUT);
  pinMode(WATER_LEAK_PIN, INPUT_PULLDOWN);
  pinMode(LEVEL_FLOAT_PIN, INPUT);

  digitalWrite(WATER_PUMP_PIN, 0);
  digitalWrite(AIR_PUMP_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limitsw_handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEVEL_FLOAT_PIN), levelfloat_handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WATER_LEAK_PIN), waterleak_handleInterrupt, CHANGE);

  UART2.begin(9600, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
  ads_setting();

  _Volfor100PercentH2 = ADS.getMaxVoltage();  // Need to change if max voltage is not 100% H2
  _Volfor1000ppmTDS = ADS.getMaxVoltage();    // Need to change if max voltage is not 1000ppm TDS

}

void loop() {
  // handleSerialCommands();
  handleKnobCommand();
  if (limitsw_interruptFlag) {
    if (!digitalRead(LIMIT_SW_PIN)) {
      updatePWMVoltage(0);
      send_error_to_Knob(HEPA_FILTER_MISSING);
      delay(1000);
    } else {
      limitsw_interruptFlag = false;
    }
  }

  if(waterleak_interruptFlag) {
    delay(50);
    if (digitalRead(WATER_LEAK_PIN)) {
      updatePWMVoltage(0);
      send_error_to_Knob(WATER_LEAK_DETECTED);
      delay(1000);
    } else {
      waterleak_interruptFlag = false;
    }
  }

  if (levelfloat_interruptFlag) {
    if (!digitalRead(LEVEL_FLOAT_PIN)) {
        updatePWMVoltage(0);
        send_error_to_Knob(FLOAT_DETECTED);
        delay(1000);
    }
    else {
      levelfloat_interruptFlag = false;
    }
  }

  if (millis() - lastReadTimeSensor > SENSOR_READ_TIME) {
    lastReadTimeSensor = millis();
    h2 = readH2Sensor();
    tds = readTDS();

    if (!tds) {
      send_error_to_Knob(POOR_WATER_QUALITY);
      delay(1000);
    }
    if (!h2) {
      send_error_to_Knob(H2_ABOVE_1_PERCENT);
      delay(1000);
    }

    if (h2 && tds && !levelfloat_interruptFlag && !limitsw_interruptFlag && !waterleak_interruptFlag) {
      digitalWrite(WATER_PUMP_PIN, 1);
      digitalWrite(AIR_PUMP_PIN, 1);
      UART2.println("X");

    } else {
      digitalWrite(WATER_PUMP_PIN, 0);
      digitalWrite(AIR_PUMP_PIN, 0);
    }
  }
  delay(1);
}
