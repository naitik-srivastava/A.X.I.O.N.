#include <ESP8266WiFi.h>
extern "C" {
#include <espnow.h>
}
#include <ArduinoJson.h>

#define ENA D4  /////left motor
#define ENB D8  /////right motor
#define IN_1 D7
#define IN_2 D3
#define IN_3 D5
#define IN_4 D6
// #define BUZZER D0  // Changed from D8 to avoid conflict with encoder

// Wheel encoder pins
#define SENSOR_PIN D1   // Right wheel (GPIO13)
#define SENSOR_PIN2 D2  // Left wheel (GPIO15)

// Wheel encoder variables
volatile int holeCount1 = 0;  // Right wheel
volatile int holeCount2 = 0;  // Left wheel
volatile unsigned long lastInterruptTime1 = 0;
volatile unsigned long lastInterruptTime2 = 0;


int PWM=0;


const float wheelCircumference = 21.4;  // cm
const int holesPerRevolution = 20;
unsigned long lastEncoderUpdate = 0;
const unsigned long ENCODER_UPDATE_INTERVAL = 100;  // Update every 100ms

float FixedV = 25.0;
float CalibV = 1589;
float volt;
int Rd;
int ALGOCMD;

int ReadyToSendRadarDataTrig;

bool lastButtonState = 0;
unsigned long lastToggleTime = 0;
const unsigned long debounceDelay = 400;

unsigned long lastScanDisUpdate = 0;
const unsigned long SCAN_DIS_INTERVAL = 200;  // Update every 200ms

// Global radar variables
float a1, a2, a3, a4, a5, a6, a7;
float d1, d2, d3, d4, d5, d6, d7;
bool radarScanComplete = false;
unsigned long radarProcessingStartTime = 0;
const unsigned long radarProcessingDuration = 100;  // ms to process radar data

// Deadzone and minimum speed threshold
const int MIN_SPEED = 70;  // Below this, motor is off

volatile bool hasReceivedJoystick = false;

// Variables
String movementCommand;
int speedCar = 255;
float speed_Coeff = 2.5;
int distance1 = 100, distance2 = 100, distance3 = 100;
int ir1 = 0, ir2 = 0;
volatile bool isAutoMode = false;

// ESP-NOW
uint8_t masterMac[6];
bool masterKnown = false;
unsigned long lastSuccessfulSend = 0;
const unsigned long sendTimeout = 3000;  // Restart if no send in 3 sec
bool sendCallbackCalled = false;

// Timing
unsigned long lastSerialCheck = 0;
unsigned long lastMovementCheck = 0;
const unsigned long serialInterval = 20;
const unsigned long movementInterval = 10;

unsigned long lastRecvTime = 0;
const int SIGNAL_TIMEOUT = 1000;

// Data structures
struct JoystickData {
  float x;
  float y;
  int sw;
  int Btn;
};
JoystickData data;

struct SlaveData {
  int Humid;
  float Temp;
  float Pressure;
  float Alt;
  float Voltage;
  int distance1, distance2, distance3;
  int ir1, ir2, Fire, angle;
  float X, Y, Z, Ax, Ay, Az;
  int RadarDist;
  int Pir;
  int mode;
  float PosX;  // Right wheel distance
  float PosY;  // Left wheel distance
  int ALGOCMD;
  int targetYaw;
  char cmd[4];  // enough for "FL", "BR", etc. + null terminator
};
SlaveData sData;

// Encoder ISRs
void IRAM_ATTR countHole1() {
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime1 > 2000) {  // 2ms debounce
    holeCount1++;
    lastInterruptTime1 = currentTime;
  }
}

void IRAM_ATTR countHole2() {
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime2 > 2000) {
    holeCount2++;
    lastInterruptTime2 = currentTime;
  }
}

// Callbacks
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&data, incomingData, sizeof(data));
  lastRecvTime = millis();
  hasReceivedJoystick = true;

  if (!masterKnown) {
    memcpy(masterMac, mac, 6);
    esp_now_add_peer(mac, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);
    masterKnown = true;
  }
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) {
    lastSuccessfulSend = millis();
    sendCallbackCalled = true;
  } else {
    // Send failed
  }
}

void updateEncoderDistances() {
  // Calculate distances for both wheels
  sData.PosX = (float)holeCount1 / holesPerRevolution * wheelCircumference;  // Right wheel
  sData.PosY = (float)holeCount2 / holesPerRevolution * wheelCircumference;  // Left wheel
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);
  // WiFi.setOutputPower(18.5);  // Set to lowest TX power (0 = 0dBm ~ minimum)


  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(ENA, LOW);


  // Encoder pins
  pinMode(SENSOR_PIN, INPUT);
  pinMode(SENSOR_PIN2, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countHole1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN2), countHole2, FALLING);

  // ESP-NOW init
  if (esp_now_init() != 0) {
    ESP.restart();
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  stopRobot();
  lastSuccessfulSend = millis();
}

void sendDataToMaster(SlaveData &sData) {
  if (!masterKnown) return;

  uint8_t retryCount = 0;
  while (retryCount < 3) {
    esp_now_send(masterMac, (uint8_t *)&sData, sizeof(sData));
    delay(10);
    retryCount++;
  }
}

void processSerialData() {
  int sensorValue = analogRead(A0);
  volt = (sensorValue * FixedV) / CalibV;

  static String input = "";
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      input.trim();
      if (input.length() > 0) {
        DynamicJsonDocument doc(1024);
        DeserializationError err = deserializeJson(doc, input);
        if (!err) {
          sData = {
            .Humid = doc["Humid"],
            .Temp = doc["Temp"],
            .Pressure = doc["Pressure"],
            .Alt = doc["Altitude"],
            .Voltage = volt,
            .distance1 = doc["distance1"],
            .distance2 = doc["distance2"],
            .distance3 = doc["distance3"],
            .ir1 = doc["ir1"],
            .ir2 = doc["ir2"],
            .Fire = doc["Fire"],
            .angle = doc["angle"],
            .X = doc["X"],
            .Y = doc["Y"],
            .Z = doc["Z"],
            .Ax = doc["Ax"],
            .Ay = doc["Ay"],
            .Az = doc["Az"],
            .RadarDist = doc["RadarDist"],
            .Pir = doc["Pir"],
            .PosX = sData.PosX,  // Right wheel distance
            .PosY = sData.PosY,  // Left wheel distance
            .ALGOCMD = doc["ALGOCMD"],
            .targetYaw = doc["targetYaw"]
          };

          ReadyToSendRadarDataTrig = doc["ReadyToSendRadarDataTrig"];
          Rd = doc["RadarDist"];
          ALGOCMD = doc["ALGOCMD"];
          PWM= doc["PWM"];

          if (isAutoMode && ReadyToSendRadarDataTrig == 1) {
            if (doc.containsKey("a1")) {
              a1 = doc["a1"];
              d1 = doc["d1"];
            }
            if (doc.containsKey("a2")) {
              a2 = doc["a2"];
              d2 = doc["d2"];
            }
            if (doc.containsKey("a3")) {
              a3 = doc["a3"];
              d3 = doc["d3"];
            }
            if (doc.containsKey("a4")) {
              a4 = doc["a4"];
              d4 = doc["d4"];
            }
            if (doc.containsKey("a5")) {
              a5 = doc["a5"];
              d5 = doc["d5"];
            }
            if (doc.containsKey("a6")) {
              a6 = doc["a6"];
              d6 = doc["d6"];
            }
            if (doc.containsKey("a7")) {
              a7 = doc["a7"];
              d7 = doc["d7"];
            }

            if (a1 == 0 && a2 == 30 && a3 == 60 && a4 == 90 && a5 == 120 && a6 == 150 && a7 == 180) {
              radarScanComplete = true;
              radarProcessingStartTime = millis();
            }
          }
          sData.mode = isAutoMode ? 0 : 1;
          sendDataToMaster(sData);
        }
      }
      input = "";
    } else {
      input += ch;
    }
  }
}

void processMovement() {
  static bool buttonActive = false;

  // Toggle Mode Handling
  if (data.Btn == 1) {
    if (!buttonActive && (millis() - lastToggleTime) > debounceDelay) {
      isAutoMode = !isAutoMode;
      lastToggleTime = millis();
      buttonActive = true;
      Serial.print("Mode changed to: ");
      Serial.println(isAutoMode ? "ALGO" : "MANUAL");
    }
  } else {
    buttonActive = false;
  }

  // Update encoder distances regularly
  if (millis() - lastEncoderUpdate >= ENCODER_UPDATE_INTERVAL) {
    updateEncoderDistances();
    lastEncoderUpdate = millis();
  }

  // Send ScanDis every 200ms
  if (millis() - lastScanDisUpdate >= SCAN_DIS_INTERVAL) {
    Serial.print("ScanDis:");
    Serial.println(isAutoMode ? "0" : "1");
    lastScanDisUpdate = millis();
  }

  if (isAutoMode) {
    processAlgoModeCommands();
  } else {
    int xAxis = data.x;
    int yAxis = data.y;
    String cmd = "";

    if (xAxis > 1900 && yAxis < 1300) {
      cmd = "BR";
      strcpy(sData.cmd, "BR");
    } else if (xAxis > 1900 && yAxis > 1900) {
      cmd = "BL";
      strcpy(sData.cmd, "BL");
    } else if (xAxis < 1300 && yAxis < 1300) {
      cmd = "FR";
      strcpy(sData.cmd, "FR");
    } else if (xAxis < 1300 && yAxis > 1900) {
      cmd = "FL";
      strcpy(sData.cmd, "FL");
    } else if (xAxis < 1300) {
      cmd = "F";
      strcpy(sData.cmd, "F");
    } else if (xAxis > 1900) {
      cmd = "B";
      strcpy(sData.cmd, "B");
    } else if (yAxis > 1900) {
      cmd = "L";
      strcpy(sData.cmd, "L");
    } else if (yAxis < 1300) {
      cmd = "R";
      strcpy(sData.cmd, "R");
    } else {
      cmd = "S";
      strcpy(sData.cmd, "S");
    }

    if (cmd == "B") {
      int speed = map(xAxis, 1670, 4000, 0, 255);
      goBack(speed, speed);
    }
    if (cmd == "F") {
      int speed = map(xAxis, 1340, 0, 0, 255);
      goAhead(speed, speed);
    }
    if (cmd == "FL") {
      int throttle = map(xAxis, 1370, 0, 0, 255);
      throttle = constrain(throttle, 0, 255);
      int curve = map(yAxis, 1670, 4000, 255, 50);
      curve = constrain(curve, 50, 255);
      goAheadLeft(curve, throttle);
    }
    if (cmd == "FR") {
      int throttle = map(xAxis, 1370, 0, 0, 255);
      throttle = constrain(throttle, 0, 255);
      int curve = map(yAxis, 1370, 0, 255, 50);
      curve = constrain(curve, 50, 255);
      goAheadRight(throttle, curve);
    }
    if (cmd == "BL") {
      int throttle = map(xAxis, 1670, 4000, 0, 255);
      throttle = constrain(throttle, 0, 255);
      int curve = map(yAxis, 1670, 4000, 255, 50);
      curve = constrain(curve, 50, 255);
      goBackLeft(curve, throttle);
    }
    if (cmd == "BR") {
      int throttle = map(xAxis, 1670, 4000, 0, 255);
      throttle = constrain(throttle, 0, 255);
      int curve = map(yAxis, 1370, 0, 255, 50);
      curve = constrain(curve, 50, 255);
      goBackRight(throttle, curve);
    }
    if (cmd == "L") {
      int speed = map(yAxis, 1670, 4000, 0, 255);
      goLeft(speed, speed);
    }
    if (cmd == "R") {
      int speed = map(yAxis, 1350, 0, 0, 255);
      goRight(speed, speed);
    }
    if (cmd == "S") {
      stopRobot();
    }
  }
}

void loop() {
  unsigned long now = millis();

  // Watchdog
  if (masterKnown && (now - lastSuccessfulSend > sendTimeout)) {
    ESP.restart();
  }
  if (!sendCallbackCalled && masterKnown) {
    lastSuccessfulSend = now;
  }

  // Main tasks
  if (now - lastMovementCheck >= movementInterval) {
    lastMovementCheck = now;
    if (hasReceivedJoystick) {
      processMovement();
    }
  }
  if (now - lastSerialCheck >= serialInterval) {
    lastSerialCheck = now;
    processSerialData();
  }

  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
    stopRobot();
  }

  delay(3);
  yield();
}

// Motor control functions remain the same as in your original code
void goAhead(int speedA, int speedB) {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(ENA, speedA);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(ENB, speedB);
}

void goBack(int speedA, int speedB) {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(ENA, speedA);
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(ENB, speedB);
}

void goRight(int speedA, int speedB) {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(ENA, speedA);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(ENB, speedB);
}

void goLeft(int speedA, int speedB) {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(ENA, speedA);
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(ENB, speedB);
}

void goAheadRight(int speedA, int speedB) {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(ENA, speedA);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(ENB, speedB);
}

void goAheadLeft(int speedA, int speedB) {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(ENA, speedA);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(ENB, speedB);
}

void goBackRight(int speedA, int speedB) {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(ENA, speedA);
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(ENB, speedB);
}

void goBackLeft(int speedA, int speedB) {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(ENA, speedA);
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(ENB, speedB);
}

void stopRobot() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
  analogWrite(ENA, speedCar);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
  analogWrite(ENB, speedCar);
}


void printRadarOneLine() {
  // Angles then distances
  Serial.print(a1);
  Serial.print(" ");
  Serial.print(a2);
  Serial.print(" ");
  Serial.print(a3);
  Serial.print(" ");
  Serial.print(a4);
  Serial.print(" ");
  Serial.print(a5);
  Serial.print(" ");
  Serial.print(a6);
  Serial.print(" ");
  Serial.print(a7);
  Serial.println(" | ");

  Serial.print(d1);
  Serial.print(" ");
  Serial.print(d2);
  Serial.print(" ");
  Serial.print(d3);
  Serial.print(" ");
  Serial.print(d4);
  Serial.print(" ");
  Serial.print(d5);
  Serial.print(" ");
  Serial.print(d6);
  Serial.print(" ");
  Serial.print(d7);
  Serial.println();
}


void processAlgoModeCommands() {

  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
    stopRobot();
    return;
  }


  switch (ALGOCMD) {
    case 1:
      goAhead(240, 240);  // Forward
      // Serial.println("🚗 Moving FORWARD");
      break;
    case 2:
      goBack(225, 225);  // Backward
      // Serial.println("🔙 Moving BACKWARD");
      break;
    case 3:
      goLeft(PWM, PWM);  // Turn left
      // Serial.println("↩ Turning LEFT");
      break;
    case 4:
      goRight(PWM, PWM);  // Turn right
      // Serial.println("↪ Turning RIGHT");
      break;
    case 5:
      stopRobot();  // Stop
      // Serial.println("🛑 Stopping");
      break;
    default:
      // Serial.println("⚠️ Unknown command");
      stopRobot();  // Safe fallback
      break;
  }
}
//Tried and tested code By - Naitik Srivastava
