#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <DHT11.h>

// Create an instance of the DHT11 class.
// - For Arduino: Connect the sensor to Digital I/O Pin 2.
// - For ESP32: Connect the sensor to pin GPIO2 or P2.
// - For ESP8266: Connect the sensor to GPIO2 or D4.
DHT11 dht11(D5);

unsigned long lastDHTRead = 0;
const unsigned long DHTdataInterval = 5000;





Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

float RadarDist;

int frontAngle = 107;
// float targetYaw = -1;     // Alignment target
bool isAligning = false;  // Alignment mode active
// const float yawTolerance = 2;


// --- Yaw Control Variables ---
float filteredYaw = 0.0;
float alpha = 0.22;  // Filter strength (0.1 to 0.3 works well)
//alpha = 1.0 → No filtering (use raw magYaw)

//alpha = 0.5 → Medium smoothing (average new and previous)

//alpha = 0.1–0.3 → Good range for smoothing noisy yaw data




float yawError = 0.0;
float yawTolerance = 5.0;  // Deadband for alignment in degrees
float targetYaw = 0.0;

// --- Timing Control ---
unsigned long lastAlignCmdTime = 0;
const unsigned long minTurnGap = 150;  // Minimum 80 ms between turn commands

// bool isAligning = false;
// int alignmentPhase = 0;
// ---- Alignment State ----
bool aligningToYaw = false;  // Whether robot is currently aligning

// Alignment control


int alignmentPhase = 0;
// 0 = normal navigation
// 1 = alignment in progress
// 2 = alignment just completed


float magYaw;
int ALGOCMD;
int a1, a2, a3, a4, a5, a6, a7;
int d1, d2, d3, d4, d5, d6, d7;

#include "MPU6050_6Axis_MotionApps20.h"

// ========== BMP280 Setup ==========
Adafruit_BMP280 bmp;

// ========== MPU6050 Setup ==========
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define OUTPUT_READABLE_REALACCEL

// Simplified struct (1 byte payload)
typedef struct {
  bool scanDisValue;
  int alignmentPhase;
} message;
message outgoingData;

bool val;



#define TRIG_PIN_3 D7  // D5
#define ECHO_PIN_3 D8  // D0 (optional, ok to use if not using deep sleep)



float accX, accY;
float velX = 0, velY = 0;
float posX = 0, posY = 0;
unsigned long lastTime = 0;
int flag;

// ===== ESP-NOW Receiver MAC =====
uint8_t receiverMac[] = { 0x48, 0x3F, 0xDA, 0x47, 0x8E, 0x32 };  // MAC in byte array format

// ===== ESP-NOW Send Status Tracking =====
unsigned long lastEspNowSend = 0;
const unsigned long ESP_NOW_INTERVAL = 100;  // Send every 100ms




String serialBuffer = "";
bool scanDisValue = false;  // Stores latest ScanDis (0=auto, 1=manual)
unsigned long lastScanDisPrint = 0;
// Add these with your other global variables
unsigned long lastScanDisPrintTime = 0;
const unsigned long SCAN_DIS_PRINT_INTERVAL = 200;  // 500ms print interval





#define INTERRUPT_PIN D4  // Example for NodeMCU D5
#define LED_PIN 13        // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

// ========== MPU SETUP DONE ==========





// ========== Timers ==========
unsigned long lastSensorRead = 0;
unsigned long lastSerialSend = 0;
const unsigned long sensorInterval = 75;
const unsigned long sendInterval = 80;

// ========== Distance Tracking ==========
volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;
const float wheelCircumference = 21.8;  // cm

// ========== JSON Setup ==========
DynamicJsonDocument doc(1024);
bool dataReady = false;

// ========== ESP-NOW Struct ==========
typedef struct struct_message {
  float distance1;
  float distance2;
  float distance3;
  float angle;
  float RadarDist;
  float Pir;
  float ReadyToSendRadarDataTrig;
} struct_message;

struct_message incomingData;

// ========== Callbacks ==========
void onDataRecv(uint8_t *mac, uint8_t *incomingDataRaw, uint8_t len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
}



void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  // Serial.println("------------------------------------");
  // Serial.print("Sensor:       ");
  // Serial.println(sensor.name);
  // Serial.print("Driver Ver:   ");
  // Serial.println(sensor.version);
  // Serial.print("Unique ID:    ");
  // Serial.println(sensor.sensor_id);
  // Serial.print("Max Value:    ");
  // Serial.print(sensor.max_value);
  // Serial.println(" uT");
  // Serial.print("Min Value:    ");
  // Serial.print(sensor.min_value);
  // Serial.println(" uT");
  // Serial.print("Resolution:   ");
  // Serial.print(sensor.resolution);
  // Serial.println(" uT");
  // Serial.println("------------------------------------");
  // Serial.println("");
  delay(5);
}




void setup() {
  Serial.begin(115200);
  Wire.begin();
  mag.begin();


  // ===== BMP280 Init =====
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found!");
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 initialized.");
  }

  // ===== MPU6050 Init =====


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Wire.begin(4, 5);  // SDA=D2, SCL=D1 for NodeMCU

  // while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  delay(200);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);


  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  // pinMode(LED_PIN, OUTPUT);












  // ===== ESP-NOW Init =====
  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(0);  // Set to lowest TX power (0 = 0dBm ~ minimum)

  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onDataRecv);

  // ===== Hall Sensor Interrupts =====
  // attachInterrupt(digitalPinToInterrupt(D5), onLeftPulse, FALLING);
  // attachInterrupt(digitalPinToInterrupt(D6), onRightPulse, FALLING);
  // ===== ESP-NOW Sender Setup =====
  esp_now_add_peer(receiverMac, ESP_NOW_ROLE_COMBO, 1, NULL, 0);  // Add receiver
  displaySensorDetails();
}

void loop() {

  unsigned long now = millis();



  // ===== [1] Non-blocking ScanDis Receiver =====

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    // Serial.print("Raw input: "); Serial.println(input);
    if (input.startsWith("ScanDis:")) {
      val = input.substring(8).toInt() == 1;
      // Serial.print("Parsed value: "); Serial.println(val);
    }
  }


  // ===== [2] Print ScanDis every 500ms =====
  if (now - lastScanDisPrintTime >= SCAN_DIS_PRINT_INTERVAL) {
    // Serial.print("ScanDis:");
    // Serial.println(val);
    outgoingData.scanDisValue = val;
    outgoingData.alignmentPhase = alignmentPhase;

    esp_now_send(receiverMac, (uint8_t *)&outgoingData, sizeof(outgoingData));
    lastScanDisPrintTime = now;
  }



  if (now - lastSensorRead >= sensorInterval) {
    lastSensorRead = now;

    // ========== Read MPU6050 ==========

    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // Serial.print("ypr\t");
      // Serial.print(ypr[0] * 180 / M_PI);
      // Serial.print("\t");
      // Serial.print(ypr[1] * 180 / M_PI);
      // Serial.print("\t");
      // Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      // Serial.print("areal\t");
      // Serial.print(aaReal.x);
      // Serial.print("\t");
      // Serial.print(aaReal.y);
      // Serial.print("\t");
      // Serial.println(aaReal.z);
#endif


      // Constants
      const float accThreshold = 0.1;         // m/s², ignore small accelerations (noise)
      const float friction = 0.85;            // decay factor for velocity to simulate friction
      const float accScale = 9.81 / 16384.0;  // raw accel to m/s²

      // Get acceleration in m/s²
      accX = aaReal.x * accScale;
      accY = aaReal.y * accScale;

      // Apply noise threshold
      if (abs(accX) < accThreshold) accX = 0;
      if (abs(accY) < accThreshold) accY = 0;

      // Time delta in seconds
      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      lastTime = now;

      // Integrate acceleration to velocity (only when significant)
      if (accX != 0 || accY != 0) {
        velX += accX * dt;
        velY += accY * dt;
      } else {
        velX *= friction;  // decay when idle
        velY *= friction;
      }

      // Integrate velocity to position
      posX += velX * dt;
      posY += velY * dt;

      // Optional: Apply heading correction using yaw
      float yawRad = ypr[0];  // already in radians
      float globalX = cos(yawRad) * posX - sin(yawRad) * posY;
      float globalY = sin(yawRad) * posX + cos(yawRad) * posY;

      // Output
      // Serial.print("Yaw: "); Serial.print(yawRad * 180.0 / PI, 1);
      // Serial.print(" | PosX: "); Serial.print(globalX, 2);
      // Serial.print(" m | PosY: "); Serial.println(globalY, 2);

      // doc["X"] = ypr[0] * 180 / M_PI;
      doc["Y"] = ypr[1] * 180 / M_PI;
      doc["Z"] = ypr[2] * 180 / M_PI;

      doc["Ax"] = aaReal.x;
      doc["Ay"] = aaReal.y;
      doc["Az"] = aaReal.z;
      doc["PosX"] = globalX;
      doc["PosY"] = globalY;


      int taapmaan = 0;
      int aadrata;

      if (now - lastDHTRead >= DHTdataInterval) {
        lastDHTRead = now;

        dht11.readTemperatureHumidity(taapmaan, aadrata);
      }
      aadrata = constrain(aadrata, 0, 100);
      doc["Humid"] = aadrata;


      sensors_event_t event;
      mag.getEvent(&event);



      float heading = atan2(event.magnetic.y, event.magnetic.x);

      float declinationAngle = 0.01192;
      heading += declinationAngle;

      // Correct for when signs are reversed.
      if (heading < 0)
        heading += 2 * PI;

      // Check for wrap due to addition of declination.
      if (heading > 2 * PI)
        heading -= 2 * PI;

      // Convert radians to degrees for readability.
      float headingDegrees = heading * 180 / M_PI;
      magYaw = headingDegrees;
      // magYaw = alpha * magYaw + (1 - alpha) * filteredYaw;
      filteredYaw = magYaw;

      doc["magYaw"] = magYaw;
      doc["X"] = magYaw;  ////modify later cause this is supposed to be of mpu yaw reading


      // Serial.print("Heading (degrees): ");
      // Serial.println(headingDegrees);
      // if (headingDegrees > 307 && headingDegrees < 309) Serial.print("FACING NORTH ");
    }


    if (incomingData.ReadyToSendRadarDataTrig == 1) {
      // Update radar data (same as before)
      flag = 1;
      if (incomingData.angle == 0) {
        doc["a1"] = incomingData.angle;
        doc["d1"] = incomingData.RadarDist;
        a1 = incomingData.angle;
        d1 = incomingData.RadarDist;
      }
      if (incomingData.angle == 30) {
        doc["a2"] = incomingData.angle;
        doc["d2"] = incomingData.RadarDist;
        a2 = incomingData.angle;
        d2 = incomingData.RadarDist;
      }
      if (incomingData.angle == 60) {
        doc["a3"] = incomingData.angle;
        doc["d3"] = incomingData.RadarDist;
        a3 = incomingData.angle;
        d3 = incomingData.RadarDist;
      }
      if (incomingData.angle == 90) {
        doc["a4"] = incomingData.angle;
        doc["d4"] = incomingData.RadarDist;
        a4 = incomingData.angle;
        d4 = incomingData.RadarDist;
      }
      if (incomingData.angle == 120) {
        doc["a5"] = incomingData.angle;
        doc["d5"] = incomingData.RadarDist;
        a5 = incomingData.angle;
        d5 = incomingData.RadarDist;
      }
      if (incomingData.angle == 150) {
        doc["a6"] = incomingData.angle;
        doc["d6"] = incomingData.RadarDist;
        a6 = incomingData.angle;
        d6 = incomingData.RadarDist;
      }
      if (incomingData.angle == 180) {
        doc["a7"] = incomingData.angle;
        doc["d7"] = incomingData.RadarDist;
        a7 = incomingData.angle;
        d7 = incomingData.RadarDist;
      }



      // Check if ALL radar data is available (a1-a7 and d1-d7)
      bool allRadarDataReady =
        doc.containsKey("a1") && doc.containsKey("d1") && doc.containsKey("a2") && doc.containsKey("d2") && doc.containsKey("a3") && doc.containsKey("d3") && doc.containsKey("a4") && doc.containsKey("d4") && doc.containsKey("a5") && doc.containsKey("d5") && doc.containsKey("a6") && doc.containsKey("d6") && doc.containsKey("a7") && doc.containsKey("d7");

      // Only proceed if all radar data is ready
      if (!allRadarDataReady) {
        dataReady = false;  // Prevent sending incomplete radar data
      }
    } else {
      // Clear radar data if not ready to send
      doc.remove("a1");
      doc.remove("d1");
      doc.remove("a2");
      doc.remove("d2");
      doc.remove("a3");
      doc.remove("d3");
      doc.remove("a4");
      doc.remove("d4");
      doc.remove("a5");
      doc.remove("d5");
      doc.remove("a6");
      doc.remove("d6");
      doc.remove("a7");
      doc.remove("d7");
      flag = 0;
    }

    // ========== Rest of your existing code (BMP280, other sensors, etc.) ==========
    doc["Temp"] = bmp.readTemperature();
    doc["Pressure"] = bmp.readPressure() / 100.0F;
    doc["Altitude"] = bmp.readAltitude(1013.25);
    doc["distance1"] = incomingData.distance1;
    doc["distance2"] = incomingData.distance2;
    doc["distance3"] = getDistance(TRIG_PIN_3, ECHO_PIN_3);
    doc["angle"] = incomingData.angle;
    doc["RadarDist"] = incomingData.RadarDist;
    RadarDist = incomingData.RadarDist;
    doc["Pir"] = incomingData.Pir;
    doc["ReadyToSendRadarDataTrig"] = incomingData.ReadyToSendRadarDataTrig;
    doc["ir1"] = 0;
    doc["ir2"] = 1;
    doc["Fire"] = 5;
    doc["LeftPulses"] = leftCount;
    doc["RightPulses"] = rightCount;



    // Only set dataReady=true if all radar data is present (or if radar data is not being sent)
    if (incomingData.ReadyToSendRadarDataTrig != 1 || (doc.containsKey("a1") && doc.containsKey("a7"))) {
      dataReady = true;
    }
  }
  if (dataReady && now - lastSerialSend >= sendInterval) {
    lastSerialSend = now;
    // printRadarOneLine();

    //Calculations For ALGO-MODE Commands


    // 🧠 ALGO-MODE Command Calculations
    if (isAligning) {
      yawError = targetYaw - filteredYaw;

      if (yawError > 180.0) yawError -= 360.0;
      if (yawError < -180.0) yawError += 360.0;

      // Serial.print("🔁 Aligning... yawError = ");
      // Serial.println(yawError);

      if (abs(yawError) <= yawTolerance) {
        ALGOCMD = 1;  // Forward
        isAligning = false;
        alignmentPhase = 0;  // Alignment just completed
        // Serial.println("✅ Alignment complete. Moving FORWARD.");
      } else if (yawError > 0) {
        ALGOCMD = 4;  // Turn RIGHT
        // Serial.println("↪ Turning RIGHT to align...");
      } else {
        ALGOCMD = 3;  // Turn LEFT
        // Serial.println("↩ Turning LEFT to align...");
      }
    } else if (RadarDist < 49) {
      ALGOCMD = 5;
      // Serial.println("⛔ Obstacle detected! Stopping...");

      if (flag == 1) {
        // Serial.println("🔄 Scanning: Calling processRadarDecision()");
        processRadarDecision();  // Triggers alignment
      } else {
        // Serial.println("❗ flag = 0 and not aligning — no action taken.");
      }

    } else {
      ALGOCMD = 1;
      alignmentPhase = 0;  // Back to normal navigation
      // Serial.println("✅ Path clear — moving FORWARD");
    }



    // 🛠️ Debug: Alignment state print
    // switch (alignmentPhase) {
    //   case 0: Serial.println("🟢 alignmentPhase = 0 → Normal navigation"); break;
    //   case 1: Serial.println("🟡 alignmentPhase = 1 → Alignment in progress"); break;
    //   case 2: Serial.println("✅ alignmentPhase = 2 → Alignment complete"); break;
    //   default: Serial.println("⚠️ Unknown alignmentPhase state!"); break;
    // }



    // ALGOCMD = 2;

    doc["ALGOCMD"] = ALGOCMD;
    doc["targetYaw"] = targetYaw;

    // Serial.println(ALGOCMD);

    serializeJson(doc, Serial);
    Serial.println();
    dataReady = false;
  }
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




// Constants and Inputs
// float magYaw; // Current yaw from magnetometer in degrees (0 to 359)
// int a1, a2, a3, a4, a5, a6, a7; // Angles scanned
// float d1, d2, d3, d4, d5, d6, d7; // Corresponding obstacle distances in cm
// int ALGOCMD = 5; // Final decision command: 1=F, 2=B, 3=L, 4=R, 5=S (default STOP)

// int flag = 1; // Flag to trigger logic (obstacle in front)
// int frontAngle = 107; // Angle representing "front" in radar scan

void processRadarDecision() {
  // Serial.println("🔄 Obstacle ahead! Processing radar scan...");

  // Step 1: Find the angle with the farthest obstacle
  float maxDist = d1;
  int bestAngle = a1;

  if (d2 > maxDist) {
    maxDist = d2;
    bestAngle = a2;
  }
  if (d3 > maxDist) {
    maxDist = d3;
    bestAngle = a3;
  }
  if (d4 > maxDist) {
    maxDist = d4;
    bestAngle = a4;
  }
  if (d5 > maxDist) {
    maxDist = d5;
    bestAngle = a5;
  }
  if (d6 > maxDist) {
    maxDist = d6;
    bestAngle = a6;
  }
  if (d7 > maxDist) {
    maxDist = d7;
    bestAngle = a7;
  }

  // Serial.print("📍 Farthest obstacle-free angle: ");
  // Serial.print(bestAngle);
  // Serial.print("° with distance: ");
  // Serial.print(maxDist);
  // Serial.println(" cm");

  // Step 2: Check if all directions are blocked (<15 cm)
  bool allBlocked = true;
  if (d1 > 21) allBlocked = false;
  if (d2 > 21) allBlocked = false;
  if (d3 > 21) allBlocked = false;
  if (d4 > 21) allBlocked = false;
  if (d5 > 21) allBlocked = false;
  if (d6 > 21) allBlocked = false;
  if (d7 > 21) allBlocked = false;

  // Serial.print("🧱 All directions blocked? ");
  // Serial.println(allBlocked ? "YES" : "NO");


  // Step 3: Compute desired yaw to face best direction
  if (allBlocked) {
    // Turn 90° LEFT if completely blocked
    targetYaw = magYaw - 180.0;  // Physically rotate counterclockwise
    // Serial.println("🚫 All directions blocked — turning 90° LEFT");
  } else {
    float angleDiff = bestAngle - frontAngle;
    targetYaw = magYaw + angleDiff;
    // Serial.print("📐 Angle offset from front: ");
    // Serial.print(angleDiff);
    // Serial.println("°");
  }

  // Step 4: Normalize targetYaw to 0–360 range
  if (targetYaw >= 360.0) targetYaw -= 360.0;
  if (targetYaw < 0.0) targetYaw += 360.0;

  // Serial.print("🧭 Current magYaw: ");
  // Serial.print(magYaw);
  // Serial.print("°, Target yaw: ");
  // Serial.print(targetYaw);
  // Serial.println("°");

  // Step 5: Start alignment
  isAligning = true;
  alignmentPhase = 1;  // Alignment started

  // Serial.println("🚦 Alignment triggered. isAligning = true");
}





long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);  // 20ms timeout (adjust if needed)
  if (duration == 0) return -1;                   // No object detected
  return duration * 0.034 / 2;
}

//Tried and tested code By - Naitik Srivastava
