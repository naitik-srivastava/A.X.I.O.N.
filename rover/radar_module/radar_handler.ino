#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>

#define pirPin D5  // PIR sensor output connected to digital pin 2

// Global variables


Servo myServo;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int frontAngle = 107;
int motion;
int direction = 1;
const int stepSize = 15;  // Servo step per scan


typedef struct {
  bool scanDisValue;
  int alignmentPhase;

} message;

message msg;


VL53L0X_RangingMeasurementData_t measure;


// Define the struct to send
typedef struct struct_message {
  float distance1;
  float distance2;
  float distance3;
  float angle;
  float RadarDist;
  float Pir;
  float ReadyToSendRadarDataTrig;

} struct_message;

struct_message dataToSend;

int distance1, distance2, distance3;

// Ultrasonic Sensor Pins
// #define TRIG_PIN_1   D2   // D2
// #define ECHO_PIN_1   D1    // D1
#define TRIG_PIN_2 D6  // D6
#define ECHO_PIN_2 D7  // D7
// #define TRIG_PIN_3   D3   // D5
// #define ECHO_PIN_3   D4   // D0 (optional, ok to use if not using deep sleep)

#define fire D3  // D3 (not ideal, but usable as input with pull-up)
// #define IR_SENSOR_1  D4    // D4 (built-in LED, usable with caution)
// #define IR_SENSOR_2  3    // RX (if Serial not used)


// ✅ SLAVE MAC Address (your device): C4:D8:D5:39:DF:9C
uint8_t slaveAddress[] = { 0xC4, 0xD8, 0xD5, 0x39, 0xDF, 0x9C };

unsigned long lastSend = 0;
const unsigned long sendInterval = 10;
// Add this above setup() or anywhere globally
unsigned long lastTaskTime = 0;
const unsigned long taskInterval = 500;  // 1 second
                                         // Every 150 ms



void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&msg, incomingData, sizeof(msg));

  Serial.print("Received ScanDis: ");
  Serial.println(msg.scanDisValue ? "1 (Manual)" : "0 (Auto)");

  // Add your mode-change logic here if needed
  if (msg.scanDisValue) {
    // Manual mode actions
  } else {
    // Auto mode actions
  }
}




void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(0);  // Set to lowest TX power (0 = 0dBm ~ minimum)
  pinMode(pirPin, INPUT);


  // pinMode(TRIG_PIN_1, OUTPUT);
  // pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  // pinMode(TRIG_PIN_3, OUTPUT);
  // pinMode(ECHO_PIN_3, INPUT);
  // pinMode(IR_SENSOR_1, INPUT);
  // pinMode(IR_SENSOR_2, INPUT);
  pinMode(fire, INPUT);

  delay(10);

  // pinMode(XSHUT_PIN, OUTPUT);
  // digitalWrite(XSHUT_PIN, LOW);   // Reset sensor
  // delay(10);
  // digitalWrite(XSHUT_PIN, HIGH);  // Power on sensor
  // delay(50);                      // Wait for sensor to initialize

  Wire.begin();  // SDA = D1 (GPIO5), SCL = D2 (GPIO4)

  if (!lox.begin()) {
    Serial.println("VL53L0X not detected. Check wiring!");
    while (1)
      ;
  }


  myServo.attach(D8, 500, 2400);  // Servo PWM, min/max pulse width
  myServo.write(frontAngle);
  delay(50);



  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  // esp_now_register_send_cb(OnDataSent);

  esp_now_add_peer(slaveAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  unsigned long now = millis();

  if (now - lastSend >= sendInterval) {
    lastSend = now;

    // distance1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
    distance2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
    // distance3 = getDistance(TRIG_PIN_3, ECHO_PIN_3);



    // myServo.write(angle);
    delay(20);  // Let servo reach new position

    lox.rangingTest(&measure, false);

    int dist = 199;  // default out-of-range
    if (measure.RangeStatus != 4) {
      dist = measure.RangeMilliMeter / 10;
    }





    // === Dummy sensor values ===
    dataToSend.distance1 = distance1;
    dataToSend.distance2 = distance2;
    dataToSend.distance3 = 69 + random(-15, 50);  // Adds a random number from -5 to +5

    if (now - lastTaskTime >= taskInterval) {
      lastTaskTime = now;

      motion = digitalRead(pirPin);

      if (motion == HIGH) {
        // Serial.println("Motion Detected!");

      } else {
        // Serial.println("No Motion");
        // Turn LED OFF
      }
    }


    dataToSend.Pir = motion;

    if (dist < 200 && dist > 0) {
      dataToSend.RadarDist = dist;
    }
    dataToSend.angle = frontAngle;
    dataToSend.ReadyToSendRadarDataTrig = 0;

    // Serial.print(" Angle : ");
    // Serial.print(dataToSend.angle);
    // Serial.print("   ");
    // Serial.print(" Distance :  ");
    // Serial.println(dataToSend.RadarDist);
    // Serial.println("   ");
    esp_now_send(slaveAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));






    //obstacle avoidance logic start

    if (dist < 47 && msg.scanDisValue != 1 && msg.alignmentPhase==0) {
      dataToSend.ReadyToSendRadarDataTrig = 1;
      esp_now_send(slaveAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
      delay(70);

      // Simple radar scan from 0 to 180 degrees in 20° steps
      for (int angle = 0; angle <= 180; angle += 30) {
        // 1. Move servo
        myServo.write(angle);


        // distance1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
        distance2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
        // distance3 = getDistance(TRIG_PIN_3, ECHO_PIN_3);

        dataToSend.distance1 = distance1;
        dataToSend.distance2 = distance2;
        dataToSend.distance3 = 58 + random(-15, 50);  // Adds a random number from -5 to +5


        motion = digitalRead(pirPin);

        if (motion == HIGH) {
          // Serial.println("Motion Detected!");

        } else {
          // Serial.println("No Motion");
          // Turn LED OFF
        }



        dataToSend.Pir = motion;


        delay(50);  // Give servo time to reach position

        // 2. Take single measurement
        int current_dist = readTof();

        // 3. Validate and store
        if (current_dist > 0 && current_dist < 200) {
          dataToSend.angle = angle;
          dataToSend.RadarDist = current_dist;
        } else {
          dataToSend.angle = angle;
          dataToSend.RadarDist = -1;  // Invalid reading marker
        }

        // 4. Serial output
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print("°\tDistance: ");
        if (dataToSend.RadarDist == -1) {
          Serial.println("Invalid");
        } else {
          Serial.print(dataToSend.RadarDist);
          Serial.println(" cm");
        }

        // 5. Send via ESP-NOW (optional)
        esp_now_send(slaveAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));

        delay(50);  // Short pause before next position
      }

      // Return to center position
      myServo.write(frontAngle);
      delay(50);
    }
    //   dataToSend.angle = frontAngle;
    //   if (dist < 200 && dist > 0) {
    //     dataToSend.RadarDist = dist;
    //   }
    // }


    //obstacle avoidance logic end

    // Serial.print(angle);
    // Serial.print(",");
    // Serial.print(dist);
    // Serial.println(".");

    // angle += direction * stepSize;

    // if (angle >= 140 || angle <= 0) {
    //   direction *= -1;
    //   delay(3);  // Pause at ends
    // }



    // ===========================
  }


  // 1-second task block


  delay(12);
  yield();
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


int leftDekh(int pcent) {
  float tempAngle, lookAngle;
  tempAngle = frontAngle * pcent / 100;
  lookAngle = frontAngle - tempAngle;
  myServo.write(lookAngle);
  return lookAngle;
}

int rightDekh(int pcent) {
  float tempAngle, lookAngle;
  tempAngle = frontAngle * pcent / 100;
  lookAngle = frontAngle + tempAngle;
  myServo.write(lookAngle);
  return lookAngle;
}

int BackToFront() {
  myServo.write(frontAngle);
  return frontAngle;
}

int readTof() {
  lox.rangingTest(&measure, false);
  int tempDist;
  if (measure.RangeStatus != 4) {
    tempDist = measure.RangeMilliMeter / 10;  // Convert mm to cm
    return tempDist;
  } else {
    return -1;
  }
}

//Tried and tested code By - Naitik Srivastava
