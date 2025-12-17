#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h> 
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <WiFiManager.h> 
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_INA219.h> 

// ==========================================
// CONFIGURATION: FLOW RATES
// ==========================================
// Measure your pumps! How many Milliliters per Second?
// Formula: Run pump for 10s, measure mL, divide by 10.
float FLOW_RATE_12V = 30.0; // Pumps 1, 2, 3 (Zones)
float FLOW_RATE_5V  = 15.0; // Pumps 4, 5, 6 (Aux) - Measure this!

// ==========================================
// CALIBRATION MODE
// Uncomment to run Sensor Calibration
// ==========================================
// #define CALIBRATION_MODE 

// ==========================================
// 1. HARDWARE CONFIGURATION
// ==========================================

// --- I2C SENSORS (SDA=21, SCL=22) ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_INA219 ina219; 
bool hasINA219 = false;

// --- SOIL MOISTURE (ADC1 PINS) ---
// Note: Capacitive sensors are INVERTED. High Voltage = Dry.
const int pinSoil[3] = {32, 33, 35}; 
const int SOIL_DRY = 2500; // Reading in Air (3.3V)
const int SOIL_WET = 1350; // Reading in Water (~1.2V)

// --- DS18B20 TEMP SENSORS ---
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int numberOfDevices = 0;

// --- AIR SENSOR (DHT) ---
#define DHTPIN 15
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// --- ULTRASONIC SENSORS ---
const int pinTrig[2] = {5, 18};
const int pinEcho[2] = {19, 34}; 
const float tankHeight = 100.0;  // Tank depth in cm

// --- RELAYS (ACTIVE LOW) ---
// 0,1,2 = Zone 1,2,3 | 3,4,5 = Aux 1,2,3
const int pinRelay[6] = {23, 25, 26, 27, 14, 13}; 

// --- INPUTS ---
#define WIFI_RESET_PIN 0 // Boot Button

// ==========================================
// 2. NETWORK & MQTT
// ==========================================
const char* mqtt_server = "35f302d106324dacb15e3eacf32cc553.s1.eu.hivemq.cloud"; 
const int mqtt_port = 8883; 
const char* mqtt_username = "irrigo"; 
const char* mqtt_password = "Irrigo123";  

WiFiClientSecure espClient; 
PubSubClient client(espClient);

// ==========================================
// 3. SYSTEM VARIABLES
// ==========================================
// Timing
unsigned long lastSensorTime = 0;
unsigned long lastLcdTime = 0;
const long sensorInterval = 2000; 
const long lcdInterval = 3000;    

// Async Temp Variables
unsigned long lastTempRequest = 0;
bool waitingForConversion = false;
const int DS18B20_DELAY = 800; 

// Data Storage
float tankLevel[2] = {0,0}; 
int soilMoist[3] = {0,0,0};
float soilTemp[3] = {0,0,0}; 
float airTemp = 0.0, airHum = 0.0;
float systemVoltage = 0.0, systemCurrent = 0.0; 

// Control State
bool manualMode[6] = {false};  
bool manualState[6] = {false}; 
int lcdPage = 0; 

// --- MANUAL VOLUME CONTROL (User Defined mL) ---
long targetVolume[6] = {0};            // mL set by user
unsigned long volumeRunStart[6] = {0}; // Timer start
unsigned long volumeDuration[6] = {0}; // Calculated ms
bool volumeActive[6] = {false};        // Is volume mode active?

// --- AUTO PULSE CONTROL (Fixed 5s Pulse) ---
bool autoState[3] = {false};             
unsigned long pumpPulseStart[3] = {0};    
unsigned long lastPulseEndTime[3] = {0};  
bool pumpPulseActive[3] = {false};        
const unsigned long PUMP_DURATION = 5000; // Fixed 5s for Auto
const unsigned long SOAK_DURATION = 5000; // Fixed 5s Soak

// ==========================================
// 4. HELPER FUNCTIONS
// ==========================================

bool startsWith(const char* pre, const char* str) {
  return strncmp(pre, str, strlen(pre)) == 0;
}

float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 10000); 
  if (duration == 0) return 0;
  return duration * 0.034 / 2;
}

void configModeCallback(WiFiManager *myWiFiManager) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("SETUP REQUIRED");
  lcd.setCursor(0, 1); lcd.print("AP: IRRIGO_SETUP");
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConfigPortalTimeout(60); 
  wm.setAPCallback(configModeCallback);
  
  if(!wm.autoConnect("IRRIGO_SETUP", "admin123")) {
    Serial.println("WiFi Failed. Running Offline.");
  } else {
    Serial.println("WiFi Connected.");
  }
}

// --- MQTT CALLBACK ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0'; 
  
  for(int i = 0; msg[i]; i++) msg[i] = toupper(msg[i]);

  Serial.printf("RX Topic: %s | Msg: %s\n", topic, msg);

  // A. Check Global Mode
  if (strcmp(topic, "irrigo/cmd/all/mode") == 0) {
    bool newMode = (strcmp(msg, "MANUAL") == 0);
    for(int i=0; i<6; i++) {
        manualMode[i] = newMode;
        if(!newMode) {
          // Switch to AUTO: Reset manual states
          manualState[i] = false; 
          volumeActive[i] = false; 
        }
    }
    lcdPage = 3; 
    lastLcdTime = millis() - lcdInterval;
    Serial.println(">> ALL MODE UPDATED");
    return; 
  }

  // B. Check Pump Commands
  if (startsWith("irrigo/cmd/pump/", topic)) {
    char* ptr = topic + 16; 
    int inputID = atoi(ptr); 
    int pumpID = inputID - 1; 
    
    if(pumpID < 0 || pumpID > 5) return; 

    // --- CASE 1: SET MILLILITERS (MANUAL ONLY) ---
    if (strstr(topic, "/ml")) {
        int ml = atoi(msg);
        targetVolume[pumpID] = ml;
        Serial.printf(">> Pump %d Target Volume Set: %d mL\n", pumpID, ml);
    }
    // --- CASE 2: STATE ON/OFF ---
    else if (strstr(topic, "/state")) {
        // Only execute State changes if in Manual Mode
        if(manualMode[pumpID]) {
            bool turnOn = (strcmp(msg, "ON") == 0);
            
            if (turnOn) {
                manualState[pumpID] = true;
                // Check if we have a volume target set for Manual Mode
                if (targetVolume[pumpID] > 0) {
                    // SELECT FLOW RATE BASED ON PUMP ID
                    // Pumps 0-2 (Zones) are 12V, Pumps 3-5 (Aux) are 5V
                    float currentFlowRate = (pumpID < 3) ? FLOW_RATE_12V : FLOW_RATE_5V;

                    // CALCULATE DURATION: (mL / (mL/sec)) * 1000 = ms
                    volumeDuration[pumpID] = (targetVolume[pumpID] / currentFlowRate) * 1000;
                    volumeRunStart[pumpID] = millis();
                    volumeActive[pumpID] = true;
                    Serial.printf(">> Pump %d START VOLUME: %d mL (%lu ms) [Flow: %.1f mL/s]\n", pumpID, targetVolume[pumpID], volumeDuration[pumpID], currentFlowRate);
                } else {
                    volumeActive[pumpID] = false; // Standard Infinite ON
                    Serial.printf(">> Pump %d START MANUAL (Infinite)\n", pumpID);
                }
            } else {
                manualState[pumpID] = false;
                volumeActive[pumpID] = false; 
                Serial.printf(">> Pump %d STOP\n", pumpID);
            }
        } else {
            Serial.println(">> Ignored: System is in AUTO");
        }
    }
  }
}

void reconnect() {
  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    Serial.print("Connecting MQTT...");
    String clientId = "ESP32Irrigo-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password, 
                       "irrigo/device/status", 0, true, "false")) {
      Serial.println("Connected.");
      client.publish("irrigo/device/status", "true", true);
      client.subscribe("irrigo/cmd/all/mode");
      client.subscribe("irrigo/cmd/pump/+/state");
      client.subscribe("irrigo/cmd/pump/+/ml");
    } else {
      Serial.print("Failed, rc="); Serial.println(client.state());
    }
  }
}

// ==========================================
// 5. MAIN SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  #ifdef CALIBRATION_MODE
    Wire.begin(21, 22);
    lcd.init(); lcd.backlight();
    lcd.setCursor(0,0); lcd.print("CALIBRATION MODE");
    Serial.println("\n=== SENSOR CALIBRATION ===");
    return;
  #endif

  Wire.begin(21, 22); 
  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("   IRRIGO SYS   ");
  lcd.setCursor(0,1); lcd.print("  SYSTEM START  ");
  delay(1000);

  if (!ina219.begin()) hasINA219 = false; else hasINA219 = true;

  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  sensors.setWaitForConversion(false); 

  pinMode(DHTPIN, INPUT_PULLUP);
  dht.begin();
  delay(2000); 
  dht.readTemperature(); // Dummy read

  for(int i=0; i<3; i++) pinMode(pinSoil[i], INPUT); 
  for(int i=0; i<2; i++) { pinMode(pinTrig[i], OUTPUT); pinMode(pinEcho[i], INPUT); }
  for(int i=0; i<6; i++) { pinMode(pinRelay[i], OUTPUT); digitalWrite(pinRelay[i], HIGH); }

  pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
  
  setup_wifi();
  
  espClient.setInsecure(); 
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(512); 
}

// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
  #ifdef CALIBRATION_MODE
    // ... Calibration Code ...
    return;
  #endif

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) reconnect();
    client.loop(); 
  }

  // WiFi Reset
  if (digitalRead(WIFI_RESET_PIN) == LOW) {
    // ... Reset Logic ...
    WiFiManager wm; wm.resetSettings(); ESP.restart();
  }

  unsigned long now = millis();

  // --- SENSOR READING & PUBLISHING ---
  if (now - lastSensorTime > sensorInterval && !waitingForConversion) {
    sensors.requestTemperatures(); 
    waitingForConversion = true;
    lastTempRequest = now;
  }

  if (waitingForConversion && (now - lastTempRequest >= DS18B20_DELAY)) {
    lastSensorTime = now;
    waitingForConversion = false;
    
    char buf[16]; 

    // Read Sensors
    if (hasINA219) {
      systemVoltage = ina219.getBusVoltage_V();
      systemCurrent = ina219.getCurrent_mA();
    }
    client.loop(); 

    float newT = dht.readTemperature();
    float newH = dht.readHumidity();
    if (isnan(newT)) { delay(10); newT = dht.readTemperature(); newH = dht.readHumidity(); }
    if (!isnan(newT)) airTemp = newT;
    if (!isnan(newH)) airHum = newH;
    
    for(int i=0; i<2; i++) {
      float dist = getDistance(pinTrig[i], pinEcho[i]);
      if (tankHeight > 0) {
        if (dist > tankHeight) dist = tankHeight;
        tankLevel[i] = ((tankHeight - dist) / tankHeight) * 100.0;
      }
      delay(10); 
    }

    for(int i=0; i<3; i++) {
        if (i < numberOfDevices) {
            float t = sensors.getTempCByIndex(i);
            if(t > -50 && t < 100) soilTemp[i] = t;
        } else soilTemp[i] = 0.0;
    }

    for(int i=0; i<3; i++) {
      int raw = analogRead(pinSoil[i]);
      // 0% = Dry, 100% = Wet
      soilMoist[i] = map(raw, SOIL_DRY, SOIL_WET, 0, 100); 
      soilMoist[i] = constrain(soilMoist[i], 0, 100);
    }

    // Publish Data
    if (client.connected()) {
        dtostrf(systemVoltage, 4, 2, buf); client.publish("irrigo/sensor/system/voltage", buf);
        dtostrf(systemCurrent, 4, 1, buf); client.publish("irrigo/sensor/system/current", buf);
        delay(20); client.loop(); 

        dtostrf(airTemp, 4, 1, buf); client.publish("irrigo/sensor/air/temp", buf);
        dtostrf(airHum, 4, 0, buf);  client.publish("irrigo/sensor/air/hum", buf);
        delay(20); client.loop(); 
        
        dtostrf(tankLevel[0], 3, 0, buf); client.publish("irrigo/sensor/tank/water", buf);
        dtostrf(tankLevel[1], 3, 0, buf); client.publish("irrigo/sensor/tank/nutrient", buf);
        delay(20); client.loop(); 
        
        for(int i=0; i<3; i++) {
           String base = "irrigo/sensor/zone/" + String(i+1);
           itoa(soilMoist[i], buf, 10);      client.publish((base + "/moist").c_str(), buf);
           dtostrf(soilTemp[i], 4, 1, buf);  client.publish((base + "/temp").c_str(), buf);
           
           String stat = "irrigo/status/zone/" + String(i+1);
           bool isPumpOn = (digitalRead(pinRelay[i]) == LOW);
           client.publish((stat + "/pump").c_str(), isPumpOn ? "ON" : "OFF");
           client.publish((stat + "/mode").c_str(), manualMode[i] ? "MANUAL" : "AUTO");
           
           delay(20); client.loop(); 
        }
        
        for(int i=3; i<6; i++) {
           String stat = "irrigo/status/aux/" + String(i-2); 
           bool isPumpOn = (digitalRead(pinRelay[i]) == LOW);
           client.publish((stat + "/pump").c_str(), isPumpOn ? "ON" : "OFF");
           client.publish((stat + "/mode").c_str(), manualMode[i] ? "MANUAL" : "AUTO");
           delay(10);
        }
    }
  }

  // --- STEP 3: ACTUATION LOGIC ---
  for(int i=0; i<3; i++) {
      bool pumpActive = false;
      
      if(manualMode[i]) {
        // === MANUAL MODE ===
        // Uses Volume Control (mL) OR Infinite
        if(manualState[i]) {
            if(volumeActive[i]) {
                // Check Timer for mL run
                if(now - volumeRunStart[i] >= volumeDuration[i]) {
                    // FINISHED: Turn Off
                    volumeActive[i] = false;
                    manualState[i] = false;
                    pumpActive = false;
                    String stat = "irrigo/status/zone/" + String(i+1);
                    client.publish((stat + "/pump").c_str(), "OFF");
                } else {
                    pumpActive = true;
                }
            } else {
                // Infinite Run (if mL was 0)
                pumpActive = true;
            }
        } else {
            pumpActive = false;
        }

        // Reset Auto variables to be safe
        pumpPulseActive[i] = false; 
        autoState[i] = false; 

      } else {
        // === AUTO MODE ===
        // Strictly uses Fixed Pulse (5s), ignores mL settings
        
        if(soilMoist[i] < 35) autoState[i] = true;  
        if(soilMoist[i] > 60) autoState[i] = false; 

        if(autoState[i]) {
            if(!pumpPulseActive[i] && (now - lastPulseEndTime[i] > SOAK_DURATION)) {
                pumpPulseActive[i] = true;
                pumpPulseStart[i] = now;
            }
            if(pumpPulseActive[i]) {
                if(now - pumpPulseStart[i] > PUMP_DURATION) {
                    pumpPulseActive[i] = false;
                    lastPulseEndTime[i] = now;
                }
            }
        } else {
            pumpPulseActive[i] = false;
        }

        pumpActive = pumpPulseActive[i];
        if (tankLevel[0] < 5) pumpActive = false; 
      }
      digitalWrite(pinRelay[i], pumpActive ? LOW : HIGH);
  }

  // AUX PUMPS (Manual Only - Supports Volume)
  for(int i=3; i<6; i++) {
      bool pumpActive = false;
      if(manualMode[i]) {
         if(manualState[i]) {
            if(volumeActive[i]) {
                if(now - volumeRunStart[i] >= volumeDuration[i]) {
                    volumeActive[i] = false;
                    manualState[i] = false;
                    pumpActive = false;
                    String stat = "irrigo/status/aux/" + String(i-2); 
                    client.publish((stat + "/pump").c_str(), "OFF");
                } else {
                    pumpActive = true;
                }
            } else {
                pumpActive = true;
            }
         }
      }
      digitalWrite(pinRelay[i], pumpActive ? LOW : HIGH);
  }

  // LCD Rotation
  if (now - lastLcdTime > lcdInterval) {
    lastLcdTime = now;
    lcd.clear();
    switch(lcdPage) {
      case 0: lcd.print("PWR:"); lcd.print(systemVoltage); lcd.print("V"); 
              lcd.setCursor(0,1); lcd.print("I:"); lcd.print(systemCurrent); lcd.print("mA"); break;
      case 1: lcd.print("Air T:"); lcd.print(airTemp); 
              lcd.setCursor(0,1); lcd.print("Air H:"); lcd.print(airHum); lcd.print("%"); break;
      case 2: lcd.print("W:"); lcd.print((int)tankLevel[0]); lcd.print("% N:"); lcd.print((int)tankLevel[1]); lcd.print("%"); break;
      case 3: case 4: case 5: {
        int z = lcdPage - 2; 
        if(z >= 1 && z <= 3) {
            lcd.print("Z"); lcd.print(z); lcd.print(" "); lcd.print(manualMode[z-1]?"MAN":"AUTO");
            lcd.setCursor(0,1); lcd.print("M:"); lcd.print(soilMoist[z-1]); lcd.print("% "); lcd.print((digitalRead(pinRelay[z-1])==LOW)?"ON":"OFF");
        }
        break;
      }
    }
    lcdPage++;
    if(lcdPage > 5) lcdPage = 0;
  }
}
