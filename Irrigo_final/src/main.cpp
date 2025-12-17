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
// CALIBRATION MODE
// Uncomment the line below to run Sensor Calibration
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

// --- PULSE WATERING VARIABLES ---
bool autoState[3] = {false};             // Memory: TRUE = Needs Water, FALSE = Done
unsigned long pumpPulseStart[3] = {0};    // Time when pump started
unsigned long lastPulseEndTime[3] = {0};  // Time when pump stopped (for soak time)
bool pumpPulseActive[3] = {false};        // Is pump currently executing a pulse?
const unsigned long PUMP_DURATION = 5000; // 5 Seconds ON
const unsigned long SOAK_DURATION = 10000; // 10 Seconds OFF (Wait for water to soak)

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
        if(!newMode) manualState[i] = false; 
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

    if (strstr(topic, "/state")) {
        if(manualMode[pumpID]) {
            manualState[pumpID] = (strcmp(msg, "ON") == 0);
            Serial.printf(">> Pump %d (Zone %d) State: %s\n", pumpID, inputID, manualState[pumpID]?"ON":"OFF");
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
    
    // Last Will & Testament (LWT)
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password, 
                       "irrigo/device/status", 0, true, "false")) {
      
      Serial.println("Connected.");
      client.publish("irrigo/device/status", "true", true);
      
      client.subscribe("irrigo/cmd/all/mode");
      client.subscribe("irrigo/cmd/pump/+/state");
    } else {
      Serial.print("Failed, rc="); Serial.println(client.state());
    }
  }
}

// ==========================================
// 5. MAIN SETUP (WITH SENSOR SCAN)
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // --- CALIBRATION MODE LOGIC ---
  #ifdef CALIBRATION_MODE
    Wire.begin(21, 22);
    lcd.init(); lcd.backlight();
    lcd.setCursor(0,0); lcd.print("CALIBRATION MODE");
    Serial.println("\n=== SENSOR CALIBRATION ===");
    Serial.println("1. Dip sensor in water -> Record 'Wet' value");
    Serial.println("2. Hold sensor in air  -> Record 'Dry' value");
    Serial.println("----------------------------------------------");
    // Skip the rest of normal setup
    return;
  #endif
  // ------------------------------

  Wire.begin(21, 22); 
  
  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("   IRRIGO SYS   ");
  lcd.setCursor(0,1); lcd.print("  SYSTEM START  ");
  delay(1000);

  Serial.println("\n\n=================================");
  Serial.println("   IRRIGO SYSTEM DIAGNOSTICS     ");
  Serial.println("=================================");

  // 1. I2C SENSORS
  Serial.print("[I2C] Scanning INA219 Power Sensor... ");
  if (!ina219.begin()) {
    Serial.println("FAILED (Check Wiring)");
    hasINA219 = false;
  } else {
    Serial.println("OK");
    hasINA219 = true;
  }

  // 2. DS18B20 TEMP SENSORS
  Serial.print("[ONEWIRE] Scanning DS18B20... ");
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  Serial.print("Found: "); Serial.println(numberOfDevices);
  sensors.setWaitForConversion(false); // ASYNC MODE

  // 3. AIR SENSOR (DHT)
  pinMode(DHTPIN, INPUT_PULLUP);
  dht.begin();
  Serial.print("[DHT] Warming up... ");
  delay(2000); 
  Serial.print("Testing... ");
  
  float t_test = dht.readTemperature();
  if(isnan(t_test)) Serial.println("ERROR (Check Connection)");
  else Serial.println("OK");

  // 4. SOIL SENSORS
  Serial.println("[SOIL] Reading Raw Values (0-4095):");
  for(int i=0; i<3; i++) pinMode(pinSoil[i], INPUT); 
  for(int i=0; i<3; i++) {
    int val = analogRead(pinSoil[i]);
    Serial.printf("  - Soil %d: %d\n", i+1, val);
  }

  // 5. ULTRASONIC
  Serial.println("[SONAR] Testing Distance Sensors:");
  for(int i=0; i<2; i++) { pinMode(pinTrig[i], OUTPUT); pinMode(pinEcho[i], INPUT); }
  for(int i=0; i<2; i++) {
     float d = getDistance(pinTrig[i], pinEcho[i]);
     Serial.printf("  - Tank %d: %.1f cm\n", i+1, d);
  }

  // 6. RELAYS
  Serial.println("[GPIO] Initializing Relays...");
  for(int i=0; i<6; i++) { pinMode(pinRelay[i], OUTPUT); digitalWrite(pinRelay[i], HIGH); }

  Serial.println("=================================");
  Serial.println("   DIAGNOSTICS COMPLETE          ");
  Serial.println("=================================\n");

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
  // --- CALIBRATION MODE LOOP ---
  #ifdef CALIBRATION_MODE
    Serial.println("\n--- Raw ADC Readings ---");
    lcd.clear();
    for(int i=0; i<3; i++) {
       int raw = analogRead(pinSoil[i]);
       Serial.printf("Zone %d (Pin %d): %d\n", i+1, pinSoil[i], raw);
       if(i < 2) {
         lcd.setCursor(0, i); 
         lcd.print("Z"); lcd.print(i+1); lcd.print(": "); lcd.print(raw);
       } else {
         if(millis() % 4000 > 2000) {
             lcd.setCursor(0, 1);
             lcd.print("Z3: "); lcd.print(raw); lcd.print("    ");
         }
       }
    }
    delay(500);
    return;
  #endif
  // -----------------------------

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) reconnect();
    client.loop(); 
  }

  if (digitalRead(WIFI_RESET_PIN) == LOW) {
    unsigned long startPress = millis();
    bool longPress = false;
    while(digitalRead(WIFI_RESET_PIN) == LOW) {
      if(millis() - startPress > 3000) { longPress = true; break; }
      delay(50);
    }
    if(longPress) {
      lcd.clear(); lcd.print("RESETTING WIFI");
      WiFiManager wm; wm.resetSettings();
      delay(1000); ESP.restart();
    }
  }

  unsigned long now = millis();

  // --- STEP 1: REQUEST TEMP ---
  if (now - lastSensorTime > sensorInterval && !waitingForConversion) {
    sensors.requestTemperatures(); 
    waitingForConversion = true;
    lastTempRequest = now;
  }

  // --- STEP 2: SENSOR READING & PUBLISHING ---
  if (waitingForConversion && (now - lastTempRequest >= DS18B20_DELAY)) {
    lastSensorTime = now;
    waitingForConversion = false;
    
    char buf[16]; 

    // A. Read Sensors
    if (hasINA219) {
      systemVoltage = ina219.getBusVoltage_V();
      systemCurrent = ina219.getCurrent_mA();
    }
    client.loop(); 

    float newT = dht.readTemperature();
    float newH = dht.readHumidity();
    
    if (isnan(newT) || isnan(newH)) {
        delay(10); 
        newT = dht.readTemperature(); 
        newH = dht.readHumidity();
    }

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
      // Standard Moisture: 0% = Dry, 100% = Wet
      soilMoist[i] = map(raw, SOIL_DRY, SOIL_WET, 0, 100); 
      soilMoist[i] = constrain(soilMoist[i], 0, 100);
    }

    // B. Publish Global Data
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

  // --- STEP 3: ACTUATION LOGIC (FIXED FOR FILLING) ---
  for(int i=0; i<3; i++) {
      bool pumpActive = false;
      
      if(manualMode[i]) {
        pumpActive = manualState[i];
        pumpPulseActive[i] = false; 
        autoState[i] = false; // Reset auto memory in manual mode
      } else {
        // === AUTO LOGIC: HYSTERESIS + PULSING ===
        // Goal: If < 30%, start pulsing until > 70%.

        // 1. UPDATE STATE (Memory)
        if(soilMoist[i] < 35) autoState[i] = true;  // Needs Water (START)
        if(soilMoist[i] > 60) autoState[i] = false; // Enough Water (STOP)

        // 2. EXECUTE PULSING IF STATE IS ACTIVE
        if(autoState[i]) {
            // Trigger a new pulse if:
            // - Not currently pulsing AND
            // - Wait time (soak duration) has passed
            if(!pumpPulseActive[i] && (now - lastPulseEndTime[i] > SOAK_DURATION)) {
                pumpPulseActive[i] = true;
                pumpPulseStart[i] = now;
            }

            // If pulse is running, check duration
            if(pumpPulseActive[i]) {
                if(now - pumpPulseStart[i] > PUMP_DURATION) {
                    pumpPulseActive[i] = false;
                    lastPulseEndTime[i] = now;
                }
            }
        } else {
            // Target reached (>70%), ensure pump is off
            pumpPulseActive[i] = false;
        }

        pumpActive = pumpPulseActive[i];

        // SAFETY: Only run if we have water
        if (tankLevel[0] < 5) pumpActive = false; 
      }
      digitalWrite(pinRelay[i], pumpActive ? LOW : HIGH);
  }

  for(int i=3; i<6; i++) {
      bool pumpActive = manualMode[i] ? manualState[i] : false;
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
