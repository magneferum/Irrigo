#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h> // REQUIRED for Port 8883 (SSL)
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
// #include <ArduinoJson.h> 
#include <WiFiManager.h> 
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// --- 1. HARDWARE CONFIGURATION ---

// LCD I2C (Address 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Soil Moisture Sensors (ADC - Input Only pins are best for stability)
const int pinSoil[3] = {32, 33, 35}; 

// Voltage Sensor (New) - Use GPIO 36 (VP) for best ADC results
const int pinVoltage = 36;

// DS18B20 Temperature Sensors
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- MANUAL DS18B20 ADDRESSING ---
// REPLACE THESE EXAMPLES WITH YOUR REAL ADDRESSES FOUND IN SERIAL MONITOR
// Format: {0x28, 0xFF, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX}
uint8_t sensorAddress1[8] = {0x28, 0xEF, 0x8B, 0xB5, 0x40, 0x24, 0x0B, 0x53}; // Zone 1
uint8_t sensorAddress2[8] = {0x28, 0x0C, 0x32, 0x53, 0x40, 0x24, 0x0B, 0xDF}; // Zone 2
uint8_t sensorAddress3[8] = {0x28, 0x56, 0xC8, 0x3D, 0x40, 0x24, 0x0B, 0xA0}; // Zone 3

// Global variables for DS18B20 Addresses
int numberOfDevices;
DeviceAddress tempDeviceAddress; 

// Air Sensor (DHT11)
#define DHTPIN 15
#define DHTTYPE DHT11 // Changed from DHT22 to DHT11
DHT dht(DHTPIN, DHTTYPE);

// Ultrasonic Sensors (0: Water Tank, 1: Nutrient Tank)
const int pinTrig[2] = {5, 18};
const int pinEcho[2] = {19, 34}; 

// Pump Relays (Active LOW) - Controls 6 Pumps
const int pinRelay[6] = {23, 25, 26, 27, 14, 13}; 

// WiFi Reset Button (BOOT Button)
#define TRIGGER_PIN 0 

// --- 2. MQTT & SYSTEM VARIABLES ---
// UPDATE THESE WITH YOUR SECURE BROKER DETAILS
const char* mqtt_server = "35f302d106324dacb15e3eacf32cc553.s1.eu.hivemq.cloud"; // e.g., xxxx.s1.eu.hivemq.cloud
const int mqtt_port = 8883; // SSL Port
const char* mqtt_username = "irrigo"; 
const char* mqtt_password = "Irrigo123"; 
WiFiClientSecure espClient; // Changed to Secure Client
PubSubClient client(espClient);

// Timing Variables
unsigned long lastSensorTime = 0;
unsigned long lastLcdTime = 1000;
const long sensorInterval = 2000; // Read sensors every 2 seconds
const long lcdInterval = 2000;    // Rotate LCD every 2 seconds

// Data Storage
float tankLevel[2]; // 0: Water, 1: Nutrients (%)
int soilMoist[3];
float soilTemp[3];  // Stores temp for Zone 1, 2, 3
float airTemp = 0.0, airHum = 0.0;
float systemVoltage = 0.0;
bool manualMode[6] = {false}; // Tracks if user manually overrode a pump
int lcdPage = 0; 

// Physical Constants
const float tankHeight = 100.0; // cm

// --- 3. HELPER FUNCTIONS ---

// Helper: Print DS18B20 Address (Debug)
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// Helper: Ultrasonic Distance
float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 0;
  return duration * 0.034 / 2;
}

// WiFiManager Callback (UPDATED)
void configModeCallback(WiFiManager *myWiFiManager) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("AP: IRRIGO_SETUP");
  // Show the actual IP address so you know where to connect
  lcd.setCursor(0, 1); lcd.print(WiFi.softAPIP());
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setAPCallback(configModeCallback);
  
  // REMOVED TIMEOUT so it stays on the screen indefinitely if connection fails
  // wm.setConfigPortalTimeout(180); 
  
  lcd.clear(); lcd.print("Connecting WiFi..");
  
  // This will block here forever if it can't connect, showing the IP on the LCD
  if(!wm.autoConnect("IRRIGO_SETUP", "admin123")) {
    lcd.clear(); lcd.print("Failed! Rebooting");
    delay(2000); ESP.restart();
  }

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("WiFi Connected!");
  lcd.setCursor(0,1); lcd.print(WiFi.localIP());
  delay(2000);
}

// MQTT Callback - Handles Commands from Android
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  
  Serial.print("Message arrived ["); Serial.print(topic); Serial.print("]: "); Serial.println(msg);

  // Check which pump is being controlled
  for(int i=0; i<6; i++) {
    String target = "irrigo/control/pump" + String(i);
    if(String(topic) == target) {
      Serial.print("Controlling Pump "); Serial.println(i);
      
      if(msg == "ON") {
        manualMode[i] = true; // Enter manual mode
        digitalWrite(pinRelay[i], LOW); // Active LOW -> ON
        Serial.println(" -> TURNED ON");
      } 
      else if (msg == "OFF") {
        manualMode[i] = false; // Return to Auto mode (or just Off)
        digitalWrite(pinRelay[i], HIGH); // Active LOW -> OFF
        Serial.println(" -> TURNED OFF");
      }
    }
  }
}

void reconnect() {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Irrigo-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.publish("irrigo/status", "ONLINE");
      // Subscribe to control topics
      client.subscribe("irrigo/control/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}

// --- 4. MAIN SETUP ---
void setup() {
  Serial.begin(115200);
  Wire.begin(); // Join I2C bus
  
  // 1. Init LCD
  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("   IRRIGO SYS   ");
  lcd.setCursor(0,1); lcd.print("   Booting...   ");
  delay(1000);

  // 2. Init Pins
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(pinVoltage, INPUT); // Voltage Sensor
  for(int i=0; i<3; i++) pinMode(pinSoil[i], INPUT);
  for(int i=0; i<2; i++) { pinMode(pinTrig[i], OUTPUT); pinMode(pinEcho[i], INPUT); }
  for(int i=0; i<6; i++) { pinMode(pinRelay[i], OUTPUT); digitalWrite(pinRelay[i], HIGH); }

  // 3. Reset Button Check
  if (digitalRead(TRIGGER_PIN) == LOW) {
    lcd.clear(); lcd.print("RESET WIFI settings?");
    delay(2000);
    if (digitalRead(TRIGGER_PIN) == LOW) {
      WiFiManager wm; wm.resetSettings();
      lcd.setCursor(0,1); lcd.print("DONE -> RESTART");
      delay(1000); ESP.restart();
    }
  }

  // 4. Init DS18B20 (Address Scanning Logic)
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  Serial.print("DS18B20 Found: "); Serial.println(numberOfDevices);
  
  // Debug print addresses
  for(int i=0;i<numberOfDevices; i++){
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Device "); Serial.print(i); Serial.print(": ");
      printAddress(tempDeviceAddress); Serial.println();
    }
  }

  // 5. Init Other Sensors
  dht.begin();

  // 6. Connect Network
  setup_wifi();
  
  // IMPORTANT: Set Insecure for SSL/TLS without Cert Verification
  espClient.setInsecure(); 
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

// --- 5. MAIN LOOP ---
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  // ==========================
  // TASK 1: SENSOR ACQUISITION
  // ==========================
  if (now - lastSensorTime > sensorInterval) {
    lastSensorTime = now;
    
    // A. Read DHT
    airTemp = dht.readTemperature();
    airHum = dht.readHumidity();
    if(isnan(airTemp)) airTemp = 0.0;

    // B. Read Voltage Sensor (0-25V Module)
    // Formula: ADC * (3.3 / 4095) * 5 (Divider Factor)
    // Calibrate the '5.0' value based on real multimeter reading
    int voltRaw = analogRead(pinVoltage);
    systemVoltage = (voltRaw * 3.3 / 4095.0) * 5.0; 

    // C. Read Tanks
    for(int i=0; i<2; i++) {
      float dist = getDistance(pinTrig[i], pinEcho[i]);
      // Prevent negative values or division errors
      if (dist > tankHeight) dist = tankHeight;
      tankLevel[i] = ((tankHeight - dist) / tankHeight) * 100.0;
      if(tankLevel[i] < 0) tankLevel[i] = 0;
    }

    // D. Read DS18B20 (Manual Address)
    sensors.requestTemperatures(); 
    
    // Manually read specific sensors for each zone
    // If the sensor is disconnected, it returns -127.00
    soilTemp[0] = sensors.getTempC(sensorAddress1);
    soilTemp[1] = sensors.getTempC(sensorAddress2);
    soilTemp[2] = sensors.getTempC(sensorAddress3);

    // Filter out disconnected sensors (optional visual cleanup)
    for(int i=0; i<3; i++) {
      if(soilTemp[i] < -100) soilTemp[i] = 0.0; 
    }

    // E. Read Soil Moisture & Logic & Publish Raw Data (No JSON)
    
    // 1. Publish General Data
    client.publish("irrigo/voltage", String(systemVoltage, 2).c_str());
    client.publish("irrigo/air/temp", String(airTemp, 1).c_str());
    client.publish("irrigo/air/hum", String(airHum, 1).c_str());
    client.publish("irrigo/tank/water", String(tankLevel[0], 0).c_str());
    client.publish("irrigo/tank/nutrient", String(tankLevel[1], 0).c_str());

    // 2. Loop through Zones
    for(int i=0; i<3; i++) {
      // Moisture Map
      int raw = analogRead(pinSoil[i]);
      soilMoist[i] = map(raw, 4095, 1500, 0, 100); 
      soilMoist[i] = constrain(soilMoist[i], 0, 100);
      
      // PUMP LOGIC
      bool pumpState = false;
      
      // Only run auto logic if NOT in manual mode
      if(!manualMode[i]) {
        // Auto: Water if dry (<30%) AND Tank has water (>10%)
        if(soilMoist[i] < 30 && tankLevel[0] > 10) {
          digitalWrite(pinRelay[i], LOW); // ON
          pumpState = true;
        } else {
          digitalWrite(pinRelay[i], HIGH); // OFF
          pumpState = false;
        }
      } else {
        // In manual mode, just read current state
        pumpState = (digitalRead(pinRelay[i]) == LOW);
      }

      // Safety: Emergency Tank Cutoff
      if(tankLevel[0] < 5) {
        digitalWrite(pinRelay[i], HIGH);
        pumpState = false;
        manualMode[i] = false; 
      }
      
      // Publish Zone Specific Data
      String baseTopic = "irrigo/zone/" + String(i+1);
      client.publish((baseTopic + "/moist").c_str(), String(soilMoist[i]).c_str());
      client.publish((baseTopic + "/temp").c_str(), String(soilTemp[i], 1).c_str());
      client.publish((baseTopic + "/pump").c_str(), pumpState ? "ON" : "OFF");
    }
  }

  // ==========================
  // TASK 2: LCD DISPLAY ROTATION
  // ==========================
  if (now - lastLcdTime > lcdInterval) {
    lastLcdTime = now;
    lcd.clear();

    switch(lcdPage) {
      case 0: // Air Status & Voltage
        lcd.setCursor(0,0); lcd.print("AIR STATUS");
        lcd.setCursor(0,1); lcd.printf("T:%.1f V:%.1fv", airTemp, systemVoltage);
        lcdPage++;
        break;
        
      case 1: // Tank Levels
        lcd.setCursor(0,0); lcd.print("TANK LEVELS");
        lcd.setCursor(0,1); lcd.printf("W:%d%% Nut:%d%%", (int)tankLevel[0], (int)tankLevel[1]);
        lcdPage++;
        break;

      case 2: // Zone 1
        lcd.setCursor(0,0); lcd.printf("Z1 P:%s", (digitalRead(pinRelay[0])==LOW)?"ON":"OFF");
        lcd.setCursor(0,1); lcd.printf("M:%d%% T:%.1fC", soilMoist[0], soilTemp[0]);
        lcdPage++;
        break;

      case 3: // Zone 2
        lcd.setCursor(0,0); lcd.printf("Z2 P:%s", (digitalRead(pinRelay[1])==LOW)?"ON":"OFF");
        lcd.setCursor(0,1); lcd.printf("M:%d%% T:%.1fC", soilMoist[1], soilTemp[1]);
        lcdPage++;
        break;

      case 4: // Zone 3
        lcd.setCursor(0,0); lcd.printf("Z3 P:%s", (digitalRead(pinRelay[2])==LOW)?"ON":"OFF");
        lcd.setCursor(0,1); lcd.printf("M:%d%% T:%.1fC", soilMoist[2], soilTemp[2]);
        lcdPage = 0; 
        break;
    }
  }
}