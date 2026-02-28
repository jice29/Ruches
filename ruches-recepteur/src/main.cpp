/*
 * RECEPTEUR LoRa - Heltec WiFi LoRa 32 V3
 * Version avec RadioLib - CORRIGEE FINALE
 */

#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>

#ifndef WIFI_SSID_VALUE
#define WIFI_SSID_VALUE ""
#endif

#ifndef WIFI_PASSWORD_VALUE
#define WIFI_PASSWORD_VALUE ""
#endif

#ifndef DEVICE_LOGIN_NAME_VALUE
#define DEVICE_LOGIN_NAME_VALUE ""
#endif

#ifndef DEVICE_KEY_VALUE
#define DEVICE_KEY_VALUE ""
#endif

// ===== Configuration OLED =====
#define OLED_SDA   17
#define OLED_SCL   18
#define OLED_RST   21
#define VEXT_PIN   36
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// ===== Configuration LoRa =====
#define LORA_CS    8
#define LORA_SCK   9
#define LORA_MOSI  10
#define LORA_MISO  11
#define LORA_BUSY  13
#define LORA_RST   12
#define LORA_DIO1  14

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// ===== Configuration WiFi / Arduino Cloud =====
const char* WIFI_SSID = WIFI_SSID_VALUE;
const char* WIFI_PASSWORD = WIFI_PASSWORD_VALUE;
const char* DEVICE_LOGIN_NAME = DEVICE_LOGIN_NAME_VALUE;
const char* DEVICE_KEY = DEVICE_KEY_VALUE;
const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TOPIC_TELEMETRY = "ruches/telemetry";
const char* MQTT_TOPIC_ALERT = "ruches/alert";

float weight_g = 0.0f;
float temp_c = 0.0f;
float hum_pct = 0.0f;
float batt_pct = 0.0f;
int rssi_dbm = 0;

// ===== Variables =====
uint32_t packetCount = 0;
float lastWeight = 0.0;
int16_t lastRSSI = 0;
float lastTempC = NAN;
float lastHumPct = NAN;
float lastBattPct = -1.0f;
bool oled_working = false;
volatile bool receivedFlag = false;
bool radio_receiveMode = false;
unsigned long lastReceiveCheck = 0;
int receiveErrorCount = 0;
unsigned long lastLoraPacketMs = 0;
bool wifiInfoPrinted = false;
bool wifiConfigWarningPrinted = false;
bool cloudEnabled = false;
const long CLOUD_UPDATE_INTERVAL_S = 15;
unsigned long lastMqttTryMs = 0;
bool mqttInfoPrinted = false;
unsigned long lastWifiTryMs = 0;
unsigned long lastHealthyNetworkMs = 0;
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
const unsigned long LORA_SIGNAL_LOSS_MS = 45000;
const int BATT_LOW_THRESHOLD_PCT = 20;
const unsigned long NETWORK_STALL_RESTART_MS = 300000;
bool alertSignalLost = false;
bool alertBatteryLow = false;
bool prevAlertSignalLost = false;
bool prevAlertBatteryLow = false;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

WiFiConnectionHandler ArduinoIoTPreferredConnection(WIFI_SSID, WIFI_PASSWORD);

void initProperties() {
    ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
    ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
    ArduinoCloud.addProperty(weight_g, READ, CLOUD_UPDATE_INTERVAL_S, NULL);
    ArduinoCloud.addProperty(temp_c, READ, CLOUD_UPDATE_INTERVAL_S, NULL);
    ArduinoCloud.addProperty(hum_pct, READ, CLOUD_UPDATE_INTERVAL_S, NULL);
    ArduinoCloud.addProperty(batt_pct, READ, CLOUD_UPDATE_INTERVAL_S, NULL);
    ArduinoCloud.addProperty(rssi_dbm, READ, CLOUD_UPDATE_INTERVAL_S, NULL);
}

void setFlag() {
    receivedFlag = true;
}

void publishAlertEvent(const char* type, bool active) {
    if (!mqttClient.connected()) return;
    char json[160];
    int n = snprintf(
        json,
        sizeof(json),
        "{\"type\":\"%s\",\"active\":%d,\"packet\":%lu,\"ts_ms\":%lu}",
        type,
        active ? 1 : 0,
        (unsigned long)packetCount,
        (unsigned long)millis()
    );
    if (n > 0 && n < (int)sizeof(json)) {
        mqttClient.publish(MQTT_TOPIC_ALERT, json, true);
    }
}

void updateAlertStates(unsigned long now) {
    alertSignalLost = (lastLoraPacketMs > 0) && ((now - lastLoraPacketMs) > LORA_SIGNAL_LOSS_MS);
    alertBatteryLow = (lastBattPct >= 0.0f) && (lastBattPct <= (float)BATT_LOW_THRESHOLD_PCT);

    if (alertSignalLost != prevAlertSignalLost) {
        Serial.print("ALERTE Signal ");
        Serial.println(alertSignalLost ? "PERDU" : "RETABLI");
        publishAlertEvent("signal_lost", alertSignalLost);
        prevAlertSignalLost = alertSignalLost;
    }
    if (alertBatteryLow != prevAlertBatteryLow) {
        Serial.print("ALERTE Batterie ");
        Serial.println(alertBatteryLow ? "BASSE" : "OK");
        publishAlertEvent("battery_low", alertBatteryLow);
        prevAlertBatteryLow = alertBatteryLow;
    }
}

void ensureWiFiConnection() {
    if (strlen(WIFI_SSID) == 0) return;
    if (WiFi.status() == WL_CONNECTED) return;

    unsigned long now = millis();
    if (now - lastWifiTryMs < WIFI_RETRY_INTERVAL_MS) return;
    lastWifiTryMs = now;

    Serial.print("WiFi reconnexion: ");
    Serial.println(WIFI_SSID);
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

uint8_t rssiToBars(int16_t rssi) {
    if (rssi >= -65) return 5;
    if (rssi >= -75) return 4;
    if (rssi >= -85) return 3;
    if (rssi >= -95) return 2;
    if (rssi >= -105) return 1;
    return 0;
}

void drawSignalBars(int16_t rssi, int x, int y) {
    const int barW = 3;
    const int gap = 2;
    const int maxH = 10;
    const uint8_t bars = rssiToBars(rssi);

    for (uint8_t i = 0; i < 5; i++) {
        int h = 2 + i * 2;   // 2,4,6,8,10
        int bx = x + i * (barW + gap);
        int by = y + (maxH - h);
        oled.drawRect(bx, by, barW, h, SSD1306_WHITE);
        if (i < bars) {
            oled.fillRect(bx, by, barW, h, SSD1306_WHITE);
        }
    }
}

uint8_t battToBars(float battPct) {
    if (battPct < 0.0f) return 0;
    if (battPct >= 90.0f) return 5;
    if (battPct >= 70.0f) return 4;
    if (battPct >= 50.0f) return 3;
    if (battPct >= 30.0f) return 2;
    if (battPct >= 10.0f) return 1;
    return 0;
}

void drawBatteryBars(float battPct, int x, int y) {
    const int barW = 3;
    const int gap = 2;
    const int maxH = 10;
    const uint8_t bars = battToBars(battPct);

    for (uint8_t i = 0; i < 5; i++) {
        int h = 2 + i * 2;   // 2,4,6,8,10
        int bx = x + i * (barW + gap);
        int by = y + (maxH - h);
        oled.drawRect(bx, by, barW, h, SSD1306_WHITE);
        if (i < bars) {
            oled.fillRect(bx, by, barW, h, SSD1306_WHITE);
        }
    }
}

// Fonction pour verifier et remettre en reception si necessaire
void ensureReceiveMode() {
    if (!radio_receiveMode) {
        Serial.println("Remise en reception...");
        
        int state = radio.startReceive();
        if (state == RADIOLIB_ERR_NONE) {
            radio_receiveMode = true;
            Serial.println("OK");
        } else {
            Serial.print("Erreur: ");
            Serial.println(state);
            receiveErrorCount++;
        }
    }
}

// ===== Fonctions OLED =====
bool initOLED() {
    Serial.print("Init OLED... ");
    
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(100);
    
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    delay(50);
    
    Wire.begin(OLED_SDA, OLED_SCL);
    
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("ECHEC");
        return false;
    }
    
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("Recepteur LoRa");
    oled.display();
    
    Serial.println("OK");
    return true;
}

void updateDisplay() {
    if (!oled_working) return;
    
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);

    // P = Poids (grammes)
    oled.setCursor(0, 0);
    oled.print("P:");
    float displayWeight = lastWeight;
    if (displayWeight < 0) displayWeight = -displayWeight;
    oled.print(displayWeight, 0);
    oled.print("g");

    oled.setCursor(0, 22);
    oled.print("T:");
    if (!isnan(lastTempC)) {
        oled.print(lastTempC, 0);
        oled.print("C");
    } else {
        oled.print("--C");
    }

    oled.setCursor(0, 44);
    oled.print("H:");
    if (!isnan(lastHumPct)) {
        oled.print(lastHumPct, 0);
        oled.print("%");
    } else {
        oled.print("--%");
    }

    // Batterie en face de la temperature, label petit + barres
    oled.setTextSize(1);
    oled.setCursor(82, 23);
    oled.print("BAT");
    drawBatteryBars(lastBattPct, 100, 22);

    // Signal en bas a droite avec label SGN
    oled.setCursor(82, 54);
    oled.print("SGN");
    drawSignalBars(lastRSSI, 100, 52);

    if (alertSignalLost || alertBatteryLow) {
        oled.setTextSize(1);
        oled.setCursor(80, 0);
        oled.print("ALR:");
        if (alertSignalLost) oled.print("SIG ");
        if (alertBatteryLow) oled.print("BAT");
    }
    
    oled.display();
}

float parseFieldValue(const String& payload, const String& key, float fallback) {
    int idx = payload.indexOf(key);
    if (idx < 0) return fallback;
    int start = idx + key.length();
    int end = payload.indexOf(',', start);
    String token = (end >= 0) ? payload.substring(start, end) : payload.substring(start);
    return token.toFloat();
}

void ensureMqtt() {
    if (mqttClient.connected()) {
        if (!mqttInfoPrinted) {
            Serial.println("MQTT connecte");
            mqttInfoPrinted = true;
        }
        return;
    }
    mqttInfoPrinted = false;
    if (WiFi.status() != WL_CONNECTED) return;

    unsigned long now = millis();
    if (now - lastMqttTryMs < 5000) return;
    lastMqttTryMs = now;

    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    String clientId = "ruches-rx-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool ok = mqttClient.connect(clientId.c_str());
    if (ok) {
        Serial.print("MQTT connecte sur ");
        Serial.print(MQTT_HOST);
        Serial.print(":");
        Serial.println(MQTT_PORT);
    } else {
        Serial.print("MQTT echec state=");
        Serial.println(mqttClient.state());
    }
}

void publishTelemetryMqtt(const String& rawPayload) {
    if (!mqttClient.connected()) return;

    char json[320];
    unsigned long lastLoraAgeSec = (lastLoraPacketMs == 0) ? 0 : (millis() - lastLoraPacketMs) / 1000UL;
    int n = snprintf(
        json,
        sizeof(json),
        "{\"packet\":%lu,\"weight_g\":%.2f,\"temp_c\":%.1f,\"hum_pct\":%.1f,\"batt_pct\":%.0f,\"rssi\":%d,\"rssi_dbm\":%d,\"alert_signal_lost\":%d,\"alert_batt_low\":%d,\"last_lora_s\":%lu,\"raw\":\"%s\"}",
        (unsigned long)packetCount,
        lastWeight,
        isnan(lastTempC) ? -999.0f : lastTempC,
        isnan(lastHumPct) ? -999.0f : lastHumPct,
        (lastBattPct < 0.0f) ? -1.0f : lastBattPct,
        (int)lastRSSI,
        (int)lastRSSI,
        alertSignalLost ? 1 : 0,
        alertBatteryLow ? 1 : 0,
        lastLoraAgeSec,
        rawPayload.c_str()
    );
    if (n <= 0 || n >= (int)sizeof(json)) return;

    mqttClient.publish(MQTT_TOPIC_TELEMETRY, json, true);
}

// ===== Initialisation LoRa =====
bool initLoRa() {
    Serial.print("Init LoRa... ");
    
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(100);
    
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    
    int state = radio.begin(868.0);
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("OK");
        
        radio.setOutputPower(14);
        radio.setSpreadingFactor(7);
        radio.setBandwidth(125.0);
        radio.setCodingRate(5);
        radio.setPreambleLength(8);
        radio.setSyncWord(0x12);
        radio.setCRC(true);
        
        Serial.println("   Frequence: 868.000 MHz");
        
        radio.setDio1Action(setFlag);
        
        state = radio.startReceive();
        if (state == RADIOLIB_ERR_NONE) {
            radio_receiveMode = true;
            Serial.println("   Mode reception actif");
            return true;
        } else {
            Serial.print("   Erreur startReceive: ");
            Serial.println(state);
            return false;
        }
    } else {
        Serial.print("ECHEC Code: ");
        Serial.println(state);
        return false;
    }
}

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n================================");
    Serial.println("RECEPTEUR LORA - RADIOLIB V5");
    Serial.println("================================\n");

    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);

    WiFi.mode(WIFI_STA);
    if (strlen(WIFI_SSID) == 0) {
        Serial.println("Arduino Cloud desactive: SSID vide.");
        wifiConfigWarningPrinted = true;
    } else if (strlen(DEVICE_LOGIN_NAME) == 0 || strlen(DEVICE_KEY) == 0) {
        Serial.println("Arduino Cloud desactive: DEVICE_LOGIN_NAME/DEVICE_KEY vides.");
    } else {
        initProperties();
        ArduinoCloud.begin(ArduinoIoTPreferredConnection);
        setDebugMessageLevel(2);
        ArduinoCloud.printDebugInfo();
        cloudEnabled = true;
        Serial.println("Arduino Cloud actif.");
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    }
    
    oled_working = initOLED();
    delay(500);
    
    if (!initLoRa()) {
        Serial.println("ERREUR LORA");
        while(1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }
    
    Serial.println("\n=== EN ATTENTE ===\n");
    updateDisplay();
}

// ===== Loop =====
void loop() {
    unsigned long now = millis();
    esp_task_wdt_reset();

    ensureWiFiConnection();
    updateAlertStates(now);

    if (cloudEnabled) {
        ArduinoCloud.update();
        ensureMqtt();
        mqttClient.loop();
        if (WiFi.status() == WL_CONNECTED && !wifiInfoPrinted) {
            Serial.print("WiFi OK IP=");
            Serial.print(WiFi.localIP());
            Serial.print(" RSSI=");
            Serial.println(WiFi.RSSI());
            wifiInfoPrinted = true;
        } else if (WiFi.status() != WL_CONNECTED) {
            wifiInfoPrinted = false;
        }
    }

    if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
        lastHealthyNetworkMs = now;
    }
    if (lastHealthyNetworkMs > 0 && (now - lastHealthyNetworkMs) > NETWORK_STALL_RESTART_MS) {
        Serial.println("Redemarrage securite reseau (stall > 5 min)...");
        delay(100);
        ESP.restart();
    }
    
    // POLLING toutes les 100ms
    if (now - lastReceiveCheck > 100) {
        lastReceiveCheck = now;
        
        // Via interruption
        if (receivedFlag) {
            receivedFlag = false;
            
            String received;
            int state = radio.readData(received);
            
            if (state == RADIOLIB_ERR_NONE) {
                packetCount++;
                lastLoraPacketMs = now;
                lastRSSI = radio.getRSSI();
                
                int idx = received.indexOf("POIDS_G:");
                int offset = 8;
                if (idx < 0) {
                    idx = received.indexOf("POIDS:");
                    offset = 6;
                }
                if (idx >= 0) {
                    lastWeight = received.substring(idx + offset).toFloat();
                    Serial.print("Poids recu: ");
                    Serial.print(lastWeight, 2);
                    Serial.println(" g");
                }
                lastTempC = parseFieldValue(received, "T_C:", lastTempC);
                lastHumPct = parseFieldValue(received, "H_P:", lastHumPct);
                lastBattPct = parseFieldValue(received, "B_P:", lastBattPct);
                weight_g = lastWeight;
                temp_c = isnan(lastTempC) ? temp_c : lastTempC;
                hum_pct = isnan(lastHumPct) ? hum_pct : lastHumPct;
                batt_pct = (lastBattPct < 0.0f) ? batt_pct : lastBattPct;
                rssi_dbm = (int)lastRSSI;
                publishTelemetryMqtt(received);
                
                Serial.print("Paquet #");
                Serial.print(packetCount);
                Serial.print(": ");
                Serial.println(received);
                
                updateDisplay();
            }
            
            radio_receiveMode = false;
            ensureReceiveMode();
        }
        
        // Polling direct
        int irq = radio.getIrqStatus();
        if (irq > 0) {
            if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
                String received;
                int state = radio.readData(received);
                
                if (state == RADIOLIB_ERR_NONE) {
                    packetCount++;
                    lastLoraPacketMs = now;
                    lastRSSI = radio.getRSSI();
                    
                    int idx = received.indexOf("POIDS_G:");
                    int offset = 8;
                    if (idx < 0) {
                        idx = received.indexOf("POIDS:");
                        offset = 6;
                    }
                    if (idx >= 0) {
                        lastWeight = received.substring(idx + offset).toFloat();
                        Serial.print("Poids recu: ");
                        Serial.print(lastWeight, 2);
                        Serial.println(" g");
                    }
                    lastTempC = parseFieldValue(received, "T_C:", lastTempC);
                    lastHumPct = parseFieldValue(received, "H_P:", lastHumPct);
                    lastBattPct = parseFieldValue(received, "B_P:", lastBattPct);
                    weight_g = lastWeight;
                    temp_c = isnan(lastTempC) ? temp_c : lastTempC;
                    hum_pct = isnan(lastHumPct) ? hum_pct : lastHumPct;
                    batt_pct = (lastBattPct < 0.0f) ? batt_pct : lastBattPct;
                    rssi_dbm = (int)lastRSSI;
                    publishTelemetryMqtt(received);
                    
                    Serial.print("Paquet #");
                    Serial.print(packetCount);
                    Serial.print(": ");
                    Serial.println(received);
                    
                    updateDisplay();
                }
                
                radio_receiveMode = false;
                ensureReceiveMode();
            }
        }
    }
    
    // Verification periodique
    static unsigned long lastCheckMode = 0;
    if (now - lastCheckMode > 2000) {
        lastCheckMode = now;
        
        if (!radio_receiveMode) {
            ensureReceiveMode();
        }
    }
    
    // Mise a jour affichage
    static unsigned long lastDisplay = 0;
    if (now - lastDisplay > 200) {
        lastDisplay = now;
        updateAlertStates(now);
        updateDisplay();
    }
    
    delay(5);
}















