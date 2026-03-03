/*
 * EMETTEUR LoRa - Heltec WiFi LoRa 32 V3 + HX711 + OLED
 * Version FINALE V2 avec parametres exacts
 * Converti en .cpp
 */

#include <HX711_ADC.h>
#include <EEPROM.h>
#include <SPI.h>
#include <RadioLib.h>
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <limits.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>

// ===== Configuration HX711 =====
const int HX711_dout = 19;
const int HX711_sck = 20;
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
const int tareOffset_eepromAdress = calVal_eepromAdress + (int)sizeof(float);
const int eepromMagic_eepromAdress = tareOffset_eepromAdress + (int)sizeof(long);
const uint32_t EEPROM_MAGIC = 0x52554348; // "RUCH"
const int EEPROM_TOTAL_SIZE = 64;

// ===== Configuration DHT11 =====
#define DHT_PIN   26
#define DHT_TYPE  DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// ===== Mode economie d'energie =====
const bool LOW_POWER_MODE = true;
const uint8_t LOW_POWER_WAKE_MIN = 15;         // 5, 10 ou 15 minutes
const uint32_t LOW_POWER_SLEEP_S = (uint32_t)LOW_POWER_WAKE_MIN * 60U;
const uint32_t LOW_POWER_ACTIVE_WINDOW_MS = 12000; // fenetre de mesure avant envoi
const bool KEEP_AWAKE_WHEN_USB_SERIAL = true;
const uint32_t STARTUP_SETTLE_IGNORE_MS = 5000;
const uint32_t STARTUP_FORCE_SEND_MS = 25000;
const uint8_t STARTUP_STABLE_SAMPLES = 12;
const float STARTUP_STABLE_SPAN_G = 18.0f;
const float WEIGHT_EMA_ALPHA_OPT = 0.02f;      // configurable
const float WEIGHT_TEMP_COEFF = 0.30f;         // g / degC, configurable
const float SEND_DELTA_THRESHOLD_G = 10.0f;    // anti-envoi inutile
const uint16_t SEND_MAX_INTERVAL_MIN = 30;     // envoi de securite
const uint8_t HX711_SAMPLE_COUNT = 30;
const uint8_t HX711_TRIM_PERCENT = 20;
const uint16_t HX711_WAKE_SETTLE_MS = 200;
const uint16_t HX711_RATE_MS = 100;            // 10 Hz
const uint16_t HX711_MEASURE_PERIOD_MS = 2000;

// ===== Configuration batterie (Heltec V3) =====
// Ajuster ces broches si votre revision de carte differe.
const int BAT_ADC_PIN = 1;
const int BAT_ADC_EN_PIN = 37; // Heltec V3: enable mesure batterie actif a LOW
// Sur Heltec LoRa32 V3, la mesure VBAT passe par un diviseur ~4.9.
const float BAT_DIVIDER_RATIO = 4.9f;
const uint8_t BAT_ADC_SAMPLES = 8;
const unsigned long BAT_READ_INTERVAL_MS = 5000;
const float BAT_VOLTAGE_FULL = 4.14f;   // batterie 18650 pleine charge mesuree
const float BAT_VOLTAGE_EMPTY = 3.30f;  // seuil bas pratique sous charge

// ===== Configuration OLED =====
#define OLED_SDA   17
#define OLED_SCL   18
#define OLED_RST   21
#define VEXT_PIN   36
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// ===== Configuration LoRa avec RadioLib =====
#define LORA_CS    8
#define LORA_SCK   9
#define LORA_MOSI  10
#define LORA_MISO  11
#define LORA_BUSY  13
#define LORA_RST   12
#define LORA_DIO1  14
// Bouton PRG/BOOT Heltec V3
#define PRG_BUTTON_PIN 0

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
const char* LORA_NODE_ID = "RUCHE1";

// ===== Variables globales =====
float lastWeight = 0.0;
uint32_t packetsSent = 0;
uint32_t packetsFailed = 0;
bool oled_working = false;
bool tareInProgress = false;
bool waitingKnownMass = false;
unsigned long previousMillis = 0;
const long interval = 15000;
char txpacket[64];
float currentCalFactor = 696.0f;
const float DEFAULT_CAL_FACTOR = 696.0f;
const float MIN_VALID_CAL_FACTOR = 100.0f;
const float MAX_VALID_CAL_FACTOR = 5000.0f;
float lastTempC = NAN;
float lastHumPct = NAN;
unsigned long lastDhtReadMs = 0;
const unsigned long DHT_READ_INTERVAL_MS = 10000;
float batteryVoltage = NAN;
int batteryPercent = -1;
bool batteryChargingLikely = false;
float previousBatteryVoltage = NAN;
unsigned long lastBatteryReadMs = 0;
int activeBatteryAdcPin = BAT_ADC_PIN;
int16_t lastRxRSSI = -120;
bool prgLastRawState = HIGH;
bool prgStableState = HIGH;
unsigned long prgLastChangeMs = 0;
const unsigned long PRG_DEBOUNCE_MS = 35;
bool prgPressedLatched = false;
volatile bool loraRxFlag = false;
bool radioReceiveMode = false;
bool lowPowerFrameSent = false;
unsigned long bootMs = 0;
bool startupReady = false;
float startupBuf[STARTUP_STABLE_SAMPLES];
uint8_t startupBufCount = 0;
uint8_t startupBufIndex = 0;
float lastSentWeight = 0.0f;
bool lastSentWeightReady = false;
bool forceFastSend = false;
float calibrationBaseWeight = 0.0f;
bool calibrationBaseReady = false;
bool telemetryCycleDone = false;
SemaphoreHandle_t gDataMutex = NULL;
TaskHandle_t hxTaskHandle = NULL;
TaskHandle_t dhtTaskHandle = NULL;
TaskHandle_t loraTaskHandle = NULL;
char serialLine[32];
size_t serialLineLen = 0;
RTC_DATA_ATTR float rtcLastFilteredWeight = NAN;
RTC_DATA_ATTR float rtcLastSentWeight = NAN;
RTC_DATA_ATTR uint16_t rtcCyclesSinceLastSend = 0;
RTC_DATA_ATTR uint32_t rtcBootCount = 0;

bool envoyerPaquet(const char* message);
void readDhtSensor();
void readBatteryStatus();
int batteryPercentFromVoltage(float voltage);
int detectBatteryAdcPin();
uint8_t battToBars(int battPct);
void drawBatteryBars(int battPct, int x, int y);
uint8_t rssiToBars(int16_t rssi);
void drawSignalBars(int16_t rssi, int x, int y);
void beginLoRaReceive();
void onLoraDio1();
void handleLoRaCommand(const char* message);
void enterDeepSleep();
bool isUsbSerialActive();
void pushStartupSample(float w);
bool isStartupStable();
void displayMessage(const char* line1, const char* line2 = "", const char* line3 = "");
void processSerialLine(char* line);
void taskHX711(void* parameter);
void taskDht(void* parameter);
void taskLoRa(void* parameter);
float readWeightOptimized();
bool shouldSendTelemetry(float weight);

// ===== Filtrage HX711 =====
const uint8_t FILTER_WINDOW_SIZE = 20;
const float EMA_ALPHA_SLOW = 0.05f;
const float EMA_ALPHA_MED = 0.10f;
const float EMA_ALPHA_FAST = 0.25f;
const float FAST_TRACK_DELTA_G = 120.0f;
const float OUTLIER_JUMP_G = 100.0f;
const float DISPLAY_DEADBAND_G = 12.0f;
const uint8_t STEP_CONFIRM_SAMPLES = 2;
const uint8_t MEDIAN_WINDOW_SIZE = 7;
const uint8_t STABLE_CONFIRM_SAMPLES = 4;
const float AUTO_ZERO_WINDOW_G = 120.0f;      // zone "balance vide" pour corriger la derive
const float AUTO_ZERO_MAX_STEP_G = 1.5f;      // variation max entre 2 mesures pour corriger
const float AUTO_ZERO_ALPHA = 0.015f;         // vitesse de correction derive
const float ZERO_LOCK_G = 8.0f;               // affichage force a 0 sous ce seuil
const float TELEMETRY_EMA_ALPHA = 0.10f;      // lissage dedie aux trames envoyees
const float TELEMETRY_MAX_STEP_G = 120.0f;    // limite de variation par echantillon pour la telemetrie
const float FAST_CHANGE_TRIGGER_G = 60.0f;    // envoi immediat sur variation brusque
float filterWindow[FILTER_WINDOW_SIZE];
uint8_t filterIndex = 0;
uint8_t filterCount = 0;
float emaWeight = 0.0f;
bool emaReady = false;
float stableWeight = 0.0f;
uint8_t jumpStreak = 0;
float softwareZeroOffset = 0.0f;
float prevCorrectedRaw = 0.0f;
bool prevCorrectedRawReady = false;
float medianWindow[MEDIAN_WINDOW_SIZE];
uint8_t medianIndex = 0;
uint8_t medianCount = 0;
float pendingStableCandidate = 0.0f;
uint8_t pendingStableCount = 0;
double minuteWeightSum = 0.0;
uint16_t minuteWeightCount = 0;
float telemetryWeight = 0.0f;
bool telemetryWeightReady = false;
bool tareResidualPending = false;
float tareResidualAcc = 0.0f;
uint8_t tareResidualCount = 0;
const uint8_t TARE_RESIDUAL_SAMPLES = 8;

float filterTelemetryWeight(float inputWeight) {
    if (!telemetryWeightReady) {
        telemetryWeight = inputWeight;
        telemetryWeightReady = true;
        return telemetryWeight;
    }
    float delta = inputWeight - telemetryWeight;
    if (delta > TELEMETRY_MAX_STEP_G) delta = TELEMETRY_MAX_STEP_G;
    if (delta < -TELEMETRY_MAX_STEP_G) delta = -TELEMETRY_MAX_STEP_G;
    telemetryWeight += TELEMETRY_EMA_ALPHA * delta;
    return telemetryWeight;
}

float medianFilter(float raw) {
    medianWindow[medianIndex] = raw;
    medianIndex = (medianIndex + 1) % MEDIAN_WINDOW_SIZE;
    if (medianCount < MEDIAN_WINDOW_SIZE) {
        medianCount++;
    }

    float tmp[MEDIAN_WINDOW_SIZE];
    for (uint8_t i = 0; i < medianCount; i++) {
        tmp[i] = medianWindow[i];
    }
    for (uint8_t i = 0; i < medianCount; i++) {
        for (uint8_t j = i + 1; j < medianCount; j++) {
            if (tmp[j] < tmp[i]) {
                float t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    return tmp[medianCount / 2];
}

float filterWeight(float raw) {
    if (emaReady) {
        float diff = fabs(raw - emaWeight);
        if (diff > OUTLIER_JUMP_G) {
            jumpStreak++;
            if (jumpStreak >= STEP_CONFIRM_SAMPLES) {
                // Changement reel de charge: recaler rapidement le filtre.
                emaWeight = raw;
                filterWindow[0] = raw;
                filterIndex = 1;
                filterCount = 1;
                jumpStreak = 0;
                return emaWeight;
            }
            // Pic isole: ignorer.
            return emaWeight;
        }
        jumpStreak = 0;
    }

    filterWindow[filterIndex] = raw;
    filterIndex = (filterIndex + 1) % FILTER_WINDOW_SIZE;
    if (filterCount < FILTER_WINDOW_SIZE) {
        filterCount++;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < filterCount; i++) {
        sum += filterWindow[i];
    }
    float avg = sum / filterCount;

    if (!emaReady) {
        emaWeight = avg;
        emaReady = true;
    } else {
        float delta = fabs(avg - emaWeight);
        float alpha = EMA_ALPHA_SLOW;
        if (delta >= FAST_TRACK_DELTA_G) {
            alpha = EMA_ALPHA_FAST;
        } else if (delta >= 40.0f) {
            alpha = EMA_ALPHA_MED;
        }
        emaWeight = alpha * avg + (1.0f - alpha) * emaWeight;
    }

    return emaWeight;
}

float readWeightOptimized() {
    // Coupe HX711 entre acquisitions pour minimiser la conso batterie.
    LoadCell.powerUp();
    vTaskDelay(pdMS_TO_TICKS(HX711_WAKE_SETTLE_MS));

    float samples[HX711_SAMPLE_COUNT];
    uint8_t count = 0;
    for (uint8_t i = 0; i < HX711_SAMPLE_COUNT; i++) {
        unsigned long t0 = millis();
        bool got = false;
        while ((millis() - t0) < 250) {
            if (LoadCell.update()) {
                samples[count++] = LoadCell.getData();
                got = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (!got) continue;
        // Cadence mesure 10 Hz.
        vTaskDelay(pdMS_TO_TICKS(HX711_RATE_MS));
    }

    if (count < 5) {
        LoadCell.powerDown();
        return NAN;
    }

    for (uint8_t i = 0; i < count; i++) {
        for (uint8_t j = i + 1; j < count; j++) {
            if (samples[j] < samples[i]) {
                float tmp = samples[i];
                samples[i] = samples[j];
                samples[j] = tmp;
            }
        }
    }

    uint8_t trim = (uint8_t)((count * HX711_TRIM_PERCENT) / 100U);
    if ((trim * 2U) >= count) trim = 0;
    uint8_t begin = trim;
    uint8_t end = count - trim;
    float sum = 0.0f;
    uint8_t used = 0;
    for (uint8_t i = begin; i < end; i++) {
        sum += samples[i];
        used++;
    }
    if (used == 0) {
        LoadCell.powerDown();
        return NAN;
    }

    float avg = sum / (float)used;
    if (!emaReady || isnan(emaWeight)) {
        emaWeight = avg;
        emaReady = true;
    } else {
        emaWeight += WEIGHT_EMA_ALPHA_OPT * (avg - emaWeight);
    }

    float corrected = emaWeight;
    if (!isnan(lastTempC)) {
        // Compensation thermique simple autour de 20 C.
        corrected = emaWeight - ((lastTempC - 20.0f) * WEIGHT_TEMP_COEFF);
    }

    rtcLastFilteredWeight = corrected;
    LoadCell.powerDown();
    return corrected;
}

bool shouldSendTelemetry(float weight) {
    if (isnan(rtcLastSentWeight)) return true;
    if (fabs(weight - rtcLastSentWeight) > SEND_DELTA_THRESHOLD_G) return true;
    if (((uint32_t)rtcCyclesSinceLastSend * (uint32_t)LOW_POWER_WAKE_MIN) >= (uint32_t)SEND_MAX_INTERVAL_MIN) return true;
    return false;
}

uint8_t battToBars(int battPct) {
    if (battPct >= 90) return 5;
    if (battPct >= 70) return 4;
    if (battPct >= 50) return 3;
    if (battPct >= 30) return 2;
    if (battPct >= 10) return 1;
    return 0;
}

void drawBatteryBars(int battPct, int x, int y) {
    const int barW = 3;
    const int gap = 2;
    const int maxH = 10;
    const uint8_t bars = battToBars(battPct);

    for (uint8_t i = 0; i < 5; i++) {
        int h = 2 + i * 2;
        int bx = x + i * (barW + gap);
        int by = y + (maxH - h);
        oled.drawRect(bx, by, barW, h, SSD1306_WHITE);
        if (i < bars) {
            oled.fillRect(bx, by, barW, h, SSD1306_WHITE);
        }
    }
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
        int h = 2 + i * 2;
        int bx = x + i * (barW + gap);
        int by = y + (maxH - h);
        oled.drawRect(bx, by, barW, h, SSD1306_WHITE);
        if (i < bars) {
            oled.fillRect(bx, by, barW, h, SSD1306_WHITE);
        }
    }
}

// ===== Fonctions OLED =====
bool initOLED() {
    Serial.print("Init OLED... ");
    
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    vTaskDelay(pdMS_TO_TICKS(20));
    digitalWrite(OLED_RST, HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    Wire.begin(OLED_SDA, OLED_SCL);
    
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("ECHEC");
        return false;
    }
    
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("Balance LoRa");
    oled.display();
    
    Serial.println("OK");
    return true;
}

void updateDisplay() {
    if (!oled_working) return;
    float displayWeightLocal = 0.0f;
    float tempLocal = NAN;
    float humLocal = NAN;
    int batteryPercentLocal = -1;
    int16_t lastRxRSSILocal = -120;
    if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
        displayWeightLocal = fabs(lastWeight);
        tempLocal = lastTempC;
        humLocal = lastHumPct;
        batteryPercentLocal = batteryPercent;
        lastRxRSSILocal = lastRxRSSI;
        xSemaphoreGive(gDataMutex);
    }
    
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);

    oled.setCursor(0, 0);
    oled.print("P:");
    if (displayWeightLocal >= 1000.0f) {
        oled.print(displayWeightLocal / 1000.0f, 3);
        oled.print("kg");
    } else {
        oled.print(displayWeightLocal, 0);
        oled.print("g");
    }

    oled.setCursor(0, 22);
    oled.print("T:");
    if (!isnan(tempLocal)) {
        oled.print(tempLocal, 0);
        oled.print("C");
    } else {
        oled.print("--C");
    }

    oled.setCursor(0, 44);
    oled.print("H:");
    if (!isnan(humLocal)) {
        oled.print(humLocal, 0);
        oled.print("%");
    } else {
        oled.print("--%");
    }

    // BAT a droite
    oled.setTextSize(1);
    oled.setCursor(80, 44);
    oled.print("BAT");
    drawBatteryBars(batteryPercentLocal, 100, 42);

    // SGN juste dessous BAT
    oled.setCursor(80, 56);
    oled.print("SGN");
    drawSignalBars(lastRxRSSILocal, 100, 54);
    
    oled.display();
}

void readDhtSensor() {
    unsigned long now = millis();
    if (now - lastDhtReadMs < DHT_READ_INTERVAL_MS) return;
    lastDhtReadMs = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
        Serial.println("DHT11 lecture KO");
        return;
    }

    if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
        lastTempC = t;
        lastHumPct = h;
        xSemaphoreGive(gDataMutex);
    }
}

int batteryPercentFromVoltage(float voltage) {
    if (isnan(voltage) || voltage <= BAT_VOLTAGE_EMPTY) return 0;
    if (voltage >= BAT_VOLTAGE_FULL) return 100;

    // Approximation Li-ion 1S avec courbe par segments.
    if (voltage < 3.70f) {
        return (int)((voltage - 3.30f) * (45.0f / 0.40f));
    }
    if (voltage < 4.00f) {
        return 45 + (int)((voltage - 3.70f) * (35.0f / 0.30f));
    }
    return 80 + (int)((voltage - 4.00f) * (20.0f / (BAT_VOLTAGE_FULL - 4.00f)));
}

int detectBatteryAdcPin() {
    const int candidates[] = {1, 2, 3, 4, 5, 6, 7};
    int bestPin = BAT_ADC_PIN;
    uint32_t bestMv = 0;

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        int pin = candidates[i];
        analogSetPinAttenuation(pin, ADC_11db);
        uint32_t sum = 0;
        for (uint8_t s = 0; s < 4; s++) {
            sum += analogReadMilliVolts(pin);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        uint32_t avg = sum / 4;
        if (avg > bestMv) {
            bestMv = avg;
            bestPin = pin;
        }
    }

    Serial.print("BAT scan ADC best pin=");
    Serial.print(bestPin);
    Serial.print(" raw=");
    Serial.print(bestMv);
    Serial.println("mV");

    return bestPin;
}

void readBatteryStatus() {
    unsigned long now = millis();
    if (now - lastBatteryReadMs < BAT_READ_INTERVAL_MS) return;
    lastBatteryReadMs = now;

    // Sur Heltec V3, la voie batterie est generalement active quand EN=LOW.
    digitalWrite(BAT_ADC_EN_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(3));

    uint32_t mvSum = 0;
    for (uint8_t i = 0; i < BAT_ADC_SAMPLES; i++) {
        mvSum += analogReadMilliVolts(activeBatteryAdcPin);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    digitalWrite(BAT_ADC_EN_PIN, HIGH);

    float adcMvAvg = (float)mvSum / BAT_ADC_SAMPLES;

    // Si lecture nulle, chercher automatiquement le bon ADC batterie.
    if (adcMvAvg < 5.0f) {
        activeBatteryAdcPin = detectBatteryAdcPin();
        mvSum = 0;
        for (uint8_t i = 0; i < BAT_ADC_SAMPLES; i++) {
            mvSum += analogReadMilliVolts(activeBatteryAdcPin);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        adcMvAvg = (float)mvSum / BAT_ADC_SAMPLES;
    }

    float batteryVoltageLocal = (adcMvAvg / 1000.0f) * BAT_DIVIDER_RATIO;
    int batteryPercentLocal = batteryPercentFromVoltage(batteryVoltageLocal);
    bool batteryChargingLikelyLocal = false;
    if (!isnan(previousBatteryVoltage)) {
        batteryChargingLikelyLocal = (batteryVoltageLocal - previousBatteryVoltage) > 0.01f;
    }
    previousBatteryVoltage = batteryVoltageLocal;

    if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
        batteryVoltage = batteryVoltageLocal;
        batteryPercent = batteryPercentLocal;
        batteryChargingLikely = batteryChargingLikelyLocal;
        xSemaphoreGive(gDataMutex);
    }

    Serial.print("BAT raw=");
    Serial.print(adcMvAvg, 0);
    Serial.print("mV pin=");
    Serial.print(activeBatteryAdcPin);
    Serial.print(" calc=");
    Serial.print(batteryVoltageLocal, 2);
    Serial.print("V ");
    Serial.print(batteryPercentLocal);
    Serial.println("%");
}

void displayMessage(const char* line1, const char* line2, const char* line3) {
    if (!oled_working) return;
    
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 15);
    oled.println(line1);
    if (line2 != NULL && line2[0] != '\0') {
        oled.setCursor(0, 30);
        oled.println(line2);
    }
    if (line3 != NULL && line3[0] != '\0') {
        oled.setCursor(0, 45);
        oled.println(line3);
    }
    oled.display();
}

void saveCalFactor(float factor) {
    EEPROM.put(calVal_eepromAdress, factor);
    EEPROM.commit();
}

void saveTareOffset(long tareOffset) {
    EEPROM.put(tareOffset_eepromAdress, tareOffset);
    EEPROM.put(eepromMagic_eepromAdress, EEPROM_MAGIC);
    EEPROM.commit();
}

bool performCalibrationWithMass(float knownMass) {
    if (knownMass <= 0.0f) {
        Serial.println("Masse invalide. Exemple: c500");
        return false;
    }

    // Calibrage terrain: on ajoute une masse connue sur la ruche deja en place.
    // Utilise le delta de poids entre "avant" et "apres ajout".
    if (!calibrationBaseReady) {
        Serial.println("Calib: base absente. Utiliser 'c' puis poser la masse.");
        displayMessage("Calib KO", "Base absente");
        return false;
    }

    float currentWeight = stableWeight;
    float measuredDelta = fabs(currentWeight - calibrationBaseWeight);
    calibrationBaseReady = false;

    if (measuredDelta < 20.0f) {
        Serial.println("Calib KO: delta trop faible");
        displayMessage("Calib KO", "Delta trop faible");
        return false;
    }

    float ratio = measuredDelta / knownMass;
    float newCal = currentCalFactor * ratio;
    if (isnan(newCal) || isinf(newCal) || newCal == 0.0f) {
        Serial.println("Calibration echouee");
        displayMessage("Calib KO", "Valeur invalide");
        return false;
    }
    if (newCal < MIN_VALID_CAL_FACTOR || newCal > MAX_VALID_CAL_FACTOR) {
        Serial.println("Calib KO: facteur hors limite");
        displayMessage("Calib KO", "Hors limites");
        return false;
    }

    currentCalFactor = newCal;
    LoadCell.setCalFactor(currentCalFactor);
    saveCalFactor(currentCalFactor);

    // Reinitialise les filtres pour appliquer la nouvelle echelle immediatement.
    filterIndex = 0;
    filterCount = 0;
    emaWeight = 0.0f;
    emaReady = false;
    pendingStableCount = 0;
    telemetryWeightReady = false;

    Serial.print("Delta mesure: ");
    Serial.print(measuredDelta, 2);
    Serial.print(" g pour masse ref ");
    Serial.print(knownMass, 2);
    Serial.println(" g");
    Serial.print("Nouveau calFactor: ");
    Serial.println(currentCalFactor, 2);
    char calMsg[24];
    snprintf(calMsg, sizeof(calMsg), "%.2f", currentCalFactor);
    displayMessage("Calib OK", calMsg);
    return true;
}

bool performLegacyCalibrationWithMass(float knownMass) {
    if (knownMass <= 0.0f) {
        Serial.println("Masse invalide. Exemple: c500");
        return false;
    }

    // Compatibilite dashboard ancien: calibrage direct en une commande.
    LoadCell.refreshDataSet();
    float newCal = LoadCell.getNewCalibration(knownMass);
    if (isnan(newCal) || isinf(newCal) || newCal == 0.0f) {
        Serial.println("Calibration echouee (legacy)");
        displayMessage("Calib KO", "Legacy invalide");
        return false;
    }
    if (newCal < MIN_VALID_CAL_FACTOR || newCal > MAX_VALID_CAL_FACTOR) {
        Serial.println("Calib KO legacy: facteur hors limite");
        displayMessage("Calib KO", "Legacy hors lim");
        return false;
    }

    currentCalFactor = newCal;
    LoadCell.setCalFactor(currentCalFactor);
    saveCalFactor(currentCalFactor);
    filterIndex = 0;
    filterCount = 0;
    emaWeight = 0.0f;
    emaReady = false;
    pendingStableCount = 0;
    telemetryWeightReady = false;

    Serial.print("Legacy calFactor: ");
    Serial.println(currentCalFactor, 2);
    displayMessage("Calib OK", "Legacy");
    return true;
}

bool isUsbSerialActive() {
    if (!KEEP_AWAKE_WHEN_USB_SERIAL) return false;
    // Sur ESP32-S3 en USB CDC, Serial est "true" quand la liaison hote est active.
    return (bool)Serial;
}

void pushStartupSample(float w) {
    startupBuf[startupBufIndex] = w;
    startupBufIndex = (startupBufIndex + 1) % STARTUP_STABLE_SAMPLES;
    if (startupBufCount < STARTUP_STABLE_SAMPLES) startupBufCount++;
}

bool isStartupStable() {
    if (startupBufCount < STARTUP_STABLE_SAMPLES) return false;
    float mn = startupBuf[0];
    float mx = startupBuf[0];
    for (uint8_t i = 1; i < startupBufCount; i++) {
        if (startupBuf[i] < mn) mn = startupBuf[i];
        if (startupBuf[i] > mx) mx = startupBuf[i];
    }
    return (mx - mn) <= STARTUP_STABLE_SPAN_G;
}

void enterDeepSleep() {
    if (oled_working && !isUsbSerialActive()) {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(0, 18);
        oled.println("Sleep...");
        oled.display();
        vTaskDelay(pdMS_TO_TICKS(80));
        // Coupe l'alim OLED via VEXT.
        digitalWrite(VEXT_PIN, HIGH);
    }

    digitalWrite(BAT_ADC_EN_PIN, HIGH);
    LoadCell.powerDown();
    radio.sleep();
    WiFi.mode(WIFI_OFF);
    SPI.end();
    // Reveil periodique + reveil bouton PRG pour tare locale.
    esp_sleep_enable_timer_wakeup((uint64_t)LOW_POWER_SLEEP_S * 1000000ULL);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PRG_BUTTON_PIN, 0);
    esp_deep_sleep_start();
}

void beginLoRaReceive() {
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        radioReceiveMode = true;
    } else {
        radioReceiveMode = false;
        Serial.print("Erreur startReceive TX: ");
        Serial.println(state);
    }
}

void onLoraDio1() {
    loraRxFlag = true;
}

static char* trimInPlace(char* s) {
    if (s == NULL) return s;
    while (*s != '\0' && isspace((unsigned char)*s)) {
        s++;
    }
    if (*s == '\0') return s;
    char* end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
    return s;
}

void handleLoRaCommand(const char* message) {
    if (message == NULL) return;
    char msg[128];
    strncpy(msg, message, sizeof(msg) - 1);
    msg[sizeof(msg) - 1] = '\0';
    char* pMsg = trimInPlace(msg);
    if (strncmp(pMsg, "CMD:", 4) != 0) return;

    char* payload = pMsg + 4;
    char* firstSep = strchr(payload, ':');
    char* target = payload;
    char* commandPart = payload;
    if (firstSep != NULL) {
        *firstSep = '\0';
        commandPart = firstSep + 1;
    } else {
        target = (char*)"";
    }

    target = trimInPlace(target);
    commandPart = trimInPlace(commandPart);

    if (target[0] != '\0' && strcmp(target, "*") != 0 && strcmp(target, LORA_NODE_ID) != 0) {
        return;
    }

    if (strcasecmp(commandPart, "TARE") == 0) {
        if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
            LoadCell.tareNoDelay();
            tareInProgress = true;
            xSemaphoreGive(gDataMutex);
        }
        Serial.println("Commande LoRa: TARE");
        displayMessage("Tare distante");
        char ack[64];
        snprintf(ack, sizeof(ack), "ACK:%s:TARE:STARTED", LORA_NODE_ID);
        envoyerPaquet(ack);
        return;
    }

    if (strcasecmp(commandPart, "CAL_START") == 0) {
        if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
            calibrationBaseWeight = stableWeight;
            calibrationBaseReady = true;
            xSemaphoreGive(gDataMutex);
        }
        Serial.print("Commande LoRa: CAL_START base=");
        Serial.println(calibrationBaseWeight, 2);
        char ack[80];
        snprintf(ack, sizeof(ack), "ACK:%s:CAL_START:OK:%.2f", LORA_NODE_ID, calibrationBaseWeight);
        envoyerPaquet(ack);
        return;
    }

    if (strncasecmp(commandPart, "CAL:", 4) == 0) {
        char* massToken = trimInPlace(commandPart + 4);
        float knownMass = strtof(massToken, NULL);
        bool ok = false;
        if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
            if (calibrationBaseReady) {
                ok = performCalibrationWithMass(knownMass);
            } else {
                // Fallback pour compatibilite dashboard existant (commande unique).
                ok = performLegacyCalibrationWithMass(knownMass);
            }
            xSemaphoreGive(gDataMutex);
        }
        char ack[80];
        if (ok) {
            snprintf(ack, sizeof(ack), "ACK:%s:CAL:OK:%.2f", LORA_NODE_ID, knownMass);
        } else {
            snprintf(ack, sizeof(ack), "ACK:%s:CAL:ERR", LORA_NODE_ID);
        }
        envoyerPaquet(ack);
        return;
    }

    {
        char ack[72];
        snprintf(ack, sizeof(ack), "ACK:%s:ERR:UNKNOWN_CMD", LORA_NODE_ID);
        envoyerPaquet(ack);
    }
}

void processSerialLine(char* line) {
    char* pLine = trimInPlace(line);
    if (pLine == NULL || pLine[0] == '\0') return;

    if (waitingKnownMass) {
        float knownMass = strtof(pLine, NULL);
        if (knownMass <= 0.0f) {
            Serial.println("Masse invalide. Exemple: 500.0");
            return;
        }

        bool calOk = false;
        if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
            calOk = performCalibrationWithMass(knownMass);
            xSemaphoreGive(gDataMutex);
        }
        if (!calOk) {
            waitingKnownMass = false;
            return;
        }
        waitingKnownMass = false;
        return;
    }

    char cmd = (char)tolower((unsigned char)pLine[0]);
    switch(cmd) {
        case 't':
            if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
                LoadCell.tareNoDelay();
                tareInProgress = true;
                xSemaphoreGive(gDataMutex);
            }
            Serial.println("Tare lancee...");
            displayMessage("Tare...");
            break;
        case 'c':
            {
                char* massToken = trimInPlace(pLine + 1);
                if (massToken[0] != '\0') {
                    float knownMass = strtof(massToken, NULL);
                    bool calOk = false;
                    if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
                        calOk = performCalibrationWithMass(knownMass);
                        xSemaphoreGive(gDataMutex);
                    }
                    if (!calOk) {
                        Serial.println("Usage: c500  (ou c puis 500)");
                    }
                } else {
                    if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
                        calibrationBaseWeight = stableWeight;
                        calibrationBaseReady = true;
                        xSemaphoreGive(gDataMutex);
                    }
                    waitingKnownMass = true;
                    Serial.println("Poser masse connue puis saisir la masse en g (ex: 500.0)");
                    Serial.println("Astuce rapide: c500");
                    Serial.print("Base calib: ");
                    Serial.print(calibrationBaseWeight, 2);
                    Serial.println(" g");
                    displayMessage("Calibration", "Entrez masse g");
                }
            }
            break;
        case 'x':
            envoyerPaquet("TEST");
            break;
        case 'h':
            Serial.println("Commandes: t=tare, c=calibrage, c500=calib rapide, x=test envoi, h=aide");
            break;
        default:
            Serial.println("Commande inconnue. h pour aide.");
            break;
    }
}

// ===== Initialisation HX711 =====
bool initHX711() {
    Serial.print("Init HX711... ");
    
    LoadCell.begin();
    LoadCell.start(2500, false);
    
    if (LoadCell.getTareTimeoutFlag()) {
        Serial.println("ECHEC TIMEOUT");
        return false;
    }
    
    EEPROM.begin(EEPROM_TOTAL_SIZE);
    float calVal;
    EEPROM.get(calVal_eepromAdress, calVal);
    
    if (isnan(calVal) || isinf(calVal) || calVal == 0 ||
        calVal < MIN_VALID_CAL_FACTOR || calVal > MAX_VALID_CAL_FACTOR) {
        calVal = DEFAULT_CAL_FACTOR;
        EEPROM.put(calVal_eepromAdress, calVal);
        EEPROM.commit();
        Serial.print("(def-reset) ");
    }
    
    currentCalFactor = calVal;
    LoadCell.setCalFactor(currentCalFactor);
    uint32_t magic = 0;
    long storedTareOffset = 0;
    EEPROM.get(eepromMagic_eepromAdress, magic);
    if (magic == EEPROM_MAGIC) {
        EEPROM.get(tareOffset_eepromAdress, storedTareOffset);
        if (storedTareOffset != LONG_MIN && storedTareOffset != LONG_MAX) {
            LoadCell.setTareOffset(storedTareOffset);
            Serial.print(" tareOfs=");
            Serial.print(storedTareOffset);
        }
    } else {
        Serial.print(" tareOfs=none");
    }
    // Une mesure brute par cycle, le lissage est gere en logiciel optimise.
    LoadCell.setSamplesInUse(1);
    LoadCell.refreshDataSet();
    LoadCell.powerDown();
    Serial.print("OK, cal=");
    Serial.println(currentCalFactor, 2);
    
    return true;
}

// ===== Initialisation LoRa avec RadioLib =====
bool initLoRa() {
    Serial.print("Init LoRa (RadioLib)... ");
    
    // Reset materiel
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(LORA_RST, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialisation SPI
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    
    // Initialisation SX1262 avec frequence exacte
    int state = radio.begin(868.0); // 868 MHz
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("OK");
        
        // Configuration EXACTE pour compatibilite
        radio.setOutputPower(14);      // 14 dBm
        radio.setSpreadingFactor(7);    // SF7
        radio.setBandwidth(125.0);      // 125 kHz
        radio.setCodingRate(5);         // 4/5
        radio.setPreambleLength(8);     // 8 symboles
        radio.setSyncWord(0x12);        // Sync word standard
        radio.setCRC(true);              // CRC active
        radio.setDio1Action(onLoraDio1);
        beginLoRaReceive();
        
        Serial.println("   Frequence: 868.000 MHz");
        Serial.println("   SF: 7, BW: 125 kHz, CR: 4/5");
        return true;
    } else {
        Serial.print("ECHEC Code: ");
        Serial.println(state);
        return false;
    }
}

// ===== Fonction d'envoi avec RadioLib =====
bool envoyerPaquet(const char* message) {
    Serial.print("Envoi: ");
    Serial.print(message);
    Serial.print(" ... ");
    
    digitalWrite(LED_BUILTIN, HIGH);
    
    radioReceiveMode = false;
    // Envoi synchrone
    int state = radio.transmit(message); // 5 secondes timeout
    
    digitalWrite(LED_BUILTIN, LOW);
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("OK");
        packetsSent++;
        if (!LOW_POWER_MODE || isUsbSerialActive()) {
            beginLoRaReceive();
        }
        return true;
    } else {
        Serial.print("ECHEC Code: ");
        Serial.println(state);
        packetsFailed++;
        if (!LOW_POWER_MODE || isUsbSerialActive()) {
            beginLoRaReceive();
        }
        return false;
    }
}

// ===== Setup =====
void taskHX711(void* parameter) {
    (void)parameter;
    unsigned long lastRead = 0;
    while (true) {
        unsigned long now = millis();

        bool prgRaw = digitalRead(PRG_BUTTON_PIN);
        if (prgRaw != prgLastRawState) {
            prgLastRawState = prgRaw;
            prgLastChangeMs = now;
        }
        if ((now - prgLastChangeMs) > PRG_DEBOUNCE_MS && prgStableState != prgRaw) {
            prgStableState = prgRaw;
            if (prgStableState == LOW && !prgPressedLatched && !tareInProgress && !waitingKnownMass) {
                if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
                    prgPressedLatched = true;
                    LoadCell.tareNoDelay();
                    tareInProgress = true;
                    xSemaphoreGive(gDataMutex);
                }
                Serial.println("Tare lancee (bouton PRG)...");
                displayMessage("Tare...");
            } else if (prgStableState == HIGH) {
                prgPressedLatched = false;
            }
        }

        if (!tareInProgress && (now - lastRead > HX711_MEASURE_PERIOD_MS)) {
            float optWeight = readWeightOptimized();
            if (!isnan(optWeight)) {
                if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
                    stableWeight = optWeight;
                    if (fabs(stableWeight) < ZERO_LOCK_G) {
                        stableWeight = 0.0f;
                    }
                    lastWeight = stableWeight;
                    pushStartupSample(stableWeight);
                    if (!startupReady && (now - bootMs) > STARTUP_SETTLE_IGNORE_MS && isStartupStable()) {
                        startupReady = true;
                        Serial.println("Startup HX711 stable");
                    }
                    xSemaphoreGive(gDataMutex);
                }
            }
            lastRead = now;
        }

        if (tareInProgress) {
            LoadCell.powerUp();
            (void)LoadCell.update();
        }

        if (tareInProgress && LoadCell.getTareStatus()) {
            if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
                tareInProgress = false;
                saveTareOffset(LoadCell.getTareOffset());
                filterIndex = 0;
                filterCount = 0;
                emaWeight = 0.0f;
                emaReady = false;
                stableWeight = 0.0f;
                jumpStreak = 0;
                lastWeight = 0.0f;
                softwareZeroOffset = 0.0f;
                prevCorrectedRaw = 0.0f;
                prevCorrectedRawReady = false;
                medianIndex = 0;
                medianCount = 0;
                pendingStableCandidate = 0.0f;
                pendingStableCount = 0;
                minuteWeightSum = 0.0;
                minuteWeightCount = 0;
                telemetryWeight = 0.0f;
                telemetryWeightReady = false;
                tareResidualPending = true;
                tareResidualAcc = 0.0f;
                tareResidualCount = 0;
                lastSentWeight = 0.0f;
                lastSentWeightReady = false;
                forceFastSend = false;
                rtcLastFilteredWeight = NAN;
                xSemaphoreGive(gDataMutex);
            }
            LoadCell.powerDown();
            Serial.println("Tare terminee");
            displayMessage("Tare OK");
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void taskDht(void* parameter) {
    (void)parameter;
    while (true) {
        readDhtSensor();
        readBatteryStatus();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void taskLoRa(void* parameter) {
    (void)parameter;
    while (true) {
        unsigned long now = millis();

        if (loraRxFlag) {
            loraRxFlag = false;
            char incoming[128];
            memset(incoming, 0, sizeof(incoming));
            int state = radio.readData((uint8_t*)incoming, sizeof(incoming) - 1);
            if (state == RADIOLIB_ERR_NONE) {
                int16_t rssi = radio.getRSSI();
                if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
                    lastRxRSSI = rssi;
                    xSemaphoreGive(gDataMutex);
                }
                Serial.print("LoRa RX: ");
                Serial.println(incoming);
                handleLoRaCommand(incoming);
            }
            radioReceiveMode = false;
            beginLoRaReceive();
        }

        bool doSend = false;
        bool startupReadyLocal = false;
        float sendWeight = 0.0f;
        float tempLocal = NAN;
        float humLocal = NAN;
        int batteryPercentLocal = -1;

        if (gDataMutex != NULL && xSemaphoreTake(gDataMutex, portMAX_DELAY) == pdTRUE) {
            startupReadyLocal = startupReady;
            if (!telemetryCycleDone && startupReady && shouldSendTelemetry(lastWeight)) {
                doSend = true;
                sendWeight = lastWeight;
                tempLocal = lastTempC;
                humLocal = lastHumPct;
                batteryPercentLocal = batteryPercent;
                lastSentWeight = sendWeight;
                lastSentWeightReady = true;
                telemetryCycleDone = true;
                previousMillis = now;
            }
            xSemaphoreGive(gDataMutex);
        }

        if (doSend) {
            if (!startupReadyLocal && (now - bootMs) < STARTUP_FORCE_SEND_MS) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            char poidsMsg[96];
            if (!isnan(tempLocal) && !isnan(humLocal) && batteryPercentLocal >= 0) {
                snprintf(poidsMsg, sizeof(poidsMsg), "POIDS_G:%.2f,T_C:%.1f,H_P:%.0f,B_P:%d",
                         fabs(sendWeight), tempLocal, humLocal, batteryPercentLocal);
            } else if (!isnan(tempLocal) && !isnan(humLocal)) {
                snprintf(poidsMsg, sizeof(poidsMsg), "POIDS_G:%.2f,T_C:%.1f,H_P:%.0f",
                         fabs(sendWeight), tempLocal, humLocal);
            } else if (batteryPercentLocal >= 0) {
                snprintf(poidsMsg, sizeof(poidsMsg), "POIDS_G:%.2f,B_P:%d",
                         fabs(sendWeight), batteryPercentLocal);
            } else {
                snprintf(poidsMsg, sizeof(poidsMsg), "POIDS_G:%.2f", fabs(sendWeight));
            }
            envoyerPaquet(poidsMsg);
            rtcLastSentWeight = sendWeight;
            rtcCyclesSinceLastSend = 0;

            if (LOW_POWER_MODE && !isUsbSerialActive()) {
                lowPowerFrameSent = true;
                Serial.println("Low power: trame envoyee, passage en deep sleep");
                vTaskDelay(pdMS_TO_TICKS(50));
                enterDeepSleep();
            }
        }

        if (LOW_POWER_MODE && !lowPowerFrameSent && !isUsbSerialActive() && (now - bootMs) >= LOW_POWER_ACTIVE_WINDOW_MS) {
            // Pas de variation significative, on rend la main au deep sleep.
            Serial.println("Low power: pas de variation utile, deep sleep");
            enterDeepSleep();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));
    dht.begin();
    WiFi.mode(WIFI_OFF);
    pinMode(BAT_ADC_EN_PIN, OUTPUT);
    digitalWrite(BAT_ADC_EN_PIN, HIGH);
    analogReadResolution(12);
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
    pinMode(PRG_BUTTON_PIN, INPUT_PULLUP);
    bootMs = millis();
    rtcBootCount++;
    if (rtcCyclesSinceLastSend < 65000U) rtcCyclesSinceLastSend++;
    bool wakeByPrgButton = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0);

    Serial.println("\n================================");
    Serial.println("BALANCE LORA - RADIOLIB V2");
    Serial.println("================================\n");
    if (LOW_POWER_MODE) {
        Serial.println("Mode: LOW POWER");
        if (isUsbSerialActive()) {
            Serial.println("USB detecte: deep sleep suspendu");
        } else {
            Serial.print("Wake periodique: ");
            Serial.print(LOW_POWER_WAKE_MIN);
            Serial.println(" min");
        }
    }

    bool useDisplay = (!LOW_POWER_MODE || isUsbSerialActive());
    if (useDisplay) {
        oled_working = initOLED();
        vTaskDelay(pdMS_TO_TICKS(500));
    } else {
        oled_working = false;
        pinMode(VEXT_PIN, OUTPUT);
        digitalWrite(VEXT_PIN, HIGH);
    }

    if (!initHX711()) {
        Serial.println("ERREUR HX711");
        displayMessage("ERREUR HX711", "Verifiez cablage");
        while(1) {
            digitalWrite(LED_BUILTIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(200));
            digitalWrite(LED_BUILTIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    if (wakeByPrgButton) {
        Serial.println("Wake PRG: tare locale auto");
        LoadCell.powerUp();
        LoadCell.tareNoDelay();
        tareInProgress = true;
    }

    if (!initLoRa()) {
        Serial.println("ERREUR LORA");
        displayMessage("ERREUR LORA", "Module defectueux");
        while(1) {
            digitalWrite(LED_BUILTIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(LED_BUILTIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("\n=== PRET ===");
    Serial.println("t=tare, c=calibrage, x=test envoi, h=aide");
    Serial.println("============================\n");
    if (oled_working) {
        displayMessage("PRET", "Attente...");
    }

    if (!LOW_POWER_MODE) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        envoyerPaquet("DEMARRAGE");
    }

    gDataMutex = xSemaphoreCreateMutex();
    if (gDataMutex == NULL) {
        Serial.println("ERREUR mutex");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    xTaskCreatePinnedToCore(taskLoRa, "task_lora", 8192, NULL, 4, &loraTaskHandle, 1);
    xTaskCreatePinnedToCore(taskHX711, "task_hx711", 6144, NULL, 3, &hxTaskHandle, 1);
    xTaskCreatePinnedToCore(taskDht, "task_dht11", 4096, NULL, 2, &dhtTaskHandle, 1);
}

void loop() {
    static unsigned long lastDisplay = 0;

    while (Serial.available() > 0) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (serialLineLen > 0) {
                serialLine[serialLineLen] = '\0';
                processSerialLine(serialLine);
                serialLineLen = 0;
            }
        } else if (isPrintable(ch) && serialLineLen < (sizeof(serialLine) - 1)) {
            serialLine[serialLineLen++] = ch;
        }
    }

    unsigned long now = millis();
    if (now - lastDisplay > 500) {
        lastDisplay = now;
        updateDisplay();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

