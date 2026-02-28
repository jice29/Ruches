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

// ===== Configuration HX711 =====
const int HX711_dout = 19;
const int HX711_sck = 20;
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;

// ===== Configuration DHT11 =====
#define DHT_PIN   26
#define DHT_TYPE  DHT11
DHT dht(DHT_PIN, DHT_TYPE);

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

// ===== Variables globales =====
float lastWeight = 0.0;
uint32_t packetsSent = 0;
uint32_t packetsFailed = 0;
bool oled_working = false;
bool tareInProgress = false;
bool waitingKnownMass = false;
unsigned long previousMillis = 0;
const long interval = 60000;
char txpacket[64];
String serialLine;
float currentCalFactor = 696.0f;
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
bool prgLastRawState = HIGH;
bool prgStableState = HIGH;
unsigned long prgLastChangeMs = 0;
const unsigned long PRG_DEBOUNCE_MS = 35;
bool prgPressedLatched = false;

bool envoyerPaquet(const char* message);
void readDhtSensor();
void readBatteryStatus();
int batteryPercentFromVoltage(float voltage);
int detectBatteryAdcPin();
uint8_t battToBars(int battPct);
void drawBatteryBars(int battPct, int x, int y);

// ===== Filtrage HX711 =====
const uint8_t FILTER_WINDOW_SIZE = 24;
const float EMA_ALPHA = 0.08f;
const float OUTLIER_JUMP_G = 80.0f;
const float DISPLAY_DEADBAND_G = 12.0f;
const uint8_t STEP_CONFIRM_SAMPLES = 2;
const uint8_t MEDIAN_WINDOW_SIZE = 7;
const uint8_t STABLE_CONFIRM_SAMPLES = 3;
const float AUTO_ZERO_WINDOW_G = 120.0f;      // zone "balance vide" pour corriger la derive
const float AUTO_ZERO_MAX_STEP_G = 1.5f;      // variation max entre 2 mesures pour corriger
const float AUTO_ZERO_ALPHA = 0.015f;         // vitesse de correction derive
const float ZERO_LOCK_G = 8.0f;               // affichage force a 0 sous ce seuil
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
        emaWeight = EMA_ALPHA * avg + (1.0f - EMA_ALPHA) * emaWeight;
    }

    return emaWeight;
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
    oled.println("Balance LoRa");
    oled.display();
    
    Serial.println("OK");
    return true;
}

void updateDisplay() {
    if (!oled_working) return;
    
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);

    oled.setCursor(0, 0);
    oled.print("P:");
    oled.print(fabs(lastWeight), 0);
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

    // BAT en bas a droite avec barres
    oled.setTextSize(1);
    oled.setCursor(80, 54);
    oled.print("BAT");
    drawBatteryBars(batteryPercent, 100, 52);
    
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

    lastTempC = t;
    lastHumPct = h;
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
            delay(1);
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
    delay(3);

    uint32_t mvSum = 0;
    for (uint8_t i = 0; i < BAT_ADC_SAMPLES; i++) {
        mvSum += analogReadMilliVolts(activeBatteryAdcPin);
        delay(2);
    }

    digitalWrite(BAT_ADC_EN_PIN, HIGH);

    float adcMvAvg = (float)mvSum / BAT_ADC_SAMPLES;

    // Si lecture nulle, chercher automatiquement le bon ADC batterie.
    if (adcMvAvg < 5.0f) {
        activeBatteryAdcPin = detectBatteryAdcPin();
        mvSum = 0;
        for (uint8_t i = 0; i < BAT_ADC_SAMPLES; i++) {
            mvSum += analogReadMilliVolts(activeBatteryAdcPin);
            delay(2);
        }
        adcMvAvg = (float)mvSum / BAT_ADC_SAMPLES;
    }

    batteryVoltage = (adcMvAvg / 1000.0f) * BAT_DIVIDER_RATIO;
    batteryPercent = batteryPercentFromVoltage(batteryVoltage);

    if (!isnan(previousBatteryVoltage)) {
        batteryChargingLikely = (batteryVoltage - previousBatteryVoltage) > 0.01f;
    }
    previousBatteryVoltage = batteryVoltage;

    Serial.print("BAT raw=");
    Serial.print(adcMvAvg, 0);
    Serial.print("mV pin=");
    Serial.print(activeBatteryAdcPin);
    Serial.print(" calc=");
    Serial.print(batteryVoltage, 2);
    Serial.print("V ");
    Serial.print(batteryPercent);
    Serial.println("%");
}

void displayMessage(const String& line1, const String& line2 = "", const String& line3 = "") {
    if (!oled_working) return;
    
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 15);
    oled.println(line1);
    if (line2.length() > 0) {
        oled.setCursor(0, 30);
        oled.println(line2);
    }
    if (line3.length() > 0) {
        oled.setCursor(0, 45);
        oled.println(line3);
    }
    oled.display();
}

void saveCalFactor(float factor) {
    EEPROM.put(calVal_eepromAdress, factor);
    EEPROM.commit();
}

void processSerialLine(String line) {
    line.trim();
    if (line.length() == 0) return;

    if (waitingKnownMass) {
        float knownMass = line.toFloat();
        if (knownMass <= 0.0f) {
            Serial.println("Masse invalide. Exemple: 500.0");
            return;
        }

        LoadCell.refreshDataSet();
        float newCal = LoadCell.getNewCalibration(knownMass);
        if (isnan(newCal) || isinf(newCal) || newCal == 0.0f) {
            Serial.println("Calibration echouee");
            displayMessage("Calib KO", "Valeur invalide");
            waitingKnownMass = false;
            return;
        }

        currentCalFactor = newCal;
        LoadCell.setCalFactor(currentCalFactor);
        saveCalFactor(currentCalFactor);
        waitingKnownMass = false;

        Serial.print("Nouveau calFactor: ");
        Serial.println(currentCalFactor, 2);
        displayMessage("Calib OK", String(currentCalFactor, 2));
        return;
    }

    char cmd = tolower(line.charAt(0));
    switch(cmd) {
        case 't':
            LoadCell.tareNoDelay();
            tareInProgress = true;
            Serial.println("Tare lancee...");
            displayMessage("Tare...");
            break;
        case 'c':
            waitingKnownMass = true;
            Serial.println("Poser masse connue puis saisir la masse en g (ex: 500.0)");
            displayMessage("Calibration", "Entrez masse g");
            break;
        case 'x':
            envoyerPaquet("TEST");
            break;
        case 'h':
            Serial.println("Commandes: t=tare, c=calibrage, x=test envoi, h=aide");
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
    
    EEPROM.begin(512);
    float calVal;
    EEPROM.get(calVal_eepromAdress, calVal);
    
    if (isnan(calVal) || isinf(calVal) || calVal == 0) {
        calVal = 696.0;
        Serial.print("(def) ");
    }
    
    currentCalFactor = calVal;
    LoadCell.setCalFactor(currentCalFactor);
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
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(100);
    
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
    
    // Envoi synchrone avec timeout plus long
    int state = radio.transmit(message); // 5 secondes timeout
    
    digitalWrite(LED_BUILTIN, LOW);
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("OK");
        packetsSent++;
        return true;
    } else {
        Serial.print("ECHEC Code: ");
        Serial.println(state);
        packetsFailed++;
        return false;
    }
}

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    dht.begin();
    pinMode(BAT_ADC_EN_PIN, OUTPUT);
    // Laisser desactive au repos pour limiter la conso.
    digitalWrite(BAT_ADC_EN_PIN, HIGH);
    analogReadResolution(12);
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
    pinMode(PRG_BUTTON_PIN, INPUT_PULLUP);
    
    Serial.println("\n================================");
    Serial.println("BALANCE LORA - RADIOLIB V2");
    Serial.println("================================\n");
    
    oled_working = initOLED();
    delay(500);
    
    if (!initHX711()) {
        Serial.println("ERREUR HX711");
        displayMessage("ERREUR HX711", "Verifiez cablage");
        while(1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
    }
    delay(500);
    
    if (!initLoRa()) {
        Serial.println("ERREUR LORA");
        displayMessage("ERREUR LORA", "Module defectueux");
        while(1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }
    delay(500);
    
    Serial.println("\n=== PRET ===");
    Serial.println("t=tare, c=calibrage, x=test envoi, h=aide");
    Serial.println("============================\n");
    
    displayMessage("PRET", "Attente...");
    
    // Test au demarrage
    delay(2000);
    envoyerPaquet("DEMARRAGE");
}

// ===== Loop =====
void loop() {
    static unsigned long lastRead = 0;
    static unsigned long lastDisplay = 0;
    unsigned long now = millis();

    // Bouton PRG -> tare (declenchement unique sur appui, avec anti-rebond)
    bool prgRaw = digitalRead(PRG_BUTTON_PIN);
    if (prgRaw != prgLastRawState) {
        prgLastRawState = prgRaw;
        prgLastChangeMs = now;
    }
    if ((now - prgLastChangeMs) > PRG_DEBOUNCE_MS && prgStableState != prgRaw) {
        prgStableState = prgRaw;
        if (prgStableState == LOW && !prgPressedLatched && !tareInProgress && !waitingKnownMass) {
            prgPressedLatched = true;
            LoadCell.tareNoDelay();
            tareInProgress = true;
            Serial.println("Tare lancee (bouton PRG)...");
            displayMessage("Tare...");
        } else if (prgStableState == HIGH) {
            prgPressedLatched = false;
        }
    }
    
    // Lecture HX711
    if (LoadCell.update()) {
        if (now - lastRead > 200) {
            float rawWeight = LoadCell.getData();

            // Auto-zero lent: corrige seulement autour de zero et si le signal est stable.
            float correctedRaw = rawWeight - softwareZeroOffset;
            if (prevCorrectedRawReady) {
                float d = fabs(correctedRaw - prevCorrectedRaw);
                if (fabs(correctedRaw) <= AUTO_ZERO_WINDOW_G && d <= AUTO_ZERO_MAX_STEP_G) {
                    softwareZeroOffset += AUTO_ZERO_ALPHA * correctedRaw;
                    correctedRaw = rawWeight - softwareZeroOffset;
                }
            }
            prevCorrectedRaw = correctedRaw;
            prevCorrectedRawReady = true;

            float medWeight = medianFilter(correctedRaw);
            float filteredWeight = filterWeight(medWeight);
            if (fabs(filteredWeight - stableWeight) >= DISPLAY_DEADBAND_G) {
                if (pendingStableCount == 0 || fabs(filteredWeight - pendingStableCandidate) >= DISPLAY_DEADBAND_G) {
                    pendingStableCandidate = filteredWeight;
                    pendingStableCount = 1;
                } else {
                    pendingStableCount++;
                    if (pendingStableCount >= STABLE_CONFIRM_SAMPLES) {
                        stableWeight = pendingStableCandidate;
                        pendingStableCount = 0;
                    }
                }
            } else {
                pendingStableCount = 0;
            }
            if (fabs(stableWeight) < ZERO_LOCK_G) {
                stableWeight = 0.0f;
            }
            lastWeight = stableWeight;
            minuteWeightSum += (double)stableWeight;
            minuteWeightCount++;
            lastRead = now;
        }
    }

    readDhtSensor();
    readBatteryStatus();
    
    if (tareInProgress && LoadCell.getTareStatus()) {
        tareInProgress = false;
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
        Serial.println("Tare terminee");
        displayMessage("Tare OK");
    }

    // Commandes serie (ligne complete)
    while (Serial.available() > 0) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (serialLine.length() > 0) {
                processSerialLine(serialLine);
                serialLine = "";
            }
        } else if (isPrintable(ch) && serialLine.length() < 24) {
            serialLine += ch;
        }
    }
    
    // Envoi periodique
    if (now - previousMillis >= interval) {
        previousMillis = now;
        float sendWeight = lastWeight;
        if (minuteWeightCount > 0) {
            sendWeight = (float)(minuteWeightSum / (double)minuteWeightCount);
            minuteWeightSum = 0.0;
            minuteWeightCount = 0;
        }
        
        char poidsMsg[96];
        // CORRECTION: valeur absolue pour eviter les valeurs negatives
        if (!isnan(lastTempC) && !isnan(lastHumPct) && batteryPercent >= 0) {
            snprintf(
                poidsMsg,
                sizeof(poidsMsg),
                "POIDS_G:%.2f,T_C:%.1f,H_P:%.0f,B_P:%d",
                fabs(sendWeight),
                lastTempC,
                lastHumPct,
                batteryPercent
            );
        } else if (!isnan(lastTempC) && !isnan(lastHumPct)) {
            snprintf(
                poidsMsg,
                sizeof(poidsMsg),
                "POIDS_G:%.2f,T_C:%.1f,H_P:%.0f",
                fabs(sendWeight),
                lastTempC,
                lastHumPct
            );
        } else if (batteryPercent >= 0) {
            snprintf(
                poidsMsg,
                sizeof(poidsMsg),
                "POIDS_G:%.2f,B_P:%d",
                fabs(sendWeight),
                batteryPercent
            );
        } else {
            snprintf(poidsMsg, sizeof(poidsMsg), "POIDS_G:%.2f", fabs(sendWeight));
        }
        envoyerPaquet(poidsMsg);
    }
    
    // Mise a jour affichage
    if (now - lastDisplay > 500) {
        lastDisplay = now;
        updateDisplay();
    }
    
    delay(10);
}

