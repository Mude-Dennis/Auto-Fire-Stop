#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <time.h>


#include <OneWire.h>
#include <DallasTemperature.h>


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//  Firebase 
#include <FirebaseESP32.h>

//  Telegram 
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

//  USER CONFIGURATION  — replace before deployment

static constexpr const char* WIFI_SSID        = "YOUR_SSID";
static constexpr const char* WIFI_PASSWORD    = "YOUR_PASSWORD";
static constexpr const char* FIREBASE_HOST    = "YOUR_PROJECT.firebaseio.com";
static constexpr const char* FIREBASE_API_KEY = "YOUR_FIREBASE_API_KEY";
static constexpr const char* BOT_TOKEN        = "YOUR_BOT_TOKEN";
static constexpr const char* CHAT_ID          = "YOUR_CHAT_ID";
static constexpr const char* NTP_SERVER       = "pool.ntp.org";

//  PIN DEFINITIONS

static constexpr uint8_t PIN_MQ2_ANALOG  = 34;
static constexpr uint8_t PIN_DS18B20     = 4;
static constexpr uint8_t PIN_FLAME       = 27;
static constexpr uint8_t PIN_RELAY       = 26;
static constexpr uint8_t PIN_BUZZER      = 25;
static constexpr uint8_t PIN_OLED_SDA    = 21;
static constexpr uint8_t PIN_OLED_SCL    = 22;

//  THRESHOLDS & TIMING

static constexpr float    TEMP_WARN_HIGH        = 60.0f;
static constexpr float    TEMP_WARN_LOW         = 55.0f;   // hysteresis
static constexpr int      SMOKE_WARN_HIGH       = 400;
static constexpr int      SMOKE_WARN_LOW        = 350;     // hysteresis
static constexpr float    TEMP_FAULT_LOW        = -55.0f;  // DS18B20 error sentinel
static constexpr float    TEMP_FAULT_HIGH       = 150.0f;
static constexpr int      SMOKE_FAULT_LOW       = 0;
static constexpr int      SMOKE_FAULT_HIGH      = 4095;

static constexpr uint32_t INTERVAL_SENSOR_MS    = 500;
static constexpr uint32_t INTERVAL_DISPLAY_MS   = 750;
static constexpr uint32_t INTERVAL_FIREBASE_MS  = 5000;
static constexpr uint32_t INTERVAL_WIFI_MS      = 10000;
static constexpr uint32_t INTERVAL_BUZZER_MS    = 300;

static constexpr uint32_t DANGER_TO_EMERGENCY_MS = 5000;
static constexpr uint32_t EMERGENCY_CONFIRM_MS   = 3000;
static constexpr uint32_t WDT_TIMEOUT_S          = 10;

static constexpr uint8_t  SENSOR_AVG_SAMPLES     = 8;
static constexpr uint8_t  OLED_WIDTH             = 128;
static constexpr uint8_t  OLED_HEIGHT            = 64;
static constexpr int8_t   OLED_RESET             = -1;

//  STATE MACHINE

enum class SystemState : uint8_t {
    SAFE         = 0,
    WARNING      = 1,
    DANGER       = 2,
    EMERGENCY    = 3,
    SENSOR_FAULT = 4,
    WIFI_FAULT   = 5
};

//  DATA STRUCTURES

struct SensorData {
    float    temperature    = 0.0f;
    int      smokeRaw       = 0;
    bool     flameDetected  = false;
    bool     tempValid      = true;
    bool     smokeValid     = true;
    bool     flameValid     = true;
};

struct SystemStatus {
    SystemState  state            = SystemState::SAFE;
    SystemState  prevState        = SystemState::SAFE;
    bool         suppressionOn    = false;
    bool         wifiConnected    = false;
    bool         firebaseOk       = false;
    int8_t       rssi             = 0;
    char         timestamp[32]    = {};
};

struct AlertFlags {
    SystemState  lastAlerted      = SystemState::SAFE;
    bool         suppressionAlert = false;
    bool         faultAlert       = false;
    bool         safeAlert        = false;
};

//  GLOBALS (minimised — only objects that must be global)

OneWire            oneWire(PIN_DS18B20);
DallasTemperature  tempSensor(&oneWire);

Adafruit_SSD1306   display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

FirebaseData       fbData;
FirebaseAuth       fbAuth;
FirebaseConfig     fbConfig;

WiFiClientSecure   secureClient;
UniversalTelegramBot bot(BOT_TOKEN, secureClient);

Preferences        prefs;

SensorData   sensors;
SystemStatus sysStatus;
AlertFlags   alerts;

// millis() timers
static uint32_t tSensor    = 0;
static uint32_t tDisplay   = 0;
static uint32_t tFirebase  = 0;
static uint32_t tWifi      = 0;
static uint32_t tBuzzer    = 0;
static uint32_t tDangerStart   = 0;
static uint32_t tEmergencyConf = 0;
static bool     dangerTimerActive   = false;
static bool     emergencyConfActive = false;
static bool     buzzerState         = false;

// Smoke averaging ring buffer
static int   smokeBuffer[SENSOR_AVG_SAMPLES] = {};
static uint8_t smokeIdx = 0;
static bool  smokeBufferFull = false;

//  FORWARD DECLARATIONS

void     initHardware();
void     selfTest();
void     connectWiFi();
void     initFirebase();
void     syncNTP();
void     updateTimestamp();

void     taskSensors();
void     taskDisplay();
void     taskFirebase();
void     taskWiFiWatchdog();
void     taskBuzzer();

void     readTemperature();
void     readSmoke();
void     readFlame();
bool     detectSensorFaults();

int      averageSmoke(int raw);
bool     tempTriggered();
bool     smokeTriggered();

void     updateStateMachine();
uint8_t  countTriggeredSensors();
void     evaluateSupression();

void     activateSuppression();
void     deactivateSuppression();

void     sendTelegramAlert(const char* msg);
void     handleTelegramAlerts();

void     uploadFirebase();

void     renderDisplay();
const char* stateString(SystemState s);

void     logPrefs(SystemState s, bool suppression);
void     loadPrefs();

//  SETUP

void setup() {
    Serial.begin(115200);

    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    initHardware();
    loadPrefs();
    selfTest();
    connectWiFi();
    initFirebase();
    syncNTP();

    Serial.println(F("[BOOT] System ready."));
}

//  LOOP

void loop() {
    esp_task_wdt_reset();

    const uint32_t now = millis();

    if (now - tSensor   >= INTERVAL_SENSOR_MS)   { tSensor   = now; taskSensors();       }
    if (now - tDisplay  >= INTERVAL_DISPLAY_MS)  { tDisplay  = now; taskDisplay();        }
    if (now - tFirebase >= INTERVAL_FIREBASE_MS) { tFirebase = now; taskFirebase();       }
    if (now - tWifi     >= INTERVAL_WIFI_MS)     { tWifi     = now; taskWiFiWatchdog();   }
    if (now - tBuzzer   >= INTERVAL_BUZZER_MS)   { tBuzzer   = now; taskBuzzer();         }

    updateStateMachine();
    evaluateSupression();
    handleTelegramAlerts();
}

//  INITIALISATION

void initHardware() {
    pinMode(PIN_RELAY,  OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_FLAME,  INPUT);

    digitalWrite(PIN_RELAY,  LOW);
    digitalWrite(PIN_BUZZER, LOW);

    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("[OLED] Init failed"));
    } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("Fire Guard v1.0"));
        display.display();
    }

    tempSensor.begin();
    tempSensor.setResolution(12);
    tempSensor.setWaitForConversion(false);
    tempSensor.requestTemperatures();

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
}

void selfTest() {
    Serial.println(F("[TEST] Self-test start"));

    // Relay pulse
    digitalWrite(PIN_RELAY, HIGH);  delay(200);
    digitalWrite(PIN_RELAY, LOW);

    // Buzzer beep
    digitalWrite(PIN_BUZZER, HIGH); delay(150);
    digitalWrite(PIN_BUZZER, LOW);

    // Check DS18B20 presence
    DeviceAddress addr;
    if (!tempSensor.getAddress(addr, 0)) {
        Serial.println(F("[TEST] DS18B20 not found"));
        sensors.tempValid = false;
    }

    // Smoke sanity read
    int raw = analogRead(PIN_MQ2_ANALOG);
    if (raw <= SMOKE_FAULT_LOW || raw >= SMOKE_FAULT_HIGH) {
        Serial.printf("[TEST] Smoke raw=%d suspect\n", raw);
    }

    Serial.println(F("[TEST] Self-test complete"));
}

//  WIFI & CONNECTIVITY

void connectWiFi() {
    Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        attempts++;
        esp_task_wdt_reset();
    }

    if (WiFi.status() == WL_CONNECTED) {
        sysStatus.wifiConnected = true;
        sysStatus.rssi = WiFi.RSSI();
        Serial.printf("[WiFi] Connected, RSSI=%d\n", sysStatus.rssi);
    } else {
        sysStatus.wifiConnected = false;
        Serial.println(F("[WiFi] Connection failed"));
    }
}

void initFirebase() {
    fbConfig.host              = FIREBASE_HOST;
    fbConfig.api_key           = FIREBASE_API_KEY;
    fbConfig.token_status_callback = tokenStatusCallback;

    Firebase.begin(&fbConfig, &fbAuth);
    Firebase.reconnectWiFi(true);
    sysStatus.firebaseOk = Firebase.ready();
}

void syncNTP() {
    if (!sysStatus.wifiConnected) return;
    configTime(0, 0, NTP_SERVER);
    uint8_t retries = 0;
    time_t now = 0;
    while (now < 1000000000UL && retries++ < 10) {
        delay(500);
        time(&now);
        esp_task_wdt_reset();
    }
    Serial.printf("[NTP] Time synced: %lu\n", (unsigned long)now);
}

void updateTimestamp() {
    time_t now;
    struct tm ti;
    time(&now);
    gmtime_r(&now, &ti);
    strftime(sysStatus.timestamp, sizeof(sysStatus.timestamp),
             "%Y-%m-%dT%H:%M:%SZ", &ti);
}

void taskWiFiWatchdog() {
    if (WiFi.status() != WL_CONNECTED) {
        sysStatus.wifiConnected = false;
        Serial.println(F("[WiFi] Lost — reconnecting"));
        WiFi.reconnect();
    } else {
        sysStatus.wifiConnected = true;
        sysStatus.rssi = WiFi.RSSI();
    }
}

//  SENSOR TASKS

void taskSensors() {
    readTemperature();
    readSmoke();
    readFlame();
    detectSensorFaults();
}

void readTemperature() {
    float t = tempSensor.getTempCByIndex(0);
    tempSensor.requestTemperatures();   // async for next cycle

    if (t == DEVICE_DISCONNECTED_C || t < TEMP_FAULT_LOW || t > TEMP_FAULT_HIGH) {
        sensors.tempValid = false;
    } else {
        sensors.tempValid   = true;
        sensors.temperature = t;
    }
}

void readSmoke() {
    int raw = analogRead(PIN_MQ2_ANALOG);

    if (raw <= SMOKE_FAULT_LOW || raw >= SMOKE_FAULT_HIGH) {
        sensors.smokeValid = false;
    } else {
        sensors.smokeValid = true;
        sensors.smokeRaw   = averageSmoke(raw);
    }
}

void readFlame() {
    // Flame sensor: LOW = flame detected (active-low)
    sensors.flameDetected = (digitalRead(PIN_FLAME) == LOW);
    sensors.flameValid    = true;   // digital pin has no fault mode; assume valid
}

int averageSmoke(int raw) {
    smokeBuffer[smokeIdx] = raw;
    smokeIdx = (smokeIdx + 1) % SENSOR_AVG_SAMPLES;
    if (smokeIdx == 0) smokeBufferFull = true;

    uint8_t count = smokeBufferFull ? SENSOR_AVG_SAMPLES : smokeIdx;
    long sum = 0;
    for (uint8_t i = 0; i < count; i++) sum += smokeBuffer[i];
    return (int)(sum / count);
}

bool tempTriggered() {
    static bool above = false;
    if (!sensors.tempValid) return false;
    if (!above && sensors.temperature >= TEMP_WARN_HIGH) above = true;
    if (above  && sensors.temperature <  TEMP_WARN_LOW)  above = false;
    return above;
}

bool smokeTriggered() {
    static bool above = false;
    if (!sensors.smokeValid) return false;
    if (!above && sensors.smokeRaw >= SMOKE_WARN_HIGH) above = true;
    if (above  && sensors.smokeRaw <  SMOKE_WARN_LOW)  above = false;
    return above;
}

bool detectSensorFaults() {
    bool fault = false;
    if (!sensors.tempValid)  { fault = true; Serial.println(F("[FAULT] Temp sensor")); }
    if (!sensors.smokeValid) { fault = true; Serial.println(F("[FAULT] Smoke sensor")); }
    return fault;
}


//  STATE MACHINE

uint8_t countTriggeredSensors() {
    uint8_t count = 0;
    if (tempTriggered())           count++;
    if (smokeTriggered())          count++;
    if (sensors.flameDetected)     count++;
    return count;
}

void updateStateMachine() {
    sysStatus.prevState = sysStatus.state;

    // Sensor fault takes priority
    if (!sensors.tempValid || !sensors.smokeValid) {
        sysStatus.state = SystemState::SENSOR_FAULT;
        return;
    }

    if (!sysStatus.wifiConnected) {
        // Don't override EMERGENCY for wifi fault
        if (sysStatus.state != SystemState::EMERGENCY &&
            sysStatus.state != SystemState::DANGER) {
            sysStatus.state = SystemState::WIFI_FAULT;
        }
        return;
    }

    const uint8_t triggered = countTriggeredSensors();
    const uint32_t now = millis();

    switch (triggered) {
        case 0:
            sysStatus.state         = SystemState::SAFE;
            dangerTimerActive       = false;
            emergencyConfActive     = false;
            break;

        case 1:
            sysStatus.state     = SystemState::WARNING;
            dangerTimerActive   = false;
            emergencyConfActive = false;
            break;

        case 2:
            if (sysStatus.state != SystemState::DANGER &&
                sysStatus.state != SystemState::EMERGENCY) {
                sysStatus.state   = SystemState::DANGER;
                tDangerStart      = now;
                dangerTimerActive = true;
            }
            if (dangerTimerActive && (now - tDangerStart >= DANGER_TO_EMERGENCY_MS)) {
                sysStatus.state     = SystemState::EMERGENCY;
                dangerTimerActive   = false;
                emergencyConfActive = true;
                tEmergencyConf      = now;
            }
            break;

        case 3:
            sysStatus.state     = SystemState::EMERGENCY;
            dangerTimerActive   = false;
            if (!emergencyConfActive) {
                emergencyConfActive = true;
                tEmergencyConf      = now;
            }
            break;

        default:
            break;
    }
}


//  SUPPRESSION

void evaluateSupression() {
    const uint32_t now = millis();

    if (sysStatus.state == SystemState::EMERGENCY) {
        if (emergencyConfActive && (now - tEmergencyConf >= EMERGENCY_CONFIRM_MS)) {
            if (!sysStatus.suppressionOn) activateSuppression();
        }
    } else {
        if (sysStatus.suppressionOn) deactivateSuppression();
        emergencyConfActive = false;
    }
}

void activateSuppression() {
    sysStatus.suppressionOn = true;
    digitalWrite(PIN_RELAY, HIGH);
    logPrefs(sysStatus.state, true);
    Serial.println(F("[SUPPRESS] RELAY ON"));
}

void deactivateSuppression() {
    sysStatus.suppressionOn = false;
    digitalWrite(PIN_RELAY, LOW);
    logPrefs(sysStatus.state, false);
    Serial.println(F("[SUPPRESS] RELAY OFF"));
}

//  BUZZER

void taskBuzzer() {
    switch (sysStatus.state) {
        case SystemState::EMERGENCY:
            buzzerState = !buzzerState;
            digitalWrite(PIN_BUZZER, buzzerState ? HIGH : LOW);
            break;
        case SystemState::DANGER:
            // Slow blink
            { static uint8_t cnt = 0;
              cnt++;
              if (cnt % 2 == 0) { buzzerState = !buzzerState; }
              digitalWrite(PIN_BUZZER, buzzerState ? HIGH : LOW);
            }
            break;
        case SystemState::WARNING:
            // Single short beep every ~3 s (10 * 300ms)
            { static uint8_t cnt2 = 0;
              cnt2++;
              digitalWrite(PIN_BUZZER, (cnt2 % 10 == 0) ? HIGH : LOW);
            }
            break;
        default:
            buzzerState = false;
            digitalWrite(PIN_BUZZER, LOW);
            break;
    }
}

//  DISPLAY

void taskDisplay() {
    renderDisplay();
}

void renderDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Row 0: State
    display.setCursor(0, 0);
    display.print(F("State: "));
    display.println(stateString(sysStatus.state));

    // Row 1: Temp & Smoke
    display.setCursor(0, 10);
    display.printf("T:%.1fC  Sm:%d", sensors.temperature, sensors.smokeRaw);

    // Row 2: Flame
    display.setCursor(0, 20);
    display.print(F("Flame: "));
    display.println(sensors.flameDetected ? F("YES !!") : F("No"));

    // Row 3: WiFi + RSSI
    display.setCursor(0, 30);
    if (sysStatus.wifiConnected) {
        display.printf("WiFi:OK  %ddBm", sysStatus.rssi);
    } else {
        display.print(F("WiFi: DISCONNECTED"));
    }

    // Row 4: Firebase
    display.setCursor(0, 40);
    display.print(F("Firebase: "));
    display.println(sysStatus.firebaseOk ? F("OK") : F("FAIL"));

    // Row 5: Suppression
    display.setCursor(0, 52);
    display.setTextSize(1);
    if (sysStatus.suppressionOn) {
        display.print(F(">>> SUPPRESSION ON <<<"));
    } else {
        display.print(F("Suppression: Off"));
    }

    display.display();
}

//  FIREBASE

void taskFirebase() {
    if (!sysStatus.wifiConnected) return;

    sysStatus.firebaseOk = Firebase.ready();
    if (!sysStatus.firebaseOk) return;

    uploadFirebase();
}

void uploadFirebase() {
    updateTimestamp();

    FirebaseJson json;
    json.set("temperature",  sensors.temperature);
    json.set("smoke",        sensors.smokeRaw);
    json.set("flame",        sensors.flameDetected);
    json.set("state",        stateString(sysStatus.state));
    json.set("suppression",  sysStatus.suppressionOn);
    json.set("timestamp",    sysStatus.timestamp);
    json.set("rssi",         sysStatus.rssi);

    if (!Firebase.updateNode(fbData, "/fire_system/status", json)) {
        Serial.printf("[FB] Error: %s\n", fbData.errorReason().c_str());
        sysStatus.firebaseOk = false;
    }
}

//  TELEGRAM

void sendTelegramAlert(const char* msg) {
    if (!sysStatus.wifiConnected) return;
    secureClient.setInsecure();   // skip cert verification
    bot.sendMessage(CHAT_ID, msg, "");
    Serial.printf("[TG] Sent: %s\n", msg);
}

void handleTelegramAlerts() {
    const SystemState s = sysStatus.state;

    // State-change alerts
    if (s != alerts.lastAlerted) {
        char buf[128];
        switch (s) {
            case SystemState::WARNING:
                snprintf(buf, sizeof(buf),
                    "⚠️ FIRE WARNING\nT:%.1f°C Smoke:%d Flame:%s",
                    sensors.temperature, sensors.smokeRaw,
                    sensors.flameDetected ? "YES" : "No");
                sendTelegramAlert(buf);
                break;
            case SystemState::DANGER:
                snprintf(buf, sizeof(buf),
                    "🔴 FIRE DANGER!\nT:%.1f°C Smoke:%d Flame:%s",
                    sensors.temperature, sensors.smokeRaw,
                    sensors.flameDetected ? "YES" : "No");
                sendTelegramAlert(buf);
                break;
            case SystemState::EMERGENCY:
                snprintf(buf, sizeof(buf),
                    "🚨 EMERGENCY! Fire confirmed!\nT:%.1f°C Smoke:%d Flame:%s",
                    sensors.temperature, sensors.smokeRaw,
                    sensors.flameDetected ? "YES" : "No");
                sendTelegramAlert(buf);
                break;
            case SystemState::SAFE:
                if (alerts.lastAlerted != SystemState::SAFE && !alerts.safeAlert) {
                    sendTelegramAlert("✅ System returned to SAFE state.");
                    alerts.safeAlert = true;
                }
                break;
            case SystemState::SENSOR_FAULT:
                if (!alerts.faultAlert) {
                    sendTelegramAlert("❌ SENSOR FAULT detected! Check hardware.");
                    alerts.faultAlert = true;
                }
                break;
            default:
                break;
        }

        if (s != SystemState::SAFE) alerts.safeAlert = false;
        if (s != SystemState::SENSOR_FAULT) alerts.faultAlert = false;
        alerts.lastAlerted = s;
    }

    // Suppression alert (one-shot)
    if (sysStatus.suppressionOn && !alerts.suppressionAlert) {
        sendTelegramAlert("💧 SUPPRESSION ACTIVATED — Relay ON");
        alerts.suppressionAlert = true;
    }
    if (!sysStatus.suppressionOn && alerts.suppressionAlert) {
        sendTelegramAlert("💧 Suppression deactivated — Relay OFF");
        alerts.suppressionAlert = false;
    }
}

//  PREFERENCES LOGGING

void logPrefs(SystemState s, bool suppression) {
    prefs.begin("fireGuard", false);
    prefs.putUChar("lastState",  (uint8_t)s);
    prefs.putBool("lastSupp",    suppression);

    time_t now;
    time(&now);
    if (s == SystemState::EMERGENCY) {
        prefs.putULong("lastEmergency", (unsigned long)now);
    }
    if (suppression) {
        prefs.putULong("lastSuppEvent", (unsigned long)now);
    }
    prefs.end();
}

void loadPrefs() {
    prefs.begin("fireGuard", true);
    uint8_t lastState    = prefs.getUChar("lastState",  0);
    bool    lastSupp     = prefs.getBool("lastSupp",    false);
    unsigned long lastEM = prefs.getULong("lastEmergency", 0);
    unsigned long lastSE = prefs.getULong("lastSuppEvent", 0);
    prefs.end();

    Serial.printf("[PREFS] LastState=%u Suppression=%d LastEmerg=%lu LastSupp=%lu\n",
                  lastState, lastSupp, lastEM, lastSE);
}

//  UTILITIES

const char* stateString(SystemState s) {
    switch (s) {
        case SystemState::SAFE:         return "SAFE";
        case SystemState::WARNING:      return "WARNING";
        case SystemState::DANGER:       return "DANGER";
        case SystemState::EMERGENCY:    return "EMERGENCY";
        case SystemState::SENSOR_FAULT: return "SENSOR_FAULT";
        case SystemState::WIFI_FAULT:   return "WIFI_FAULT";
        default:                        return "UNKNOWN";
    }
}

// Firebase library token callback (required by library)
void tokenStatusCallback(TokenInfo info) {
    if (info.status == token_status_error) {
        Serial.printf("[FB] Token error: %s\n", info.error.message.c_str());
        sysStatus.firebaseOk = false;
    }
}
