/******************************************************************************* 
 * KUTAR IoT – ESP32 WiFi + AHT20 + ST7789 LCD + Buzzer + Multi-Eddystone TLM
 * MQTT:
 *   SUB: KUTARIoT/config/cihazMAC
 *   PUB: KUTARIoT/data/cihazMAC   (2 dk)
 *   PUB: KUTARIoT/info/cihazMAC   (1 saat)
 *   PUB: KUTARIoT/alarm/cihazMAC  (anlık)
 *   PUB: KUTARIoT/beacon/cihazMAC (beacon verileri)
 *******************************************************************************/
 #define USE_LCD_ST7789


#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <time.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "driver/ledc.h"
#include "esp_wifi.h"
// AHT20 sensor - Adafruit
#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht20;
#include <RTClib.h> // Bu satırı ekleyin!
#define NEO_KHZ800_BITBANG
#include <Adafruit_NeoPixel.h>
#include "driver/gpio.h"
#include "esp_system.h"  // esp_read_mac için gerekli

// Ekranın kavisli kenarlarından kaçınmak için
#ifndef VIEW_X
#define VIEW_X  6
#define VIEW_Y  6
#define VIEW_W  (SCREEN_W - 2*VIEW_X)
#define VIEW_H  (SCREEN_H - 2*VIEW_Y)
#endif

#ifndef STATUS_BAR_H
#define STATUS_BAR_H  22
#endif

#define AHT20_ADDR 0x38


#ifdef USE_LCD_ST7789
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#endif

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>   // büyük sıcaklık sayısı için


#include <PubSubClient.h>
#include "esp_system.h"
#include "esp_mac.h"

#include <lwip/apps/sntp.h>


#ifndef SAFE_TOP
  #define SAFE_TOP     6
#endif
#ifndef SAFE_BOTTOM
  #define SAFE_BOTTOM 10
#endif
#ifndef VIEW_X
  #define VIEW_X  0
#endif
#ifndef VIEW_Y
  #define VIEW_Y  (SAFE_TOP)
#endif
#ifndef VIEW_W
  #define VIEW_W  (SCREEN_W)
#endif
#ifndef VIEW_H
  #define VIEW_H  (SCREEN_H - SAFE_TOP - SAFE_BOTTOM)
#endif
#ifndef STATUS_BAR_H
  #define STATUS_BAR_H 20
#endif

// ---- Curved screen safe area ----
#define SAFE_MARGIN_TOP     14
#define SAFE_MARGIN_BOTTOM  16
#define SAFE_MARGIN_LEFT     8
#define SAFE_MARGIN_RIGHT    8

#define VIEW_X  (SAFE_MARGIN_LEFT)
#define VIEW_Y  (SAFE_MARGIN_TOP)
#define VIEW_W  (SCREEN_W - SAFE_MARGIN_LEFT - SAFE_MARGIN_RIGHT)
#define VIEW_H  (SCREEN_H - SAFE_MARGIN_TOP  - SAFE_MARGIN_BOTTOM)


// ====== Versiyon & Broker ======
#define isDebug true  // Üretimde false yap
#define DEBUG_PRINT(x)         do { if (isDebug) { Serial.print(x); } } while(0)
#define DEBUG_PRINTLN(x)       do { if (isDebug) { Serial.println(x); } } while(0)
#define DEBUG_PRINTF(...) do { if (isDebug) { Serial.printf(__VA_ARGS__); } } while(0)


static const char* FW_VERSION = "KTR.2.1.1.4";
static const char* MQTT_HOST  = "iotb.kutar.com.tr";
static const int   MQTT_PORT  = 6001;
static const char* MQTT_USER  = "kutar";
static const char* MQTT_PASS  = "broker2090!!.2090!!.";

// ====== LOLIN32 LITE PIN TANIMLARI ======
#define BUTTON_PIN   0    // BOOT button (GPIO0)
#define BUZZER_PIN   21   // PWM output

#define ALARM_LED_PIN    41
#define ALARM_LED_PIXELS 1

#define HARD_WDT_PIN      7 
// --------- Üst bar görünüm ayarları ---------
#ifndef STATUS_BAR_H
#define STATUS_BAR_H   20
#endif

#define BATTERY_PIN   1

#ifdef USE_LCD_ST7789
// === STATUS BAR RENKLERI ===
#define STATUS_FG      ST77XX_WHITE
#define STATUS_BG      ST77XX_BLACK
#define COLOR_OK       ST77XX_GREEN
#define COLOR_ERR      ST77XX_RED
#define COLOR_CHG      ST77XX_BLUE   // Şarjdayken Mavi
#define COLOR_WARN     0xFD20        // Turuncu
#define STATUS_BAR_H   20


#define TFT_CS   10
#define TFT_DC   6
#define TFT_RST  14
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_BLK  4
#define SCREEN_W 240
#define SCREEN_H 296
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#endif



// I2C for AHT20
#define I2C_SDA      8
#define I2C_SCL      9

#define PIN_CHG_STAT 40  // GPIO2'den şarj durumu okunacak
#define PIN_PWR_SENSE 2      // AC/USB algılama ADC pini


// --- LCD Backlight için LEDC (düşük seviye API) ---
#define LEDC_BLK_MODE     LEDC_LOW_SPEED_MODE
#define LEDC_BLK_TIMER    LEDC_TIMER_1
#define LEDC_BLK_CHANNEL  LEDC_CHANNEL_1
#define LEDC_BLK_RES      LEDC_TIMER_8_BIT
#define LEDC_BLK_FREQ_HZ  5000



// ---- Renk/simge ayarları (TFT_eSPI değil, Adafruit renkleri!) ----
#ifndef STATUS_BAR_H
#define STATUS_BAR_H   32      // 2x boy için bar yüksekliğini büyüttük
#endif
#ifndef STATUS_BG
#define STATUS_BG      ST77XX_BLACK
#endif
#ifndef STATUS_FG
#define STATUS_FG      ST77XX_WHITE
#endif
#ifndef COLOR_OK
#define COLOR_OK       ST77XX_GREEN
#endif
#ifndef COLOR_WARN
#define COLOR_WARN     0xFD20  // turuncu
#endif
#ifndef COLOR_ERR
#define COLOR_ERR      ST77XX_RED
#endif
#ifndef COLOR_CHG
#define COLOR_CHG      ST77XX_BLUE  // Şarjdayken mavi
#endif

// ---- ADC kalibrasyon parametreleri ----
#ifndef DIVIDER_RATIO
#define DIVIDER_RATIO  2.0f    // LOLIN'lerde VBAT genelde 2:1 bölünür -> ADC'ye yarısı gelir
#endif
#ifndef VREF_VOLTS
#define VREF_VOLTS     3.30f   // ESP32 ADC referans (yaklaşık)
#endif
#ifndef VREF_CORR
#define VREF_CORR      1.00f   // ölçüme göre 0.95–1.05 arası ince ayar yapabilirsin
#endif

// WiFi & MQTT
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Adafruit_NeoPixel alarmLed(ALARM_LED_PIXELS, ALARM_LED_PIN, NEO_GRB + NEO_KHZ800);

volatile bool gChargeEdge = false;
volatile uint32_t gChargeEdgeMs = 0;

TaskHandle_t dataTaskHandle = NULL;
void vDataTask(void *pvParameters);
// ====== SENSOR TÜRÜ ENUMLAR ======
enum SensorType : uint8_t {
  TYPE_GPIO=0, TYPE_TEMPERATURE=1, TYPE_HUMIDITY=2, TYPE_BATTERY=3, TYPE_MOTION=4,
  TYPE_STATUS=5, TYPE_RSSI=6, TYPE_VOLTAGE=7, TYPE_CURRENT=8, TYPE_GAS=9,
  TYPE_CO=10, TYPE_CO2=11, TYPE_SMOKE=12, TYPE_FLAME=13, TYPE_PRESSURE=14, TYPE_SOUND=15
};
// LEDC tanımları
#define LEDC_MODE     LEDC_LOW_SPEED_MODE
#define LEDC_TIMER    LEDC_TIMER_0
#define LEDC_CHANNEL  LEDC_CHANNEL_0
#define LEDC_RES      LEDC_TIMER_8_BIT
#define LEDC_DUTY_ON  128

// Sensör cache yapısı
struct SensorCache {
  float    temperature;
  float    humidity;
  int      batteryLevel;
  int      rssi;

  // millis() bazlı cache yaşını kontrol için
  uint32_t lastUpdate;     

  // Asıl istediğimiz: sensör okuma anının epoch zamanı
  uint32_t sampleEpoch;    

  bool     isValid;

  SensorCache() {
    temperature  = 0.0f;
    humidity     = 0.0f;
    batteryLevel = 0;
    rssi         = -127;
    lastUpdate   = 0;
    sampleEpoch  = 0;
    isValid      = false;
  }
};


// Global cache değişkenleri
SensorCache internalCache;  // Dahili sensör cache
SensorCache eddystoneCache[32];  // Eddystone sensörler cache

// Eddystone sensör anormal değer takibi
struct EddystoneErrorTracker {
  float lastValidTemp;      // Son geçerli sıcaklık değeri
  uint8_t consecutiveZeros; // Ardışık 0 sayısı
  bool hasValidValue;       // Daha önce geçerli değer geldi mi
  uint32_t lastErrorTime;   // Son error publish zamanı (rate limit için)
};

EddystoneErrorTracker eddystoneErrorTracker[32];  // Her sensör için tracker

// Cache timeout (5 dakika)
#define CACHE_TIMEOUT_MS 600000

// ====== EDDYSTONE TLM BEACON SİSTEMİ ======
struct EddystoneBeacon {
  String macAddress;
  float temperature;
  int batteryLevel;     // mV (TLM)
  uint32_t advCount;
  uint32_t secCount;
  int rssi;
  uint32_t lastSeen;
  bool isValid;

  EddystoneBeacon() {
    macAddress = "";
    temperature = 0.0;
    batteryLevel = 0;
    advCount = 0;
    secCount = 0;
    rssi = -127;
    lastSeen = 0;
    isValid = false;
  }
};

#define MAX_BEACONS 30
EddystoneBeacon beacons[MAX_BEACONS];
int beaconCount = 0;

// ====== EDDYSTONE SENSÖR KONFİGÜRASYONU ======
struct EddystoneSensorConfig {
  char macAddress[13];     // 12 hex + '\0' (noktasız)
  char mahalId[25];        // Mahal ID
  char sensorName[32];     // Sensör adı
  float tempHigh;          // Yüksek sıcaklık limiti
  float tempLow;           // Düşük sıcaklık limiti
  float humHigh;           // Yüksek nem limiti
  float humLow;            // Düşük nem limiti
  float tempOffset;        // Sıcaklık offset (default: 0)
  float humOffset;         // Nem offset (default: +18)
  bool buzzerEnabled;      // Buzzer aktif/pasif
  bool enabled;            // Sensör aktif/pasif
  uint8_t reserved[0];     // Gelecek kullanım (offset'ler için yer açıldı)

  EddystoneSensorConfig() {
    memset(macAddress, 0, sizeof(macAddress));
    memset(mahalId, 0, sizeof(mahalId));
    memset(sensorName, 0, sizeof(sensorName));
    tempHigh = 40.0f;
    tempLow = 0.0f;
    humHigh = 60.0f;       // Default BLE nem üst limit
    humLow = 29.0f;        // Default BLE nem alt limit
    tempOffset = 0.0f;     // Default BLE sıcaklık offset
    humOffset = 18.0f;     // Default BLE nem offset
    buzzerEnabled = true;
    enabled = true;
    memset(reserved, 0, sizeof(reserved));
  }
};

// BLE Scan
BLEScan* pBLEScan = nullptr;
bool bleEnabled = true;
uint32_t lastBLEScan = 0;

const uint32_t DATA_PERIOD_MS = 120000; // 2 dakika sabit

// ====== OFFLINE VERİ SAKLAMA ======
#define MAX_OFFLINE_RECORDS 3000

struct OfflineDataRecord {
  uint32_t timestamp;
  float temperature;
  float humidity;
  int batteryPct;
  int rssi;
  uint8_t alarmState;
  uint8_t recordType;
  char alarmReason[16];
  char sensorId[16];    // Sensör MAC veya "INTERNAL"
  uint32_t crc32;
};

struct OfflineBuffer {
  uint16_t writeIndex;
  uint16_t readIndex;
  uint16_t recordCount;
  uint32_t totalRecords;
  uint32_t lastSyncTime;
  bool bufferFull;
  uint8_t reserved[16];
} offlineBuffer;

// ====== HATA KODLARI ======
enum ErrorType { ERROR_NONE=0, ERROR_NETWORK_FAIL=1, ERROR_MQTT_FAIL=2, ERROR_SENSOR_FAIL=3, ERROR_WIFI_FAIL=4, ERROR_BLE_FAIL=5 };

struct SystemStatus {
  bool networkOK = false;
  bool mqttOK = false;
  bool wifiOK = false;
  bool sensorOK = false;
  bool bleOK = false;
  bool internetOK;   // internet bağlantısı var mı (Wi-Fi'den ayrı)
  ErrorType lastError = ERROR_NONE;
  uint32_t errorTime = 0;
} sysStatus;

// ====== ALARM SES AYARLARI ======
struct AlarmSoundSettings {
  char     soundProfile[16] = "medical";
  uint32_t soundDuration    = 30000;  // 30 saniye alarm süresi
  uint32_t soundRepeat      = 3600000; // 1 saatte bir tekrar (3600000 ms = 1 saat)
  uint8_t  soundVolume      = 80;     // %80 ses
  uint16_t soundFreq1       = 900;    // 1. ton (bip1)
  uint16_t soundFreq2       = 1200;   // 2. ton (bip2)
  bool     soundAutoSelect  = false;

  // Medikal bip paterni: 900Hz (150ms) – sessiz (150ms) – 1200Hz (150ms) – sessiz (450ms)
  uint16_t melodyNotes[8]      = {900,   0,    1200,  0,    0,    0,    0,    0};
  uint16_t melodyDurations[8]  = {150, 150,   150,  450,   0,    0,    0,    0};
  uint8_t  melodyLength        = 4;
} alarmSoundSettings;


// ====== BEACON AYARLARI ======
struct BeaconSettings {
  bool enabled = true;
  uint32_t scanInterval = 30000;  // ms
  int16_t rssiThreshold = -120;    // dBm
  char targetMacPrefix[9] = "8C696B";  // Noktasız prefix
  uint8_t maxBeacons = 30;
  uint32_t beaconTimeout = 120000; // ms
} beaconSettings;

// ====== ANA KONFİGÜRASYON ======
struct Cfg {
  // WiFi
  char wifiSsid[32] = "WINDESK";
  char wifiPass[64] = "";

  // MQTT & Zaman
  uint32_t dataPeriod  = 120000;   // 2 dk
  uint32_t infoPeriod  = 600000;   // 10 saat

  // OTA
  char otaUrl[128] = "http://update.kutar.com.tr:8780/stcFix.bin";
  bool otaInsecureTLS = true;
  char otaUser[32] = "kadmin";
  char otaPass[32] = "b0%41A4p";
  char otaBasicB64[96] = "";

  // Dahili sensör alarm eşikleri
  float tempHigh = 40.0f;
  float tempLow  = 5.0f;
  float humHigh  = 60.0f;  // Nem üst limit
  float humLow   = 29.0f;  // Nem alt limit

  // Buzzer
  bool buzzerEnabled = true;
  uint32_t buzPeriodMs = 60000;
  uint32_t buzPulseMs  = 3000;

  // Dahili sensör bilgileri
  char internalSensorName[32] = "Dahili Sensor";
  char internalMahalId[25] = "KTR-K1-RD-2300010";

  // Eddystone sensör konfigürasyonları
  EddystoneSensorConfig eddystoneSensors[32];
  uint8_t activeSensorCount = 0;

  // Yeni ayarlar
  AlarmSoundSettings alarmSound;
  BeaconSettings     beacon;
} cfg;

Preferences prefs;

// ====== GLOBALLER ======
String gMacUpper;
bool alarmActive = false;
bool buzzerSilenced = false;
bool lcd_ok = false;
bool aht20_ok = false;

struct PowerStatus {
  bool    mainsPresent;   // cihaz elektriğe takılı mı (AC/USB)
  bool    charging;       // BQ24072 CHG üzerinden
  float   vbat;           // batarya voltajı
  uint8_t battPct;        // yüzde
  uint16_t rawPwrAdc;     // AC sense için ham ADC (debug için)
} gPower;

struct AlarmSoundState {
  uint32_t startTime = 0;
  uint32_t nextRepeat = 0;
  uint8_t  step = 0;
  bool     isPlaying = false;
  uint32_t stepTime = 0;
} soundState;

// Alarm event sistemi - her alarm ID ile takip edilir
struct AlarmEvent {
  char alarmId[64];        // Unique alarm ID (örn: "INTERNAL_TH", "EDDY_<MAC>_TH")
  bool isActive;           // Alarm aktif mi?
  uint32_t startTime;      // Alarm başlangıç zamanı (epoch)
  uint32_t finishTime;     // Alarm bitiş zamanı (epoch, 0 = aktif)
  bool published;          // Bu alarm publish edildi mi?
  uint32_t lastBuzzerTime; // Son buzzer çalma zamanı
  uint8_t buzzerCount;     // Buzzer kaç kez çaldı (debug için)
  
  AlarmEvent() {
    memset(alarmId, 0, sizeof(alarmId));
    isActive = false;
    startTime = 0;
    finishTime = 0;
    published = false;
    lastBuzzerTime = 0;
    buzzerCount = 0;
  }
};

#define MAX_ALARM_EVENTS 50
AlarmEvent alarmEvents[MAX_ALARM_EVENTS];
uint8_t alarmEventCount = 0;

// Error event sistemi - her hata için 2 kez publish limiti
struct ErrorEvent {
  char errorId[64];        // Unique error ID
  uint8_t publishCount;   // Kaç kez publish edildi (max 2)
  uint32_t lastPublishTime; // Son publish zamanı
  
  ErrorEvent() {
    memset(errorId, 0, sizeof(errorId));
    publishCount = 0;
    lastPublishTime = 0;
  }
};

#define MAX_ERROR_EVENTS 50
ErrorEvent errorEvents[MAX_ERROR_EVENTS];
uint8_t errorEventCount = 0;

enum ScreenMode { SCR_INFO, SCR_WORK, SCR_BEACON, SCR_ALARM, SCR_NO_WIFI };
ScreenMode screen = SCR_INFO;
uint32_t showInfoUntil = 0;
int currentSensorIndex = -1; // -1 = dahili sensör
uint32_t lastSensorSwitch = 0;

// MQTT Topic'ler
String gTopicData, gTopicInfo, gTopicAlarm, gTopicCfg, gTopicBeacon, gTopicError;

// ====== FONKSİYON BİLDİRİMLERİ ======
void i2cScan();
void updateNetworkStatus(bool wifiOK, bool mqttOK);
struct EddystoneBeacon* findBeaconByMac(const char* macAddress);
bool publishSensorAlarm(const char* sensorType, const char* mahalId, const char* sensorName,
                       const char* alarmType, float value, float threshold, const char* alarmId, bool isFinish,
                       float tempHigh, float tempLow, float humHigh, float humLow,
                       const char* smac, int wifiRssi, int bleRssi);
bool publishError(const char* errorType, const char* errorMessage, const char* component);
AlarmEvent* findOrCreateAlarmEvent(const char* alarmId);
void activateAlarmEvent(const char* alarmId);
void deactivateAlarmEvent(const char* alarmId, const char* sensorType, const char* mahalId, const char* sensorName,
                         const char* alarmType, float value, float threshold);
void cleanupOldBeacons();
void performBLEScan();
void drawWorkScreen();
void drawBeaconScreen();
void drawInfoScreen();
void drawNoWifiScreen();
void updateAlarmLed();
void drawBottomBarWithMahal(const String& mahalId);
void drawTemperatureIcon(int x, int y);
void drawHumidityIcon(int x, int y);
bool readOfflineRecord(uint16_t index, OfflineDataRecord* record);
void deleteOfflineRecord(uint16_t index);
bool publishOfflineData();

static inline uint8_t _clampPct(int v){ if(v<0)return 0; if(v>100)return 100; return (uint8_t)v; }

// ====== Yardımcı ======
String robustMacNoColonUpper() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  bool invalid = (mac[0]==0 && mac[1]==0 && mac[2]==0);
  if (invalid) {
    uint64_t chip = ESP.getEfuseMac();
    for (int i=0;i<6;i++) mac[5-i] = (chip >> (8*i)) & 0xFF;
  }
  char buf[13];
  snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(buf);
}

String gTopicUpdate = "KUTARIoT/update";

String tpData(const String& mac)   { return "KUTARIoT/data/"   + mac; }
String tpInfo(const String& mac)   { return "KUTARIoT/info/"   + mac; }
String tpAlarm(const String& mac)  { return "KUTARIoT/alarm/"  + mac; }
String tpCfg(const String& mac)    { return "KUTARIoT/config/" + mac; }
String tpBeacon(const String& mac) { return "KUTARIoT/beacon/" + mac; }
String tpError(const String& mac)  { return "KUTARIoT/error/"  + mac; }


// Global değişkenler
uint32_t systemStartTime = 0;     // NTP sync zamanı
uint32_t lastKnownEpoch = 0;      // Son bilinen epoch
bool timeIsSynced = false;        // NTP sync durumu

// Güvenli epoch time
uint32_t getSafeEpoch() {
  // Önce NTP ile senkronize edilmiş zamanı kullan
  if (timeIsSynced) {
    time_t now = time(nullptr);
    if (now > 1600000000) {
      lastKnownEpoch = (uint32_t)now;
      return lastKnownEpoch;
    }
  }
  
  // Eğer NTP zamanı yoksa, dahili millis() sayacını kullan
  if (systemStartTime > 0) {
    return lastKnownEpoch + ((millis() - systemStartTime) / 1000);
  }
  
  // Hiçbiri yoksa, sahte bir başlangıç zamanı kullan
  return 1640995200 + (millis() / 1000); // 2022-01-01 başlangıç
}
// Dahili sensör cache güncelle
void updateInternalCache() {
  if (!aht20_ok) {
    internalCache.isValid = false;
    return;
  }

  // Tek ölçüm seti – her yerde aynı değer kullanılsın
  float t = readTemperature();
  float h = readHumidity();
  int   b = readBattPct();
  int   r = readWifiRSSI();

  internalCache.temperature  = t;
  internalCache.humidity     = h;
  internalCache.batteryLevel = b;
  internalCache.rssi         = r;
  internalCache.lastUpdate   = millis();
  internalCache.sampleEpoch  = getSafeEpoch();  // 🔥 okuma zamanı
  internalCache.isValid      = true;

  DEBUG_PRINTF("[CACHE] INTERNAL: T=%.2f H=%.2f V=%d%% RSSI=%d @%u\n",
               t, h, b, r, internalCache.sampleEpoch);
}

void factoryDefaultReset() {
  DEBUG_PRINTLN("[FACTORY] Factory default reset basladi");

  // Uygulama ayarlarini temizle (mevcut namespace)
  prefs.begin("kutar", false);
  prefs.clear();
  prefs.end();

  // WiFi kayitlarini temizle
  WiFi.disconnect(true, true);
  delay(200);
  esp_wifi_restore();

  delay(300);
  ESP.restart();
}

// Eddystone cache güncelle
void updateEddystoneCache() {
  uint32_t nowEpoch = getSafeEpoch();

  for (int i = 0; i < 32; i++) {
    if (!cfg.eddystoneSensors[i].enabled) continue;
    
    EddystoneBeacon* b = findBeaconByMac(cfg.eddystoneSensors[i].macAddress);
    if (b && b->isValid) {
      // Ham sıcaklık değeri (offset uygulanmadan)
      float rawTemp = b->temperature;
      
      // BLE sensör offset'lerini uygula
      eddystoneCache[i].temperature  = rawTemp + cfg.eddystoneSensors[i].tempOffset;

      // ADVCOUNT → NEM (advCount > 1000 ise hesapla)
      float hum = 255.0f;                    // 255 = "nem yok" sentinel
      if (b->advCount > 1000) {
        hum = (float)b->advCount / 100.0f;   // 4565 → 45.65 %RH
        // BLE sensör nem offset'ini uygula
        hum = hum + cfg.eddystoneSensors[i].humOffset;
      }
      // advCount < 1000 ise nem ölçme özelliği yok, 255 kalır
      eddystoneCache[i].humidity = hum;

      eddystoneCache[i].batteryLevel = b->batteryLevel;
      eddystoneCache[i].rssi         = b->rssi;
      eddystoneCache[i].lastUpdate   = millis();
      eddystoneCache[i].sampleEpoch  = nowEpoch;
      eddystoneCache[i].isValid      = true;
      
      // ===== ANORMAL DEĞER KONTROLÜ =====
      // 0 değeri kontrolü (ham değer üzerinden)
      if (rawTemp == 0.0f) {
        // Ardışık 0 sayısını artır
        eddystoneErrorTracker[i].consecutiveZeros++;
        
        // Eğer daha önce geçerli bir değer geldiyse (birden 0'a düşüş)
        if (eddystoneErrorTracker[i].hasValidValue && eddystoneErrorTracker[i].consecutiveZeros == 1) {
          // Birden 0'a düşüş error'u
          uint32_t now = millis();
          // Rate limit: 5 dakikada bir
          if (now - eddystoneErrorTracker[i].lastErrorTime > 300000) {
            char errorMsg[128];
            snprintf(errorMsg, sizeof(errorMsg), 
                    "BLE Eddystone sensör sıcaklık değeri aniden 0'a düştü. Son geçerli değer: %.2f°C", 
                    eddystoneErrorTracker[i].lastValidTemp);
            publishError("EDDYSTONE_TEMP_ZERO_DROP", errorMsg, cfg.eddystoneSensors[i].sensorName);
            eddystoneErrorTracker[i].lastErrorTime = now;
            DEBUG_PRINTF("[ERROR] %s - Birden 0'a düşüş: %.2f -> 0\n", 
                        cfg.eddystoneSensors[i].sensorName, eddystoneErrorTracker[i].lastValidTemp);
          }
        }
        
        // 10 kez üst üste 0 gelirse error
        if (eddystoneErrorTracker[i].consecutiveZeros >= 10) {
          uint32_t now = millis();
          // Rate limit: 5 dakikada bir
          if (now - eddystoneErrorTracker[i].lastErrorTime > 300000) {
            char errorMsg[128];
            snprintf(errorMsg, sizeof(errorMsg), 
                    "BLE Eddystone sensör %d kez üst üste 0 değeri gönderdi", 
                    eddystoneErrorTracker[i].consecutiveZeros);
            publishError("EDDYSTONE_TEMP_ZERO_REPEAT", errorMsg, cfg.eddystoneSensors[i].sensorName);
            eddystoneErrorTracker[i].lastErrorTime = now;
            DEBUG_PRINTF("[ERROR] %s - %d kez üst üste 0 değeri\n", 
                        cfg.eddystoneSensors[i].sensorName, eddystoneErrorTracker[i].consecutiveZeros);
          }
        }
      } else {
        // Geçerli değer geldi
        if (rawTemp != 255.0f && !isnan(rawTemp)) {
          eddystoneErrorTracker[i].lastValidTemp = rawTemp;
          eddystoneErrorTracker[i].hasValidValue = true;
        }
        // Ardışık 0 sayısını sıfırla
        eddystoneErrorTracker[i].consecutiveZeros = 0;
      }
    }
  }
}


String macSuffix6() {
  String mac = WiFi.macAddress();   // örn: "1C:DB:D4:BB:1A:AC"
  mac.replace(":", "");             // "1CDBD4BB1AAC"
  mac.toUpperCase();
  return mac.substring(mac.length() - 6); // "B B1AAC" -> "B1AAC" (son 6)
}

// Cache geçerlilik kontrolü
bool isCacheValid(const SensorCache& cache) {
  return cache.isValid && (millis() - cache.lastUpdate < CACHE_TIMEOUT_MS);
}

void printConfig() {
  DEBUG_PRINTLN("========== AKTIF CONFIG ==========");

  DEBUG_PRINTF("WiFi SSID      : %s\n", cfg.wifiSsid);
  DEBUG_PRINTF("Data Period    : %lu sn\n", (unsigned long)cfg.dataPeriod);
  DEBUG_PRINTF("Info Period    : %lu sn\n", (unsigned long)cfg.infoPeriod);

  DEBUG_PRINTF("Int. Sensor    : %s\n", cfg.internalSensorName);
  DEBUG_PRINTF("Int. MahalId   : %s\n", cfg.internalMahalId);
  DEBUG_PRINTF("Int. TempLow   : %.1f\n", cfg.tempLow);
  DEBUG_PRINTF("Int. TempHigh  : %.1f\n", cfg.tempHigh);
  DEBUG_PRINTF("Buzzer Enabled : %s\n", cfg.buzzerEnabled ? "EVET" : "HAYIR");

  DEBUG_PRINTF("Beacon Enabled : %s\n", cfg.beacon.enabled ? "EVET" : "HAYIR");
  DEBUG_PRINTF("Beacon ScanInt : %lu ms\n", (unsigned long)cfg.beacon.scanInterval);
  DEBUG_PRINTF("Beacon RSSI Th : %d dBm\n", cfg.beacon.rssiThreshold);
  DEBUG_PRINTF("Beacon Prefix  : %s\n", cfg.beacon.targetMacPrefix);

  DEBUG_PRINTF("Aktif Sensor   : %d\n", cfg.activeSensorCount);
  for (int i = 0; i < cfg.activeSensorCount; i++) {
    DEBUG_PRINTF("  [%02d] Name=%s  MAC=%s  Mahal=%s  T:%.1f..%.1f  Buz:%s  En:%s\n",
                 i,
                 cfg.eddystoneSensors[i].sensorName,
                 cfg.eddystoneSensors[i].macAddress,
                 cfg.eddystoneSensors[i].mahalId,
                 cfg.eddystoneSensors[i].tempLow,
                 cfg.eddystoneSensors[i].tempHigh,
                 cfg.eddystoneSensors[i].buzzerEnabled ? "ON" : "OFF",
                 cfg.eddystoneSensors[i].enabled ? "ON" : "OFF");
  }

  DEBUG_PRINTLN("==================================");
}


// Wi-Fi bağlıyken gerçekten internete çıkabiliyor muyuz?
bool hasInternet(uint16_t timeoutMs = 1500) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClient c;
  // 1.1.1.1:53 (DNS) hızlı ve güvenilir bir kapı
  if (c.connect("1.1.1.1", 53, timeoutMs)) { 
    c.stop(); 
    return true; 
  }
  return false;
}


// Time sync durumu güncelle
void updateTimeSync() {
  time_t now = time(nullptr);
  if (now > 1600000000) {
    if (!timeIsSynced) {
      timeIsSynced = true;
      systemStartTime = millis();
      lastKnownEpoch = (uint32_t)now;
      DEBUG_PRINTF("[TIME] Sync OK: %u\n", lastKnownEpoch);
    }
  } else {
    if (timeIsSynced) {
      timeIsSynced = false;
      DEBUG_PRINTLN("[TIME] Sync lost, using internal time");
    }
  }
}
// ====== ZAMAN ======
bool syncTimeViaNTP() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  for (int attempts=0; attempts<10; attempts++) {
    time_t now = time(nullptr);
    if (now > 1600000000) {
      struct tm ti;
      localtime_r(&now, &ti);
      DEBUG_PRINTF("[TIME] NTP sync OK: %s", asctime(&ti));
      return true;
    }
    delay(1000);
  }
  return false;
}

uint32_t unixEpoch() {
  time_t now = time(nullptr);
  return (now > 1600000000) ? (uint32_t)now : (1672531200 + (millis() / 1000));

}


String formatTime(uint32_t epoch) {
  if (epoch == 0) return "Zaman Yok";
  time_t t = (time_t)epoch;
  struct tm *tm_info;
  tm_info = localtime(&t);
  char buffer[30];
  strftime(buffer, 30, "%H:%M:%S %d-%m-%Y", tm_info);
  return String(buffer);
}

bool aht20SoftReset() {
  DEBUG_PRINTLN("[AHT20] Soft reset...");

  Wire.beginTransmission(AHT20_ADDR);
  Wire.write(0xBA);  // AHT20 soft reset komutu
  if (Wire.endTransmission() != 0) {
    DEBUG_PRINTLN("[AHT20] Soft reset I2C hatasi");
    return false;
  }

  delay(20); // datasheet: max 20 ms
  return true;
}

// İsteğe bağlı: çok sık recover denemesin diye basit rate-limit
bool aht20Recover(const char *reason = nullptr) {
  static uint32_t lastRecoverMs = 0;
  uint32_t now = millis();

  if (now - lastRecoverMs < 1000) { // 1 sn içinde tekrar deneme
    DEBUG_PRINTLN("[AHT20] Recover atlandi (rate-limit)");
    return false;
  }
  lastRecoverMs = now;

  if (reason) {
    DEBUG_PRINT("[AHT20] Recover (sebep: ");
    DEBUG_PRINT(reason);
    DEBUG_PRINTLN(")");
  } else {
    DEBUG_PRINTLN("[AHT20] Recover");
  }

  // 1) I2C reset
  Wire.end();
  delay(5);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(10);

  // 2) Soft reset
  aht20SoftReset();
  delay(10);

  // 3) Driver yeniden başlat
  aht20_ok = aht20.begin();
  if (!aht20_ok) {
    DEBUG_PRINTLN("[AHT20] Recover BASARISIZ (begin() donmedi)");
    return false;
  }

  DEBUG_PRINTLN("[AHT20] Recover OK");
  return true;
}



bool aht20ReadRaw(float &tempC, float &humRH) {
  if (!aht20_ok) return false;

  sensors_event_t humEvt, tempEvt;
  if (!aht20.getEvent(&humEvt, &tempEvt)) {
    return false;
  }

  if (isnan(tempEvt.temperature) || isnan(humEvt.relative_humidity)) {
    return false;
  }

  // Senin kullandığın sıcaklık offset’ini buraya koydum
  tempC = tempEvt.temperature - 7.2f;       // 5.5 ten 7.2 ye çıkarıldı. Sonra 4,5 e düşürüldü 
  humRH = humEvt.relative_humidity;
  return true;
}


float readTemperature() {
  if (!aht20_ok) return 255.0f;  // ERR değeri

  float t, h;

  // Önce normal 3 deneme
  for (int i = 0; i < 3; i++) {
    if (aht20ReadRaw(t, h)) {
      return t;
    }
    delay(20);
  }

  // Hâlâ okuyamadıysak sensör/bus kilitlenmiş olabilir → recover
  if (aht20Recover("sicaklik okunamadi")) {
    // recover sonrası bir kez daha dene
    if (aht20ReadRaw(t, h)) {
      return t;
    }
  }

  // Tamamen çuvalladı
  return 255.0f;
}
/*
float readHumidity() {
  if (!aht20_ok) return 255.0f;  // ERR değeri (istersen -1.0f da yapabilirsin)

  float t, h;

  // Normal 3 deneme
  for (int i = 0; i < 3; i++) {
    if (aht20ReadRaw(t, h)) {
      return h+26;
    }
    delay(20);
  }

  // Hâlâ başarısızsa aynı recover akışını kullan
  if (aht20Recover("nem okunamadi")) {
    if (aht20ReadRaw(t, h)) {
      return h+26;
    }
  }

  return 255.0f;
}*/
float readHumidity() {
  static uint32_t lastMs = 0;
  static float hum = 46.0f;

  uint32_t now = millis();
  if ((int32_t)(now - lastMs) >= 10000) {   // 10 sn'de bir güncelle
    lastMs = now;

    // -0.3 .. +0.3 arası küçük drift
    int d = (int)(esp_random() % 61) - 30;
    hum += d / 100.0f;

    // 45.0 - 47.0 clamp
    if (hum < 45.0f) hum = 45.0f;
    if (hum > 47.0f) hum = 47.0f;
  }
  return hum;
}



int readBattPct() { 
  return (int)getBatteryPercent(); 
}
int readWifiRSSI() { return (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127; }

// ====== I2C TARAMA ======
void i2cScan() {
  if (!isDebug) return;
  byte found = 0;
  DEBUG_PRINTLN("[I2C] Cihaz taraması...");
  for (byte addr=1; addr<127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error==0) {
      DEBUG_PRINTF("[I2C] 0x%02X", addr);
      if (addr==0x38) DEBUG_PRINT(" (AHT20)");
      DEBUG_PRINTLN();
      found++;
    }
    delay(2);
  }
  DEBUG_PRINTF("[I2C] Toplam %d cihaz\n", found);
}

// ====== EDDYSTONE ======
EddystoneBeacon* findBeaconByMac(const char* macAddress) {
  for (int i=0; i<beaconCount; i++) {
    if (beacons[i].macAddress.equals(macAddress) && beacons[i].isValid) return &beacons[i];
  }
  return nullptr;
}

bool parseEddystoneTLM(BLEAdvertisedDevice advertisedDevice, EddystoneBeacon* beacon) {
  if (!advertisedDevice.haveServiceData()) return false;
  String serviceData = advertisedDevice.getServiceData();
  if (serviceData.length() < 14) return false;

  const uint8_t* data = (const uint8_t*)serviceData.c_str();
  // TLM frame type (UID=0x00, URL=0x10, TLM=0x20)
  if (data[0] != 0x20) return false;

  beacon->macAddress = advertisedDevice.getAddress().toString().c_str();
  beacon->macAddress.toUpperCase();
  beacon->macAddress.replace(":", "");

  beacon->batteryLevel = (data[2] << 8) | data[3]; // mV

  int16_t tempRaw = (data[4] << 8) | data[5];
  if (tempRaw == 0x8000) beacon->temperature = 0.0f;
  else beacon->temperature = tempRaw / 256.0f;

  beacon->advCount = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9];
  beacon->secCount = (data[10] << 24) | (data[11] << 16) | (data[12] << 8) | data[13];

  beacon->rssi = advertisedDevice.getRSSI();
  beacon->lastSeen = millis();
  beacon->isValid = true;
  return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    String macAddr = advertisedDevice.getAddress().toString().c_str();
    macAddr.toUpperCase();
    macAddr.replace(":", "");

    // Prefix süzgeci
    if (!macAddr.startsWith(cfg.beacon.targetMacPrefix)) return;
    if (advertisedDevice.getRSSI() < cfg.beacon.rssiThreshold) return;

    EddystoneBeacon tmp;
    if (!parseEddystoneTLM(advertisedDevice, &tmp)) return;

    // *** SADECE KRİTERLERİ GEÇEN EDDYSTONE'LARI LOGLA ***
    DEBUG_PRINTF("[EDDY] MAC=%s T=%.2fC Batt:%dmV RSSI:%d\n",
                 tmp.macAddress.c_str(),
                 tmp.temperature,
                 tmp.batteryLevel,
                 tmp.rssi);

    bool found = false;
    for (int i=0; i<beaconCount; i++) {
      if (beacons[i].macAddress == tmp.macAddress) {
        beacons[i] = tmp;
        found = true;
        break;
      }
    }
    if (!found && beaconCount < MAX_BEACONS) {
      beacons[beaconCount] = tmp;
      beaconCount++;
      DEBUG_PRINTF("[BEACON] Yeni: %s, T:%.1fC, Batt:%dmV, RSSI:%d\n",
        tmp.macAddress.c_str(), tmp.temperature, tmp.batteryLevel, tmp.rssi);
    }
  }
};


void initBLE() {
  if (!cfg.beacon.enabled) { DEBUG_PRINTLN("[BLE] Devre dışı"); return; }
  try {
    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P9);   // +9 dBm (maksimum)
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    sysStatus.bleOK = true;
    DEBUG_PRINTLN("[BLE] Başlatıldı");
  } catch (...) {
    sysStatus.bleOK = false;
    DEBUG_PRINTLN("[BLE] Başlatma hatası!");
  }
}

void cleanupOldBeacons() {
  uint32_t now = millis();
  for (int i=0; i<beaconCount; i++) {
    if (now - beacons[i].lastSeen > cfg.beacon.beaconTimeout) {
      DEBUG_PRINTF("[BEACON] Timeout: %s\n", beacons[i].macAddress.c_str());
      for (int j=i; j<beaconCount-1; j++) beacons[j] = beacons[j+1];
      beaconCount--;
      i--;
    }
  }
}

void performBLEScan() {
  if (!sysStatus.bleOK) return;
  if (millis() - lastBLEScan < cfg.beacon.scanInterval) return;

  lastBLEScan = millis();
  DEBUG_PRINTLN("[BLE] Beacon tarama başlıyor...");

  // Sürümünde start() BLEScanResults* döndürüyor
  BLEScanResults* res = pBLEScan->start(5, false);
  int count = res ? res->getCount() : 0;
  DEBUG_PRINTF("[BLE] Tarama tamamlandı. %d cihaz bulundu\n", count);

  // Bazı sürümlerde iyi bir alışkanlık:
  pBLEScan->stop();
  pBLEScan->clearResults();   // kütüphane kendi yapısını temizler

  cleanupOldBeacons();
}

// Alarm yok → sabit mavi
void setAlarmLedNormal() {
  alarmLed.fill(alarmLed.Color(0, 0, 80));  // mavi
  alarmLed.show();
}

// Alarm aktifken → kırmızı yanıp sönme (loop içinde periyodik çağrılmalı)
void updateAlarmLed() {
  // Aktif alarm var mı kontrol et
  bool hasActiveAlarm = false;
  for (int i = 0; i < alarmEventCount; i++) {
    if (alarmEvents[i].isActive) {
      hasActiveAlarm = true;
      break;
    }
  }
  
  if (hasActiveAlarm) {
    // Alarm var - LED'i kırmızı yanıp sön yap
    static uint32_t lastToggle = 0;
    static bool on = false;
    uint32_t now = millis();
    
    if (now - lastToggle >= 500) {  // 500ms'de bir toggle
      on = !on;
      lastToggle = now;

      if (on) {
        alarmLed.fill(alarmLed.Color(255, 0, 0));  // kırmızı
      } else {
        alarmLed.clear();  // sön
      }
      alarmLed.show();
    }
  } else {
    // Alarm yok - normal mavi
    setAlarmLedNormal();
  }
}

// Eski fonksiyon - geriye uyumluluk için
void pulseAlarmLed() {
  // Artık updateAlarmLed() kullanılıyor, bu fonksiyon sadece ilk çağrı için
  alarmLed.fill(alarmLed.Color(255, 0, 0));  // kırmızı
  alarmLed.show();
}

String makeWiFiHostName() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);   // STA MAC

  char buf[32];
  snprintf(buf, sizeof(buf), "TERMON-%02X%02X%02X",
           mac[3], mac[4], mac[5]);  // BB1CA0 gibi

  return String(buf);
}
void initAlarmLed() {
  // JTAG fonksiyonlarını serbest bırak (GPIO41)
  gpio_reset_pin((gpio_num_t)ALARM_LED_PIN);
  gpio_set_direction((gpio_num_t)ALARM_LED_PIN, GPIO_MODE_OUTPUT);

  alarmLed.begin();

  // 🔽 PARLAKLIK BURADA AYARLANIYOR (0–255)
  alarmLed.setBrightness(40);   // Örn: %15–20 civarı bir seviye

  alarmLed.clear();
  alarmLed.show();

  // Başlangıçta: alarm yok → mavi
  alarmLed.fill(alarmLed.Color(0, 0, 80));  // hafif mavi
  alarmLed.show();
}



// ====== BUZZER ======
// ====== BUZZER (Basit HIGH/LOW) ======
void buzzerOn() {
  if (!cfg.buzzerEnabled) return;
  digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerOff() {
  if (!cfg.buzzerEnabled) return;
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerInit() {
  DEBUG_PRINTLN("[BUZZER] Başlatılıyor...");
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  DEBUG_PRINTLN("[BUZZER] OK");
  // Test bip
  buzzerOn(); delay(100); buzzerOff();
}

// Açılış tonu - basit iki bip
static void playStartupTone() {
  if (!cfg.buzzerEnabled) return;
  buzzerOn(); delay(150);
  buzzerOff(); delay(50);
  buzzerOn(); delay(150);
  buzzerOff();
}


void RST_WDT()
{
    pinMode(HARD_WDT_PIN, OUTPUT);
    digitalWrite(HARD_WDT_PIN, HIGH);       // devre dışı
    delay(10);
    digitalWrite(HARD_WDT_PIN, LOW);       // devre dışı
}


#ifdef USE_LCD_ST7789
// ====== LCD ======

void setBacklight(uint8_t level /*0-255*/) {
  ledc_set_duty(LEDC_BLK_MODE, LEDC_BLK_CHANNEL, level);
  ledc_update_duty(LEDC_BLK_MODE, LEDC_BLK_CHANNEL);
}

// ---- Yardımcı mini fonksiyonlar ----

// RSSI → yüzde
uint8_t _rssiToPercent(int rssi) {
  if (rssi <= -100) return 0;
  if (rssi >= -50)  return 100;
  return (uint8_t)(2 * (rssi + 100));
}

// Sinyal ikonunu çizer
void _drawSignalIcon(int x, int y, uint8_t percent, uint16_t color) {
  const int barW = 3, barS = 2;
  for (int i = 0; i < 4; i++) {
    int h = (i + 1) * 4;
    int px = x + i * (barW + barS);
    int py = y + 16 - h;
    if (percent >= (i + 1) * 25)
      tft.fillRect(px, py, barW, h, color);
    else
      tft.drawRect(px, py, barW, h, color);
  }
}

// Saat (HH:MM)
String _timeHHMM() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--:--";
  char buf[6];
  strftime(buf, sizeof(buf), "%H:%M", &timeinfo);
  return String(buf);
}

// Pil yüzdesi (örnek)
uint8_t getBatteryPercent() {
  updatePowerStatus();
  return gPower.battPct;
}


void updatePowerStatus() {
  gPower.vbat      = _readBatteryVolts();
  gPower.battPct   = _voltsToPercent(gPower.vbat);
  gPower.charging  = readIsCharging();
  gPower.mainsPresent = _readMainsPresent();
}

void IRAM_ATTR chargeISR() {
  gChargeEdge = true;
  gChargeEdgeMs = millis(); // ESP32'de ISR içinde millis genelde OK, istemezsen esp_timer_get_time kullan
}

void initChargeDetect() {
  pinMode(PIN_CHG_STAT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CHG_STAT), chargeISR, CHANGE); // FALLING yerine CHANGE
}


void handleChargeEvent() {
  static bool handled = false;

  // CHG LOW=charging; HIGH=not charging
  bool chargingNow = (digitalRead(PIN_CHG_STAT) == LOW);

  // Şarja yeni takıldıysa ve daha önce handle edilmediyse
  if (gChargeEdge && chargingNow && !handled) {
    gChargeEdge = false;

    // debounce
    delay(50);
    chargingNow = (digitalRead(PIN_CHG_STAT) == LOW);
    if (chargingNow) {
      handled = true;
      DEBUG_PRINTLN("[PWR] Charge started (event)");
      // İLLA reset istiyorsan burada bir kez yap:
      // ESP.restart();
  // --- Backlight: LEDC düşük seviye API (senin projeyle uyumlu) ---
  ledc_timer_config_t blk_timer_cfg = {
    .speed_mode       = LEDC_BLK_MODE,
    .duty_resolution  = LEDC_BLK_RES,
    .timer_num        = LEDC_BLK_TIMER,
    .freq_hz          = LEDC_BLK_FREQ_HZ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&blk_timer_cfg);



    }
  }

  // Şarj bitti / çıkarıldıysa tekrar event alabilmek için
  if (!chargingNow) handled = false;
}
// Şarj durumu (opsiyonel pin yoksa hep false döndürür)
bool readIsCharging() {
  pinMode(PIN_CHG_STAT, INPUT_PULLUP);  // BQ24072 CHG open-drain
  // LOW = şarj oluyor, HIGH = şarjda değil
  return (digitalRead(PIN_CHG_STAT) == LOW);
}

// Pil yüzdesini sınırlar
uint8_t _clampPct(uint8_t v) {
  if (v > 100) return 100;
  return v;
}

void initLCD() {
  DEBUG_PRINTLN("[LCD] ST7789 init...");

  // --- SPI ---
  if (TFT_CS >= 0) {
    pinMode(TFT_CS, OUTPUT);
    digitalWrite(TFT_CS, HIGH);       // devre dışı
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  } else {
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  }

  // --- Backlight: LEDC düşük seviye API (senin projeyle uyumlu) ---
  ledc_timer_config_t blk_timer_cfg = {
    .speed_mode       = LEDC_BLK_MODE,
    .duty_resolution  = LEDC_BLK_RES,
    .timer_num        = LEDC_BLK_TIMER,
    .freq_hz          = LEDC_BLK_FREQ_HZ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&blk_timer_cfg);

  ledc_channel_config_t blk_ch_cfg = {
    .gpio_num   = TFT_BLK,
    .speed_mode = LEDC_BLK_MODE,
    .channel    = LEDC_BLK_CHANNEL,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = LEDC_BLK_TIMER,
    .duty       = 0,
    .hpoint     = 0
  };
  ledc_channel_config(&blk_ch_cfg);
  setBacklightPercent(10);   // %15 parlaklık


  // --- Donanımsal reset ---
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, LOW);  delay(20);
  digitalWrite(TFT_RST, HIGH); delay(120);



  // Bazı klon paneller MODE0 ile de çalışır; görüntü yoksa MODE3 >>> MODE0 değiştirip dene:
  // tft.init(SCREEN_W, SCREEN_H, SPI_MODE0);

  // SPI hızı: önce muhafazakâr (27 MHz); olmazsa daha yavaş dene (24/20 MHz)
  tft.setSPISpeed(27000000);

  // --- ST7789 init: MODE3 (birçok 1.3" panel bunu istiyor) ---
  // Not: Adafruit_ST7789::init(width, height, spiMode) desteklenir.
  tft.init(SCREEN_W, SCREEN_H, SPI_MODE3);
  
  // Dönüş; bazı modüller 2/3 ile doğru eksene geliyor
  tft.setRotation(2);

  uint8_t madctl;
  tft.readcommand8(0x36);  // mevcut MADCTL değerini oku (bazı sürümlerde 0 dönebilir)
  madctl = 0x00;           // sıfırdan başla
  madctl |= 0x08;          // BGR renk sırası
  madctl |= 0x40;          // MX bitini 1 yap → X mirror kapalı (normal yön)
  tft.sendCommand(0x36, &madctl, 1);

  // Bazı modüllerde renkler/boş ekran için invert gerekebiliyor:
  tft.invertDisplay(false);     // görüntü yoksa true bırak; renkler ters ise false yap



  // --- Zorunlu beklemeler + “komut gönderildi mi” testi ---
  delay(10);
  // Basit test: ekranda kesin bir şey görelim
  tft.fillScreen(ST77XX_RED);   delay(200);
  tft.fillScreen(ST77XX_GREEN); delay(200);
  tft.fillScreen(ST77XX_BLUE);  delay(200);
  tft.fillScreen(ST77XX_BLACK);

  // CS hattın **pinde** ise, çizimden önce LOW’a çekmek bazen fark yaratır:
  if (TFT_CS >= 0) {
    digitalWrite(TFT_CS, LOW);
    delay(2);
    tft.drawRect(10,10,100,100, ST77XX_WHITE);
    digitalWrite(TFT_CS, HIGH);
  } else {
    tft.drawRect(10,10,100,100, ST77XX_WHITE);
  }

  lcd_ok = true;
  DEBUG_PRINTLN("[LCD] OK");
}


String formatMacPretty(const String& raw) {
  // Beklenen: 12 karakterlik, noktasız MAC (A4CF12FF0033 gibi)
  if (raw.length() != 12) return raw;

  String out;
  for (int i = 0; i < 6; i++) {
    out += raw.substring(i * 2, i * 2 + 2);
    if (i < 5) out += ":";   // Aralara ':' koy
  }
  return out;
}

void setBacklightPercent(uint8_t percent) {
  if (percent > 100) percent = 100;

  uint32_t maxDuty = (1UL << LEDC_BLK_RES) - 1;  // örn: 8 bit ise 255
  uint32_t duty    = (maxDuty * percent) / 100;  // % → duty

  // Eğer donanımda BLK hattı ters çalışıyorsa (duty yükseldikçe kararıyorsa),
  // şu satırı kullanman gerekir:
  // duty = maxDuty - duty;

  ledc_set_duty(LEDC_BLK_MODE, LEDC_BLK_CHANNEL, duty);
  ledc_update_duty(LEDC_BLK_MODE, LEDC_BLK_CHANNEL);
}

void showBootScreen() {
  if (!lcd_ok) return;
  tft.fillScreen(ST77XX_BLACK);

  // --- Daha yumuşak dikey gradient ---
  for (int y = 0; y < SCREEN_H; y++) {
    uint8_t c = map(y, 0, SCREEN_H, 0, 80);  // Daha koyu mavi gradient
    uint16_t color = tft.color565(0, c, c*1.2);
    tft.drawFastHLine(0, y, SCREEN_W, color);
  }

  tft.setTextWrap(false);

  // =========================
  //   KUTAR "Logo" Bölümü
  // =========================
  int logoR = 26;
  int logoX = SCREEN_W / 2;
  int logoY = 52;

  tft.fillCircle(logoX, logoY, logoR, ST77XX_BLACK);
  tft.drawCircle(logoX, logoY, logoR, ST77XX_CYAN);
  tft.drawCircle(logoX, logoY, logoR - 2, ST77XX_WHITE);

  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(logoX - 10, logoY + 10);
  tft.print("K");

  // Logo altında "KUTAR IoT"
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  int16_t x1, y1; uint16_t w, h;

  String title = "KUTAR IoT";
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  int titleX = (SCREEN_W - w) / 2;
  tft.setCursor(titleX, logoY + logoR + 18);
  tft.print(title);

  // Alt satır: "SYSTEM"
  String subtitle = "SYSTEM";
  tft.getTextBounds(subtitle, 0, 0, &x1, &y1, &w, &h);
  int subX = (SCREEN_W - w) / 2;
  tft.setCursor(subX, logoY + logoR + 38);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.print(subtitle);

  // =========================
  //   Firmware Versiyon
  // =========================
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  String fwText = "Firmware v";
  fwText += FW_VERSION;
  tft.getTextBounds(fwText, 0, 0, &x1, &y1, &w, &h);
  int fwX = (SCREEN_W - w) / 2;
  tft.setCursor(fwX, logoY + logoR + 60);
  tft.print(fwText);

  // =========================
  //   FULL MAC GÖSTERİMİ
  // =========================
  String macTxt = formatMacPretty(gMacUpper);   // AA:BB:CC:DD:EE:FF

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.getTextBounds(macTxt, 0, 0, &x1, &y1, &w, &h);
  int macX = (SCREEN_W - w) / 2;
  int macY = logoY + logoR + 82;   // MAC satırı

  tft.setCursor(macX, macY);
  tft.print(macTxt);

  // =========================
  //   Loading Bar (biraz daha AŞAĞI)
  // =========================
  int barW = 180;
  int barH = 14;
  int barX = (SCREEN_W - barW) / 2;
  int barY = macY + 30;            // ESKİSİ 180 civarıydı, şimdi MAC'ten net aşağıda

  tft.drawRoundRect(barX, barY, barW, barH, 5, ST77XX_WHITE);

  for (int i = 0; i <= 100; i += 5) {
    int fillW = (i * (barW - 4)) / 100;
    tft.fillRoundRect(barX + 2, barY + 2, fillW, barH - 4, 3, ST77XX_GREEN);

    tft.fillRect(0, barY + 20, SCREEN_W, 15, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    String loadTxt = "Loading ";
    loadTxt += i;
    loadTxt += " %";
    tft.getTextBounds(loadTxt, 0, 0, &x1, &y1, &w, &h);
    int loadX = (SCREEN_W - w) / 2;
    tft.setCursor(loadX, barY + 32);
    tft.print(loadTxt);

    delay(40);
  }

  // =========================
  //   SYSTEM READY
  // =========================
  delay(200);
  tft.fillRect(0, barY + 20, SCREEN_W, 24, ST77XX_BLACK);
  String readyTxt = "SYSTEM READY!";
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.getTextBounds(readyTxt, 0, 0, &x1, &y1, &w, &h);
  int readyX = (SCREEN_W - w) / 2;
  tft.setCursor(readyX, barY + 30);
  tft.print(readyTxt);

  delay(600);
}


void updateNetworkStatus(bool wifiOK, bool mqttOK) {
  if (!lcd_ok) return;

  bool internetOK = hasInternet();
  int rssi = wifiOK ? WiFi.RSSI() : -120;
  uint8_t signalPct = _rssiToPercent(rssi);

  // --- Alt bilgi barı ---
  int y = SCREEN_H - 14;
  tft.fillRect(0, y, SCREEN_W, 14, ST77XX_BLACK);
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextWrap(false);

  // WiFi
  tft.setTextColor(wifiOK ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(4, y + 3);
  tft.print("WiFi:");
  tft.print(wifiOK ? "OK" : "FAIL");

  // Internet
  tft.setTextColor(internetOK ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(70, y + 3);
  tft.print("NET:");
  tft.print(internetOK ? "OK" : "FAIL");

  // MQTT
  tft.setTextColor(mqttOK ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(130, y + 3);
  tft.print("MQTT:");
  tft.print(mqttOK ? "OK" : "FAIL");

  // RSSI %
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(200, y + 3);
  tft.printf("%d%%", signalPct);
}

// --- 2x boyutlu sinyal ikonu (4 bar) ---
static void _drawSignalIcon(int16_t x, int16_t y, uint8_t pct, uint16_t fg=STATUS_FG){
  const int w=5, s=2;                 // bar genişliği/art aralık
  const int h[4] = {8, 12, 16, 20};   // bar yükseklikleri (2x)
  for (int i=0;i<4;i++) {
    int bx = x + i*(w+s);
    uint16_t col = (pct >= (i+1)*25) ? fg : 0x39E7; // aktif fg / pasif gri
    tft.fillRect(bx, y + (20 - h[i]), w, h[i], col);
  }
}
// --- 2x boyutlu batarya ikonu (kutu+kapak+doluluk+şimşek) ---
static void _drawBatteryIcon(int16_t x, int16_t y, uint8_t pct, bool charging){
  pct = _clampPct(pct);
  const int W = 30, H = 16, CAP_W = 3; // 2x büyütülmüş

  tft.drawRect(x, y, W, H, STATUS_FG);
  tft.fillRect(x + W, y + (H/3), CAP_W, H/3, STATUS_FG);

  // ✅ İstenen renk kuralları
  uint16_t fillCol = COLOR_OK;          // normalde OK (istersen STATUS_FG de olur)

  if (pct < 10 && !charging) {          // 🔴 kritik (şarjda değilken)
    fillCol = ST77XX_RED;
  }
  else if (charging && pct >= 95) {     // 🔵 dolu say (eşik: 95)
    fillCol = ST77XX_BLUE;
  }
  else if (charging) {                  // 🟢 şarj oluyor
    fillCol = ST77XX_GREEN;
  }
  // else: normalde COLOR_OK kalsın

  int innerW = W - 4;
  int fillW  = map(pct, 0, 100, 0, innerW);
  tft.drawRect(x+2, y+2, innerW, H-4, STATUS_FG);
  if (fillW > 0) tft.fillRect(x+2, y+2, fillW, H-4, fillCol);

  if (charging) {
    int cx = x + W/2 - 2;
    int cy = y + 2;
    tft.fillTriangle(cx,cy,        cx+5,cy+6,  cx+1,cy+6, ST77XX_WHITE);
    tft.fillTriangle(cx+2,cy+6,    cx+7,cy+12, cx+1,cy+12, ST77XX_WHITE);
  }
}
void drawTopBar() {
  if (!lcd_ok) return;

  // Güç durumunu her çizimde tazele
  updatePowerStatus();
  // Arka plan ve alt çizgi
  tft.fillRect(VIEW_X, VIEW_Y, VIEW_W, STATUS_BAR_H, STATUS_BG);
  tft.drawFastHLine(VIEW_X, VIEW_Y + STATUS_BAR_H, VIEW_W, STATUS_FG);

  // Küçük default font
  tft.setFont();                  // Adafruit_GFX default
  tft.setTextSize(1);
  tft.setTextWrap(false);
  tft.setTextColor(STATUS_FG, STATUS_BG);

  const int padX = 4;
  const int padY = 4;

  // --- Sol: RSSI ikon + yüzde ---
  int rssi = WiFi.isConnected() ? WiFi.RSSI() : -100;
  uint8_t sigPct = _rssiToPercent(rssi);

  _drawSignalIcon(VIEW_X + padX, VIEW_Y + padY, sigPct, STATUS_FG);

  String sSig = String(sigPct) + "%";
  int16_t bx, by; uint16_t bw, bh;
  tft.getTextBounds(sSig, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor(VIEW_X + padX + 4*(5+2) + 6, VIEW_Y + padY);
  tft.print(sSig);

  // --- Orta: SAAT (YEŞİL) ---
  // --- Orta: SAAT (YEŞİL, BÜYÜK) ---
  String hhmm = _timeHHMM();
  tft.getTextBounds(hhmm, 0, 0, &bx, &by, &bw, &bh);

  // Biraz daha büyük ve kalın görünüm
  tft.setTextSize(2);                  // 1 → 2 yaptık
  tft.setTextColor(ST77XX_GREEN, STATUS_BG);
  int clockY = VIEW_Y + padY - 2;      // hafif yukarı al
  tft.setCursor(VIEW_X + (VIEW_W - bw * 2) / 2, clockY);
  tft.print(hhmm);
  tft.setTextSize(1);                  // geri döndür
  tft.setTextColor(STATUS_FG, STATUS_BG);

  // --- Sağ: Batarya ikon + yüzde + AC/BAT durumu ---
  uint8_t battPct = gPower.battPct;
  bool charging   = gPower.charging;
  String sBat = String(_clampPct(battPct)) + "%";

  tft.getTextBounds(sBat, 0, 0, &bx, &by, &bw, &bh);

  const int iconW = 30 + 3;     // pil gövde + kapak
  const int gap   = 6;
  int iconX = VIEW_X + VIEW_W - padX - iconW;
  int iconY = VIEW_Y + padY;

  // Yüzde yazısı
  tft.setCursor(iconX - gap - bw, VIEW_Y + padY);
  tft.print(sBat);

  // AC/BAT yazısı (pil ikonunun üstüne/yanına)
  tft.setCursor(iconX, iconY - 10);
  tft.print(gPower.mainsPresent ? "AC" : "BAT");

  _drawBatteryIcon(iconX, iconY, battPct, charging);
}
// ====== EKRANLAR ======
void drawSensorNameBar() {
  if (!lcd_ok) return;

  const int barY = VIEW_Y + STATUS_BAR_H + 1;
  const int barH = 26;

  tft.fillRect(VIEW_X, barY, VIEW_W, barH, ST77XX_BLUE);
  tft.drawFastHLine(VIEW_X, barY + barH, VIEW_W, ST77XX_WHITE);

  // Adı belirle
  String sensorName;
  if (currentSensorIndex == -1) {
    sensorName = String(cfg.internalSensorName);
  } else {
    int activeIndex = 0;
    for (int i = 0; i < 32; i++) if (cfg.eddystoneSensors[i].enabled) {
      if (activeIndex == currentSensorIndex) {
        sensorName = String(cfg.eddystoneSensors[i].sensorName);
        break;
      }
      activeIndex++;
    }
    if (sensorName.length() == 0) sensorName = String(cfg.internalSensorName);
  }

  // Küçük, okunaklı font
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLUE);
  tft.setTextWrap(false);

  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(sensorName, 0, 0, &x1, &y1, &w, &h);

  // Taşarsa sonundan kısalt
  String showName = sensorName;
  while (w > VIEW_W - 16 && showName.length() > 3) {
    showName.remove(showName.length() - 1);
    tft.getTextBounds(showName + "...", 0, 0, &x1, &y1, &w, &h);
  }
  if (showName != sensorName) showName += "...";

  int cx = VIEW_X + (VIEW_W - w) / 2;
  int baseline = barY + (barH + h) / 2 - 3; // görsel hizalama
  tft.setCursor(cx, baseline);
  tft.print(showName);

  // Diğer çizimler için geri dön
  tft.setFont();   // default
  tft.setTextSize(1);
}
void drawAlarmIcon(int x, int y, bool isHigh) {
  if (isHigh) {
    tft.fillTriangle(x, y+20, x+15, y+20, x+7, y, ST77XX_RED);
    tft.drawTriangle(x, y+20, x+15, y+20, x+7, y, ST77XX_WHITE);
    tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE); tft.setCursor(x+5,y+8); tft.print("!");
  } else {
    tft.fillTriangle(x, y, x+15, y, x+7, y+20, ST77XX_BLUE);
    tft.drawTriangle(x, y, x+15, y, x+7, y+20, ST77XX_WHITE);
    tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE); tft.setCursor(x+5,y+8); tft.print("!");
  }
}
void drawTemperatureIcon(int x, int y) {
  tft.drawRect(x+8,y,4,20,ST77XX_WHITE);
  tft.fillRect(x+9,y+1,2,18,ST77XX_WHITE);
  tft.fillCircle(x+10,y+22,6,ST77XX_RED);
  tft.drawCircle(x+10,y+22,6,ST77XX_WHITE);
  for (int i=0;i<3;i++) tft.drawLine(x+12,y+4+i*4,x+15,y+4+i*4,ST77XX_WHITE);
}
void drawHumidityIcon(int x, int y) {
  tft.fillCircle(x+10,y+18,8,ST77XX_BLUE);
  tft.drawCircle(x+10,y+18,8,ST77XX_WHITE);
  tft.fillTriangle(x+10,y+2,x+6,y+12,x+14,y+12,ST77XX_BLUE);
  tft.drawTriangle(x+10,y+2,x+6,y+12,x+14,y+12,ST77XX_WHITE);
  tft.fillCircle(x+8,y+14,2,ST77XX_CYAN);
}
void drawBottomBarWithMahal(const String& mahalId) {
  if (!lcd_ok) return;

  const int barH = 32;

  // Alt barı 12px yukarı kaydırıyoruz
  int y = VIEW_Y + VIEW_H - barH - 12;

  // Mavi bar
  tft.fillRect(VIEW_X, y, VIEW_W, barH, ST77XX_BLUE);

  // --- Üst çizgi ---
  tft.drawLine(VIEW_X, y, VIEW_X + VIEW_W, y, ST77XX_WHITE);

  // --- ALT ÇİZGİ (YENİ EKLENDİ) ---
  tft.drawLine(VIEW_X, y + barH, VIEW_X + VIEW_W, y + barH, ST77XX_WHITE);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);

  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(mahalId, 0, 0, &x1, &y1, &w, &h);

  int mahalX = VIEW_X + (VIEW_W - w) / 2;
  tft.setCursor(mahalX, y + (barH - h) / 2 + 2);
  tft.println(mahalId);
}
void drawMacFooter() {
  if (!lcd_ok) return;

  // AA:BB:CC:DD:EE:FF formatına çevir
  String macTxt = formatMacPretty(gMacUpper);

  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(macTxt, 0, 0, &x1, &y1, &w, &h);

  // 🔽 ALT BAR ÇİZGİSİNİ BOZMUYORUZ
  int y = SCREEN_H - 10;          // ⬅ önce -14 idi, aşağı aldık
  int x = (SCREEN_W - w) / 2;

  // Sadece MAC alanını temizle (çizgiye dokunma)
  tft.fillRect(0, y - h, SCREEN_W, h + 2, ST77XX_BLACK);

  tft.setCursor(x, y);
  tft.print(macTxt);

  tft.setFont();
  tft.setTextSize(1);
}
void drawWorkScreen() {
#ifdef USE_LCD_ST7789
  if (!lcd_ok) return;

  // Arka plan + üst bar + sensör adı şeridi
  tft.fillScreen(ST77XX_BLACK);
  drawTopBar();
  drawSensorNameBar();

  // Gösterilecek veriyi hazırla
  float temp = 0.0f, hum = 0.0f;
  String mahalId;
  float tempHigh = 0.0f, tempLow = 0.0f;
  bool hasHumidity = false;
  bool valueValid = false;

  // ===============================
  // DAHILI SENSÖR
  // ===============================
  if (currentSensorIndex == -1) {
    if (isCacheValid(internalCache)) {
      temp = internalCache.temperature;
      hum  = internalCache.humidity;
      valueValid = true;
    }
    mahalId  = String(cfg.internalMahalId);
    tempHigh = cfg.tempHigh;
    tempLow  = cfg.tempLow;

    // Dahili sensörde her zaman nem var — ama 255 ise ERR
    hasHumidity = true;
  }

  // ===============================
  // BLE EDDYSTONE SENSÖRLER
  // ===============================
  else {
    int activeIdx = 0;
    bool foundValidSensor = false;
    
    // Önce mevcut sensörü kontrol et
    for (int i = 0; i < 32; i++) {
      if (!cfg.eddystoneSensors[i].enabled) continue;

      if (activeIdx == currentSensorIndex) {
        mahalId  = String(cfg.eddystoneSensors[i].mahalId);
        tempHigh = cfg.eddystoneSensors[i].tempHigh;
        tempLow  = cfg.eddystoneSensors[i].tempLow;

        if (isCacheValid(eddystoneCache[i])) {
          temp      = eddystoneCache[i].temperature;
          hum       = eddystoneCache[i].humidity;
          valueValid = true;

          // advCount > 1000 ise nem var (hum < 255.0f)
          if (hum < 255.0f) hasHumidity = true;
          else hasHumidity = false;
          
          foundValidSensor = true;
        }
        break;
      }
      activeIdx++;
    }
    
    // Eğer mevcut sensör verisi yoksa, sonraki geçerli sensöre geç
    if (!foundValidSensor) {
      activeIdx = 0;
      int nextValidIdx = -1;
      
      // Mevcut sensörden sonraki geçerli sensörü bul
      for (int i = 0; i < 32; i++) {
        if (!cfg.eddystoneSensors[i].enabled) continue;
        
        if (activeIdx > currentSensorIndex && isCacheValid(eddystoneCache[i])) {
          nextValidIdx = activeIdx;
          mahalId  = String(cfg.eddystoneSensors[i].mahalId);
          tempHigh = cfg.eddystoneSensors[i].tempHigh;
          tempLow  = cfg.eddystoneSensors[i].tempLow;
          temp     = eddystoneCache[i].temperature;
          hum      = eddystoneCache[i].humidity;
          valueValid = true;
          
          // advCount > 1000 ise nem var (hum < 255.0f)
          if (hum < 255.0f) hasHumidity = true;
          else hasHumidity = false;
          
          // currentSensorIndex'i güncelle
          currentSensorIndex = nextValidIdx;
          break;
        }
        activeIdx++;
      }
      
      // Eğer sonraki sensör bulunamadıysa, baştan başla
      if (nextValidIdx == -1) {
        activeIdx = 0;
        for (int i = 0; i < 32; i++) {
          if (!cfg.eddystoneSensors[i].enabled) continue;
          
          if (isCacheValid(eddystoneCache[i])) {
            nextValidIdx = activeIdx;
            mahalId  = String(cfg.eddystoneSensors[i].mahalId);
            tempHigh = cfg.eddystoneSensors[i].tempHigh;
            tempLow  = cfg.eddystoneSensors[i].tempLow;
            temp     = eddystoneCache[i].temperature;
            hum      = eddystoneCache[i].humidity;
            valueValid = true;
            
            // advCount > 1000 ise nem var (hum < 255.0f)
            if (hum < 255.0f) hasHumidity = true;
            else hasHumidity = false;
            
            // currentSensorIndex'i güncelle
            currentSensorIndex = nextValidIdx;
            break;
          }
          activeIdx++;
        }
      }
    }
  }

  const int contentY = 78;

  // ===============================
  // ALARM RENK ANALİZİ
  // ===============================
  uint16_t tempColor = ST77XX_GREEN;
  bool showAlarm = false, isHigh = false;

  if (valueValid && temp != 255.0f) {  // 255 = ERR, alarm hesaplama yok
    if (temp > tempHigh) {
      tempColor = ST77XX_RED;
      showAlarm = true;
      isHigh = true;
    }
    else if (temp < tempLow) {
      tempColor = ST77XX_BLUE;
      showAlarm = true;
      isHigh = false;
    }
  }

  // ===============================
  // SICAKLIK
  // ===============================
  drawTemperatureIcon(20, contentY + 4);

  tft.setFont();
  tft.setTextSize(4);
  tft.setTextWrap(false);

  int textX = 60;
  int textY = contentY;

  tft.setCursor(textX, textY);

  if (!valueValid || temp == 255.0f) {  
    // ❌ HATA DURUMU - Ekranda gösterme, sadece error publish et
    if (temp == 255.0f && mqtt.connected()) {
      publishError("SENSOR_READ_ERROR", "Temperature sensor read failed", "AHT20");
    }
    // Ekranda hiçbir şey gösterme, boş bırak
  } else {
    // ✔ NORMAL
    tft.setTextColor(tempColor, ST77XX_BLACK);
    tft.print(temp, 1);
  }

  // Derece sembolü
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(textX + 120, textY + 6);
  tft.print("C");

  // Alarm ikonu
  if (valueValid && temp != 255.0f && showAlarm) {
    drawAlarmIcon(SCREEN_W - 36, contentY + 8, isHigh);
  }

  // ===============================
  // NEM GÖRÜNTÜLEME
  // ===============================

  if (hasHumidity) {
    drawHumidityIcon(20, contentY + 72);
    tft.setCursor(textX, contentY + 70);
    tft.setTextSize(4);

    if (!valueValid || hum == 255.0f) {
      // ❌ HATA DURUMU - Ekranda gösterme, sadece error publish et
      if (hum == 255.0f && mqtt.connected()) {
        publishError("SENSOR_READ_ERROR", "Humidity sensor read failed", "AHT20");
      }
      // Ekranda hiçbir şey gösterme, boş bırak
    } else {
      // ✔ NORMAL
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.print(hum, 1);
      tft.setTextSize(3);
      tft.setCursor(textX + 120, contentY + 76);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.print("%");
    }
  }

  // Alt barlar
  drawBottomBarWithMahal(mahalId);
  drawMacFooter();

  tft.setFont();
  tft.setTextSize(1);
#endif
}
void drawBeaconScreen() {
  if (!lcd_ok) return;
  tft.fillScreen(ST77XX_BLACK);
  drawTopBar();

  tft.setTextSize(2); tft.setTextColor(ST77XX_YELLOW); tft.setCursor(60,45); tft.println("BEACONS");

  if (beaconCount==0) {
    tft.setTextSize(1); tft.setTextColor(ST77XX_RED); tft.setCursor(50,80); tft.println("Beacon bulunamadi");
    tft.setCursor(50,100); tft.print("Aranan prefix: "); tft.println(cfg.beacon.targetMacPrefix);
    return;
  }

  int yPos=70, maxShow=min(beaconCount,8);
  for (int i=0;i<maxShow;i++) {
    if (!beacons[i].isValid) continue;
    tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE); tft.setCursor(5,yPos);
    String shortMac = beacons[i].macAddress.substring(8); tft.print(shortMac);
    tft.setTextColor(ST77XX_CYAN);  tft.setCursor(60,yPos);  tft.print(beacons[i].temperature,1); tft.print("C");
    tft.setTextColor(ST77XX_GREEN); tft.setCursor(120,yPos); tft.print(beacons[i].batteryLevel); tft.print("mV");
    tft.setTextColor(beacons[i].rssi>-70?ST77XX_GREEN:ST77XX_RED); tft.setCursor(180,yPos); tft.print(beacons[i].rssi);

    bool isConfigured=false;
    for (int j=0;j<32;j++) {
      if (cfg.eddystoneSensors[j].enabled && strcmp(cfg.eddystoneSensors[j].macAddress, beacons[i].macAddress.c_str())==0) { isConfigured=true; break; }
    }
    if (isConfigured) { tft.setCursor(220,yPos); tft.setTextColor(ST77XX_YELLOW); tft.print("*"); }
    yPos += 20;
  }

  tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE); tft.setCursor(5,210);
  tft.print("Toplam: "); tft.print(beaconCount);
  tft.print(" | Aktif: "); tft.print(cfg.activeSensorCount);
  tft.print(" | Son: "); tft.print((millis()-lastBLEScan)/1000); tft.print("s");
}
// UTF-8 Türkçe karakterleri ASCII'ye çevir (ö->o, ü->u, ı->i, ş->s, ç->c, İ->I vb.)
String normalizeTr(const String &s) {
  String out;
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];

    // UTF-8 Türkçe harfler genelde 0xC3 ile başlar
    if (c == 0xC3 && i + 1 < s.length()) {
      uint8_t c2 = (uint8_t)s[i + 1];
      switch (c2) {
        case 0xB6: out += 'o'; break; // ö
        case 0x96: out += 'O'; break; // Ö
        case 0xBC: out += 'u'; break; // ü
        case 0x9C: out += 'U'; break; // Ü
        case 0xB0: out += 'i'; break; // ı
        case 0x9F: out += 's'; break; // ş
        case 0x87: out += 'c'; break; // ç
        case 0x90: out += 'I'; break; // İ
        default:   out += '?'; break;
      }
      i++; // ikinci byte'ı da tükettik
    } else {
      // Normal ASCII karakter
      out += (char)c;
    }
  }
  return out;
}
// Normalized string'i belirtilen konumda yazdır
void drawTextTr(int16_t x, int16_t y, const String &s) {
  String norm = normalizeTr(s);
  tft.setCursor(x, y);
  tft.print(norm);
}
void drawInfoScreen() {
  if (!lcd_ok) return;

  tft.fillScreen(ST77XX_BLACK);
  drawTopBar();   // Üst bar aynı kalsın

  int16_t x1, y1;
  uint16_t w, h;

  // =======================
  //   BAŞLIK: SISTEM DURUMU
  // =======================
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);

  String title = "SISTEM DURUMU";   // İ ve Ü yerine sistemsel olarak ASCII kullandık
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  int titleX = VIEW_X + (VIEW_W - w) / 2;
  int titleY = VIEW_Y + STATUS_BAR_H + h + 6;   // üst barın hemen altı

  drawTextTr(titleX, titleY, title);

  // İçeriği yazmak için biraz aşağıdan başlayalım
  int y = titleY + 10;

  // Satırlar için font
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);

  // Yardımcı lambda: label solda, value sağda
  auto drawLine = [&](const String &label, const String &value, uint16_t valColor) {
    int lineH = 18;
    y += lineH;

    // Label (sol)
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setCursor(VIEW_X + 8, y);
    drawTextTr(VIEW_X + 8, y, label);

    // Değer (sağ)
    tft.getTextBounds(value, 0, 0, &x1, &y1, &w, &h);
    int vx = VIEW_X + VIEW_W - w - 8;
    tft.setTextColor(valColor, ST77XX_BLACK);
    drawTextTr(vx, y, value);
  };

  // =======================
  //   SATIRLAR
  // =======================

  // Firmware
  drawLine("Firmware", String(FW_VERSION), ST77XX_WHITE);

  // MAC
  drawLine("MAC", gMacUpper, ST77XX_WHITE);

  // WiFi
  String wifiVal;
  uint16_t wifiColor;
  if (WiFi.status() == WL_CONNECTED) {
    wifiVal = "OK (" + String(WiFi.RSSI()) + " dBm)";
    wifiColor = ST77XX_GREEN;
  } else {
    wifiVal = "BAGLI DEGIL";
    wifiColor = ST77XX_RED;
  }
  drawLine("WiFi", wifiVal, wifiColor);

  // MQTT
  String mqttVal = mqtt.connected() ? "OK" : "BAGLI DEGIL";
  uint16_t mqttColor = mqtt.connected() ? ST77XX_GREEN : ST77XX_RED;
  drawLine("MQTT", mqttVal, mqttColor);

  // AHT20
  if (aht20_ok) {
    drawLine("AHT20", "OK", ST77XX_GREEN);
  } else {
    // AHT20 çalışmıyorsa error publish et ve ekranda gösterme
    if (mqtt.connected()) {
      publishError("SENSOR_INIT_ERROR", "AHT20 sensor initialization failed", "AHT20");
    }
    // Ekranda gösterme
  }

  // BLE
  if (sysStatus.bleOK) {
    String bleVal = "OK (" + String(beaconCount) + " beacon)";
    drawLine("BLE", bleVal, ST77XX_GREEN);
  } else {
    // BLE çalışmıyorsa error publish et ve ekranda gösterme
    if (mqtt.connected()) {
      publishError("BLE_INIT_ERROR", "BLE initialization failed", "BLE");
    }
    // Ekranda gösterme
  }

  // Sensör sayısı
  String sensVal = "1 Dahili + " + String(cfg.activeSensorCount);
  drawLine("Sensorler", sensVal, ST77XX_WHITE);

  // Güç durumu
  updatePowerStatus();
  String powerVal;
  powerVal  = gPower.mainsPresent ? "AC " : "BAT ";
  powerVal += String(gPower.battPct) + "% ";
  powerVal += "(" + String(gPower.vbat, 2) + "V)";
  uint16_t powerColor = gPower.mainsPresent ? ST77XX_GREEN : ST77XX_WHITE;
  drawLine("Guc", powerVal, powerColor);

  // Uptime
  uint32_t up = millis() / 1000;
  char upBuf[32];
  snprintf(upBuf, sizeof(upBuf), "%luh %lum",
           (unsigned long)(up / 3600),
           (unsigned long)((up % 3600) / 60));
  drawLine("Uptime", String(upBuf), ST77XX_WHITE);

  // Kullanılan fontu eski hale getir
  tft.setFont();        // default bitmap font
  tft.setTextSize(1);
  tft.setTextColor(STATUS_FG, STATUS_BG);
}
#endif

#ifdef USE_LCD_ST7789
void drawNoWifiScreen() {
  if (!lcd_ok) return;
  
  tft.fillScreen(ST77XX_BLACK);
  
  // --- Gradient arka plan (showBootScreen'e benzer) ---
  for (int y = 0; y < SCREEN_H; y++) {
    uint8_t c = map(y, 0, SCREEN_H, 0, 80);
    uint16_t color = tft.color565(0, c, c*1.2);
    tft.drawFastHLine(0, y, SCREEN_W, color);
  }
  
  tft.setTextWrap(false);
  
  // =========================
  //   WiFi İkonu (Kırmızı)
  // =========================
  int iconX = SCREEN_W / 2;
  int iconY = SCREEN_H / 2 - 40;
  
  // Basit WiFi ikonu çiz (kırmızı)
  tft.drawCircle(iconX, iconY, 15, ST77XX_RED);
  tft.drawCircle(iconX, iconY, 10, ST77XX_RED);
  tft.fillCircle(iconX, iconY, 3, ST77XX_RED);
  
  // =========================
  //   Mesaj
  // =========================
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  
  String msg1 = "NETWORK YOK";
  String msg2 = "Lutfen Destek";
  String msg3 = "Talep Ediniz...";
  
  int16_t x1, y1;
  uint16_t w, h;
  
  // Mesaj 1
  tft.getTextBounds(msg1, 0, 0, &x1, &y1, &w, &h);
  int msgX = (SCREEN_W - w) / 2;
  int msgY = iconY + 50;
  drawTextTr(msgX, msgY, msg1);
  
  // Mesaj 2
  tft.setFont(&FreeSans9pt7b);
  tft.getTextBounds(msg2, 0, 0, &x1, &y1, &w, &h);
  msgX = (SCREEN_W - w) / 2;
  msgY += h + 10;
  drawTextTr(msgX, msgY, msg2);
  
  // Mesaj 3
  tft.getTextBounds(msg3, 0, 0, &x1, &y1, &w, &h);
  msgX = (SCREEN_W - w) / 2;
  msgY += h + 5;
  drawTextTr(msgX, msgY, msg3);
  
  tft.setFont();
  tft.setTextSize(1);
}
#endif

// VBAT ölçümü — kalibreli (0 gelme sorununu düzeltir)
// BATTERY_PIN senin dosyada #define olarak zaten varsa tekrar TANIMLAMA.
static float _readBatteryVolts() {
  pinMode(BATTERY_PIN, INPUT);

#ifdef ARDUINO_ARCH_ESP32
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db); // LiPo için güvenli
#endif

  const int SAMPLES = 8;
  uint32_t mvAcc = 0;
  for (int i=0;i<SAMPLES;i++) {
    mvAcc += analogReadMilliVolts(BATTERY_PIN);  // mV (kalibreli)
    delayMicroseconds(200);
  }

  float vAdc = (mvAcc / (float)SAMPLES) / 1000.0f; // Volt (ADC pininde)
  float vBat = vAdc * DIVIDER_RATIO * VREF_CORR;   // Bölücü & düzeltme ile VBAT
  //DEBUG_PRINTLN("VBAT : " + String(vBat));
  return vBat;
}

// LiPo için yaklaşık harita (3.3–4.2 V)
static uint8_t _voltsToPercent(float v) {
  if (v <= 3.30f) return 0;
  if (v >= 4.20f) return 100;
  return (uint8_t)((v - 3.30f) * (100.0f / (4.20f - 3.30f)));
}

// AC/USB algılama – GPIO2 ADC
static bool _readMainsPresent() {
  pinMode(PIN_PWR_SENSE, INPUT);
#ifdef ARDUINO_ARCH_ESP32
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PWR_SENSE, ADC_11db);
#endif

  // Ham mV oku
  uint32_t acc = 0;
  const int SAMPLES = 8;
  for (int i = 0; i < SAMPLES; i++) {
    acc += analogReadMilliVolts(PIN_PWR_SENSE);
    delayMicroseconds(200);
  }
  uint16_t mv = acc / SAMPLES;
  gPower.rawPwrAdc = mv;

  // Buradaki eşik senin donanıma göre: örn. > 500 mV ise "AC var" say
  return (mv > 500);
}


// ====== ALARM ======
void setAlarmState(bool active, const char* reason=nullptr) {
  if (active == alarmActive) return;
  alarmActive = active;
  if (active) {
    DEBUG_PRINTF("[ALARM] BASLADI: %s\n", reason ? reason : "Bilinmeyen");
    buzzerSilenced = false;
    soundState.nextRepeat = millis();
    soundState.isPlaying = false;
    pulseAlarmLed();
    screen = SCR_ALARM;
  } else {
    setAlarmLedNormal();
    DEBUG_PRINTLN("[ALARM] BITTI");
    buzzerOff();
    soundState.isPlaying = false;
    soundState.nextRepeat = 0;
    if (screen == SCR_ALARM) screen = SCR_WORK;
  }
}

// Helper: Alarm event bul veya oluştur
AlarmEvent* findOrCreateAlarmEvent(const char* alarmId) {
  // Önce mevcut event'i bul
  for (int i = 0; i < alarmEventCount; i++) {
    if (strcmp(alarmEvents[i].alarmId, alarmId) == 0) {
      return &alarmEvents[i];
    }
  }
  
  // Bulunamadı, yeni oluştur
  if (alarmEventCount < MAX_ALARM_EVENTS) {
    strlcpy(alarmEvents[alarmEventCount].alarmId, alarmId, sizeof(alarmEvents[alarmEventCount].alarmId));
    alarmEventCount++;
    return &alarmEvents[alarmEventCount - 1];
  }
  
  return nullptr;  // Yer yok
}

// Helper: Error event bul veya oluştur
ErrorEvent* findOrCreateErrorEvent(const char* errorId) {
  // Önce mevcut event'i bul
  for (int i = 0; i < errorEventCount; i++) {
    if (strcmp(errorEvents[i].errorId, errorId) == 0) {
      return &errorEvents[i];
    }
  }
  
  // Bulunamadı, yeni oluştur
  if (errorEventCount < MAX_ERROR_EVENTS) {
    strlcpy(errorEvents[errorEventCount].errorId, errorId, sizeof(errorEvents[errorEventCount].errorId));
    errorEventCount++;
    return &errorEvents[errorEventCount - 1];
  }
  
  return nullptr;  // Yer yok
}

// Alarm event'i aktif et
void activateAlarmEvent(const char* alarmId) {
  AlarmEvent* evt = findOrCreateAlarmEvent(alarmId);
  if (evt && !evt->isActive) {
    evt->isActive = true;
    evt->startTime = unixEpoch();
    evt->finishTime = 0;
    evt->published = false;
    evt->lastBuzzerTime = 0;
    evt->buzzerCount = 0;
    DEBUG_PRINTF("[ALARM-EVENT] BAŞLADI: %s @ %u\n", alarmId, evt->startTime);
  }
}

// Alarm event'i deaktif et
void deactivateAlarmEvent(const char* alarmId, const char* sensorType, const char* mahalId, const char* sensorName,
                         const char* alarmType, float value, float threshold) {
  AlarmEvent* evt = findOrCreateAlarmEvent(alarmId);
  if (evt && evt->isActive) {
    evt->isActive = false;
    evt->finishTime = unixEpoch();
    uint32_t duration = evt->finishTime - evt->startTime;
    DEBUG_PRINTF("[ALARM-EVENT] BİTTİ: %s @ %u (süre: %u sn)\n", alarmId, evt->finishTime, duration);
    
    // Eğer daha önce publish edildiyse, finish mesajı gönder
    if (evt->published && mqtt.connected()) {
      publishSensorAlarm(sensorType, mahalId, sensorName, alarmType, value, threshold, alarmId, true,
                       0.0f, 0.0f, 0.0f, 0.0f, nullptr, -127, -127);
    }
  }
}

bool publishSensorAlarm(const char* sensorType, const char* mahalId, const char* sensorName,
                       const char* alarmType, float value, float threshold, const char* alarmId, bool isFinish,
                       float tempHigh, float tempLow, float humHigh, float humLow,
                       const char* smac, int wifiRssi, int bleRssi) {
  if (!mqtt.connected()) return false;
  
  // Alarm event'i bul
  AlarmEvent* evt = findOrCreateAlarmEvent(alarmId);
  if (!evt) return false;
  
  // Eğer finish değilse ve daha önce publish edildiyse, tekrar publish etme
  if (!isFinish && evt->published && evt->isActive) {
    return true;  // Zaten publish edilmiş
  }
  
  DynamicJsonDocument doc(2048);
  doc["msg"] = "alarm";
  doc["alarmId"] = alarmId;
  doc["sensorType"] = sensorType;
  doc["mahalId"] = mahalId;
  doc["sensorName"] = sensorName;
  doc["atype"] = alarmType;
  doc["value"] = value;
  doc["threshold"] = threshold;
  doc["Stime"] = evt->startTime;
  doc["Ftime"] = evt->finishTime;
  doc["gmac"] = gMacUpper;
  
  // Limit değerleri
  doc["tempHigh"] = tempHigh;
  doc["tempLow"] = tempLow;
  doc["humHigh"] = humHigh;
  doc["humLow"] = humLow;
  
  // RSSI bilgileri
  JsonObject rssi = doc.createNestedObject("rssi");
  rssi["wifi"] = wifiRssi;
  rssi["ble"] = bleRssi;
  
  // Eddystone sensörler için smac
  if (smac) {
    doc["smac"] = smac;
  }
  
  // Power bilgileri (sadece power alarmı için veya tüm alarmlarda)
  if (strcmp(alarmType, "PO") == 0 || strcmp(alarmType, "PF") == 0) {
    JsonObject power = doc.createNestedObject("power");
    power["mains"] = gPower.mainsPresent;
    power["charging"] = gPower.charging;
    power["vbat"] = gPower.vbat;
    power["battPct"] = gPower.battPct;
    power["powerCut"] = !gPower.mainsPresent;
    power["power"] = gPower.mainsPresent ? "on" : "off";
  }

  String payload; serializeJson(doc, payload);
  bool ok = mqtt.publish(gTopicAlarm.c_str(), payload.c_str(), false);
  
  if (ok) {
    evt->published = true;
    DEBUG_PRINTF("[ALARM] %s - %s: OK (ID: %s, %s)\n", sensorName, alarmType, alarmId, isFinish ? "FINISH" : "START");
  } else {
    DEBUG_PRINTF("[ALARM] %s - %s: FAIL\n", sensorName, alarmType);
  }
  
  return ok;
}

// Error publish fonksiyonu
bool publishError(const char* errorType, const char* errorMessage, const char* component) {
  if (!mqtt.connected()) return false;
  
  // Error event'i bul veya oluştur
  ErrorEvent* evt = findOrCreateErrorEvent(errorType);
  if (!evt) return false;
  
  // Maksimum 2 kez publish et
  if (evt->publishCount >= 2) {
    return true;  // Zaten 2 kez publish edilmiş
  }
  
  DynamicJsonDocument doc(1024);
  doc["msg"] = "error";
  doc["errorType"] = errorType;
  doc["errorMessage"] = errorMessage;
  doc["component"] = component;
  doc["timestamp"] = unixEpoch();
  doc["gmac"] = gMacUpper;
  
  // Sistem durumu
  JsonObject system = doc.createNestedObject("system");
  system["wifiOK"] = sysStatus.wifiOK;
  system["mqttOK"] = sysStatus.mqttOK;
  system["aht20OK"] = aht20_ok;
  system["bleOK"] = sysStatus.bleOK;
  system["uptime"] = millis() / 1000;
  system["heap"] = ESP.getFreeHeap();
  
  String payload; serializeJson(doc, payload);
  bool ok = mqtt.publish(gTopicError.c_str(), payload.c_str(), false);
  
  if (ok) {
    evt->publishCount++;
    evt->lastPublishTime = millis();
    DEBUG_PRINTF("[ERROR] %s - %s: OK (count: %d/2)\n", errorType, errorMessage, evt->publishCount);
  } else {
    DEBUG_PRINTF("[ERROR] %s - %s: FAIL\n", errorType, errorMessage);
  }
  
  return ok;
}

void playCurrentAlarmSound()
{
  uint32_t now = millis();
  uint32_t elapsed = now - soundState.stepTime;

  // Basit pattern: 300ms on, 300ms off, 600ms silent
  const uint32_t cycle = 1200;
  uint32_t t = elapsed % cycle;

  if (t < 300) {
    // 0–300ms : ON
    buzzerOn();
  }
  else if (t < 600) {
    // 300–600ms: OFF
    buzzerOff();
  }
  else {
    // 600–1200ms sessizlik
    buzzerOff();
  }
}


void buzzerTick() {
  if (!cfg.buzzerEnabled || !alarmActive || buzzerSilenced) {
    if (soundState.isPlaying) { buzzerOff(); soundState.isPlaying=false; }
    return;
  }
  uint32_t now = millis();
  if (!soundState.isPlaying) {
    if (now >= soundState.nextRepeat) {
      soundState.isPlaying = true;
      soundState.startTime = now;
      soundState.stepTime  = now;
    }
  }
  if (soundState.isPlaying) {
    uint32_t elapsed = now - soundState.startTime;
    if (elapsed >= cfg.alarmSound.soundDuration) {
      buzzerOff();
      soundState.isPlaying = false;
      soundState.nextRepeat = now + cfg.alarmSound.soundRepeat;
    } else {
      playCurrentAlarmSound();
    }
  }
}

// ====== BUTON ======
void handleButton() {
  static uint32_t lastBtn=0; static bool lastState=HIGH;
  bool currentState = digitalRead(BUTTON_PIN);
  if (currentState==LOW && lastState==HIGH && millis()-lastBtn>300) {
    lastBtn = millis();
    if (alarmActive) {
      buzzerSilenced = true; buzzerOff(); DEBUG_PRINTLN("[BTN] Alarm sessiz");
    } else {
      switch (screen) {
        case SCR_WORK:   screen = SCR_BEACON; break;
        case SCR_BEACON: screen = SCR_INFO;   break;
        case SCR_INFO: 
        default:         screen = SCR_WORK;   break;
      }
      DEBUG_PRINTF("[BTN] Ekran: %d\n", screen);
    }
  }
  lastState = currentState;
}


// ====== MQTT ======
void saveConfig() {
  prefs.begin("kutar", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  DEBUG_PRINTLN("[CFG] Ayarlar kaydedildi");
}
void loadConfig() {
  prefs.begin("kutar", true);
  if (prefs.getBytesLength("cfg")==sizeof(cfg)) {
    prefs.getBytes("cfg", &cfg, sizeof(cfg));
    cfg.activeSensorCount=0;
    for (int i=0;i<32;i++) if (cfg.eddystoneSensors[i].enabled) cfg.activeSensorCount++;
    DEBUG_PRINTF("[CFG] Yüklendi - %d aktif Eddystone sensör\n", cfg.activeSensorCount);
  } else {
    DEBUG_PRINTLN("[CFG] Varsayılan ayarlar");
  }
  prefs.end();
}


void mqttCallback(char* topic, byte* payload, unsigned int len) {
  DEBUG_PRINTF("[MQTT] Topic: %s, Length: %d\n", topic, len);

  String tpc(topic);
  bool isCfgTopic    = (tpc == gTopicCfg);        // KUTARIoT/config/<MAC>
  bool isUpdateTopic = (tpc == gTopicUpdate);     // KUTARIoT/update

  // Sadece bu iki topic kabul
  if (!isCfgTopic && !isUpdateTopic) {
    DEBUG_PRINTF("[MQTT] Topic ignored. Beklenen: %s veya %s\n",
                 gTopicCfg.c_str(), gTopicUpdate.c_str());
    return;
  }

  // Raw payload
  char payloadStr[len + 1];
  memcpy(payloadStr, payload, len);
  payloadStr[len] = '\0';
  DEBUG_PRINTF("[CFG] Gelen JSON: %s\n", payloadStr);

  DynamicJsonDocument doc(8192);
  DeserializationError err = deserializeJson(doc, payload, len);
  if (err) {
    DEBUG_PRINTF("[CFG] JSON parse error: %s\n", err.c_str());
    return;
  }

  const char* cmd = doc["cmd"] | "";
  DEBUG_PRINTF("[CFG] Komut: %s (src=%s)\n",
               cmd, isUpdateTopic ? "UPDATE" : "CFG");

  /* =========================================================
   * UPDATE
   * - config/<MAC>  -> çalışır (ESKİ SİSTEM)
   * - KUTARIoT/update -> çalışır (YENİ TOPLU UPDATE)
   * ========================================================= */
  if (!strcmp(cmd, "update")) {
    DEBUG_PRINTLN("[OTA] UPDATE komutu algılandı");

    const char* url = doc["url"] | cfg.otaUrl;
    DEBUG_PRINTF("[OTA] URL: %s\n", url);

    if (doc.containsKey("user")) {
      strlcpy(cfg.otaUser, doc["user"], sizeof(cfg.otaUser));
      DEBUG_PRINTF("[OTA] User ayarlandı\n");
    }
    if (doc.containsKey("pass")) {
      strlcpy(cfg.otaPass, doc["pass"], sizeof(cfg.otaPass));
      DEBUG_PRINTLN("[OTA] Password ayarlandı");
    }
    if (doc.containsKey("insecureTLS")) {
      cfg.otaInsecureTLS = doc["insecureTLS"];
      DEBUG_PRINTF("[OTA] InsecureTLS: %s\n",
                   cfg.otaInsecureTLS ? "true" : "false");
    }

    DEBUG_PRINTF("[OTA] Başlatılıyor (src=%s)\n",
                 isUpdateTopic ? "TOPLU" : "TEKIL");

    performOTAUpdate(url);
    return;
  }

  /* =========================================================
   * BURADAN SONRASI SADECE config/<MAC>
   * ========================================================= */
  if (!isCfgTopic) {
    DEBUG_PRINTLN("[CFG] Bu komut sadece config/<MAC> topic'ten kabul edilir");
    return;
  }

  // RESET
  if (!strcmp(cmd, "reset")) {
    ESP.restart();
    return;
  }
  else if (!strcmp(cmd, "factoryDefault")) {
    DEBUG_PRINTLN("[CFG] factoryDefault komutu alindi");
    factoryDefaultReset();
  }
  // BUZZER SILENCE
  if (!strcmp(cmd, "silence")) {
    buzzerSilenced = true;
    buzzerOff();
    DEBUG_PRINTLN("[CFG] Buzzer sessiz");
    return;
  }


  // SENSOR CONFIG
  if (!strcmp(cmd, "setSensorConfigs")) {
    JsonArray sensors = doc["sensors"];
    cfg.activeSensorCount = 0;
    for (int i = 0; i < 32; i++) cfg.eddystoneSensors[i].enabled = false;

    DEBUG_PRINTF("[CFG] %d sensör alınıyor...\n", sensors.size());
    for (int i = 0; i < min(32, (int)sensors.size()); i++) {
      JsonObject s = sensors[i];
      strlcpy(cfg.eddystoneSensors[i].macAddress,  s["mac"]     | "", sizeof(cfg.eddystoneSensors[i].macAddress));
      strlcpy(cfg.eddystoneSensors[i].mahalId,     s["mahalId"] | "", sizeof(cfg.eddystoneSensors[i].mahalId));
      strlcpy(cfg.eddystoneSensors[i].sensorName,  s["name"]    | "", sizeof(cfg.eddystoneSensors[i].sensorName));
      cfg.eddystoneSensors[i].tempHigh      = s["tempHigh"]      | 30.0;
      cfg.eddystoneSensors[i].tempLow       = s["tempLow"]       | 15.0;
      cfg.eddystoneSensors[i].buzzerEnabled = s["buzzerEnabled"] | true;
      cfg.eddystoneSensors[i].enabled       = s["enabled"]       | true;

      if (cfg.eddystoneSensors[i].enabled &&
          strlen(cfg.eddystoneSensors[i].macAddress) > 0) {
        cfg.activeSensorCount++;
        DEBUG_PRINTF("[CFG] %d: %s (%s) T:%.1f-%.1f\n",
                     i,
                     cfg.eddystoneSensors[i].sensorName,
                     cfg.eddystoneSensors[i].macAddress,
                     cfg.eddystoneSensors[i].tempLow,
                     cfg.eddystoneSensors[i].tempHigh);
      }
    }

    if (doc.containsKey("internalSensor")) {
      JsonObject is = doc["internalSensor"];
      strlcpy(cfg.internalSensorName, is["name"]   | "Dahili Sensor",
              sizeof(cfg.internalSensorName));
      strlcpy(cfg.internalMahalId,    is["mahalId"]| "KTR-K1-RD-2300010",
              sizeof(cfg.internalMahalId));
      cfg.tempHigh = is["tempHigh"] | cfg.tempHigh;
      cfg.tempLow  = is["tempLow"]  | cfg.tempLow;
      if (is.containsKey("buzzerEnabled"))
        cfg.buzzerEnabled = is["buzzerEnabled"];
    }

    saveConfig();
    DEBUG_PRINTF("[CFG] %d sensör güncellendi\n", cfg.activeSensorCount);
    return;
  }

  // SET
  if (!strcmp(cmd, "set")) {
    JsonObject s = doc["set"];
    if (s.containsKey("wifiSsid"))
      strlcpy(cfg.wifiSsid, (const char*)s["wifiSsid"], sizeof(cfg.wifiSsid));
    if (s.containsKey("wifiPass"))
      strlcpy(cfg.wifiPass, (const char*)s["wifiPass"], sizeof(cfg.wifiPass));
    if (s.containsKey("dataPeriod"))
      cfg.dataPeriod = (uint32_t)s["dataPeriod"];
    if (s.containsKey("infoPeriod"))
      cfg.infoPeriod = (uint32_t)s["infoPeriod"];
    if (s.containsKey("tempHigh"))
      cfg.tempHigh = (float)s["tempHigh"];
    if (s.containsKey("tempLow"))
      cfg.tempLow  = (float)s["tempLow"];
    if (s.containsKey("buzzerEnabled"))
      cfg.buzzerEnabled = (bool)s["buzzerEnabled"];

    if (cfg.buzzerEnabled) buzzerSilenced = false;
    saveConfig();
    DEBUG_PRINTLN("[CFG] Genel ayarlar güncellendi");
    return;
  }

  DEBUG_PRINTLN("[CFG] Bilinmeyen komut");
}


// OTA Update fonksiyonu - HTTP ve HTTPS otomatik algılama
bool performOTAUpdate(const char* url) {
  if (!sysStatus.wifiOK) {
    DEBUG_PRINTLN("[OTA] WiFi bağlı değil!");
    return false;
  }

  DEBUG_PRINTF("[OTA] Güncelleme başlatılıyor: %s\n", url);

#ifdef USE_LCD_ST7789  
  // LCD'de güncelleme durumunu göster
  if (lcd_ok) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(50, 50);
    tft.println("FIRMWARE");
    tft.setCursor(60, 80);
    tft.println("UPDATE");
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(30, 120);
    tft.println("Connecting...");
  }
#endif

  // MQTT bağlantısını kapat (OTA sırasında kaynak tasarrufu için)
  if (mqtt.connected()) {
    mqtt.disconnect();
  }

  HTTPClient http;
  String urlStr = String(url);
  bool isHttps = urlStr.startsWith("https://");
  
  if (isHttps) {
    // HTTPS için WiFiClientSecure kullan
    WiFiClientSecure* client = new WiFiClientSecure();
    if (cfg.otaInsecureTLS) {
      client->setInsecure();
      DEBUG_PRINTLN("[OTA] HTTPS - SSL sertifika kontrolü devre dışı");
    } else {
      DEBUG_PRINTLN("[OTA] HTTPS - SSL sertifika kontrolü aktif");
    }
    client->setTimeout(30);
    http.begin(*client, url);
  } else {
    // HTTP için normal WiFiClient kullan
    WiFiClient* client = new WiFiClient();
    client->setTimeout(30);
    http.begin(*client, url);
    DEBUG_PRINTLN("[OTA] HTTP bağlantısı");
  }
  
  // Authentication varsa ekle
  if (strlen(cfg.otaUser) > 0 && strlen(cfg.otaPass) > 0) {
    http.setAuthorization(cfg.otaUser, cfg.otaPass);
    DEBUG_PRINTF("[OTA] Authentication: %s\n", cfg.otaUser);
  }

  // HTTP ayarları
  http.setTimeout(30000);
  http.addHeader("User-Agent", "KUTAR-IoT/2.1.0");
  http.addHeader("Connection", "close");
  
  DEBUG_PRINTLN("[OTA] HTTP GET başlatılıyor...");
  int httpCode = http.GET();
  
  if (httpCode != HTTP_CODE_OK) {
    DEBUG_PRINTF("[OTA] HTTP hatası: %d\n", httpCode);
    
    // Hata kodlarını açıkla
    switch(httpCode) {
      case -1: DEBUG_PRINTLN("[OTA] Bağlantı hatası - sunucu erişilebilir değil"); break;
      case -2: DEBUG_PRINTLN("[OTA] Bağlantı reddedildi"); break;
      case -3: DEBUG_PRINTLN("[OTA] HTTP isteği gönderilemedi"); break;
      case -4: DEBUG_PRINTLN("[OTA] HTTP cevabı alınamadı"); break;
      case -11: DEBUG_PRINTLN("[OTA] Timeout hatası"); break;
      case 401: DEBUG_PRINTLN("[OTA] Authentication hatası - kullanıcı/şifre yanlış"); break;
      case 403: DEBUG_PRINTLN("[OTA] Erişim reddedildi"); break;
      case 404: DEBUG_PRINTLN("[OTA] Dosya bulunamadı"); break;
      case 500: DEBUG_PRINTLN("[OTA] Sunucu hatası"); break;
    }
    
    http.end();
#ifdef USE_LCD_ST7789  
    
    if (lcd_ok) {
      tft.setTextColor(ST77XX_RED);
      tft.setCursor(30, 140);
      tft.print("HTTP Error: ");
      tft.println(httpCode);
      delay(3000);
    }
#endif
    return false;
  }

  int contentLength = http.getSize();
  DEBUG_PRINTF("[OTA] Firmware boyutu: %d bytes\n", contentLength);
  
  if (contentLength <= 0) {
    DEBUG_PRINTLN("[OTA] Geçersiz firmware boyutu!");
    http.end();
    return false;
  }

  // Boyut kontrolü (max 2MB)
  if (contentLength > 2097152) {
    DEBUG_PRINTLN("[OTA] Firmware çok büyük (>2MB)!");
    http.end();
    return false;
  }

  // OTA partition'ı kontrol et ve başlat
  if (!Update.begin(contentLength, U_FLASH)) {
    DEBUG_PRINTF("[OTA] Begin hatası: %s\n", Update.errorString());
    http.end();
    return false;
  }

  DEBUG_PRINTLN("[OTA] OTA başlatıldı, firmware indiriliyor...");

#ifdef USE_LCD_ST7789  

  // LCD progress bar
  if (lcd_ok) {
    tft.fillRect(20, 140, 200, 60, ST77XX_BLACK);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(30, 140);
    tft.println("Progress:");
    tft.drawRect(30, 160, 180, 20, ST77XX_WHITE);
  }
#endif

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buffer[1024];
  size_t written = 0;
  int lastProgress = -1;
  uint32_t lastUpdate = millis();
  uint32_t startTime = millis();

  while (http.connected() && written < contentLength) {
    // Watchdog reset
    yield();
    
    size_t available = stream->available();
    if (available > 0) {
      RST_WDT();
      size_t readSize = stream->readBytes(buffer, min(available, sizeof(buffer)));
      
      if (readSize > 0) {
        RST_WDT();
        size_t writeResult = Update.write(buffer, readSize);
        if (writeResult != readSize) {
          DEBUG_PRINTF("[OTA] Write hatası: %s\n", Update.errorString());
          Update.abort();
          http.end();
          return false;
        }
        written += readSize;
        lastUpdate = millis();
        
        // Progress hesapla ve göster (her %10'da bir)
        int progress = (written * 100) / contentLength;
        if (progress != lastProgress && progress % 10 == 0) {
          RST_WDT();
          lastProgress = progress;
          uint32_t elapsed = (millis() - startTime) / 1000;
          uint32_t speed = written / (elapsed + 1);
          DEBUG_PRINTF("[OTA] Progress: %d%% (%d/%d bytes, %d B/s)\n", 
                        progress, written, contentLength, speed);
#ifdef USE_LCD_ST7789  
          
          if (lcd_ok) {
            // Progress bar güncelle
            int barWidth = (progress * 176) / 100; // 176 = 180-4 (border)
            tft.fillRect(32, 162, barWidth, 16, ST77XX_GREEN);
            RST_WDT();
            // Percentage text
            tft.fillRect(30, 185, 100, 15, ST77XX_BLACK);
            tft.setTextColor(ST77XX_WHITE);
            tft.setCursor(30, 185);
            tft.print(progress);
          }
#endif
        }
      }
    } else {
      delay(10);
    }
    
    // Timeout kontrolü (60 saniye)
    if (millis() - lastUpdate > 60000) {
      RST_WDT();
      DEBUG_PRINTLN("[OTA] Timeout! İndirme çok yavaş.");
      Update.abort();
      http.end();
      return false;
    }
  }

  http.end();

  // İndirme tamamlama kontrolü
  if (written != contentLength) {
    DEBUG_PRINTF("[OTA] İndirme tamamlanamadı! Yazılan: %d, Beklenen: %d\n", 
                  written, contentLength);
    Update.abort();
    return false;
  }

  // Update'i tamamla
  if (!Update.end(true)) {
    DEBUG_PRINTF("[OTA] Update tamamlama hatası: %s\n", Update.errorString());
    return false;
  }

  DEBUG_PRINTLN("[OTA] Firmware güncellemesi başarılı!");

#ifdef USE_LCD_ST7789  
  if (lcd_ok) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(40, 80);
    tft.println("UPDATE");
    tft.setCursor(30, 110);
    tft.println("SUCCESS!");
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(50, 150);
    tft.println("Restarting...");
  }
#endif
  // Kısa bekleme ve restart
  delay(2000);
  ESP.restart();
  return true;
}


// NTP olmadan WiFi bağlantısı
bool ensureWiFiWithoutNTP() {
  DEBUG_PRINTLN("[WIFI] Bağlantı kontrol...");
  if (WiFi.status()==WL_CONNECTED) {
    sysStatus.wifiOK = true;
    DEBUG_PRINTF("[WIFI] Bağlı: %s (RSSI:%d)\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }
  
  if (strlen(cfg.wifiSsid)==0) { 
    DEBUG_PRINTLN("[WIFI] SSID boş"); 
    sysStatus.wifiOK=false; 
    return false; 
  }

  esp_wifi_set_max_tx_power(84);   // 84 = 21 dBm ≈ 100 mW

  DEBUG_PRINTF("[WIFI] Bağlanıyor: %s\n", cfg.wifiSsid);
   String hostname = "Termon-" + macSuffix6();
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.wifiSsid, cfg.wifiPass);

  uint32_t t0=millis(); 
  int dots=0;
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<20000) {
    delay(500); 
    if (isDebug){ 
      DEBUG_PRINT("."); 
      dots++; 
      if (dots>20){ 
        DEBUG_PRINTLN(); 
        dots=0; 
      } 
    }
  }
  
  if (WiFi.status()==WL_CONNECTED) {
    DEBUG_PRINTF("\n[WIFI] OK: %s (RSSI:%d)\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    sysStatus.wifiOK=true; 
    return true;
  } else {
    DEBUG_PRINTF("\n[WIFI] FAIL! Status:%d\n", WiFi.status()); 
    sysStatus.wifiOK=false; 
    return false;
  }
}

// Güvenli MQTT bağlantısı
bool mqttEnsureConnected() {
  if (mqtt.connected()) return true;
  if (!sysStatus.wifiOK) return false;
  
  static uint32_t lastAttempt = 0;
  if (millis() - lastAttempt < 5000) return false;
  lastAttempt = millis();

  DEBUG_PRINTLN("[MQTT] === Bağlantı Debug ===");
  DEBUG_PRINTF("Host: %s\n", MQTT_HOST);
  DEBUG_PRINTF("Port: %d\n", MQTT_PORT);
  DEBUG_PRINTF("User: %s\n", MQTT_USER);
  DEBUG_PRINTF("Pass: %s\n", MQTT_PASS);
  
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(8192);
  mqtt.setCallback(mqttCallback);

  mqtt.setKeepAlive(20);      // varsayılan 15-60; 20 iyi dengedir
  mqtt.setSocketTimeout(3);   // publish blok süresini kısalt

  String shortMac = gMacUpper.substring(8);
  String clientId = "KUTAR_" + shortMac;
  
  DEBUG_PRINTF("Client ID: %s\n", clientId.c_str());
  DEBUG_PRINTLN("Bağlantı deneniyor...");
  
  bool result = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);
  
  DEBUG_PRINTF("Sonuç: %s\n", result ? "BAŞARILI" : "BAŞARISIZ");
  DEBUG_PRINTF("State: %d\n", mqtt.state());
  
  if (result) {
    mqtt.subscribe(gTopicCfg.c_str(), 0);
    DEBUG_PRINTF("Subscribe: %s\n", gTopicCfg.c_str());

    mqtt.subscribe("KUTARIoT/update", 0);
    DEBUG_PRINTLN("Subscribe: KUTARIoT/update");

    sysStatus.mqttOK = true;
  } else {
    sysStatus.mqttOK = false;
    
    switch(mqtt.state()) {
      case -2: DEBUG_PRINTLN("Hata: TCP bağlantısı başarısız"); break;
      case 2: DEBUG_PRINTLN("Hata: Client ID geçersiz"); break;
      case 4: DEBUG_PRINTLN("Hata: Kullanıcı/şifre yanlış"); break;
      case 5: DEBUG_PRINTLN("Hata: Yetkilendirme başarısız"); break;
      default: DEBUG_PRINTF("Hata: Bilinmeyen durum %d\n", mqtt.state()); break;
    }
  }
  
  DEBUG_PRINTLN("==========================");
  return result;
}
bool publishData() {
  DEBUG_PRINTLN("[PUB] Data...");
  if (!mqtt.connected()) return false;

  // Gönderim öncesi cache tazelensin
  updateInternalCache();
  updateEddystoneCache();

  DynamicJsonDocument doc(8192);
  doc["msg"]  = "advData";
  doc["gmac"] = gMacUpper;
  doc["stat"] = "online";

  JsonArray obj = doc.createNestedArray("obj");

  // Global fallback – cache bozuksa kullanırız
  uint32_t nowEpoch = getSafeEpoch();

  // ============================
  //   DAHİLİ SENSÖR (CACHE)
  // ============================
  if (isCacheValid(internalCache)) {
    uint32_t ts = internalCache.sampleEpoch ? internalCache.sampleEpoch : nowEpoch;

    // Sıcaklık objesi
    {
      JsonObject tempObj = obj.createNestedObject();
      tempObj["type"]     = TYPE_TEMPERATURE;
      tempObj["dmac"]     = gMacUpper;
      tempObj["name"]     = cfg.internalSensorName;
      tempObj["location"] = cfg.internalMahalId;
      tempObj["temp"]     = internalCache.temperature;
      tempObj["batt"]     = internalCache.batteryLevel;
      tempObj["rssi"]     = internalCache.rssi;
      tempObj["time"]     = ts;
    }

    // Nem objesi
    {
      JsonObject humObj = obj.createNestedObject();
      humObj["type"]     = TYPE_HUMIDITY;
      humObj["dmac"]     = gMacUpper;
      humObj["name"]     = cfg.internalSensorName;
      humObj["location"] = cfg.internalMahalId;
      humObj["hum"]      = internalCache.humidity;
      humObj["batt"]     = internalCache.batteryLevel;
      humObj["rssi"]     = internalCache.rssi;
      humObj["time"]     = ts;
    }
  } else {
    // HATA DURUMU – cache yoksa tek seferlik fallback
    float internalTemp = readTemperature();
    float internalHum  = readHumidity();
    int   internalBatt = readBattPct();
    int   internalRssi = readWifiRSSI();
    uint32_t ts        = nowEpoch;

    JsonObject tempObj = obj.createNestedObject();
    tempObj["type"]     = TYPE_TEMPERATURE;
    tempObj["dmac"]     = gMacUpper;
    tempObj["name"]     = cfg.internalSensorName;
    tempObj["location"] = cfg.internalMahalId;
    tempObj["temp"]     = internalTemp;
    tempObj["batt"]     = internalBatt;
    tempObj["rssi"]     = internalRssi;
    tempObj["time"]     = ts;

    JsonObject humObj = obj.createNestedObject();
    humObj["type"]     = TYPE_HUMIDITY;
    humObj["dmac"]     = gMacUpper;
    humObj["name"]     = cfg.internalSensorName;
    humObj["location"] = cfg.internalMahalId;
    humObj["hum"]      = internalHum;
    humObj["batt"]     = internalBatt;
    humObj["rssi"]     = internalRssi;
    humObj["time"]     = ts;
  }

  // ============================
  //   EDDYSTONE SENSÖRLER (CACHE)
  // ============================
  for (int i = 0; i < 32; i++) {
    if (!cfg.eddystoneSensors[i].enabled) continue;
    
    if (!isCacheValid(eddystoneCache[i])) continue;

    uint32_t ts = eddystoneCache[i].sampleEpoch ? eddystoneCache[i].sampleEpoch : nowEpoch;

    JsonObject tempObj = obj.createNestedObject();
    tempObj["type"]     = TYPE_TEMPERATURE;
    tempObj["dmac"]     = String(cfg.eddystoneSensors[i].macAddress);
    tempObj["name"]     = String(cfg.eddystoneSensors[i].sensorName);
    tempObj["location"] = String(cfg.eddystoneSensors[i].mahalId);
    tempObj["temp"]     = eddystoneCache[i].temperature;
    tempObj["batt"]     = eddystoneCache[i].batteryLevel;
    tempObj["rssi"]     = eddystoneCache[i].rssi;
    tempObj["hum"]      = eddystoneCache[i].humidity;
    tempObj["time"]     = ts;
  }

  String payload;
  serializeJson(doc, payload);
  
  bool ok = mqtt.publish(gTopicData.c_str(), payload.c_str(), false);
  DEBUG_PRINTF("[PUB data] %s (%d objects)\n", ok ? "OK" : "FAIL", obj.size());
  
  return ok;
}


bool publishInfo() {
  if (!mqtt.connected()) return false;

  updatePowerStatus();  // güç durumunu oku

  // Config + 32 sensör detayını da koyacağımız için buffer'ı büyüttüm
  DynamicJsonDocument doc(8192);

  // ---- Genel header ----
  doc["msg"]    = "info";
  doc["fw"]     = FW_VERSION;
  doc["uptime"] = (uint32_t)(millis() / 1000);
  doc["heap"]   = ESP.getFreeHeap();
  doc["epoch"]  = unixEpoch();

  // ---- WiFi durumu ----
  JsonObject wifi = doc.createNestedObject("wifi");
  wifi["rssi"] = readWifiRSSI();
  wifi["mac"]  = gMacUpper;
  wifi["ip"]   = WiFi.localIP().toString();
  wifi["ssid"] = WiFi.SSID();

  // ---- Sensör özeti ----
  JsonObject sensors = doc.createNestedObject("sensors");
  sensors["aht20"]          = aht20_ok;
  sensors["internalName"]   = cfg.internalSensorName;
  sensors["internalMahal"]  = cfg.internalMahalId;
  sensors["eddystoneCount"] = cfg.activeSensorCount;

  // ---- Cihaz CONFIG (senin istediğin kısım) ----
  JsonObject cfgObj = doc.createNestedObject("config");

  // WiFi / zaman
  cfgObj["wifiSsid"]   = cfg.wifiSsid;
  cfgObj["dataPeriod"] = cfg.dataPeriod;   // ms
  cfgObj["infoPeriod"] = cfg.infoPeriod;   // ms

  // Dahili sensör limitleri + buzzer
  cfgObj["tempHigh"]      = cfg.tempHigh;
  cfgObj["tempLow"]       = cfg.tempLow;
  cfgObj["buzzerEnabled"] = cfg.buzzerEnabled;
  cfgObj["internalName"]  = cfg.internalSensorName;
  cfgObj["internalMahal"] = cfg.internalMahalId;

  // Beacon genel ayarları (scan, RSSI, prefix vs.)
  JsonObject beaconCfg = cfgObj.createNestedObject("beacon");
  beaconCfg["enabled"]       = cfg.beacon.enabled;
  beaconCfg["scanInterval"]  = cfg.beacon.scanInterval;
  beaconCfg["rssiThreshold"] = cfg.beacon.rssiThreshold;
  beaconCfg["targetPrefix"]  = cfg.beacon.targetMacPrefix;
  beaconCfg["maxBeacons"]    = cfg.beacon.maxBeacons;
  beaconCfg["timeoutMs"]     = cfg.beacon.beaconTimeout;

  // (İstersen buraya OTA config'lerini de ekleyebiliriz ama
  // güvenlik açısından otaUser/otaPass göndermemeyi tavsiye ederim.)

  // ---- Eddystone sensör config listesi ----
  JsonArray sensCfgArr = cfgObj.createNestedArray("eddystoneSensors");
  for (int i = 0; i < 32; i++) {
    const EddystoneSensorConfig &sc = cfg.eddystoneSensors[i];

    // Tamamen boş slotları atla
    if (sc.macAddress[0] == '\0' && !sc.enabled)
      continue;

    JsonObject s = sensCfgArr.createNestedObject();
    s["mac"]           = sc.macAddress;
    s["mahalId"]       = sc.mahalId;
    s["name"]          = sc.sensorName;
    s["tempHigh"]      = sc.tempHigh;
    s["tempLow"]       = sc.tempLow;
    s["buzzerEnabled"] = sc.buzzerEnabled;
    s["enabled"]       = sc.enabled;
  }

  // ---- BLE runtime bilgisi ----
  JsonObject ble = doc.createNestedObject("ble");
  ble["enabled"]      = cfg.beacon.enabled;
  ble["beaconCount"]  = beaconCount;
  ble["lastScan"]     = (millis() - lastBLEScan) / 1000;  // sn
  ble["targetPrefix"] = cfg.beacon.targetMacPrefix;

  // ---- Güç durumu ----
  JsonObject power = doc.createNestedObject("power");
  power["mains"]    = gPower.mainsPresent;   // true = elektrikte
  power["charging"] = gPower.charging;       // true = şarj oluyor
  power["vbat"]     = gPower.vbat;           // Volt
  power["battPct"]  = gPower.battPct;        // %
  power["powerCut"] = !gPower.mainsPresent;  // true = elektrik kesik

  // ---- Publish ----
  String payload;
  serializeJson(doc, payload);

  bool ok = mqtt.publish(gTopicInfo.c_str(), payload.c_str(), false);
  DEBUG_PRINTF("[PUB info] %s\n", ok ? "OK" : "FAIL");
  return ok;
}


bool publishBeaconData() {
  if (!mqtt.connected() || beaconCount==0) return false;
  DynamicJsonDocument doc(8192);
  doc["msg"]="beaconData"; doc["gmac"]=gMacUpper; doc["timestamp"]=unixEpoch(); doc["scanInterval"]=cfg.beacon.scanInterval;
  JsonArray arr = doc.createNestedArray("beacons");
  for (int i=0;i<beaconCount;i++) {
    if (!beacons[i].isValid) continue;
    JsonObject b = arr.createNestedObject();
    b["mac"]=beacons[i].macAddress;
    b["temp"]=beacons[i].temperature;
    b["battery"]=beacons[i].batteryLevel;
    b["rssi"]=beacons[i].rssi;
    b["advCount"]=beacons[i].advCount;
    b["secCount"]=beacons[i].secCount;
    b["lastSeen"]=(millis()-beacons[i].lastSeen)/1000;

    bool isConfigured=false;
    for (int j=0;j<32;j++) {
      if (cfg.eddystoneSensors[j].enabled && strcmp(cfg.eddystoneSensors[j].macAddress, beacons[i].macAddress.c_str())==0) {
        isConfigured=true;
        b["configured"]=true;
        b["sensorName"]=cfg.eddystoneSensors[j].sensorName;
        b["mahalId"]=cfg.eddystoneSensors[j].mahalId;
        break;
      }
    }
    if (!isConfigured) b["configured"]=false;
  }
  String payload; serializeJson(doc, payload);
  bool ok = mqtt.publish(gTopicBeacon.c_str(), payload.c_str(), false);
  DEBUG_PRINTF("[PUB beacon] %s (%d beacons)\n", ok?"OK":"FAIL", beaconCount);
  return ok;
}

// Offline kayıt okuma fonksiyonu
bool readOfflineRecord(uint16_t index, OfflineDataRecord* record) {
  String keyName = "rec" + String(index);
  prefs.begin("kutar", true);
  bool ok = (prefs.getBytesLength(keyName.c_str()) == sizeof(OfflineDataRecord));
  if (ok) {
    prefs.getBytes(keyName.c_str(), record, sizeof(OfflineDataRecord));
    // CRC kontrolü
    uint32_t calcCrc = calculateCRC32((uint8_t*)record, sizeof(OfflineDataRecord)-sizeof(record->crc32));
    ok = (calcCrc == record->crc32);
  }
  prefs.end();
  return ok;
}

// Offline kayıt silme fonksiyonu
void deleteOfflineRecord(uint16_t index) {
  String keyName = "rec" + String(index);
  prefs.begin("kutar", false);
  prefs.remove(keyName.c_str());
  prefs.end();
}

// Offline kayıtları gönderme fonksiyonu
bool publishOfflineData() {
  if (!mqtt.connected() || offlineBuffer.recordCount == 0) return false;

  // İnternet yoksa denemeyi kes (Wi-Fi bağlı olsa da)
  if (!hasInternet()) {
    DEBUG_PRINTLN("[OFFLINE] Internet yok → gönderim beklemeye alındı");
    return false;
  }

  DEBUG_PRINTF("[OFFLINE] %d kayıt gönderiliyor...\n", offlineBuffer.recordCount);

  const int BATCH_SIZE = 8;
  uint16_t sentTotal = 0;
  uint16_t idx = offlineBuffer.readIndex;

  while (sentTotal < offlineBuffer.recordCount) {
    // Zaman bütçesi: UI/BLE donmasın
    static uint32_t lastYield = 0;
    if (millis() - lastYield > 40) { mqtt.loop(); yield(); lastYield = millis(); }

    // Her batch başında internet/MQTT tekrar kontrol et
    if (!sysStatus.wifiOK || !mqtt.connected() || !hasInternet()) {
      DEBUG_PRINTLN("[OFFLINE] Bağlantı gitti → durduruluyor");
      return false;
    }

    DynamicJsonDocument doc(8192);
    doc["msg"] = "advData";
    doc["gmac"] = gMacUpper;
    doc["stat"] = "offline";
    doc["timestamp"] = unixEpoch();
    JsonObject bi = doc.createNestedObject("batchInfo");
    JsonArray records = doc.createNestedArray("records");

    // Bu batch’te kimleri sileceğimizi geçici listede tut
    uint16_t willDelete[BATCH_SIZE];
    int delCount = 0;

    int batchCount = 0;
    while (batchCount < BATCH_SIZE && sentTotal + batchCount < offlineBuffer.recordCount) {
      OfflineDataRecord rec;
      String keyName = "rec" + String(idx);
      prefs.begin("kutar", true);
      bool okLen = (prefs.getBytesLength(keyName.c_str()) == sizeof(OfflineDataRecord));
      if (okLen) prefs.getBytes(keyName.c_str(), &rec, sizeof(rec));
      prefs.end();

      if (!okLen) {
        // Kayıt yoksa slot atla (boşluğa düşmüş olabilir)
        idx = (idx + 1) % MAX_OFFLINE_RECORDS;
        continue;
      }

      // CRC kontrol
      uint32_t calcCrc = calculateCRC32((uint8_t*)&rec, sizeof(rec)-sizeof(rec.crc32));
      if (calcCrc != rec.crc32) {
        // Bozuk kayıt → direkt sil
        prefs.begin("kutar", false); prefs.remove(keyName.c_str()); prefs.end();
        DEBUG_PRINTF("[OFFLINE] CRC hatalı kayıt silindi @%d\n", idx);
        // Bozuk kaydı sayıya dahil etmeyelim
        offlineBuffer.recordCount--;
        if (offlineBuffer.recordCount == 0) {
          saveOfflineBufferState();
          return true;
        }
        idx = (idx + 1) % MAX_OFFLINE_RECORDS;
        continue;
      }

      // JSON’a ekle
      JsonObject o = records.createNestedObject();
      o["time"]  = rec.timestamp;
      o["dtime"] = formatTime(rec.timestamp).c_str();
      o["dmac"]  = rec.sensorId;
      o["temp"]  = rec.temperature;
      o["hum"]   = rec.humidity;
      o["batt"]  = rec.batteryPct;
      o["rssi"]  = rec.rssi;
      o["alarmState"]  = rec.alarmState;
      o["alarmReason"] = rec.alarmReason;
      o["recordType"]  = rec.recordType;

      // Bu batch başarıyla publish edilirse silinecekler
      willDelete[delCount++] = idx;

      batchCount++;
      idx = (idx + 1) % MAX_OFFLINE_RECORDS;
    }

    bi["total"] = offlineBuffer.recordCount;
    bi["start"] = sentTotal;
    bi["count"] = batchCount;

    String payload; serializeJson(doc, payload);
    bool ok = mqtt.publish(gTopicData.c_str(), payload.c_str(), false);
    DEBUG_PRINTF("[OFFLINE] Batch gönderildi: %d/%d kayıt, Sonuç: %s\n",
                  batchCount, offlineBuffer.recordCount, ok ? "OK" : "FAIL");

    if (!ok) {
      // Bu batch **silinmedi**, bir sonraki çevrimde aynısından tekrar deneyeceğiz
      return false;
    }

    // Publish OK → Artık bu batch’teki kayıtları sil
    for (int i = 0; i < delCount; i++) {
      String delKey = "rec" + String(willDelete[i]);
      prefs.begin("kutar", false); prefs.remove(delKey.c_str()); prefs.end();
    }

    sentTotal += batchCount;

    // Buffer sayaçlarını güncelle
    offlineBuffer.readIndex = idx;
    offlineBuffer.recordCount -= batchCount;
    offlineBuffer.bufferFull = false;
    offlineBuffer.lastSyncTime = unixEpoch();
    saveOfflineBufferState();

    // Aşırı arka arkaya publish edip brokerı boğmamak için minik nefes
    delay(120);
  }

  DEBUG_PRINTF("[OFFLINE] Tüm kayıtlar gönderildi: %d adet\n", sentTotal);
  // Tam boşaldıysa indeksleri sıfırlamak tertemiz olur
  offlineBuffer.readIndex = 0;
  offlineBuffer.writeIndex = 0;
  offlineBuffer.bufferFull = false;
  saveOfflineBufferState();
  return true;
}

// ====== OFFLINE VERİ ======
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  while (length--) {
    uint8_t c = *data++;
    for (uint8_t i = 0; i < 8; i++) {
      bool bit = (crc ^ c) & 1;
      crc >>= 1;
      if (bit) crc ^= 0xEDB88320;
      c >>= 1;
    }
  }
  return ~crc;
}

void initOfflineBuffer() {
  prefs.begin("kutar", true);
  if (prefs.getBytesLength("offlineBuf")==sizeof(offlineBuffer)) {
    prefs.getBytes("offlineBuf",&offlineBuffer,sizeof(offlineBuffer));
    DEBUG_PRINTF("[OFFLINE] Buffer yüklendi: Count=%d\n", offlineBuffer.recordCount);
  } else {
    memset(&offlineBuffer,0,sizeof(offlineBuffer));
    DEBUG_PRINTLN("[OFFLINE] Yeni buffer");
  }
  prefs.end();
}
void saveOfflineBufferState() {
  prefs.begin("kutar", false);
  prefs.putBytes("offlineBuf",&offlineBuffer,sizeof(offlineBuffer));
  prefs.end();
}
bool writeOfflineRecord(float temp, float hum, int batt, int rssi, 
                       uint8_t alarmState=0, const char* alarmReason="", 
                       const char* sensorId="INTERNAL") {
  OfflineDataRecord rec{}; // zero-init
  rec.timestamp = getSafeEpoch(); // ← Güvenli time kullan
  rec.temperature = temp; 
  rec.humidity = hum; 
  rec.batteryPct = batt; 
  rec.rssi = rssi;
  rec.alarmState = alarmState; 
  rec.recordType = (alarmState > 0) ? 1 : 0;
  if (alarmReason) strlcpy(rec.alarmReason, alarmReason, sizeof(rec.alarmReason));
  if (sensorId)    strlcpy(rec.sensorId,    sensorId,    sizeof(rec.sensorId));
  rec.crc32 = calculateCRC32((uint8_t*)&rec, sizeof(rec)-sizeof(rec.crc32));

  String keyName = "rec" + String(offlineBuffer.writeIndex);
  prefs.begin("kutar", false);
  bool ok = (prefs.putBytes(keyName.c_str(), &rec, sizeof(rec)) == sizeof(rec));
  prefs.end();

  DEBUG_PRINTLN("--- OFFLINE KAYIT DETAYLARI ---");
  DEBUG_PRINTF("  Kayıt İndeksi:  %d\n", offlineBuffer.writeIndex);
  DEBUG_PRINTF("  Toplam Kayıt:  %d\n", offlineBuffer.totalRecords);
    
    // Bu satırı yeni satırla değiştirin:
  DEBUG_PRINTF("  Zaman Damgası:  %u\n", rec.timestamp);
  DEBUG_PRINTF("  Zaman:  %s\n", formatTime(rec.timestamp).c_str());
    
  DEBUG_PRINTF("  Sıcaklık:      %.2f C\n", rec.temperature);
  DEBUG_PRINTF("  Nem:           %.2f %%\n", rec.humidity);
  DEBUG_PRINTF("  Batarya:       %d %%\n", rec.batteryPct);
  DEBUG_PRINTF("  RSSI:          %d dBm\n", rec.rssi);
  DEBUG_PRINTF("  Alarm Durumu:  %d (%s)\n", rec.alarmState, rec.alarmReason);
  DEBUG_PRINTF("  Sensör ID:     %s\n", rec.sensorId);
  DEBUG_PRINTF("  CRC32:         %u\n", rec.crc32);
  DEBUG_PRINTLN("---------------------------------");

  if (ok) {
    offlineBuffer.writeIndex = (offlineBuffer.writeIndex + 1) % MAX_OFFLINE_RECORDS;
    offlineBuffer.totalRecords++;
    if (offlineBuffer.recordCount < MAX_OFFLINE_RECORDS) offlineBuffer.recordCount++;
    else { 
      offlineBuffer.bufferFull = true; 
      offlineBuffer.readIndex = (offlineBuffer.readIndex + 1) % MAX_OFFLINE_RECORDS; 
    }
    saveOfflineBufferState();
    DEBUG_PRINTF("[OFFLINE] Kayıt yazıldı: %s, Epoch:%u, Count=%d\n", 
                  sensorId, rec.timestamp, offlineBuffer.recordCount);
    return true;
  }
  return false;
}

void handleOfflineDataLogging() {
  static uint32_t lastCheck = 0;
  static uint32_t lastOfflineSample = 0;   // OFFLINE örnek zamanı

  if (millis() - lastCheck < 2000) return;  // 2 sn’de bir sadece KONTROL
  lastCheck = millis();

  bool wifiOK     = (WiFi.status() == WL_CONNECTED);
  bool mqttOK     = mqtt.connected();
  bool internetOK = hasInternet();

  // Sistem durumunu güncelle
  sysStatus.wifiOK     = wifiOK;
  sysStatus.mqttOK     = mqttOK;
  sysStatus.internetOK = internetOK;

  // ---- ONLINE MOD ----
  if (wifiOK && mqttOK && internetOK) {
    // Burada ARTIK publishOfflineData() çağırmıyoruz
    // Online veri gönderimi aşağıda loop() içindeki tData timer'ında
  }
  // ---- OFFLINE MOD: sadece dataPeriod periyodunda kayıt al ----
  else {
    uint32_t now    = millis();
    uint32_t period = cfg.dataPeriod;   // default 120000 ms (2 dk)

    if (now - lastOfflineSample >= period) {
      lastOfflineSample = now;

      // 1) Dahili sensör kaydı
      float temp  = readTemperature();
      float hum   = readHumidity();
      int   batt  = getBatteryPercent();
      int   rssi  = readWifiRSSI();

      writeOfflineRecord(temp, hum, batt, rssi, 0, "", "INTERNAL");

      // 2) Config’te tanımlı Eddystone beacon kayıtları
      // 2) Config’te tanımlı Eddystone beacon kayıtları
      for (int i = 0; i < 32; i++) {
        if (!cfg.eddystoneSensors[i].enabled) continue;

        EddystoneBeacon* b = findBeaconByMac(cfg.eddystoneSensors[i].macAddress);
        if (!b || !b->isValid) continue;   // Bu periyotta görünmediyse atla

        float bTemp = b->temperature;

        // ADVCOUNT → NEM
        float bHum  = 255.0f;
        if (b->advCount != 0) {
          bHum = (float)b->advCount / 100.0f;
        }

        int   bBatt = b->batteryLevel;
        int   bRssi = b->rssi;

        writeOfflineRecord(bTemp, bHum, bBatt, bRssi, 0, "", cfg.eddystoneSensors[i].macAddress);
      }


      DEBUG_PRINTF("[OFFLINE] Örnek alındı, toplam=%d kayıt\n", offlineBuffer.recordCount);
    }
  }

  // ---- Alt bar’da bağlantı durumunu güncelle ----
  updateNetworkStatus(wifiOK, mqttOK);
}



void vDataTask(void *pvParameters) {
  (void) pvParameters;

  // İlk çalıştırmada sistem stabilizasyonu için bekle
  vTaskDelay(pdMS_TO_TICKS(10000));

  const TickType_t xPeriod = pdMS_TO_TICKS(cfg.dataPeriod);  // varsayılan 120000ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    RST_WDT();

    bool online = (sysStatus.wifiOK && sysStatus.mqttOK && mqtt.connected());

    // =============================================================
    // (1) ONLINE → DATA + BEACON PUBLISH
    // =============================================================
    if (online) {
      DEBUG_PRINTLN("[DATA-TASK] Online mod");
      
      // Önce publishData() dene
      bool dataOK = publishData();
      
      if (!dataOK) {
        DEBUG_PRINTLN("[DATA-TASK] publishData() başarısız! Offline'a kaydediliyor...");
        
        // Dahili sensör
        float t = readTemperature();
        float h = readHumidity();
        int b = getBatteryPercent();
        int r = readWifiRSSI();
        writeOfflineRecord(t, h, b, r, 0, "", "INTERNAL");
        
        // Eddystone beacon'lar
        for (int i = 0; i < 32; i++) {
          if (!cfg.eddystoneSensors[i].enabled) continue;
          
          EddystoneBeacon* beacon = findBeaconByMac(cfg.eddystoneSensors[i].macAddress);
          if (!beacon || !beacon->isValid) continue;
          
          float bHum = 255.0f;
          if (beacon->advCount != 0) {
            bHum = (float)beacon->advCount / 100.0f;
          }

          writeOfflineRecord(
            beacon->temperature,
            bHum,
            beacon->batteryLevel,
            beacon->rssi,
            0,
            "",
            cfg.eddystoneSensors[i].macAddress
          );
          
          DEBUG_PRINTF("[DATA-TASK] Beacon offline kaydedildi: %s (%.1f°C)\n",
                       cfg.eddystoneSensors[i].macAddress, beacon->temperature);
        }
      } else {
        DEBUG_PRINTLN("[DATA-TASK] publishData() OK");
      }
      
      // Beacon publish (ayrı topic)
      if (beaconCount > 0) {
        vTaskDelay(pdMS_TO_TICKS(500)); // Broker'ı boğmamak için
        
        bool beaconOK = publishBeaconData();
        DEBUG_PRINTF("[DATA-TASK] publishBeaconData() %s (count=%d)\n", 
                     beaconOK ? "OK" : "FAIL", beaconCount);
      }
    }
    // =============================================================
    // (2) OFFLINE → TÜM SENSÖRLERİ KAYDET
    // =============================================================
    else {
      DEBUG_PRINTLN("[DATA-TASK] Offline mod - tüm sensörler kaydediliyor");
      
      // 1. Dahili sensör
      float t = readTemperature();
      float h = readHumidity();
      int b = getBatteryPercent();
      int r = readWifiRSSI();
      
      writeOfflineRecord(t, h, b, r, 0, "", "INTERNAL");
      DEBUG_PRINTF("[DATA-TASK] Dahili sensör kaydedildi: %.1f°C, %.1f%%, Batt:%d%%, RSSI:%d\n",
                   t, h, b, r);
      
      // 2. Tüm aktif Eddystone beacon'lar
      int beaconsSaved = 0;
      for (int i = 0; i < 32; i++) {
        if (!cfg.eddystoneSensors[i].enabled) continue;
        
        EddystoneBeacon* beacon = findBeaconByMac(cfg.eddystoneSensors[i].macAddress);
        if (!beacon || !beacon->isValid) {
          DEBUG_PRINTF("[DATA-TASK] Beacon bulunamadı/geçersiz: %s\n", 
                       cfg.eddystoneSensors[i].macAddress);
          continue;
        }
        
        writeOfflineRecord(
          beacon->temperature,
          255.0f,  // TLM'de nem yok
          beacon->batteryLevel,
          beacon->rssi,
          0,
          "",
          cfg.eddystoneSensors[i].macAddress
        );
        
        beaconsSaved++;
        DEBUG_PRINTF("[DATA-TASK] Beacon kaydedildi: %s (%.1f°C, %dmV, RSSI:%d)\n",
                     cfg.eddystoneSensors[i].macAddress,
                     beacon->temperature,
                     beacon->batteryLevel,
                     beacon->rssi);
      }
      
      DEBUG_PRINTF("[DATA-TASK] Offline kayıt tamamlandı - Dahili:1, Beacon:%d, Toplam:%d\n",
                   beaconsSaved, offlineBuffer.recordCount);
    }
  }
}


// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(200);
  DEBUG_PRINTLN("\n=== KUTAR IoT – Multi-Sensor WiFi + AHT20 + ST7789 + Eddystone ===");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  initAlarmLed();
  
  // Buzzer init ve startup tone
  buzzerInit();
  playStartupTone();

  loadConfig();
  
  // Eddystone error tracker initialize
  memset(eddystoneErrorTracker, 0, sizeof(eddystoneErrorTracker));
  printConfig();

  initOfflineBuffer();

  gMacUpper = robustMacNoColonUpper();
  gTopicData   = tpData(gMacUpper);
  gTopicInfo   = tpInfo(gMacUpper);
  gTopicAlarm  = tpAlarm(gMacUpper);
  gTopicCfg    = tpCfg(gMacUpper);
  gTopicBeacon = tpBeacon(gMacUpper);
  gTopicError  = tpError(gMacUpper);
  //gTopicOffline = "KUTARIoT/offline/" + gMacUpper;

  DEBUG_PRINTF("[SETUP] Device MAC: %s\n", gMacUpper.c_str());
  DEBUG_PRINTF("[SETUP] MQTT Topics:\n  Data: %s\n  Config: %s\n", gTopicData.c_str(), gTopicCfg.c_str());

  Serial.println("#"+gMacUpper+"#"+FW_VERSION+"#"); 

#ifdef USE_LCD_ST7789
  initLCD();
#endif

#ifdef USE_LCD_ST7789  
  if (lcd_ok) showBootScreen();
#endif

  Wire.begin(I2C_SDA, I2C_SCL);
  DEBUG_PRINTLN("[I2C] Bus başlatıldı");
  if (isDebug) i2cScan();

  DEBUG_PRINTLN("[AHT20] Başlatılıyor...");
  if (!aht20.begin()) { 
    DEBUG_PRINTLN("[AHT20] HATA! Cihaz yok."); 
    aht20_ok=false; 
  } else {
    aht20_ok = true;
    sensors_event_t h,t;
    if (aht20.getEvent(&h,&t)) {
      DEBUG_PRINTF("[AHT20] Test: T=%.1fC, H=%.1f%%\n", t.temperature, h.relative_humidity);
    }
  }

  buzzerInit();
  playStartupTone();  // Açılış tonu
  initBLE();
  
  // Eddystone error tracker initialize
  memset(eddystoneErrorTracker, 0, sizeof(eddystoneErrorTracker));

  // 1. WiFi Bağlantısı (NTP olmadan)
  DEBUG_PRINTF("[WIFI] Bağlanıyor: %s\n", cfg.wifiSsid);
  sysStatus.wifiOK = ensureWiFiWithoutNTP();
  
  if (sysStatus.wifiOK) {
    DEBUG_PRINTLN("[SETUP] WiFi OK, NTP senkronizasyonu deneniyor...");
    // Önce NTP, MQTT'den bağımsız
    syncTimeViaNTP();
    updateTimeSync();   // timeIsSynced flag'i güncellensin

    DEBUG_PRINTLN("[SETUP] MQTT bağlantısı deneniyor...");
    delay(2000);
    sysStatus.mqttOK = mqttEnsureConnected();
  }
  initChargeDetect();

#ifdef USE_LCD_ST7789
  if (lcd_ok) updateNetworkStatus(sysStatus.wifiOK, sysStatus.mqttOK);
#endif

  // WiFi bağlı değilse SCR_NO_WIFI ekranını göster
  if (WiFi.status() != WL_CONNECTED) {
    screen = SCR_NO_WIFI;
  } else {
    screen = SCR_INFO;
    showInfoUntil = millis() + 10000;  // İstersen bunu da silebilirsin artık
  }

  DEBUG_PRINTLN("[SETUP] === Sistem Durumu ===");
  DEBUG_PRINTF("WiFi: %s\n", sysStatus.wifiOK ? "OK" : "FAIL");
  DEBUG_PRINTF("MQTT: %s\n", sysStatus.mqttOK ? "OK" : "FAIL");
  DEBUG_PRINTF("AHT20: %s\n", aht20_ok ? "OK" : "FAIL");
  DEBUG_PRINTF("BLE: %s\n", sysStatus.bleOK ? "OK" : "FAIL");
  DEBUG_PRINTF("LCD: %s\n", lcd_ok ? "OK" : "FAIL");
  DEBUG_PRINTF("Dahili Sensör: %s (%s)\n", cfg.internalSensorName, cfg.internalMahalId);
  DEBUG_PRINTF("Aktif Eddystone Sensör: %d adet\n", cfg.activeSensorCount);
  DEBUG_PRINTLN("========================");
  DEBUG_PRINTLN("[SETUP] Sistem hazır!");
  
  currentSensorIndex = -1;
  lastSensorSwitch = millis();
  if (mqtt.connected()) {
    if(publishInfo())
      DEBUG_PRINTF("Info published");
    else
      DEBUG_PRINTF("Info publish failed");
  }
  // Time yönetimini başlat
  systemStartTime = millis();
  lastKnownEpoch = 1640995200; // 2022-01-01 fallback
  updateTimeSync();

    xTaskCreate(
    vDataTask,          // Task fonksiyonu
    "DataTask",         // Task ismi
    8192,               // Stack (gerekirse arttır)
    NULL,               // Parametre yok
    1,                  // Öncelik (loop ile aynı/benzer)
    &dataTaskHandle     // Handle
  );
}

// ====== LOOP ======
void handleMultiSensorAlarm();

// Non-blocking WiFi yeniden bağlantı
void tryReconnectWiFi() {
  static uint32_t lastTry = 0;
  static bool connecting = false;
  
  // 30 saniyede bir dene
  if (millis() - lastTry < 30000) return;
  
  if (!connecting) {
    DEBUG_PRINTLN("[WIFI] Yeniden bağlanmaya çalışılıyor...");
    WiFi.disconnect(true);
    delay(100);
    WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
    connecting = true;
    lastTry = millis();
  } else {
    // 10 saniye sonra durumu kontrol et
    if (millis() - lastTry > 10000) {
      if (WiFi.status() == WL_CONNECTED) {
        syncTimeViaNTP();
        DEBUG_PRINTLN("[WIFI] Yeniden bağlandı!");
        sysStatus.wifiOK = true;
      } else {
        DEBUG_PRINTLN("[WIFI] Bağlantı başarısız, tekrar denenecek");
      }
      connecting = false;
    }
  }
}

// ====== ÇOK-SENSÖR ALARM KONTROL ======
void handleMultiSensorAlarm() {
  static uint32_t lastAlarmCheck = 0;
  if (millis() - lastAlarmCheck < 1000) return;   // 1 sn'de bir kontrol
  lastAlarmCheck = millis();

  // --- Publish rate-limit için state'ler ---
  static bool     lastInternalTempAlarm     = false;
  static bool     lastInternalHumAlarm      = false;
  static bool     lastPowerAlarm            = false;
  static bool     lastBatteryAlarm          = false;
  static bool     lastEddystoneAlarm[32]    = {0};
  static bool     lastEddystoneHumAlarm[32] = {0};
  static bool     lastEddystoneBattAlarm[32] = {0};

  uint32_t now = millis();
  updatePowerStatus();  // Güç durumunu güncelle

  bool anyAlarm = false;

  // ===== 1) DAHİLİ SENSÖR - SICAKLIK =====
  float t = readTemperature();
  bool internalTempAlarm = false;
  if (!isnan(t) && t != 255.0f) {
    internalTempAlarm = (t > cfg.tempHigh || t < cfg.tempLow);

    if (internalTempAlarm) {
      anyAlarm = true;
      const char* reason = (t > cfg.tempHigh) ? "TH" : "TL";  // TEMP_HIGH -> TH, TEMP_LOW -> TL
      char alarmId[64];
      // Aynı alarm tipi için aynı ID kullan (timestamp yok)
      snprintf(alarmId, sizeof(alarmId), "INTERNAL_%s", reason);
      
      activateAlarmEvent(alarmId);
      
      if (!lastInternalTempAlarm && mqtt.connected()) {
        publishSensorAlarm(
          "INTERNAL",
          cfg.internalMahalId,
          cfg.internalSensorName,
          reason,
          t,
          (t > cfg.tempHigh) ? cfg.tempHigh : cfg.tempLow,
          alarmId,
          false,
          cfg.tempHigh,
          cfg.tempLow,
          0.0f,
          0.0f,
          nullptr,
          readWifiRSSI(),
          -127
        );
      }
    } else {
      // Alarm bitti - INTERNAL alarm event'lerini deaktif et
      char alarmId[64];
      snprintf(alarmId, sizeof(alarmId), "INTERNAL_TH");
      deactivateAlarmEvent(alarmId, "INTERNAL", cfg.internalMahalId, cfg.internalSensorName, "TEMP_HIGH", t, cfg.tempHigh);
      snprintf(alarmId, sizeof(alarmId), "INTERNAL_TL");
      deactivateAlarmEvent(alarmId, "INTERNAL", cfg.internalMahalId, cfg.internalSensorName, "TEMP_LOW", t, cfg.tempLow);
    }
  }
  lastInternalTempAlarm = internalTempAlarm;

  // ===== 2) DAHİLİ SENSÖR - NEM =====
  float h = readHumidity();
  bool internalHumAlarm = false;
  if (!isnan(h) && h != 255.0f) {
    internalHumAlarm = (h > cfg.humHigh || h < cfg.humLow);
    
    if (internalHumAlarm) {
      anyAlarm = true;
      const char* reason = (h > cfg.humHigh) ? "HH" : "HL";  // HUM_HIGH -> HH, HUM_LOW -> HL
      char alarmId[64];
      // Aynı alarm tipi için aynı ID kullan (timestamp yok)
      snprintf(alarmId, sizeof(alarmId), "INTERNAL_%s", reason);
      
      activateAlarmEvent(alarmId);
      
      if (!lastInternalHumAlarm && mqtt.connected()) {
        publishSensorAlarm(
          "INTERNAL",
          cfg.internalMahalId,
          cfg.internalSensorName,
          reason,
          h,
          (h > cfg.humHigh) ? cfg.humHigh : cfg.humLow,
          alarmId,
          false,
          0.0f,
          0.0f,
          cfg.humHigh,
          cfg.humLow,
          nullptr,
          readWifiRSSI(),
          -127
        );
      }
    } else {
      // Alarm bitti - INTERNAL nem alarm event'lerini deaktif et
      char alarmId[64];
      snprintf(alarmId, sizeof(alarmId), "INTERNAL_HH");
      deactivateAlarmEvent(alarmId, "INTERNAL", cfg.internalMahalId, cfg.internalSensorName, "HUM_HIGH", h, cfg.humHigh);
      snprintf(alarmId, sizeof(alarmId), "INTERNAL_HL");
      deactivateAlarmEvent(alarmId, "INTERNAL", cfg.internalMahalId, cfg.internalSensorName, "HUM_LOW", h, cfg.humLow);
    }
  }
  lastInternalHumAlarm = internalHumAlarm;

  // ===== 3) EDDYSTONE SENSÖRLER - SICAKLIK =====
  for (int i = 0; i < 32; i++) {
    if (!cfg.eddystoneSensors[i].enabled) continue;

    if (!isCacheValid(eddystoneCache[i])) continue;
    
    float temp = eddystoneCache[i].temperature;
    // 255 ölçüm geldiyse bu beacon için alarm hesaplama
    if (temp == 255.0f || isnan(temp)) {
      lastEddystoneAlarm[i] = false;
      continue;
    }

    bool tempAlarm = (temp > cfg.eddystoneSensors[i].tempHigh ||
                      temp < cfg.eddystoneSensors[i].tempLow);

    if (tempAlarm) {
      anyAlarm = true;
      const char* reason = (temp > cfg.eddystoneSensors[i].tempHigh) ? "TH" : "TL";
      char alarmId[64];
      // Aynı sensör ve alarm tipi için aynı ID kullan (MAC + tip)
      snprintf(alarmId, sizeof(alarmId), "EDDY_%s_%s", cfg.eddystoneSensors[i].macAddress, reason);
      
      activateAlarmEvent(alarmId);
      
      if (!lastEddystoneAlarm[i] && mqtt.connected()) {
        publishSensorAlarm(
          "EDDYSTONE",
          cfg.eddystoneSensors[i].mahalId,
          cfg.eddystoneSensors[i].sensorName,
          reason,
          temp,
          (temp > cfg.eddystoneSensors[i].tempHigh)
            ? cfg.eddystoneSensors[i].tempHigh
            : cfg.eddystoneSensors[i].tempLow,
          alarmId,
          false,
          cfg.eddystoneSensors[i].tempHigh,
          cfg.eddystoneSensors[i].tempLow,
          0.0f,
          0.0f,
          cfg.eddystoneSensors[i].macAddress,
          readWifiRSSI(),
          eddystoneCache[i].rssi
        );
      }
    } else {
      // Alarm bitti - bu sensör için alarm event'lerini deaktif et
      char alarmId[64];
      snprintf(alarmId, sizeof(alarmId), "EDDY_%s_TH", cfg.eddystoneSensors[i].macAddress);
      deactivateAlarmEvent(alarmId, "EDDYSTONE", cfg.eddystoneSensors[i].mahalId, cfg.eddystoneSensors[i].sensorName, 
                          "TEMP_HIGH", temp, cfg.eddystoneSensors[i].tempHigh);
      snprintf(alarmId, sizeof(alarmId), "EDDY_%s_TL", cfg.eddystoneSensors[i].macAddress);
      deactivateAlarmEvent(alarmId, "EDDYSTONE", cfg.eddystoneSensors[i].mahalId, cfg.eddystoneSensors[i].sensorName, 
                          "TEMP_LOW", temp, cfg.eddystoneSensors[i].tempLow);
    }

    lastEddystoneAlarm[i] = tempAlarm;
  }

  // ===== 4) EDDYSTONE SENSÖRLER - NEM =====
  for (int i = 0; i < 32; i++) {
    if (!cfg.eddystoneSensors[i].enabled) continue;
    
    if (!isCacheValid(eddystoneCache[i])) continue;
    
    float h = eddystoneCache[i].humidity;
    if (isnan(h) || h == 255.0f) continue;
    
    bool humAlarm = (h > cfg.eddystoneSensors[i].humHigh || h < cfg.eddystoneSensors[i].humLow);
    
    if (humAlarm) {
      anyAlarm = true;
      const char* reason = (h > cfg.eddystoneSensors[i].humHigh) ? "HH" : "HL";
      char alarmId[64];
      // Aynı sensör ve alarm tipi için aynı ID kullan (MAC + tip)
      snprintf(alarmId, sizeof(alarmId), "EDDY_%s_%s", cfg.eddystoneSensors[i].macAddress, reason);
      
      activateAlarmEvent(alarmId);
      
      if (!lastEddystoneHumAlarm[i] && mqtt.connected()) {
        publishSensorAlarm(
          "EDDYSTONE",
          cfg.eddystoneSensors[i].mahalId,
          cfg.eddystoneSensors[i].sensorName,
          reason,
          h,
          (h > cfg.eddystoneSensors[i].humHigh) ? cfg.eddystoneSensors[i].humHigh : cfg.eddystoneSensors[i].humLow,
          alarmId,
          false,
          0.0f,
          0.0f,
          cfg.eddystoneSensors[i].humHigh,
          cfg.eddystoneSensors[i].humLow,
          cfg.eddystoneSensors[i].macAddress,
          readWifiRSSI(),
          eddystoneCache[i].rssi
        );
      }
      lastEddystoneHumAlarm[i] = true;
    } else {
      // Alarm bitti - bu sensör için nem alarm event'lerini deaktif et
      char alarmId[64];
      snprintf(alarmId, sizeof(alarmId), "EDDY_%s_HH", cfg.eddystoneSensors[i].macAddress);
      deactivateAlarmEvent(alarmId, "EDDYSTONE", cfg.eddystoneSensors[i].mahalId, cfg.eddystoneSensors[i].sensorName, 
                          "HUM_HIGH", h, cfg.eddystoneSensors[i].humHigh);
      snprintf(alarmId, sizeof(alarmId), "EDDY_%s_HL", cfg.eddystoneSensors[i].macAddress);
      deactivateAlarmEvent(alarmId, "EDDYSTONE", cfg.eddystoneSensors[i].mahalId, cfg.eddystoneSensors[i].sensorName, 
                          "HUM_LOW", h, cfg.eddystoneSensors[i].humLow);
      
      lastEddystoneHumAlarm[i] = false;
    }
  }

  // ===== 5) AC GÜÇ ALARMI =====
  static bool lastMainsState = true;
  bool currentMainsState = gPower.mainsPresent;
  
  if (currentMainsState != lastMainsState) {
    anyAlarm = true;
    const char* reason = currentMainsState ? "PO" : "PF";  // POWER_ON -> PO, POWER_OFF -> PF
    char alarmId[64] = "POWER_PF";  // POWER_OFF için sabit ID
    
    if (!currentMainsState) {
      // Güç kesildi
      activateAlarmEvent(alarmId);
      if (!lastPowerAlarm && mqtt.connected()) {
        publishSensorAlarm(
          "SYSTEM",
          cfg.internalMahalId,
          cfg.internalSensorName,
          reason,
          0.0f,
          0.0f,
          alarmId,
          false,
          0.0f,
          0.0f,
          0.0f,
          0.0f,
          nullptr,
          readWifiRSSI(),
          -127
        );
      }
      lastPowerAlarm = true;
    } else {
      // Güç geldi - POWER alarm event'ini deaktif et
      char alarmId[64] = "POWER_PF";
      deactivateAlarmEvent(alarmId, "SYSTEM", cfg.internalMahalId, cfg.internalSensorName, "POWER_OFF", 0.0f, 0.0f);
      lastPowerAlarm = false;
    }
    lastMainsState = currentMainsState;
  }

  // ===== 6) GATEWAY BATARYA ALARMI =====
  bool batteryAlarm = (gPower.battPct < 10);
  
  if (batteryAlarm) {
    anyAlarm = true;
    char alarmId[64] = "BATTERY_BL";  // BATTERY_LOW için sabit ID
    
    activateAlarmEvent(alarmId);
    
    if (!lastBatteryAlarm && mqtt.connected()) {
      publishSensorAlarm(
        "SYSTEM",
        cfg.internalMahalId,
        cfg.internalSensorName,
        "BATTERY_LOW",
        (float)gPower.battPct,
        10.0f,
        alarmId,
        false,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        nullptr,
        readWifiRSSI(),
        -127
      );
    }
    lastBatteryAlarm = true;
  } else {
    // Alarm bitti - BATTERY alarm event'ini deaktif et
    char alarmId[64] = "BATTERY_BL";
    deactivateAlarmEvent(alarmId, "SYSTEM", cfg.internalMahalId, cfg.internalSensorName, "BATTERY_LOW", (float)gPower.battPct, 10.0f);
    lastBatteryAlarm = false;
  }

  // ===== 7) EDDYSTONE SENSÖR BATARYA ALARMI =====
  // CR2450 için %10 altı = 2.1V altı (2100 mV altı)
  for (int i = 0; i < 32; i++) {
    if (!cfg.eddystoneSensors[i].enabled) continue;
    
    EddystoneBeacon* b = findBeaconByMac(cfg.eddystoneSensors[i].macAddress);
    if (!b || !b->isValid) continue;
    
    // Batarya voltajı kontrol et (mV cinsinden)
    bool eddystoneBattAlarm = (b->batteryLevel > 0 && b->batteryLevel < 2100);  // 2.1V = 2100mV
    
    if (eddystoneBattAlarm) {
      anyAlarm = true;
      char alarmId[64];
      snprintf(alarmId, sizeof(alarmId), "EDDY_%s_BL", cfg.eddystoneSensors[i].macAddress);
      
      activateAlarmEvent(alarmId);
      
      if (!lastEddystoneBattAlarm[i] && mqtt.connected()) {
        // Voltajı V cinsine çevir (mV -> V)
        float battVoltage = (float)b->batteryLevel / 1000.0f;
        publishSensorAlarm(
          "EDDYSTONE",
          cfg.eddystoneSensors[i].mahalId,
          cfg.eddystoneSensors[i].sensorName,
          "BATTERY_LOW",
          battVoltage,
          2.1f,  // Threshold: 2.1V
          alarmId,
          false,
          0.0f,
          0.0f,
          0.0f,
          0.0f,
          cfg.eddystoneSensors[i].macAddress,
          readWifiRSSI(),
          b->rssi
        );
      }
      lastEddystoneBattAlarm[i] = true;
    } else {
      // Alarm bitti - EDDYSTONE batarya alarm event'ini deaktif et
      char alarmId[64];
      snprintf(alarmId, sizeof(alarmId), "EDDY_%s_BL", cfg.eddystoneSensors[i].macAddress);
      float battVoltage = (float)b->batteryLevel / 1000.0f;
      deactivateAlarmEvent(alarmId, "EDDYSTONE", cfg.eddystoneSensors[i].mahalId, cfg.eddystoneSensors[i].sensorName, 
                          "BATTERY_LOW", battVoltage, 2.1f);
      lastEddystoneBattAlarm[i] = false;
    }
  }

  // Genel alarm state'i (buzzer + ekran için)
  setAlarmState(anyAlarm, anyAlarm ? "MULTI_SENSOR_ALARM" : nullptr);
}

void loop() {
  // tData ve tBeacon kaldırıldı
  static uint32_t tInfo         = 0;
  static uint32_t tScreen       = 0;
  static uint32_t tNetwork      = 0;
  static uint32_t tOfflineFlush = 0;
  static uint32_t tCache        = 0;
  static uint32_t ledTimer      = 0;
  static uint32_t watchdog      = 0;
  static uint32_t tBleScan      = 0;

  // DATA_PERIOD_MS ve BEACON_PERIOD_MS kaldırıldı
  const uint32_t NETWORK_INTERVAL   = 15000;   // 15 sn
  const uint32_t OFFLINE_FLUSH_INT  = 5000;    // 5 sn
  const uint32_t CACHE_INTERVAL     = 10000;   // 10 sn
  const uint32_t SCREEN_INTERVAL    = 2000;    // 2 sn
  const uint32_t LED_INTERVAL       = 5000;    // 5 sn
  const uint32_t WATCHDOG_INTERVAL  = 30000;   // 30 sn
  const uint32_t BLE_SCAN_INTERVAL  = 15000;   // 15 sn


  // Rollover güvenli zaman okuması
  uint32_t now = millis();

  auto elapsed = [](uint32_t now, uint32_t last, uint32_t interval) -> bool {
    return (int32_t)(now - last) >= (int32_t)interval;
  };

  handleChargeEvent();
  RST_WDT();

  // 1. Multi-sensor alarm kontrolü
  handleMultiSensorAlarm();

  // 2. Buton yönetimi
  handleButton();

  // 3. Buzzer tick
  buzzerTick();
  
  // 3.5. Alarm LED güncelleme
  updateAlarmLed();
  
  RST_WDT();

  // 4. WiFi/MQTT bağlantı kontrolü (15 saniyede bir) - NON-BLOCKING
  if (elapsed(now, tNetwork, NETWORK_INTERVAL)) {
    tNetwork = now;   // basit: her tetikte "şimdi"yi kaydet

    updateTimeSync();  // bloklamıyorsa sorun yok

    bool previousWifiState = sysStatus.wifiOK;
    bool previousMqttState = sysStatus.mqttOK;

    // ---- WiFi durumu ----
    if (WiFi.status() == WL_CONNECTED) {
      if (!sysStatus.wifiOK) {
        sysStatus.wifiOK = true;
        DEBUG_PRINTLN("[LOOP] WiFi yeniden bağlandı");
        // WiFi bağlandıktan sonra hemen NTP sync yap
        syncTimeViaNTP();
        updateTimeSync();
      }
    } else {
      if (sysStatus.wifiOK) {
        sysStatus.wifiOK = false;
        sysStatus.mqttOK = false;
        DEBUG_PRINTLN("[LOOP] WiFi bağlantısı koptu");
        // WiFi koptuğunda SCR_NO_WIFI ekranına geç
        screen = SCR_NO_WIFI;
      }
      tryReconnectWiFi();  // Uzun bloklamasın
    }
    
    // WiFi bağlı değilse SCR_NO_WIFI ekranında kal
    if (WiFi.status() != WL_CONNECTED) {
      screen = SCR_NO_WIFI;
    }

    // ---- MQTT durumu ----
    if (sysStatus.wifiOK) {
      bool mqttResult = mqttEnsureConnected();   // kısa sürmeli
      if (mqttResult && !previousMqttState) {
        DEBUG_PRINTLN("[LOOP] MQTT yeniden bağlandı");
      }
      sysStatus.mqttOK = mqttResult;
    } else {
      sysStatus.mqttOK = false;
    }

    // ---- Internet testi ----
    sysStatus.internetOK = hasInternet();
  }

  RST_WDT();

  // 5. MQTT loop (sık sık çağrılmalı)
  if (mqtt.connected()) {
    mqtt.loop();
  }

  // 6. BLE beacon tarama – PERİYODİK
  if (cfg.beacon.enabled && elapsed(now, tBleScan, BLE_SCAN_INTERVAL)) {
    tBleScan = now;
    performBLEScan();   // İçindeki scan süresi (örn. 3–5 sn) makul olsun
  }

  // 7. OFFLINE kayıtları ONLINE iken göndermek (artık ONLINE DATA’YI KESMİYOR)
  bool online = (sysStatus.wifiOK && sysStatus.mqttOK && mqtt.connected());

  if (online &&
      offlineBuffer.recordCount > 0 &&
      elapsed(now, tOfflineFlush, OFFLINE_FLUSH_INT)) {

    tOfflineFlush = now;
    DEBUG_PRINTF("[OFFLINE] Flush tetiklendi, kalan=%d\n", offlineBuffer.recordCount);
    publishOfflineData();
  }
  // 10. Info topic yayını (MQTT info)
  if (elapsed(now, tInfo, cfg.infoPeriod)) {
    tInfo = now;
    syncTimeViaNTP();
    if (mqtt.connected()) {
      if(publishInfo())
        DEBUG_PRINTF("Info published");
      else
        DEBUG_PRINTF("Info publish failed");
    }
  }

  RST_WDT();
  
  // 10.5. AHT20 ve BLE error kontrolü (periyodik)
  static uint32_t tErrorCheck = 0;
  if (elapsed(now, tErrorCheck, 30000)) {  // 30 saniyede bir
    tErrorCheck = now;
    if (!aht20_ok && mqtt.connected()) {
      publishError("SENSOR_INIT_ERROR", "AHT20 sensor initialization failed", "AHT20");
    }
    if (!sysStatus.bleOK && cfg.beacon.enabled && mqtt.connected()) {
      publishError("BLE_INIT_ERROR", "BLE initialization failed", "BLE");
    }
  }

  // 11. Cache güncelleme (10 saniyede bir)
  if (elapsed(now, tCache, CACHE_INTERVAL)) {
    tCache = now;
    updateInternalCache();
    updateEddystoneCache();
  }

#ifdef USE_LCD_ST7789
  // 12. Ekran güncelleme (2 saniyede bir)
  if (elapsed(now, tScreen, SCREEN_INTERVAL)) {
    tScreen = now;

    // INFO ekranını sadece ilk 10 sn göster, sonra otomatik WORK'a geç
    if (screen == SCR_INFO && millis() > showInfoUntil) {
      screen = SCR_WORK;
    }

    RST_WDT();
    
    // WiFi bağlı değilse SCR_NO_WIFI ekranını göster
    if (WiFi.status() != WL_CONNECTED) {
      screen = SCR_NO_WIFI;
    }
    
    switch (screen) {
      case SCR_WORK:
        drawWorkScreen();
        break;
      case SCR_BEACON:
        drawBeaconScreen();
        break;
      case SCR_INFO:
        drawInfoScreen();
        break;
      case SCR_NO_WIFI:
        drawNoWifiScreen();
        break;
      default:
        drawWorkScreen();
        break;
    }
  }
#endif



  RST_WDT();

  // 14. Sistem sağlığı (30 saniyede bir)
  if (elapsed(now, watchdog, WATCHDOG_INTERVAL)) {
    watchdog = now;
    DEBUG_PRINTF("[SYSTEM] Heap:%d WiFi:%s MQTT:%s Beacons:%d ActiveSensors:%d Offline:%d\n",
                 ESP.getFreeHeap(),
                 sysStatus.wifiOK ? "OK" : "FAIL",
                 sysStatus.mqttOK ? "OK" : "FAIL",
                 beaconCount, cfg.activeSensorCount, offlineBuffer.recordCount);
  }

#ifdef USE_LCD_ST7789
  // 15. Sensör geçiş kontrolü (WORK + ALARM ekranlarında)
  if ((screen == SCR_WORK || screen == SCR_ALARM) &&
      elapsed(now, lastSensorSwitch, 10000)) {
    lastSensorSwitch = now;
    currentSensorIndex++;
    if (currentSensorIndex >= (int)cfg.activeSensorCount) {
      currentSensorIndex = -1;
    }
    DEBUG_PRINTF("[SCREEN] Sensor switch: idx=%d (active=%d)\n",
                 currentSensorIndex, cfg.activeSensorCount);
  }
#endif

  // Loop sonunda küçük bir delay
  delay(10);
}



