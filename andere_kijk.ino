#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WalterModem.h>
#include <esp_mac.h>
#include <SPI.h>
#include <SD.h>
#include <esp_task_wdt.h>

// Configuratie
struct Config {
  static constexpr const char* MQTT_SERVER = "mqtt.iot-ap.be";
  static constexpr int MQTT_PORT = 1883;
  static constexpr int MQTT_TIMEOUT = 30000;  // 30 seconden
  static constexpr int RX_PIN = 44;
  static constexpr int TX_PIN = 43;
  static constexpr int DE_RE_PIN = 4;
  static constexpr int SDA_PIN = 8;
  static constexpr int SCL_PIN = 9;
  static constexpr int CS_PIN = 10;  // Voor SD-kaart
  static constexpr int MAX_PUBLISH_FAILS = 5;
  static constexpr int MAX_RETRIES = 3;
  static constexpr int WATCHDOG_TIMEOUT = 10;  // 10 seconden
  static constexpr const char* MQTT_CLIENT_ID = "em7IYh75xhnrEI7OTkr03FWm";
};

// Systeemstatus
struct SystemHealth {
  bool bme280Ok = false;
  bool sdCardOk = false;
  bool modemOk = false;
  bool mqttOk = false;
};

// Weergegevens structuur
struct WeatherData {
  float temperature = 0.0;
  float humidity = 0.0;
  float windSpeed = 0.0;
  float windGust = 0.0;
  float blackGlobe = 0.0;
  float rainRate = 0.0;
  int windDirection = 0;
  int rain = 0;
  int pressure = 0;

  struct Averages {
    float temp = 0.0;
    float humid = 0.0;
    float windSpeed = 0.0;
    float blackGlobe = 0.0;
    float sumTemp = 0.0;
    float sumHumid = 0.0;
    float sumWindSpeed = 0.0;
    float sumBlackGlobe = 0.0;
    float maxWindSpeed = 0.0;
    int readings = 0;
  } averages;

  int windDirHistogram[36] = { 0 };
};

// Sensorbeheer
class SensorManager {
public:
  SensorManager(HardwareSerial& rs485Serial)
    : rs485Serial(rs485Serial) {}

  bool begin() {
    Wire.begin(Config::SDA_PIN, Config::SCL_PIN);
    rs485Serial.begin(4800, SERIAL_8N1, Config::RX_PIN, Config::TX_PIN);
    pinMode(Config::DE_RE_PIN, OUTPUT);
    digitalWrite(Config::DE_RE_PIN, LOW);  // Ontvangstmodus

    // Probeer BME280 te initialiseren
    int retries = 0;
    while (!bme.begin(BME280_ADDRESS) && retries < Config::MAX_RETRIES) {
#if DEBUG
      Serial.println("BME280 initialisatie mislukt, opnieuw proberen...");
#endif
      delay(2000);
      retries++;
    }
    if (retries >= Config::MAX_RETRIES) {
#if DEBUG
      Serial.println("BME280 initialisatie mislukt, ga verder zonder sensor.");
#endif
      return false;
    }

    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);
#if DEBUG
    Serial.println("BME280 succesvol geïnitialiseerd!");
#endif
    bmeInitialized = true;
    return true;
  }

  void readWeatherData(WeatherData& data) {
    if (rs485Serial.available() < 6) return;

    unsigned char wxdata[8] = { 0 };
    for (uint8_t i = 0; i < 6; i++) {
      wxdata[i] = rs485Serial.read();
    }

#if DEBUG
    Serial.print("Ontvangen data: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(wxdata[i], HEX);
      Serial.print(" ");
    }
    Serial.print("Type: ");
    Serial.println(wxdata[0] >> 4, HEX);
#endif

    // Verwerk windrichting
    float windDir = (wxdata[2] * 359.0 / 255.0);
    data.windDirection = round(windDir);
    int index = round(windDir / 10.0);
    if (index >= 0 && index < 36) {
      data.windDirHistogram[index]++;
    }

    // Verwerk windsnelheid
    data.windSpeed = wxdata[1] * 1.609;
    if (data.windSpeed < 0 || data.windSpeed > 140) {
      data.windSpeed = data.averages.maxWindSpeed;
    }

    // Verwerk luchtdruk
    if (bmeOk()) {
      data.pressure = bme.readPressure();
    }

    // Verwerk gegevens op basis van type
    switch (wxdata[0] >> 4) {
      case 5:  // Regenintensiteit
        data.rainRate = (wxdata[4] & 0x40) ? 11520.0 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]) : 720.0 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]);
        break;
      case 8:  // Temperatuur
        data.temperature = (((wxdata[3] * 0xff + wxdata[4]) / 160.0) - 32.0) / 1.8;
        if (data.temperature < -20 || data.temperature > 60) {
          data.temperature = data.averages.temp;
#if DEBUG
          Serial.println("Temperature out of range, using average");
#endif
        }
        break;
      case 9:  // Windstoot
        data.windGust = wxdata[3] * 1.609;
        break;
      case 10:  // Luchtvochtigheid
        data.humidity = (((wxdata[4] >> 4) << 8) + wxdata[3]) / 10.0;
        if (data.humidity < 0 || data.humidity > 100) {
          data.humidity = data.averages.humid;
#if DEBUG
          Serial.println("Humidity out of range, using average");
#endif
        }
        break;
      case 14:  // Regen (totaal)
        data.rain = wxdata[3];
        break;
      default:
#if DEBUG
        Serial.println("Onbekend gegevenstype ontvangen!");
#endif
        break;
    }

    // Update gemiddelden
    data.averages.sumTemp += data.temperature;
    data.averages.sumHumid += data.humidity;
    data.averages.sumWindSpeed += data.windSpeed;
    data.averages.maxWindSpeed = max(data.averages.maxWindSpeed, data.windSpeed);
    data.averages.readings++;

#if DEBUG
    Serial.print("Wind: ");
    Serial.print(data.windSpeed);
    Serial.print(" km/h, Temp: ");
    Serial.print(data.temperature);
    Serial.print(" °C, Vocht: ");
    Serial.print(data.humidity);
    Serial.print(" %, Regen: ");
    Serial.print(data.rain);
    Serial.print(" mm, Windstoot: ");
    Serial.print(data.windGust);
    Serial.print(" km/h, Luchtdruk: ");
    Serial.print(data.pressure);
    Serial.println(" Pa");
#endif

    // Leeg de buffer
    while (rs485Serial.available()) rs485Serial.read();
  }

  bool bmeOk() const {
    return bmeInitialized;
  }

private:
  Adafruit_BME280 bme;
  HardwareSerial& rs485Serial;
  bool bmeInitialized = false;
};

// Communicatiebeheer
class CommunicationManager {
public:
  bool begin() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(macString, sizeof(macString), "walter%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    int retries = 0;
    while (!WalterModem::begin(&Serial2) && retries < Config::MAX_RETRIES) {
#if DEBUG
      Serial.println("Modem initialisatie mislukt, opnieuw proberen...");
#endif
      delay(2000);
      retries++;
    }
    if (retries >= Config::MAX_RETRIES) {
#if DEBUG
      Serial.println("Modem initialisatie mislukt!");
#endif
      return false;
    }

    return connectLTE();
  }

  bool connectLTE() {
    int retries = 0;
    while (retries < Config::MAX_RETRIES) {
      if (modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
#if DEBUG
        Serial.println("Successfully set operational state to NO RF");
#endif
      } else {
#if DEBUG
        Serial.println("Error: Could not set operational state to NO RF");
#endif
        retries++;
        delay(2000 * (retries + 1));
        continue;
      }

      if (modem.definePDPContext()) {
#if DEBUG
        Serial.println("Created PDP context");
#endif
      } else {
#if DEBUG
        Serial.println("Error: Could not create PDP context");
#endif
        retries++;
        delay(2000 * (retries + 1));
        continue;
      }

      if (modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
#if DEBUG
        Serial.println("Successfully set operational state to FULL");
#endif
      } else {
#if DEBUG
        Serial.println("Error: Could not set operational state to FULL");
#endif
        retries++;
        delay(2000 * (retries + 1));
        continue;
      }

      if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
#if DEBUG
        Serial.println("Network selection mode was set to automatic");
#endif
      } else {
#if DEBUG
        Serial.println("Error: Could not set the network selection mode to automatic");
#endif
        retries++;
        delay(2000 * (retries + 1));
        continue;
      }

      if (waitForNetwork()) {
        if (modem.mqttConfig(Config::MQTT_CLIENT_ID)) {
#if DEBUG
          Serial.println("MQTT configuration succeeded");
#endif
          return connectMQTT();
        } else {
#if DEBUG
          Serial.println("Error: MQTT configuration failed");
#endif
          retries++;
          delay(2000 * (retries + 1));
        }
      } else {
        retries++;
#if DEBUG
        Serial.print("Network connection attempt failed, retry ");
        Serial.print(retries);
        Serial.print(" of ");
        Serial.println(Config::MAX_RETRIES);
#endif
        delay(2000 * (retries + 1));
      }
    }

#if DEBUG
    Serial.println("All LTE connection attempts failed, resetting modem...");
#endif
    resetModem();
    return false;
  }

  bool connectMQTT() {
    if (!lteConnected()) {
#if DEBUG
      Serial.println("LTE not connected, attempting to reconnect...");
#endif
      if (!connectLTE()) {
#if DEBUG
        Serial.println("LTE reconnection failed");
#endif
        return false;
      }
    }

    if (modem.mqttConnect(Config::MQTT_SERVER, Config::MQTT_PORT, Config::MQTT_TIMEOUT)) {
#if DEBUG
      Serial.println("MQTT connection succeeded");
#endif
      publishFailCount = 0;
      return true;
    }

#if DEBUG
    Serial.println("Error: MQTT connection failed");
#endif
    return false;
  }

  bool publishData(const WeatherData& data) {
    String payload = "{\"temp\":" + String(data.temperature, 1) + ",\"humid\":" + String(data.humidity, 1) + ",\"pressure\":" + String(data.pressure) + ",\"windSpeed\":" + String(data.windSpeed, 1) + ",\"windDir\":" + String(data.windDirection) + ",\"rain\":" + String(data.rain) + ",\"windGust\":" + String(data.windGust, 1) + ",\"blackGlobe\":" + String(data.blackGlobe, 1) + "}";
    bool success = modem.mqttPublish((String(Config::MQTT_CLIENT_ID) + "/data").c_str(),
                                     (uint8_t*)payload.c_str(), payload.length());
    if (!success) {
      publishFailCount++;
#if DEBUG
      Serial.print("MQTT publish failed, failure count: ");
      Serial.println(publishFailCount);
#endif
      if (publishFailCount >= Config::MAX_PUBLISH_FAILS) {
#if DEBUG
        Serial.println("Maximum publish failures reached, restarting device...");
#endif
        restartDevice();
      }
      return false;
    }
    publishFailCount = 0;
    return true;
  }

  bool lteConnected() {
    WalterModemNetworkRegState regState = modem.getNetworkRegState();
    return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME || regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
  }

private:
  bool waitForNetwork() {
    int timeout = 0;
    const int maxTimeout = 180000;
    while (!lteConnected()) {
      delay(100);
      timeout += 100;
      if (timeout > maxTimeout) {
#if DEBUG
        Serial.println("Network connection timeout");
#endif
        return false;
      }
    }
    int rssi = modem.getSignalQuality();
#if DEBUG
    Serial.print("Signal quality (RSSI): ");
    Serial.println(rssi);
#endif
    if (rssi <= 0 || rssi > 31) {
#if DEBUG
      Serial.println("Poor or no signal detected");
#endif
      return false;
    }
#if DEBUG
    Serial.println("Connected to the network with good signal");
#endif
    return true;
  }

  void resetModem() {
#if DEBUG
    Serial.println("Resetting modem...");
#endif
    modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF);
    delay(5000);
    WalterModem::begin(&Serial2);
  }

  void restartDevice() {
#if DEBUG
    Serial.println("Restarting device...");
#endif
    delay(1000);
    ESP.restart();
  }

  WalterModem modem;
  char macString[32];
  int publishFailCount = 0;
};

// Opslagbeheer
class StorageManager {
public:
  bool begin() {
    SPI.begin(12, 13, 11, Config::CS_PIN);
    int retries = 0;
    while (!SD.begin(Config::CS_PIN) && retries < Config::MAX_RETRIES) {
#if DEBUG
      Serial.println("SD card initialization failed, retrying...");
#endif
      delay(1000);
      retries++;
    }
    if (retries >= Config::MAX_RETRIES) {
#if DEBUG
      Serial.println("SD card initialization failed!");
#endif
      return false;
    }
#if DEBUG
    Serial.println("SD card initialized successfully.");
#endif
    writeInitialLog();
    return true;
  }

  void logData(const WeatherData& data, unsigned long currentTime) {
    String logEntry = "[" + String(currentTime / 1000) + "] ";
    logEntry += "Temp=" + String(data.temperature, 1) + " °C, ";
    logEntry += "Humidity=" + String(data.humidity, 1) + " %, ";
    logEntry += "Pressure=" + String(data.pressure) + " Pa, ";
    logEntry += "WindSpeed=" + String(data.windSpeed, 1) + " km/h, ";
    logEntry += "WindDir=" + String(data.windDirection) + " deg, ";
    logEntry += "Rain=" + String(data.rain) + " mm, ";
    logEntry += "WindGust=" + String(data.windGust, 1) + " km/h, ";
    logEntry += "BlackGlobe=" + String(data.blackGlobe, 1) + " °C";

    int retries = 0;
    bool success = false;
    while (!success && retries < Config::MAX_RETRIES) {
      File logFile = SD.open("/log.txt", FILE_APPEND);
      if (logFile) {
        logFile.println(logEntry);
        logFile.close();
#if DEBUG
        Serial.println("Wrote to log.txt: " + logEntry);
#endif
        success = true;
      } else {
#if DEBUG
        Serial.println("Error: Failed to open log.txt, retrying...");
#endif
        delay(500);
        retries++;
      }
    }
  }

private:
  void writeInitialLog() {
    File logFile = SD.open("/log.txt", FILE_WRITE);
    if (logFile) {
      logFile.println("Log file created on Walter v1.6");
      logFile.close();
#if DEBUG
      Serial.println("Initial message written to log.txt");
#endif
    }
  }
};

// Globale objecten
HardwareSerial RS485Serial(1);
SensorManager sensorManager(RS485Serial);
CommunicationManager communicationManager;
StorageManager storageManager;
SystemHealth health;
WeatherData weather;

// State machine
enum class SystemState {
  INITIALIZING,
  RUNNING,
  ERROR
};
SystemState currentState = SystemState::INITIALIZING;

void setup() {
  // Stabilisatievertraging
  delay(5000);

#if DEBUG
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Setup gestart...");
#endif

  // Initialiseer watchdog
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = Config::WATCHDOG_TIMEOUT * 1000,  // Convert to milliseconds
    .idle_core_mask = 0,                            // Apply to all cores
    .trigger_panic = true                           // Trigger panic on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  // Initialiseer componenten
  randomSeed(analogRead(0));
  health.bme280Ok = sensorManager.begin();
  health.sdCardOk = storageManager.begin();
  health.modemOk = communicationManager.begin();
  health.mqttOk = health.modemOk && communicationManager.connectMQTT();

  currentState = (health.bme280Ok || health.modemOk) ? SystemState::RUNNING : SystemState::ERROR;
  esp_task_wdt_reset();
}

void loop() {
  esp_task_wdt_reset();
  static unsigned long lastUpdate = 0;

  switch (currentState) {
    case SystemState::RUNNING:
      if (millis() - lastUpdate >= 60000) {
        sensorManager.readWeatherData(weather);
        if (health.mqttOk) {
          if (!communicationManager.publishData(weather)) {
            health.mqttOk = communicationManager.connectMQTT();
          }
        }
        if (health.sdCardOk) {
          storageManager.logData(weather, millis());
        }


        $0 lastUpdate = millis();
      }
      break;
    case SystemState::ERROR:
      // Poging tot herstel
      health.bme280Ok = sensorManager.begin();
      health.sdCardOk = storageManager.begin();
      health.modemOk = communicationManager.begin();
      health.mqttOk = health.modemOk && communicationManager.connectMQTT();
      if (health.bme280Ok || health.modemOk) {
        currentState = SystemState::RUNNING;
      } else {
#if DEBUG
        Serial.println("Herstel mislukt, herstart apparaat...");
#endif
        delay(1000);
        ESP.restart();
      }
      break;
    default:
      break;
  }

  delay(100);  // Korte pauze om CPU te sparen
}
