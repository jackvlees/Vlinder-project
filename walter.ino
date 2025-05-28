#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WalterModem.h>
#include <esp_mac.h>
#include <SPI.h>
#include <SD.h>

// BME280 instantie
Adafruit_BME280 bme;

WalterModem modem;   // Modem initialisatie
WalterModemRsp rsp;  // Response model

// Definieer pinnen voor RS485-communicatie
#define RX_PIN 44     // RX-pin voor HardwareSerial
#define TX_PIN 43     // TX-pin voor HardwareSerial
#define DE_RE_PIN 4   // Data Enable/Receive Enable pin voor RS485-module

#define DEBUG 0 // Comment this line to disable debug output
// Standaard I2C-adres van BME280
#define BME280_ADDRESS 0x77

#define SDA_PIN 8
#define SCL_PIN 9

// HardwareSerial voor RS485-communicatie
HardwareSerial RS485Serial(1);

uint8_t incomingBuf[256] = {0};  // Buffer
char macString[32];
static int publishFailCount = 0;

// Buffer voor inkomende gegevens
unsigned char wxdata[8] = {0};

// Initialize SPI for ESP32-S3
void initializeSPI() {
  SPI.begin(12, 13, 11, 10);  // SCK=12, MISO=13, MOSI=11, CS=10
}

// Initialize SD card with retry logic
bool initializeSDCard() {
  int retries = 0;
  while (!SD.begin(10) && retries < 3) {
#if DEBUG
    Serial.println("Initialization failed. Things to check:");
    Serial.println("* Is the MicroSD card properly inserted?");
    Serial.println("* Is the card formatted as FAT16 or FAT32?");
    Serial.println("* Are the SPI pins correctly connected?");
    Serial.println("* Is the chipSelect pin set correctly?");
#endif
    delay(1000);
    retries++;
  }
  if (retries >= 3) {
#if DEBUG
    Serial.println("SD card initialization failed!");
#endif
    return false;
  }
#if DEBUG
  Serial.println("SD card initialized successfully.");
#endif
  return true;
}

// Write initial message to log file
void writeInitialLog() {
  File logFile = SD.open("/log.txt", FILE_WRITE);
  if (logFile) {
#if DEBUG
    Serial.println("Writing initial message to log.txt...");
#endif
    logFile.println("Log file created on Walter v1.6");
    logFile.close();
#if DEBUG
    Serial.println("Initial message written to log.txt");
#endif
  } else {
#if DEBUG
    Serial.println("Error: Failed to open log.txt for writing");
#endif
  }
}

// Definieer de WeatherData-structuur
struct WeatherData {
  float temperature;
  float humidity;
  float windSpeed;
  float windGust;
  float blackGlobe;
  float rainRate;
  int windDirection;
  int rain;
  int pressure;

  struct Averages {
    float temp;
    float humid;
    float windSpeed;
    float blackGlobe;
    float sumTemp;
    float sumHumid;
    float sumWindSpeed;
    float sumBlackGlobe;
    float maxWindSpeed;
    int readings;
  } averages;

  int windDirHistogram[36];
};

// Instantie van WeatherData
WeatherData weather;

// Write periodic log entry with retry logic
void writePeriodicLog(unsigned long currentTime) {
  int retries = 0;
  bool success = false;
  while (!success && retries < 3) {
    File logFile = SD.open("/log.txt", FILE_APPEND);
    if (logFile) {
      String logEntry = "Log entry at " + String(currentTime / 1000) + " seconds: ";
      logEntry += "Temp=" + String(weather.temperature, 1) + " °C, ";
      logEntry += "Humidity=" + String(weather.humidity, 1) + " %, ";
      logEntry += "Pressure=" + String(weather.pressure) + " Pa, ";
      logEntry += "WindSpeed=" + String(weather.windSpeed, 1) + " km/h, ";
      logEntry += "WindDir=" + String(weather.windDirection) + " deg, ";
      logEntry += "Rain=" + String(weather.rain) + " mm, ";
      logEntry += "WindGust=" + String(weather.windGust, 1) + " km/h, ";
      logEntry += "BlackGlobe=" + String(weather.blackGlobe, 1) + " °C";

      logFile.println(logEntry);
      logFile.close();
#if DEBUG
      Serial.println("Wrote to log.txt: " + logEntry);
#endif
      success = true;
    } else {
#if DEBUG
      Serial.println("Error: Failed to open log.txt for writing, retrying...");
#endif
      delay(500);
      retries++;
    }
  }
  if (!success) {
#if DEBUG
    Serial.println("Fatal: Failed to write to SD card after 3 retries");
#endif
  }
}

// Reset modem to recover from persistent failures
void resetModem() {
#if DEBUG
  Serial.println("Resetting modem...");
#endif
  modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF); // Disable RF
  delay(5000); // Wait longer to ensure modem stabilizes
  if (WalterModem::begin(&Serial2)) {
#if DEBUG
    Serial.println("Modem reinitialized successfully");
#endif
  } else {
#if DEBUG
    Serial.println("Error: Modem reinitialization failed");
#endif
  }
}

bool lteConnected() {  // Controleer verbinding met LTE-netwerk
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME || 
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

bool waitForNetwork() {  // Wacht op verbinding met LTE-netwerk
  int timeout = 0;
  const int maxTimeout = 180000; // Reduced to 3 minutes for faster recovery
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
  // Check signal quality
  int rssi = modem.getSignalQuality();
#if DEBUG
  Serial.print("Signal quality (RSSI): ");
  Serial.println(rssi);
#endif
  if (rssi <= 0 || rssi > 31) { // Invalid or poor signal
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

bool lteConnect() {  // Verbind modem met het cellulaire netwerk
  int retries = 0;
  const int maxRetries = 5; // Increased retries for robustness
  while (retries < maxRetries) {
    // Step 1: Set operational state to NO RF
    if (modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
#if DEBUG
      Serial.println("Successfully set operational state to NO RF");
#endif
    } else {
#if DEBUG
      Serial.println("Error: Could not set operational state to NO RF");
#endif
      retries++;
      delay(2000 * (retries + 1)); // Exponential backoff
      continue;
    }

    // Step 2: Define PDP context
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

    // Step 3: Set operational state to FULL
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

    // Step 4: Set network selection mode to automatic
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

    // Step 5: Wait for network connection
    if (waitForNetwork()) {
      return true;
    } else {
      retries++;
#if DEBUG
      Serial.print("Network connection attempt failed, retry ");
      Serial.print(retries);
      Serial.print(" of ");
      Serial.println(maxRetries);
#endif
      delay(2000 * (retries + 1)); // Exponential backoff
    }
  }

  // If all retries fail, reset modem
#if DEBUG
  Serial.println("All LTE connection attempts failed, resetting modem...");
#endif
  resetModem();
  return false;
}

bool ensureMqttConnected() {  // Probeer verbinding te maken of opnieuw te verbinden met MQTT-broker
  int retries = 0;
  const int maxRetries = 3;
  const int maxTimeout = 30000; // 30 seconds timeout for MQTT connection
  while (retries < maxRetries) {
    // Verify LTE connection before attempting MQTT
    if (!lteConnected()) {
#if DEBUG
      Serial.println("LTE not connected, attempting to reconnect...");
#endif
      if (!lteConnect()) {
#if DEBUG
        Serial.println("LTE reconnection failed");
#endif
        retries++;
        delay(2000 * (retries + 1)); // Exponential backoff
        continue;
      }
    }

    // Attempt MQTT connection
#if DEBUG
    Serial.println("Attempting MQTT connection...");
#endif
    if (modem.mqttConnect("mqtt.iot-ap.be", 1883, maxTimeout)) {
#if DEBUG
      Serial.println("MQTT connection succeeded");
#endif
      publishFailCount = 0;  // Reset fail counter on successful connection
      return true;
    } else {
#if DEBUG
      Serial.println("Error: MQTT connection failed");
#endif
      retries++;
      delay(2000 * (retries + 1)); // Exponential backoff
    }
  }

  // If MQTT connection fails after retries, reset modem
#if DEBUG
  Serial.println("All MQTT connection attempts failed, resetting modem...");
#endif
  resetModem();
  return false;
}

void setup() {
  // Wacht op hardwarestabilisatie
  delay(5000);

  #if DEBUG
    Serial.begin(115200);
    while (!Serial);  // Wacht tot seriële poort klaar is
    Serial.println("Setup gestart...");
  #endif

  // Initialiseer watchdog
  esp_task_wdt_init(10, true); // 10 seconden timeout
  esp_task_wdt_add(NULL);

  esp_read_mac(incomingBuf, ESP_MAC_WIFI_STA);
  sprintf(macString, "walter%02X:%02X:%02X:%02X:%02X:%02X", incomingBuf[0],
          incomingBuf[1], incomingBuf[2], incomingBuf[3], incomingBuf[4],
          incomingBuf[5]);

  // Modem initialisatie met retry-logica
  int modemRetries = 0;
  while (!WalterModem::begin(&Serial2) && modemRetries < 3) {
    #if DEBUG
      Serial.println("Modem initialisatie mislukt, opnieuw proberen...");
    #endif
    delay(2000);
    modemRetries++;
  }
  if (modemRetries >= 3) {
    #if DEBUG
      Serial.println("Modem initialisatie mislukt, herstart apparaat...");
    #endif
    restartDevice();
  }

  if (!lteConnect()) {
    #if DEBUG
      Serial.println("Error: Could Not Connect to LTE");
    #endif
    restartDevice();
  }

  if (modem.mqttConfig("em7IYh75xhnrEI7OTkr03FWm")) {
    #if DEBUG
      Serial.println("MQTT configuration succeeded");
    #endif
  } else {
    #if DEBUG
      Serial.println("Error: MQTT configuration failed");
    #endif
    restartDevice();
  }

  if (!ensureMqttConnected()) {
    #if DEBUG
      Serial.println("Error: Initial MQTT connection failed");
    #endif
    restartDevice();
  }

  randomSeed(analogRead(0));  // Initialiseer random seed

  // Start I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Start HardwareSerial voor RS485
  RS485Serial.begin(4800, SERIAL_8N1, RX_PIN, TX_PIN);
  #if DEBUG
    Serial.println("RS485Serial gestart op 4800 baud");
  #endif

  // Stel DE/RE-pin in als uitvoer
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);  // Zet in ontvangstmodus
  #if DEBUG
    Serial.println("RS485 in ontvangstmodus");
  #endif

  // Probeer BME280 te initialiseren
  if (!bme.begin(BME280_ADDRESS)) {
    #if DEBUG
      Serial.println("BME280 niet gevonden! Probeer opnieuw...");
    #endif
    delay(2000);
    if (!bme.begin(BME280_ADDRESS)) {
      #if DEBUG
        Serial.println("BME280 initialisatie mislukt, ga verder zonder sensor.");
      #endif
    }
  } else {
    #if DEBUG
      Serial.println("BME280 succesvol geïnitialiseerd!");
    #endif
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);
  }

  initializeSPI();
  if (!initializeSDCard()) {
    #if DEBUG
      Serial.println("SD-kaart initialisatie mislukt, ga verder zonder SD-kaart.");
    #endif
  } else {
    writeInitialLog();
  }

  esp_task_wdt_reset(); // Reset watchdog na setup
}

void processWeatherData() {
  // Controleer of er minimaal 6 bytes beschikbaar zijn
  if (RS485Serial.available() < 6) return;

  // Lees de 6 bytes in
  for (uint8_t i = 0; i < 6; i++) {
    wxdata[i] = RS485Serial.read();
  }

  // Debugging: print ontvangen ruwe data
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
  float windDir = (wxdata[2] * 359.0 / 255.0);  // Schaal naar 0-359 graden
  int index = round(windDir / 10.0);            // Histogramindex (0-35)
  if (index >= 0 && index < 36) {
    weather.windDirHistogram[index]++;
  }

  // Verwerk windsnelheid (in km/h)
  weather.windSpeed = wxdata[1] * 1.609;  // Omzetten naar km/h
  if (weather.windSpeed < 0 || weather.windSpeed > 140) {
    weather.windSpeed = weather.averages.maxWindSpeed;
  }

  // Verwerk luchtdruk in Pa
  weather.pressure = bme.readPressure();

  // Verwerk de gegevens op basis van het type (eerste 4 bits van wxdata[0])
  switch (wxdata[0] >> 4) {
    case 5:  // Regenintensiteit (rain rate)
      weather.rainRate = (wxdata[4] & 0x40) ? 11520.0 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]) : 720.0 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]);
      break;

    case 8:  // Temperatuur
      weather.temperature = (((wxdata[3] * 0xff + wxdata[4]) / 160.0) - 32.0) / 1.8;  // Vlinder-conversie
      if (weather.temperature < -20 || weather.temperature > 60) {
        weather.temperature = weather.averages.temp;
#if DEBUG
        Serial.println("Temperature out of range, using average");
#endif
      }
      break;

    case 9:  // Windstoot (gust)
      weather.windGust = wxdata[3] * 1.609;
      break;

    case 10:  // Luchtvochtigheid
      weather.humidity = (((wxdata[4] >> 4) << 8) + wxdata[3]) / 10.0;
      if (weather.humidity < 0 || weather.humidity > 100) {
        weather.humidity = weather.averages.humid;
#if DEBUG
        Serial.println("Humidity out of range, using average");
#endif
      }
      break;

    case 14:  // Regen (totaal)
      weather.rain = wxdata[3];
      break;

    default:
#if DEBUG
      Serial.println("Onbekend gegevenstype ontvangen!");
#endif
      break;
  }

  // Update gemiddelden
  weather.averages.sumTemp += weather.temperature;
  weather.averages.sumHumid += weather.humidity;
  weather.averages.sumWindSpeed += weather.windSpeed;
  weather.averages.maxWindSpeed = max(weather.averages.maxWindSpeed, weather.windSpeed);
  weather.averages.readings++;

  // Print de verwerkte gegevens
#if DEBUG
  Serial.print("Wind: ");
  Serial.print(weather.windSpeed);
  Serial.print(" km/h, Temp: ");
  Serial.print(weather.temperature);
  Serial.print(" °C, Vocht: ");
  Serial.print(weather.humidity);
  Serial.print(" %, Regen: ");
  Serial.print(weather.rain);
  Serial.print(" mm, Windstoot: ");
  Serial.print(weather.windGust);
  Serial.print(" km/h, Luchtdruk: ");
  Serial.print(weather.pressure);
  Serial.println(" Pa");
#endif

  // Publiceer naar MQTT
  bool publishSuccess = true;
  char outgoingMsg[64];
  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", weather.temperature);
  if (!modem.mqttPublish("em7IYh75xhnrEI7OTkr03FWm/temperature", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
#if DEBUG
    Serial.println("MQTT publish temperature failed");
#endif
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", weather.humidity);
  if (!modem.mqttPublish("em7IYh75xhnrEI7OTkr03FWm/humidity", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
#if DEBUG
    Serial.println("MQTT publish humidity failed");
#endif
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%d", weather.pressure);
  if (!modem.mqttPublish("em7IYh75xhnrEI7OTkr03FWm/pressure", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
#if DEBUG
    Serial.println("MQTT publish pressure failed");
#endif
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", weather.windSpeed);
  if (!modem.mqttPublish("em7IYh75xhnrEI7OTkr03FWm/windSpeed", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
#if DEBUG
    Serial.println("MQTT publish windSpeed failed");
#endif
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%d", weather.windDirection);
  if (!modem.mqttPublish("em7IYh75xhnrEI7OTkr03FWm/windDirection", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
#if DEBUG
    Serial.println("MQTT publish windDirection failed");
#endif
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%d", weather.rain);
  if (!modem.mqttPublish("em7IYh75xhnrEI7OTkr03FWm/rain", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
#if DEBUG
    Serial.println("MQTT publish rain failed");
#endif
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", weather.windGust);
  if (!modem.mqttPublish("em7IYh75xhnrEI7OTkr03FWm/windGust", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
#if DEBUG
    Serial.println("MQTT publish windGust failed");
#endif
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", weather.blackGlobe);
  if (!modem.mqttPublish("em7IYh75xhnrEI7OTkr03FWm/blackGlobe", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
#if DEBUG
    Serial.println("MQTT publish blackGlobe failed");
#endif
    publishSuccess = false;
  }

  if (!publishSuccess) {
    publishFailCount++;
#if DEBUG
    Serial.print("Publish failure count: ");
    Serial.println(publishFailCount);
#endif
    if (publishFailCount >= 3) {
#if DEBUG
      Serial.println("Too many publish failures, attempting MQTT reconnect...");
#endif
      if (!ensureMqttConnected()) {
#if DEBUG
        Serial.println("Reconnection failed, will retry next loop");
#endif
        writePeriodicLog(millis()); // Forceer SD-back-up bij falen
      }
    }
  } else {
    publishFailCount = 0;
    writePeriodicLog(millis()); // Schrijf naar SD-kaart na succesvolle publicatie
  }

  // Leeg de buffer om geen oude gegevens te verwerken
  while (RS485Serial.available()) RS485Serial.read();
}

void loop() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 60000) {
    if (RS485Serial.available() > 0) {
      processWeatherData();
    }
    lastUpdate = millis();
  }
  delay(100); // Kortere delay om CPU te sparen
}
