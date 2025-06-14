/*
  Vlinder MKR Arduino MQTT communication
  V7 13/03/25 - Improved Arduino-compatible version with float formatting fix
  Updated: No dummy values, sends 0 when no measurements are available
*/

#define DEBUG 1          // Debug mode: 1 = on, 0 = off
#define BLACKGLOBE 0     // BlackGlobe mode: 1 = on, 0 = off

#include <MKRNB.h>
#include "keys.h"
#include <Arduino_MKRENV.h>
#include <ArduinoRS485.h>
#include <SPI.h>
#include <SD.h>
#include <PubSubClient.h>
#if BLACKGLOBE
#include <Adafruit_MCP9600.h>
#endif

// Constants
const uint8_t I2C_ADDRESS = 0x67;
const uint8_t SETUP_LED = 6;
const uint8_t SD_CS_PIN = 4;
const uint16_t SEND_INTERVAL = 30;  // Seconds between data transmissions
const char* MQTT_SERVER = "mqtt.iot-ap.be";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASSWORD = "";

// Hardware objects
NB nbAccess;
GPRS gprs;
NBClient nbClient;
PubSubClient mqttClient(nbClient);
#if BLACKGLOBE
Adafruit_MCP9600 mcp;
#endif
File dataFile;

// Weather data structure
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

// Global variables
WeatherData weather = {0};  // Initialize all to zero
unsigned long previousMillis = 0;
unsigned long lastSuccess = 0;
unsigned char wxdata[8] = {0};
uint32_t timestamp = 0;

void flashLED(uint8_t times, uint16_t duration) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(duration);
    digitalWrite(LED_BUILTIN, LOW);
    delay(duration);
  }
}

boolean startNetwork() {
#if DEBUG
  Serial.println("Initializing network...");
#endif
  if (nbAccess.begin() != NB_READY || gprs.attachGPRS() != GPRS_READY) {
#if DEBUG
    Serial.println("Network connection failed!");
#endif
    return false;
  }
#if DEBUG
  Serial.println("Network connected successfully!");
#endif
  return true;
}

void ensureNetworkConnection() {
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
#if DEBUG
    Serial.println("Network lost, restarting modem...");
#endif
    nbAccess.shutdown();
    while (!startNetwork()) {
      flashLED(10, 200);
      delay(5000);
    }
  }
}

boolean connectMQTT() {
  ensureNetworkConnection();
  const uint8_t MAX_ATTEMPTS = 3;
  uint8_t attempt = 0;

  while (!mqttClient.connected() && attempt < MAX_ATTEMPTS) {
#if DEBUG
    Serial.print("MQTT connection attempt ");
    Serial.print(attempt + 1);
    Serial.println("...");
#endif
    if (mqttClient.connect("VlinderDevice", MQTT_USER, MQTT_PASSWORD)) {
#if DEBUG
      Serial.println("MQTT connected!");
#endif
      return true;
    }
    
#if DEBUG
    Serial.print("MQTT connection failed, state: ");
    Serial.println(mqttClient.state());
#endif
    attempt++;
    if (attempt < MAX_ATTEMPTS) delay(5000);
  }
  
#if DEBUG
  Serial.println("MQTT connection failed after max attempts");
#endif
  return false;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SETUP_LED, OUTPUT);
  digitalWrite(SETUP_LED, LOW);
  flashLED(3, 1000);

#if DEBUG
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting setup...");
#endif

  while (!startNetwork()) {
    flashLED(10, 200);
    delay(5000);
  }

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  connectMQTT();

  RS485.begin(4800);
  RS485.receive();

  if (!ENV.begin()) {
#if DEBUG
    Serial.println("MKR ENV shield initialization failed!");
#endif
    while (1) flashLED(5, 200);
  }

#if BLACKGLOBE
  if (!mcp.begin(I2C_ADDRESS)) {
#if DEBUG
    Serial.println("MCP9600 initialization failed!");
#endif
    while (1) flashLED(7, 100);
  }
  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp.setThermocoupleType(MCP9600_TYPE_K);
  mcp.setFilterCoefficient(3);
  mcp.enable(true);
#endif

  SPI.begin();
  delay(100);
  if (!SD.begin(SD_CS_PIN)) {
#if DEBUG
    Serial.println("SD card initialization failed!");
#endif
    while (1) flashLED(3, 200);
  }

  dataFile = SD.open(VLINDERNR, FILE_WRITE);
  if (dataFile) {
    dataFile.println("timestamp,temperature,humidity,pressure,WindSpeed,WindDirection,Rain,WindGust,BlackGlobe");
    dataFile.close();
  }

#if DEBUG
  Serial.println("Setup completed!");
#endif
  digitalWrite(SETUP_LED, HIGH);
}

void publishMQTT(const char* sensor, float value) {
  char topic[50];
  char payload[10];
  sprintf(topic, "%s/%s", DEVICE_ID, sensor);
  sprintf(payload, "%.2f", value);  // Using sprintf with float format specifier
  mqttClient.publish(topic, payload);
#if DEBUG
  Serial.print("MQTT Publish - ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);
#endif
}

void publishMQTT(const char* sensor, int value) {
  char topic[50];
  char payload[10];
  sprintf(topic, "%s/%s", DEVICE_ID, sensor);
  sprintf(payload, "%d", value);
  mqttClient.publish(topic, payload);
#if DEBUG
  Serial.print("MQTT Publish - ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);
#endif
}

void processWeatherData() {
  if (RS485.available() < 6) return;

  for (uint8_t i = 0; i < 6; i++) {
    wxdata[i] = RS485.read();
  }
  flashLED(1, 30);

  float windDir = (wxdata[2] * 359.0 / 255.0);
  int index = round(windDir / 10) - 1;
  if (index >= 0 && index < 36) {
    weather.windDirHistogram[index]++;
  }

  weather.windSpeed = wxdata[1] * 1.609;
  weather.windSpeed = (weather.windSpeed < 0 || weather.windSpeed > 140) ? 
                      weather.averages.maxWindSpeed : weather.windSpeed;

  switch (wxdata[0] >> 4) {
    case 5:  // Rain rate
      weather.rainRate = (wxdata[4] & 0x40) ? 
                         11520.0 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]) :
                         720.0 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]);
      break;
    case 8:  // Temperature
      weather.temperature = (((wxdata[3] * 0xff + wxdata[4]) / 160.0) - 32.0) / 1.8;
      weather.temperature = (weather.temperature < -20 || weather.temperature > 60) ?
                           weather.temperature : weather.temperature;
      break;
    case 9:  // Wind gust
      weather.windGust = wxdata[3] * 1.609;
      break;
    case 10: // Humidity
      weather.humidity = (((wxdata[4] >> 4) << 8) + wxdata[3]) / 10.0;
      weather.humidity = (weather.humidity < 0 || weather.humidity > 100) ?
                         weather.humidity : weather.humidity;
      break;
    case 14: // Rain
      weather.rain = wxdata[3];
      break;
  }

  weather.averages.sumTemp += weather.temperature;
  weather.averages.sumHumid += weather.humidity;
  weather.averages.sumWindSpeed += weather.windSpeed;
#if BLACKGLOBE
  weather.blackGlobe = mcp.readThermocouple();
  weather.averages.sumBlackGlobe += weather.blackGlobe;
#endif
  weather.averages.maxWindSpeed = max(weather.averages.maxWindSpeed, weather.windSpeed);
  weather.averages.readings++;

#if DEBUG
  Serial.print("Raw - Wind: ");
  Serial.print(weather.windSpeed);
  Serial.print(", Temp: ");
  Serial.print(weather.temperature);
  Serial.print(", Hum: ");
  Serial.print(weather.humidity);
  Serial.print(", Rain: ");
  Serial.print(weather.rain);
  Serial.print(", Gust: ");
  Serial.println(weather.windGust);
#endif

  while (RS485.available()) RS485.read();
}

void transmitData() {
  if (weather.averages.readings > 0) {
    weather.averages.temp = weather.averages.sumTemp / weather.averages.readings;
    weather.averages.humid = weather.averages.sumHumid / weather.averages.readings;
    weather.averages.windSpeed = weather.averages.sumWindSpeed / weather.averages.readings;
#if BLACKGLOBE
    weather.averages.blackGlobe = weather.averages.sumBlackGlobe / weather.averages.readings;
#else
    weather.averages.blackGlobe = 0; // Set to 0 if BLACKGLOBE is off
#endif
  } else {
    weather.averages.temp = 0;
    weather.averages.humid = 0;
    weather.averages.windSpeed = 0;
    weather.averages.blackGlobe = 0;
#if DEBUG
    Serial.println("No readings, setting values to 0");
#endif
  }

  int maxCount = 0;
  int maxIndex = 0;
  for (int i = 0; i < 36; i++) {
    if (weather.windDirHistogram[i] > maxCount) {
      maxCount = weather.windDirHistogram[i];
      maxIndex = i;
    }
  }
  weather.windDirection = (maxCount > 0) ? (maxIndex * 10) + 5 : 0; // 0 if no wind direction data

  weather.pressure = ENV.readPressure(MILLIBAR) * 100;
  timestamp = nbAccess.getTime();

  publishMQTT("temperature", weather.averages.temp);
  publishMQTT("pressure", weather.pressure);
  publishMQTT("WindSpeed", weather.averages.windSpeed);
  publishMQTT("WindDirection", weather.windDirection);
  publishMQTT("RainToDay", weather.rain);
  publishMQTT("WindGust", weather.averages.maxWindSpeed);
  publishMQTT("humidity", (int)weather.averages.humid);
#if BLACKGLOBE
  publishMQTT("BlackGlobeTemp", weather.averages.blackGlobe);
#endif

  dataFile = SD.open(VLINDERNR, FILE_WRITE);
  if (dataFile) {
    dataFile.print(timestamp); dataFile.print(",");
    dataFile.print(weather.averages.temp); dataFile.print(",");
    dataFile.print((int)weather.averages.humid); dataFile.print(",");
    dataFile.print(weather.pressure); dataFile.print(",");
    dataFile.print(weather.averages.windSpeed); dataFile.print(",");
    dataFile.print(weather.windDirection); dataFile.print(",");
    dataFile.print(weather.rain); dataFile.print(",");
    dataFile.print(weather.averages.maxWindSpeed); dataFile.print(",");
    dataFile.println(weather.averages.blackGlobe);
    dataFile.close();
  }

#if DEBUG
  Serial.print("Averages - Temp: ");
  Serial.print(weather.averages.temp);
  Serial.print(", Hum: ");
  Serial.print((int)weather.averages.humid);
  Serial.print(", Press: ");
  Serial.print(weather.pressure);
  Serial.print(", Wind: ");
  Serial.print(weather.averages.windSpeed);
  Serial.print(", Dir: ");
  Serial.print(weather.windDirection);
  Serial.print(", Rain: ");
  Serial.print(weather.rain);
  Serial.print(", Gust: ");
  Serial.print(weather.averages.maxWindSpeed);
  Serial.print(", BG: ");
  Serial.println(weather.averages.blackGlobe);
#endif

  // Reset accumulators
  weather.averages.sumTemp = 0;
  weather.averages.sumHumid = 0;
  weather.averages.sumWindSpeed = 0;
  weather.averages.maxWindSpeed = 0;
  weather.averages.readings = 0;
  weather.averages.blackGlobe = 0;
  for (int i = 0; i < 36; i++) {
    weather.windDirHistogram[i] = 0;
  }
  previousMillis = millis();
  lastSuccess = millis();
}

void loop() {
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  if (millis() - lastSuccess > 60000) {
#if DEBUG
    Serial.println("Watchdog timeout, resetting...");
#endif
    NVIC_SystemReset();
  }

  processWeatherData();

  if (millis() - previousMillis >= SEND_INTERVAL * 1000) {
    transmitData();
  }
}
