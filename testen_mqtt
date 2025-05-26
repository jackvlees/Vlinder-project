/*
 Vlinder MKR Arduino MQTT communication
 V6 27/02/25 - Aangepast met uitgebreide debugging en LED na setup
*/

#define DEBUG 1       // turn Debug mode on/off
#define BLACKGLOBE 0  // if BlackGlobe then set to 1

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

#define I2C_ADDRESS (0x67)
#define SETUP_LED 6   // Nieuwe LED-pin voor "setup complete"

// MQTT settings
const char* mqttServer = "mqtt.iot-ap.be";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";

// Hardware initialisatie
NB nbAccess;
GPRS gprs;
NBClient nbClient;
PubSubClient mqttClient(nbClient);
#if BLACKGLOBE
Adafruit_MCP9600 mcp;
#endif
File dataFile;

const int SD_CS_PIN = 4;
unsigned char wxdata[8] = { 0 };
unsigned int timestamp;
int sendInterval = 30;  // intervallen
int counter = 1;
unsigned long previousMillis;

// Weather variables
int Temperature_i = 0;
float Temperature_f = 0.0;
float OldTemperature = 0.0;
float Humidity = 0.0;
float OldHumidity = 0.0;
int Rain = 0;
float WindDir = 0.0;
int WindDirection[36] = {0};
float WindSpeed = 0;
float OldWindspeed = 0.0;
float SumTemp = 0.0;
float SumHumid = 0.0;
float SumWindSpeed = 0.0;
float MaxWindSpeed = 0.0;
int ReadingsCounter = 0;
int Index = 0;
int AverageWindDirection = 0;
float AverageTemp = 0.0;
float AverageHumid = 0.0;
float AverageWindSpeed = 0.0;
int MaxValue = 0;
float RainRate = 0.0;
float WindGust = 0.0;
float SumBlackGlobe = 0.0;
float AverageBlackGlobe = 0.0;
int Pressure = 0;
float BlackGlobe = 0.0;

void ClearDAVIS_STREAM() {
  delay(200);
  while (RS485.available()) RS485.read();
}

void flashLED(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(duration);
    digitalWrite(LED_BUILTIN, LOW);
    delay(duration);
  }
}

void startMKRModem() {
#if DEBUG
  Serial.println("Starten van MKR-modem...");
#endif
  if (nbAccess.begin() != NB_READY || gprs.attachGPRS() != GPRS_READY) {
#if DEBUG
    Serial.println("Netwerkverbinding mislukt!");
#endif
    while (1) {
      flashLED(10, 200);
      delay(5000);
    }
  }
#if DEBUG
  Serial.println("Netwerkverbinding succesvol!");
#endif
}

void checkNetwork() {
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
#if DEBUG
    Serial.println("Netwerkverbinding verloren, herstarten modem...");
#endif
    nbAccess.shutdown();
    startMKRModem();
  }
}

void connectMQTT() {
  checkNetwork(); // Controleer netwerk vóór MQTT
  const int maxAttempts = 3; // Maximum aantal pogingen
  int attempt = 0;
  unsigned long startTime = millis();
  const unsigned long timeout = 30000; // 30 seconden timeout per poging

  while (!mqttClient.connected() && attempt < maxAttempts) {
#if DEBUG
    Serial.print("Poging "); Serial.print(attempt + 1); Serial.println(" om te verbinden met MQTT-server...");
#endif
    if (mqttClient.connect("VlinderDevice", mqttUser, mqttPassword)) {
#if DEBUG
      Serial.println("Verbonden met MQTT-server!");
#endif
      return; // Succes, exit de functie
    } else {
#if DEBUG
      Serial.print("Verbinding mislukt, foutcode: ");
      Serial.println(mqttClient.state());
#endif
      attempt++;
      if (attempt < maxAttempts) {
        delay(5000); // Wacht 5 seconden voordat opnieuw wordt geprobeerd
      }
    }
    if (millis() - startTime > timeout) {
#if DEBUG
      Serial.println("Timeout bij MQTT-verbinding, volgende poging...");
#endif
      startTime = millis(); // Reset timeout voor volgende poging
    }
  }

  if (!mqttClient.connected()) {
#if DEBUG
    Serial.println("MQTT-verbinding mislukt na max pogingen, doorgaan zonder MQTT...");
#endif
    // Ga verder zonder MQTT om vastlopen te voorkomen
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SETUP_LED, OUTPUT);  // Stel de setup-LED pin in als output
  digitalWrite(SETUP_LED, LOW);  // LED uit aan het begin
  flashLED(3, 1000);
#if DEBUG
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println("Setup gestart...");
#endif
  startMKRModem();
  mqttClient.setServer(mqttServer, mqttPort);
  connectMQTT();
  
  RS485.begin(4800);
  RS485.receive();

  if (!ENV.begin()) {
#if DEBUG
    Serial.println("Failed to initialize MKR ENV shield!");
#endif
    while (1) {
      flashLED(5, 200);
      delay(5000);
    }
  }
#if BLACKGLOBE
  if (!mcp.begin(I2C_ADDRESS)) {
    while (1) {
      flashLED(7, 100);
      delay(5000);
    }
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
    Serial.println("Failed to initialize SD card!");
#endif
    while (1) {
      flashLED(3, 200);
      delay(5000);
    }
  }
  
  dataFile = SD.open(VLINDERNR, FILE_WRITE);
  delay(1000);
  dataFile.println("timestamp,temperature,humidity,pressure,WindSpeed,WindDirection,Rain,WindGust,BlackGlobe");
  dataFile.close();
#if DEBUG
  Serial.println("Setup voltooid!");
#endif
  digitalWrite(SETUP_LED, HIGH);  // Zet de LED aan als setup klaar is
}

void publishMQTTData(const char* sensor, float value) {
  char topic[50];
  char payload[50];
  sprintf(topic, "%s/%s", DEVICE_ID, sensor);
  sprintf(payload, "%.2f", value);
#if DEBUG
  Serial.print("Publiceren - Topic: ");
  Serial.print(topic);
  Serial.print(", Payload: ");
  Serial.println(payload);
#endif
  mqttClient.publish(topic, payload);
}

void publishMQTTData(const char* sensor, int value) {
  char topic[50];
  char payload[50];
  sprintf(topic, "%s/%s", DEVICE_ID, sensor);
  sprintf(payload, "%d", value);
#if DEBUG
  Serial.print("Publiceren - Topic: ");
  Serial.print(topic);
  Serial.print(", Payload: ");
  Serial.println(payload);
#endif
  mqttClient.publish(topic, payload);
}

void loop() {
  static unsigned long lastSuccess = millis(); // Houd succesvolle verzending bij

  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  // Watchdog: reset als het te lang duurt zonder succes
  if ((millis() - lastSuccess) > 60000) { // 60 seconden timeout
#if DEBUG
    Serial.println("Te lang geen succes, reset...");
#endif
    NVIC_SystemReset(); // Reset de microcontroller
  }

  if (RS485.available() >= 6) {
#if DEBUG
    Serial.println("RS485 data ontvangen:");
#endif
    for (int i = 0; i < 6; i++) {
      wxdata[i] = RS485.read();
    }
    flashLED(1, 30);
    
    WindDir = (wxdata[2] * 359.0 / 255.0);
    Index = round(WindDir / 10) - 1;
    if (Index >= 0 && Index <= 36) {
      WindDirection[Index] += 1;
    }
    WindSpeed = wxdata[1] * 1.609;
    if (WindSpeed < 0 || WindSpeed > 140) {
      WindSpeed = OldWindspeed;
    } else {
      OldWindspeed = WindSpeed;
    }

    switch (wxdata[0] >> 4) {
      case 5:
        if ((wxdata[4] & 0x40) == 0x40) {
          RainRate = 11520 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]);
        } else {
          RainRate = 720 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]);
        }
        break;
      case 8:
        Temperature_i = (wxdata[3] * 0xff + wxdata[4]);
        Temperature_f = (Temperature_i / 160.0);
        Temperature_f = (Temperature_f - 32.0) / 1.8;
        if (Temperature_f < -20 || Temperature_f > 60) {
          Temperature_f = OldTemperature;
        } else {
          OldTemperature = Temperature_f;
        }
        break;
      case 9:
        WindGust = wxdata[3] * 1.609;
        break;
      case 10:
        Humidity = (((wxdata[4] >> 4) << 8) + wxdata[3]) / 10.0;
        if (Humidity < 0 || Humidity > 100) {
          Humidity = OldHumidity;
        } else {
          OldHumidity = Humidity;
        }
        break;
      case 14:
        Rain = wxdata[3];
        break;
    }
    
#if BLACKGLOBE
    SumBlackGlobe += mcp.readThermocouple();
#endif
    SumTemp += Temperature_f;
    SumHumid += Humidity;
    SumWindSpeed += WindSpeed;
    if (MaxWindSpeed < WindSpeed) MaxWindSpeed = WindSpeed;
    ReadingsCounter++;
#if DEBUG
    Serial.print("WindSpeed: "); Serial.println(WindSpeed);
    Serial.print("Temperature: "); Serial.println(Temperature_f);
    Serial.print("Humidity: "); Serial.println(Humidity);
    Serial.print("Rain: "); Serial.println(Rain);
    Serial.print("WindGust: "); Serial.println(WindGust);
#endif
    ClearDAVIS_STREAM();
  }

  if ((unsigned long)(millis() - previousMillis) > (sendInterval * 1000)) {
#if DEBUG
    Serial.println("10 seconden voorbij, verzend data...");
#endif
    
    if (ReadingsCounter != 0) {
      AverageTemp = SumTemp / ReadingsCounter;
      AverageHumid = SumHumid / ReadingsCounter;
      AverageWindSpeed = SumWindSpeed / ReadingsCounter;
#if BLACKGLOBE
      AverageBlackGlobe = SumBlackGlobe / ReadingsCounter;
#endif
    } else {
      AverageTemp = 99;
      AverageHumid = 99;
      AverageWindSpeed = 99;
      AverageBlackGlobe = 99;
#if DEBUG
      Serial.println("Geen metingen, gebruik dummy-waarden (99)");
#endif
    }

    MaxValue = WindDirection[0];
    Index = 0;
    for (int i = 0; i <= 35; i++) {
      if (WindDirection[i] > MaxValue) {
        MaxValue = WindDirection[i];
        Index = i;
      }
    }
    AverageWindDirection = (Index * 10) + 5;

    Pressure = ENV.readPressure(MILLIBAR) * 100;
    timestamp = nbAccess.getTime();

    // Publiceer data via MQTT met verlaagde delay
    publishMQTTData("temperature", AverageTemp);
    delay(100);
    publishMQTTData("pressure", Pressure);
    delay(100);
    publishMQTTData("WindSpeed", AverageWindSpeed);
    delay(100);
    publishMQTTData("WindDirection", AverageWindDirection);
    delay(100);
    publishMQTTData("RainToDay", Rain);
    delay(100);
    publishMQTTData("WindGust", MaxWindSpeed);
    delay(100);
    publishMQTTData("humidity", (int)AverageHumid);
#if BLACKGLOBE
    delay(100);
    publishMQTTData("BlackGlobeTemp", AverageBlackGlobe);
#endif

#if DEBUG
    Serial.print("Temperature = "); Serial.println(AverageTemp);
    Serial.print("Humidity = "); Serial.println((int)AverageHumid);
    Serial.print("Pressure = "); Serial.println(Pressure);
    Serial.print("WindSpeed = "); Serial.println(AverageWindSpeed);
    Serial.print("WindDirection = "); Serial.println(AverageWindDirection);
    Serial.print("Rain = "); Serial.println(Rain);
    Serial.print("WindGust = "); Serial.println(MaxWindSpeed);
    Serial.print("BlackGlobe = "); Serial.println(AverageBlackGlobe);
    Serial.print("Timestamp = "); Serial.println(timestamp);
#endif

    // SD kaart opslag
    dataFile = SD.open(VLINDERNR, FILE_WRITE);
    delay(1000);
    dataFile.print(timestamp); dataFile.print(",");
    dataFile.print(AverageTemp); dataFile.print(",");
    dataFile.print((int)AverageHumid); dataFile.print(",");
    dataFile.print(Pressure); dataFile.print(",");
    dataFile.print(AverageWindSpeed); dataFile.print(",");
    dataFile.print(AverageWindDirection); dataFile.print(",");
    dataFile.print(Rain); dataFile.print(",");
    dataFile.print(MaxWindSpeed); dataFile.print(",");
    dataFile.println(AverageBlackGlobe);
    dataFile.close();

    // Reset variabelen en update lastSuccess
    SumTemp = 0.0;
    SumHumid = 0.0;
    SumWindSpeed = 0.0;
    MaxWindSpeed = 0.0;
    ReadingsCounter = 0;
    SumBlackGlobe = 0.0;
    for (int i = 0; i <= 35; i++) WindDirection[i] = 0;
    previousMillis = millis();
    lastSuccess = millis(); // Update na succesvolle verzending
  }
}
