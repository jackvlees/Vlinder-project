/**
 * Auteur: Daan van der Weken
 * AP-Hogeschool
 * 2ITIOT1 - VlinderProject
 * This file contains a sketch which uses the modem in Walter to publish
 * random weather data (temperature, humidity, pressure, wind speed, wind
 * direction, rain, wind gust, black globe temperature) via MQTT.
 * 
 * Met dank aan de support van Quickspot
 * gebruikte bron: https://github.com/QuickSpot/walter-arduino/blob/main/examples/mqtt/mqtt.ino 
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>

WalterModem modem; // modem initialisation
WalterModemRsp rsp; //response model

uint8_t incomingBuf[256] = {0}; //buffer
char macString[32];

static int publishFailCount = 0;

bool lteConnected() { //connection control to the lte netwerk
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

bool waitForNetwork() { //This function waits for the modem to be connected to the Lte network.
  int timeout = 0;
  while (!lteConnected()) {
    delay(100);
    timeout += 100;
    if (timeout > 300000) {
      Serial.println("Network connection timeout");
      return false;
    }
  }
  Serial.println("Connected to the network");
  return true;
}

bool lteConnect() { //This function tries to connect the modem to the cellular network.
  if (modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println("Successfully set operational state to NO RF");
  } else {
    Serial.println("Error: Could not set operational state to NO RF");
    return false;
  }

  
  if (modem.definePDPContext()) { // Create PDP context
    Serial.println("Created PDP context");
  } else {
    Serial.println("Error: Could not create PDP context");
    return false;
  }

  if (modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) { //Set the operational state to full
    Serial.println("Successfully set operational state to FULL");
  } else {
    Serial.println("Error: Could not set operational state to FULL");
    return false;
  }

  if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) { //Set the network operator selection to automatic
    Serial.println("Network selection mode was set to automatic");
  } else {
    Serial.println("Error: Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork();
}

bool ensureMqttConnected() { //Attempt to connect or reconnect to the MQTT broker.
  Serial.println("Attempting MQTT connection...");
  if (modem.mqttConnect("mqtt.iot-ap.be", 1883)) {
    Serial.println("MQTT connection succeeded");
    if (modem.mqttSubscribe("weerdata/#")) {
      Serial.println("MQTT subscribed to topic 'weerdata/#'");
      publishFailCount = 0; // Reset fail counter on successful connection
      return true;
    } else {
      Serial.println("Error: MQTT subscribe failed");
      return false;
    }
  } else {
    Serial.println("Error: MQTT connection failed");
    return false;
  }
}

void generateWeatherData(float &temperature, int &humidity, int &pressure, //Generate random weather data within realistic ranges.
                        float &windSpeed, int &windDirection, int &rain,
                        float &windGust, float &blackGlobe) {
  temperature = random(-200, 600) / 10.0; // -20.0 to 60.0°C
  humidity = random(0, 101);              // 0 to 100%
  pressure = random(90000, 110000);      // 900 to 1100 mbar * 100
  windSpeed = random(0, 1400) / 10.0;    // 0 to 140 km/h
  windDirection = random(0, 360);        // 0 to 359 degrees
  rain = random(0, 50);                  // 0 to 50 tips (0.2 L/m² per tip)
  windGust = windSpeed + random(0, 200) / 10.0; // Gust up to 20 km/h above speed
  blackGlobe = random(-200, 600) / 10.0; // -20.0 to 60.0°C
}

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Walter modem MQTT weather data example v1.0.0\r\n");

  esp_read_mac(incomingBuf, ESP_MAC_WIFI_STA);
  sprintf(macString, "walter%02X:%02X:%02X:%02X:%02X:%02X", incomingBuf[0],
          incomingBuf[1], incomingBuf[2], incomingBuf[3], incomingBuf[4],
          incomingBuf[5]);

  if (WalterModem::begin(&Serial2)) {
    Serial.println("Modem initialization OK");
  } else {
    Serial.println("Error: Modem initialization ERROR");
    return;
  }

  if (!lteConnect()) { //Connect the modem to the lte network
    Serial.println("Error: Could Not Connect to LTE");
    return;
  }

  if (modem.mqttConfig("weerdata")) { //Configure the mqtt client
    Serial.println("MQTT configuration succeeded");
  } else {
    Serial.println("Error: MQTT configuration failed");
    return;
  }

  if (!ensureMqttConnected()) { //Connect to MQTT broker
    Serial.println("Error: Initial MQTT connection failed");
    return;
  }
  randomSeed(analogRead(0)); //Initialize random seed for weather data generation
}

void loop() {
  Serial.println("Entering loop...");
  delay(5000); // Reduced delay for faster debugging

  //Generate random weather data
  float temperature, windSpeed, windGust, blackGlobe;
  int humidity, pressure, windDirection, rain;
  generateWeatherData(temperature, humidity, pressure, windSpeed, windDirection,
                      rain, windGust, blackGlobe);
  Serial.println("Generated weather data");

  //Publish weather data to separate MQTT topics
  char outgoingMsg[64];
  bool publishSuccess = true;

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", temperature);
  if (modem.mqttPublish("weerdata/temperature", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
    Serial.printf("Published temperature on topic 'weerdata/temperature': %s\r\n", outgoingMsg);
  } else {
    Serial.println("MQTT publish temperature failed");
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%d", humidity);
  if (modem.mqttPublish("weerdata/humidity", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
    Serial.printf("Published humidity on topic 'weerdata/humidity': %s\r\n", outgoingMsg);
  } else {
    Serial.println("MQTT publish humidity failed");
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%d", pressure);
  if (modem.mqttPublish("weerdata/pressure", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
    Serial.printf("Published pressure on topic 'weerdata/pressure': %s\r\n", outgoingMsg);
  } else {
    Serial.println("MQTT publish pressure failed");
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", windSpeed);
  if (modem.mqttPublish("weerdata/windSpeed", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
    Serial.printf("Published windSpeed on topic 'weerdata/windSpeed': %s\r\n", outgoingMsg);
  } else {
    Serial.println("MQTT publish windSpeed failed");
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%d", windDirection);
  if (modem.mqttPublish("weerdata/windDirection", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
    Serial.printf("Published windDirection on topic 'weerdata/windDirection': %s\r\n", outgoingMsg);
  } else {
    Serial.println("MQTT publish windDirection failed");
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%d", rain);
  if (modem.mqttPublish("weerdata/rain", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
    Serial.printf("Published rain on topic 'weerdata/rain': %s\r\n", outgoingMsg);
  } else {
    Serial.println("MQTT publish rain failed");
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", windGust);
  if (modem.mqttPublish("weerdata/windGust", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
    Serial.printf("Published windGust on topic 'weerdata/windGust': %s\r\n", outgoingMsg);
  } else {
    Serial.println("MQTT publish windGust failed");
    publishSuccess = false;
  }

  snprintf(outgoingMsg, sizeof(outgoingMsg), "%.1f", blackGlobe);
  if (modem.mqttPublish("weerdata/blackGlobe", (uint8_t *)outgoingMsg, strlen(outgoingMsg))) {
    Serial.printf("Published blackGlobe on topic 'weerdata/blackGlobe': %s\r\n", outgoingMsg);
  } else {
    Serial.println("MQTT publish blackGlobe failed");
    publishSuccess = false;
  }

  // Handle publish failures
  if (!publishSuccess) {
    publishFailCount++;
    Serial.printf("Publish failure count: %d\r\n", publishFailCount);
    if (publishFailCount >= 3) {
      Serial.println("Too many publish failures, attempting MQTT reconnect...");
      if (!ensureMqttConnected()) {
        Serial.println("Reconnection failed, will retry next loop");
      }
    }
  } else {
    publishFailCount = 0; // Reset counter on successful publish
  }

  // Handle incoming MQTT messages
  Serial.println("Checking for incoming MQTT messages...");
  while (modem.mqttDidRing("weerdata/#", incomingBuf, sizeof(incomingBuf), &rsp)) {
    Serial.printf("Incoming: qos=%d msgid=%d len=%d:\r\n",
                  rsp.data.mqttResponse.qos, rsp.data.mqttResponse.messageId,
                  rsp.data.mqttResponse.length);
    for (int i = 0; i < rsp.data.mqttResponse.length; i++) {
      Serial.printf("'%c' 0x%02x\r\n", incomingBuf[i], incomingBuf[i]);
    }
  }
  Serial.println("Loop completed");
}