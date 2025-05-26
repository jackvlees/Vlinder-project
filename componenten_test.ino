/*
 Vlinder MKR Arduino Sensor Test
 Versie: 13/03/25 - Testprogramma voor sensoren, gefixt voor switch-case fouten
*/

#define DEBUG 1       // Debug modus aan voor seriële output
#define BLACKGLOBE 0  // Zet op 1 als BlackGlobe wordt gebruikt

#include <Arduino_MKRENV.h>
#include <ArduinoRS485.h>
#if BLACKGLOBE
#include <Adafruit_MCP9600.h>
#endif

#define I2C_ADDRESS (0x67)
#define SETUP_LED 6   // LED-pin voor "setup complete"

// Weather variables
unsigned char wxdata[8] = { 0 };
float Temperature_f = 0.0;
float Humidity = 0.0;
int Rain = 0;
float WindDir = 0.0;
float WindSpeed = 0.0;
float WindGust = 0.0;
float RainRate = 0.0;
int Pressure = 0;
float BlackGlobe = 0.0;

#if BLACKGLOBE
Adafruit_MCP9600 mcp;
#endif

void ClearDAVIS_STREAM() {
  unsigned long start = millis();
  while (RS485.available() && (millis() - start < 200)) {
    RS485.read();
  }
}

void flashLED(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(duration);
    digitalWrite(LED_BUILTIN, LOW);
    delay(duration);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SETUP_LED, OUTPUT);
  digitalWrite(SETUP_LED, LOW);
  flashLED(3, 1000);

#if DEBUG
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println("Sensor Test Setup gestart...");
#endif

  // Start RS485
  RS485.begin(4800);
  RS485.receive();

  // Start MKR ENV Shield
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
  // Start BlackGlobe (MCP9600)
  if (!mcp.begin(I2C_ADDRESS)) {
#if DEBUG
    Serial.println("Failed to initialize MCP9600 (BlackGlobe)!");
#endif
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

#if DEBUG
  Serial.println("Setup voltooid!");
#endif
  digitalWrite(SETUP_LED, HIGH); // LED aan na succesvolle setup
}

void loop() {
  // Lees RS485 data
  if (RS485.available() >= 6) {
#if DEBUG
    Serial.println("RS485 data ontvangen:");
#endif
    for (int i = 0; i < 6; i++) {
      wxdata[i] = RS485.read();
    }
    flashLED(1, 30);

    // Verwerk RS485 data
    WindDir = (wxdata[2] * 359.0 / 255.0);
    WindSpeed = wxdata[1] * 1.609;

    int Temperature_i = 0; // Declareer buiten switch om "crosses initialization" te vermijden
    switch (wxdata[0] >> 4) {
      case 5: // Rain Rate
        if ((wxdata[4] & 0x40) == 0x40) {
          RainRate = 11520 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]);
        } else {
          RainRate = 720 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]);
        }
        break;
      case 8: // Temperature
        Temperature_i = (wxdata[3] * 0xff + wxdata[4]);
        Temperature_f = (Temperature_i / 160.0);
        Temperature_f = (Temperature_f - 32.0) / 1.8;
        break;
      case 9: // Wind Gust
        WindGust = wxdata[3] * 1.609;
        break;
      case 10: // Humidity
        Humidity = (((wxdata[4] >> 4) << 8) + wxdata[3]) / 10.0;
        break;
      case 14: // Rain
        Rain = wxdata[3];
        break;
    }

    ClearDAVIS_STREAM();
  }

  // Lees MKR ENV Shield druk
  Pressure = ENV.readPressure(MILLIBAR) * 100;

#if BLACKGLOBE
  // Lees BlackGlobe temperatuur
  BlackGlobe = mcp.readThermocouple();
#endif

  // Toon alle metingen via seriële monitor
#if DEBUG
  Serial.println("=== Sensor Metingen ===");
  Serial.print("Temperature (C): "); Serial.println(Temperature_f);
  Serial.print("Humidity (%): "); Serial.println(Humidity);
  Serial.print("Pressure (Pa): "); Serial.println(Pressure);
  Serial.print("Wind Speed (km/h): "); Serial.println(WindSpeed);
  Serial.print("Wind Direction (degrees): "); Serial.println(WindDir);
  Serial.print("Rain (count): "); Serial.println(Rain);
  Serial.print("Wind Gust (km/h): "); Serial.println(WindGust);
  Serial.print("Rain Rate: "); Serial.println(RainRate);
#if BLACKGLOBE
  Serial.print("BlackGlobe Temp (C): "); Serial.println(BlackGlobe);
#endif
  Serial.println("======================");
#endif

  delay(1000); // Wacht 1 seconde tussen metingen
}
