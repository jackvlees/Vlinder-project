#include <HardwareSerial.h>

// Definieer pinnen voor RS485-communicatie
#define RX_PIN 44 // RX-pin voor SoftwareSerial
#define TX_PIN 43 // TX-pin voor SoftwareSerial
#define DE_RE_PIN 12 // Data Enable/Receive Enable pin voor RS485-module
// SoftwareSerial voor RS485-communicatie
HardwareSerial RS485Serial(1);
// Buffer voor inkomende gegevens
unsigned char wxdata[8] = {0};

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
  int windDirHistogram[36];};
// Instantie van WeatherData
WeatherData weather;

void setup() {
  // Start seriële communicatie voor debugging
  Serial.begin(115200);
  while (!Serial); // Wacht tot seriële poort klaar is
  Serial.println("Setup gestart...");

  // Start SoftwareSerial voor RS485
  RS485Serial.begin(4800, SERIAL_8N1,RX_PIN,TX_PIN);
  Serial.println("RS485Serial gestart op 4800 baud");

  // Stel DE/RE-pin in als uitvoer
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Zet in ontvangstmodus
  Serial.println("RS485 in ontvangstmodus");
}

void processWeatherData() {
  // Controleer of er minimaal 6 bytes beschikbaar zijn
  if (RS485Serial.available() < 6) return;

  // Lees de 6 bytes in
  for (uint8_t i = 0; i < 6; i++) {
    wxdata[i] = RS485Serial.read();
  }

  // Debugging: print ontvangen ruwe data
  Serial.print("Ontvangen data: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(wxdata[i], HEX);
    Serial.print(" ");
  }
  Serial.print("Type: ");
  Serial.println(wxdata[0] >> 4, HEX);

  // Verwerk windrichting
  float windDir = (wxdata[2] * 359.0 / 255.0); // Schaal naar 0-359 graden
  int index = round(windDir / 10.0); // Histogramindex (0-35)
  if (index >= 0 && index < 36) {
    weather.windDirHistogram[index]++;
  }

  // Verwerk windsnelheid (in km/h)
  weather.windSpeed = wxdata[1] * 1.609; // Omzetten naar km/h
  if (weather.windSpeed < 0 || weather.windSpeed > 140) {
    weather.windSpeed = weather.averages.maxWindSpeed;
  }

  // Verwerk de gegevens op basis van het type (eerste 4 bits van wxdata[0])
  switch (wxdata[0] >> 4) {
    case 5: // Regenintensiteit (rain rate)
      weather.rainRate = (wxdata[4] & 0x40) ? 
                         11520.0 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]) :
                         720.0 / (((wxdata[4] & 0x30) / 16 * 250) + wxdata[3]);
      break;

    case 8: // Temperatuur
      weather.temperature = (((wxdata[3] * 0xff + wxdata[4]) / 160.0) - 32.0) / 1.8; // Vlinder-conversie
      if (weather.temperature < -20 || weather.temperature > 60) {
        weather.temperature = weather.averages.temp; // Gebruik gemiddelde als fallback
      }
      break;
    case 9: // Windstoot (gust)
      weather.windGust = wxdata[3] * 1.609;
      break;
    case 10: // Luchtvochtigheid
      weather.humidity = (((wxdata[4] >> 4) << 8) + wxdata[3]) / 10.0;
      if (weather.humidity < 0 || weather.humidity > 100) {
        weather.humidity = weather.averages.humid; // Gebruik gemiddelde als fallback
      }
      break;
    case 14: // Regen (totaal)
      weather.rain = wxdata[3];
      break;
    default:
      Serial.println("Onbekend gegevenstype ontvangen!");
      break;
  }
  // Update gemiddelden (uit Vlinder-code)
  weather.averages.sumTemp += weather.temperature;
  weather.averages.sumHumid += weather.humidity;
  weather.averages.sumWindSpeed += weather.windSpeed;
  weather.averages.maxWindSpeed = max(weather.averages.maxWindSpeed, weather.windSpeed);
  weather.averages.readings++;
  // Print de verwerkte gegevens
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
  Serial.println(" km/h");
  // Leeg de buffer om geen oude gegevensЛАйк data ontvangen!
  while (RS485Serial.available()) RS485Serial.read();}
void loop() {
  // Print periodiek om te controleren of de loop draait
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 5000) {
    Serial.println("Wachten op RS485-gegevens...");
    lastDebug = millis();
  }

  // Verwerk inkomende gegevens
  if (RS485Serial.available() > 0) {
    //Serial.print("Beschikbare bytes: ");
    //Serial.println(RS485Serial.available());
    processWeatherData();
  }
}
