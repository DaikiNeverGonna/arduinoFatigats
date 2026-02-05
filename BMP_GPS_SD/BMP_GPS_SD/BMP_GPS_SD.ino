// NO TOCAR
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

// GPS - pines asignados y número raro
static const uint8_t RXPin = 5;   
static const uint8_t TXPin = 4;  
static const uint32_t GPSBaud = 9600;

// SD - pin asignado 
static const uint8_t SD_CS_PIN = 9;

// Serial monitor
static const uint32_t SERIAL_BAUD = 9600;

// Pressure calibration (100 veces)
static const uint16_t samplesQuantity = 100;
static const uint32_t minPressure = 90000;
static const uint32_t maxPressure = 110000;

uint32_t calSumPa = 0;
uint16_t calValidCount = 0;
bool calibrated = false;

// cosas raras
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BMP280 bmp;

// SD
bool sdOK = false;

// paquete
uint32_t package = 0;

// Presión de nivel del mar en hPa
float seaLevelhPa = 1013.25f;

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println(F("=== START ==="));

  // GPS
  ss.begin(GPSBaud);
  ss.listen();

  pinMode(SD_CS_PIN, OUTPUT);

  // SD init
  Serial.print(F("Iniciando SD (CS="));
  Serial.print(SD_CS_PIN);
  Serial.println(F(")..."));

  sdOK = SD.begin(SD_CS_PIN);
  if (!sdOK)
  {
    Serial.println(F("ERROR: SD no inicializa. Revisa CS, cableado, y formato FAT32."));
  }
  else
  {
    Serial.println(F("SD OK"));
  }
  if (SD.exists("datalog.csv"))
  {
    Serial.println("SD existía así que borro el archivo para que esté vacío");
    SD.remove("datalog.csv");
  }

  // BMP280 init
  Serial.println(F("Iniciando BMP280..."));
  bool bmpOK = bmp.begin(0x76);
  if (!bmpOK)
    bmpOK = bmp.begin(0x77);

  if (!bmpOK)
  {
    Serial.println(F("ERROR: BMP280 no detectado (0x76/0x77)."));
  }
  else
  {
    Serial.println(F("BMP280 OK"));
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  Serial.println(F("=== LOOP ==="));
}

static void printGPS_Serial()
{
  Serial.print(F(" Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else Serial.print(F("INVALID"));

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month()); Serial.print(F("/"));
    Serial.print(gps.date.day());   Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else Serial.print(F("INVALID"));

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour()); Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute()); Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
  }
  else Serial.print(F("INVALID"));
}

static void printGPS_SD(File &f)
{
  f.print(F(" Location: "));
  if (gps.location.isValid())
  {
    f.print(gps.location.lat(), 6);
    f.print(F(","));
    f.print(gps.location.lng(), 6);
  }
  else f.print(F("INVALID"));

  f.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    f.print(gps.date.month()); f.print(F("/"));
    f.print(gps.date.day());   f.print(F("/"));
    f.print(gps.date.year());
  }
  else f.print(F("INVALID"));

  f.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) f.print(F("0"));
    f.print(gps.time.hour()); f.print(F(":"));
    if (gps.time.minute() < 10) f.print(F("0"));
    f.print(gps.time.minute()); f.print(F(":"));
    if (gps.time.second() < 10) f.print(F("0"));
    f.print(gps.time.second());
  }
  else f.print(F("INVALID"));
}

void logOnePackage()
{
  float tempC = bmp.readTemperature();
  uint32_t pressPa = (uint32_t)bmp.readPressure();
  float altM = bmp.readAltitude(seaLevelhPa);

  Serial.print(F("FATIGATS - "));
  Serial.print(package); Serial.print(F("p, "));
  Serial.print(tempC, 2); Serial.print(F("C, "));
  Serial.print(pressPa); Serial.print(F("P, "));
  Serial.print(altM, 2); Serial.print(F("m,"));
  printGPS_Serial();
  Serial.println();

  if (sdOK)
  {
    File f = SD.open("datalog.csv", FILE_WRITE);
    if (f)
    {
      f.print(F("FATIGATS - "));
      f.print(package); f.print(F("p, "));
      f.print(tempC, 2); f.print(F("C, "));
      f.print(pressPa); f.print(F("P, "));
      f.print(altM, 2); f.print(F("m,"));
      printGPS_SD(f);
      f.println();
      f.close();
    }
  }

  if (!calibrated)
  {
    if (pressPa >= minPressure && pressPa <= maxPressure)
    {
      calSumPa += pressPa;
      calValidCount++;

      if (calValidCount >= samplesQuantity)
      {
        float meanPa = (float)calSumPa / (float)calValidCount;
        seaLevelhPa = meanPa / 100.0f;

        calibrated = true;
        Serial.print(F("CALIBRADO seaLevelhPa = "));
        Serial.println(seaLevelhPa, 2);
      }
    }
  }

  package++;
}

void loop()
{
  while (ss.available() > 0)
  {
    char c = (char)ss.read();
    gps.encode(c);

    if (gps.location.isUpdated() || gps.time.isUpdated() || gps.date.isUpdated())
    {
      logOnePackage();
    }
  }

  static uint32_t lastWarn = 0;
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    if (millis() - lastWarn > 2000)
    {
      Serial.println(F("AVISO: No GPS detectado (revisa wiring). El programa sigue."));
      lastWarn = millis();
    }
  }
}
