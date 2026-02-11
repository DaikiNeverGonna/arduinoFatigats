// NO TOCAR
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

// GPS
static const uint8_t RXPin = 5;
static const uint8_t TXPin = 4;
static const uint32_t GPSBaud = 9600;

// SD
static const uint8_t SD_CS_PIN = 9;

// Serial
static const uint32_t SERIAL_BAUD = 9600;

// Pressure calibration
static const uint16_t samplesQuantity = 100;
static const uint32_t minPressure = 90000;
static const uint32_t maxPressure = 110000;

uint32_t calSumPa = 0;
uint16_t calValidCount = 0;
bool calibrated = false;

// GPS / sensores
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BMP280 bmp;

// SD
bool sdOK = false;

// paquete
uint32_t package = 0;

// tiempo
bool timeInitialized = false;
uint32_t startSeconds = 0;
uint32_t elapsedSeconds = 0;

// Presión nivel del mar
float seaLevelhPa = 1013.25f;

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(300);

  ss.begin(GPSBaud);
  ss.listen();

  pinMode(SD_CS_PIN, OUTPUT);

  sdOK = SD.begin(SD_CS_PIN);
  if (sdOK && SD.exists("datalog.txt"))
    SD.remove("datalog.txt");

  bool bmpOK = bmp.begin(0x76) || bmp.begin(0x77);
  if (bmpOK)
  {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
}

// ---------- GPS PRINT ----------

static void printGPS_Serial()
{
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID,INVALID"));
  }

  Serial.print(F(","));

  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour()); Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute()); Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}

static void printGPS_SD(File &f)
{
  if (gps.location.isValid())
  {
    f.print(gps.location.lat(), 6);
    f.print(F(","));
    f.print(gps.location.lng(), 6);
  }
  else
  {
    f.print(F("INVALID,INVALID"));
  }

  f.print(F(","));

  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) f.print(F("0"));
    f.print(gps.time.hour()); f.print(F(":"));
    if (gps.time.minute() < 10) f.print(F("0"));
    f.print(gps.time.minute()); f.print(F(":"));
    if (gps.time.second() < 10) f.print(F("0"));
    f.print(gps.time.second());
  }
  else
  {
    f.print(F("INVALID"));
  }
}

// ---------- LOG ----------

void logOnePackage()
{
  float tempC = bmp.readTemperature();
  uint32_t pressPa = (uint32_t)bmp.readPressure();
  float altM = bmp.readAltitude(seaLevelhPa);

  uint32_t currentSeconds =
    (uint32_t)gps.time.hour() * 3600UL +
    (uint32_t)gps.time.minute() * 60UL +
    (uint32_t)gps.time.second();

  if (!timeInitialized)
  {
    startSeconds = currentSeconds;
    timeInitialized = true;
  }

  elapsedSeconds = currentSeconds - startSeconds;

  Serial.print(F("FATIGATS,"));
  Serial.print(package); Serial.print(F(","));
  Serial.print(elapsedSeconds); Serial.print(F(","));
  Serial.print(tempC, 2); Serial.print(F(","));
  Serial.print(pressPa); Serial.print(F(","));
  Serial.print(altM, 2); Serial.print(F(","));
  printGPS_Serial();
  Serial.println();

  if (sdOK)
  {
    File f = SD.open("datalog.txt", FILE_WRITE);
    if (f)
    {
      f.print(F("FATIGATS,"));
      f.print(package); f.print(F(","));
      f.print(elapsedSeconds); f.print(F(","));
      f.print(tempC, 2); f.print(F(","));
      f.print(pressPa); f.print(F(","));
      f.print(altM, 2); f.print(F(","));
      printGPS_SD(f);
      f.println();
      f.close();
    }
  }

  package++;
}

// ---------- LOOP ----------

void loop()
{
  static int lastSecond = -1;

  while (ss.available() > 0)
  {
    char c = ss.read();
    gps.encode(c);

    if (gps.time.isValid())
    {
      int sec = gps.time.second();
      if (sec != lastSecond)
      {
        lastSecond = sec;
        logOnePackage();
      }
    }
  }
}
