/************** CONFIG **************/
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

// GPS
static const uint8_t RXPin = 5;   // Arduino recibe desde TX del GPS
static const uint8_t TXPin = 4;   // Arduino envía a RX del GPS (si se usa)
static const uint32_t GPSBaud = 9600;

// SD  (en Arduino UNO, la mayoría de módulos/shields usan CS = 10)
static const uint8_t SD_CS_PIN = 10;

// Serial monitor
static const uint32_t SERIAL_BAUD = 9600;

// Calibración de presión (100 lecturas)
static const uint16_t CAL_SAMPLES_TARGET = 100;
static const uint32_t PRESSURE_MIN_PA = 90000;
static const uint32_t PRESSURE_MAX_PA = 110000;
/************************************/

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BMP280 bmp; // I2C

File logFile;

// “Paquetes”
uint32_t package = 0;

// Presión a nivel del mar en hPa (BMP280 readAltitude espera hPa)
float seaLevelhPa = 1013.25f;

// Calibración sin array (MUCHA menos RAM en UNO)
uint32_t calSumPa = 0;
uint16_t calValidCount = 0;
bool calibrated = false;

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(300);

  // En UNO esto NO hace falta y a veces confunde, lo quitamos.

  Serial.println(F("=== START ==="));

  // GPS
  ss.begin(GPSBaud);
  ss.listen();

  // SPI: en UNO hay que dejar SS (pin 10) como OUTPUT para que SPI funcione bien
  pinMode(10, OUTPUT);

  // SD init
  Serial.print(F("Iniciando SD (CS="));
  Serial.print(SD_CS_PIN);
  Serial.println(F(")..."));

  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println(F("ERROR: SD no inicializa. Revisa CS, cableado, y formato FAT32."));
    // No bloqueamos el programa: seguimos sin SD para poder ver por Serial qué pasa
  }
  else
  {
    Serial.println(F("SD OK"));
  }

  // BMP280 init (probamos 0x76 y 0x77)
  Serial.println(F("Iniciando BMP280..."));
  bool bmpOK = bmp.begin(0x76);
  if (!bmpOK) bmpOK = bmp.begin(0x77);

  if (!bmpOK)
  {
    Serial.println(F("ERROR: BMP280 no detectado (0x76/0x77). Revisa cableado SDA/SCL y VCC/GND."));
    // Igual: no bloqueamos, pero ojo que sin BMP no tendrás datos válidos
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

void logOnePacket()
{
  // Lecturas sensor (si BMP no está, estas funciones devuelven cosas raras)
  float tempC = bmp.readTemperature();
  uint32_t pressPa = (uint32_t)bmp.readPressure();
  float altM = bmp.readAltitude(seaLevelhPa);

  // ---- Serial (sin String, estable en UNO)
  Serial.print(F("FATIGATS - "));
  Serial.print(package); Serial.print(F("p, "));
  Serial.print(tempC, 2); Serial.print(F("C, "));
  Serial.print(pressPa); Serial.print(F("P, "));
  Serial.print(altM, 2); Serial.print(F("m,"));
  printGPS_Serial();
  Serial.println();

  // ---- SD (si SD falla, no petamos el programa)
  if (SD.begin(SD_CS_PIN))
  {
    File f = SD.open("datalog.txt", FILE_WRITE);
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

  // ---- Calibración de seaLevelhPa (solo una vez)
  if (!calibrated)
  {
    // cogemos 100 muestras válidas
    if (pressPa >= PRESSURE_MIN_PA && pressPa <= PRESSURE_MAX_PA)
    {
      calSumPa += pressPa;
      calValidCount++;

      if (calValidCount >= CAL_SAMPLES_TARGET)
      {
        float meanPa = (float)calSumPa / (float)calValidCount;
        seaLevelhPa = meanPa / 100.0f; // Pa -> hPa

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
  // Consumimos datos GPS
  while (ss.available() > 0)
  {
    char c = (char)ss.read();
    gps.encode(c);

    // Cuando llega un paquete “completo” (cambia algo), logueamos
    // (Alternativa simple: loguear cada X ms, pero mantengo tu lógica de “según GPS”)
    if (gps.location.isUpdated() || gps.time.isUpdated() || gps.date.isUpdated())
    {
      logOnePacket();
    }
  }

  // Si NO hay GPS conectado, no bloqueamos el programa (solo avisamos cada cierto tiempo)
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
