//GPS
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 5, TXPin = 4;
static const uint32_t GPSBaud = 9600;
//

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include <SD.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
//

int package = 0;
Adafruit_BMP280 bmp; // I2C
File logFile;

int delayMillis = 2000;

int previousMillis;

const int pressuresToCalibrate = 100;
float pressures[pressuresToCalibrate];

float altitudeVariable = 1013.25;

void setup()
{
  Serial.begin(9600);

  for (int i = 0; i < pressuresToCalibrate; i++)
  {
    pressures[i] = 0;
  }

  //GPS
  ss.begin(GPSBaud);

  //
  while (!Serial) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));

  
  Serial.print(F("Iniciando SD ..."));
  if (!SD.begin(9))
  {
    Serial.println(F("Error al iniciar SD"));
    while(true);
  }
  Serial.println(F("SD iniciado correctamente"));
   
  
  unsigned status;
  status = bmp.begin();
  if (!status) {
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void ReadSD()
{
  File dataFile = SD.open("datalog.txt"); 
  if (dataFile)
  {
    while (dataFile.available())
    {
      char c = dataFile.read(); 
      Serial.write(c);  // En un caso real se realizarían las acciones oportunas
    }
    dataFile.close();
  }
  else 
  {
    Serial.println(F("Error al abrir el archivo"));
  }
  Serial.println("Se está llamando read SD");
}

//
void displayGPS()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}

void displayGPSonSD()
{
  
  logFile.print(F("Location: "));
  if (gps.location.isValid())
  {
    logFile.print(gps.location.lat(), 6);
    logFile.print(F(","));
    logFile.print(gps.location.lng(), 6);
  }
  else
  {
    logFile.print(F("INVALID"));
  }

  logFile.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    logFile.print(gps.date.month());
    logFile.print(F("/"));
    logFile.print(gps.date.day());
    logFile.print(F("/"));
    logFile.print(gps.date.year());
  }
  else
  {
    logFile.print(F("INVALID"));
  }

  logFile.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) logFile.print(F("0"));
    logFile.print(gps.time.hour());
    logFile.print(F(":"));
    if (gps.time.minute() < 10) logFile.print(F("0"));
    logFile.print(gps.time.minute());
    logFile.print(F(":"));
    if (gps.time.second() < 10) logFile.print(F("0"));
    logFile.print(gps.time.second());
    logFile.print(F("."));
    if (gps.time.centisecond() < 10) logFile.print(F("0"));
    logFile.print(gps.time.centisecond());
  }
  else
  {
    logFile.print(F("INVALID"));
  }
}
//
void loop()
{
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      delayedLoop();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

}
void newAltitudeData()
{
  
  float totalPressure = 0;
  int pressureCount = 0;

  for (int i = 0; i < pressuresToCalibrate; i++)
  {
    if (pressures[i] > 90000 && pressures[i] < 110000)
    {
      totalPressure += pressures[i];
      pressureCount++;
    }
  }
  if (pressureCount > 0)
    altitudeVariable = totalPressure / pressureCount;
}
void delayedLoop()
{
  //logFile = SD.open("datalog.txt", FILE_WRITE);
  
  logFile =  SD.open("datalog.txt", FILE_WRITE);

  Serial.print("FATIGATS - ");
  logFile.print("FATIGATS - ");

  // p-paquete C-celsius P - Pascals m-Altitud
  char unities[] = "pCPm";
  float stuff[4] = {package, bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(altitudeVariable)};
  for (int i = 0; i < 4; i++)
  {
    Serial.print(String(stuff[i]) + unities[i]);
    Serial.print(", ");

    logFile.print(String(stuff[i]) + unities[i]);
    logFile.print(", ");
  }
  displayGPS();
  displayGPSonSD();
  
  /*if (logFile)
  {
    logFile.println("Nose");
    logFile.close();
  }*/

  Serial.println();
  logFile.println();
  logFile.close();
  //delay(2000);

  //recopilar altura
  //
  //< 900 lo quito mas de 1100 quito 
  if (package < pressuresToCalibrate)
  {
    pressures[package] = bmp.readPressure();
  }
  else if (package == pressuresToCalibrate)
  {
    Serial.print("AAAAAAA");
    newAltitudeData();
  }
  //
  package++;
}
