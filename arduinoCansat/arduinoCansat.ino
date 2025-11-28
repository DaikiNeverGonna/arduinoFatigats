#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

int package;
Adafruit_BMP280 bmp;

File logFile;
File dataFile;

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);
  Serial.println(F("BMP280 test"));
  Serial.print(F("Iniciando SD ..."));
  if (!SD.begin(9))
  {
    Serial.println(F("Error al iniciar"));
    return;
  }
  Serial.println(F("Iniciado correctamente"));

  unsigned status;
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  ss.begin(GPSBaud);

  /*Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));*/
  Serial.println();
}

void ReadSD()
{ 
  dataFile = SD.open("datalog.txt"); 
  if (dataFile)
  {
    String dataLine;

    while (dataFile.available())
    {
      dataLine = dataFile.read(); 
      Serial.write(dataLine);
    }

    dataFile.close();
  }
  else 
  {
    Serial.println(F("Error al abrir el archivo"));
  }
}

void loop()
{
  String linea = "";

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
    {
      linea += "Ubicación: ";
      if (gps.location.isValid())
      {
        linea += String(gps.location.lat(), 6);
        linea += ",";
        linea += String(gps.location.lng(), 6);
      }
      else linea += "ERROR";

      linea += " Fecha y hora: ";
      if (gps.date.isValid())
      {
        linea += String(gps.date.month());
        linea += "/";
        linea += String(gps.date.day());
        linea += "/";
        linea += String(gps.date.year());
      }
      else linea += "ERROR";

      linea += " ";
      if (gps.time.isValid())
      {
        if (gps.time.hour() < 10) linea += "0";
        linea += String(gps.time.hour());
        linea += ":";
        if (gps.time.minute() < 10) linea += "0";
        linea += String(gps.time.minute());
        linea += ":";
        if (gps.time.second() < 10) linea += "0";
        linea += String(gps.time.second());
        linea += ".";
        if (gps.time.centisecond() < 10) linea += "0";
        linea += String(gps.time.centisecond());
      }
      else linea += "ERROR";
    }


  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }


  float stuff[4] = {bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1013.25)};
  
  for (int i = 0; i < 3; i++)
  {
    linea += ", ";
    linea += String(stuff[i]);
  }
  
  linea = "FATIGATS - " + String(package) + " - " + linea;

  logFile = SD.open("datalog.txt", FILE_WRITE);

  if (logFile)
  {
    logFile.println(linea);
    logFile.close();
  }
  else
  {
    Serial.println("Error al abrir el archivo");
  }

  Serial.println(linea);

  delay(2000);
  package++;
}

