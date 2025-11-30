//GPS
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
//

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>


#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

//GPS
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
//

int package = 0;
Adafruit_BMP280 bmp; // I2C
File logFile;

void setup() {
  Serial.begin(9600);
  //GPS
  ss.begin(GPSBaud);
  //
  while (!Serial) delay(100);   // wait for native usb
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
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
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
}

//GPS
String displayInfo()
{
  String lineaGPS;
  lineaGPS += F("Location: "); 
  if (gps.location.isValid())
  {
    lineaGPS += String(gps.location.lat(), 6);
    lineaGPS += ",";
    lineaGPS += String(gps.location.lng(), 6);
  }
  else
  {
    lineaGPS += F("INVALID");
  }

  lineaGPS += F("  Date/Time: ");
  if (gps.date.isValid())
  {
    lineaGPS += String(gps.date.month());
    lineaGPS += F("/");
    lineaGPS += String(gps.date.day());
    lineaGPS += F("/");
    lineaGPS += String(gps.date.year());
  }
  else
  {
    lineaGPS += F("INVALID");
  }

  lineaGPS += F(" ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) lineaGPS += F("0");
    lineaGPS += String(gps.time.hour());
    lineaGPS += F(":");
    if (gps.time.minute() < 10) lineaGPS += F("0");
    lineaGPS += String(gps.time.minute());
    lineaGPS += F(":");
    if (gps.time.second() < 10) lineaGPS += F("0");
    lineaGPS += String(gps.time.second());
    lineaGPS += F(".");
    if (gps.time.centisecond() < 10) lineaGPS += F("0");
    lineaGPS += String(gps.time.centisecond());
  }
  else
  {
    lineaGPS += F("INVALID");
  }

  return lineaGPS;
}
//
void loop()
{
  logFile = SD.open("datalog.txt", FILE_WRITE);
  String lineaGPS;
  String lineaBMP;

  //GPS
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      lineaGPS = displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

  float stuff[4] = {package, bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1013.25)};
  for (int i = 0; i < 4; i++)
  {
    lineaBMP += String(stuff[i]);
    lineaBMP += ", ";
  }
  String lineaFinal = "Fatigats, " + lineaBMP + lineaGPS;
  
  if (logFile)
  {
    logFile.println(lineaFinal);
    logFile.close();
  }
  else {
    Serial.println("Error al abrir el archivo");
  }

  Serial.println(lineaFinal);
  delay(2000);

  package++;
}
