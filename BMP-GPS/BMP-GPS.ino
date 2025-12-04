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

void setup()
{
  Serial.begin(9600);
  //GPS
  ss.begin(GPSBaud);

  //
  while (!Serial) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));

  /*
  Serial.print(F("Iniciando SD ..."));
  if (!SD.begin(9))
  {
    Serial.println(F("Error al iniciar"));
    return;
  }
  Serial.println(F("Iniciado correctamente"));*/
   
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
//
void loop()
{
  /*while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayGPS();*/

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= delayMillis) {
    delayedLoop();
    previousMillis = currentMillis;
  }

}
void delayedLoop()
{
  //logFile = SD.open("datalog.txt", FILE_WRITE);
  Serial.print("FATIGATS - ");

  // p-paquete C-celsius P - Pascals m-Altitud
  char unities[] = "pCPm";
  float stuff[4] = {package, bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1013.25)};
  for (int i = 0; i < 4; i++)
  {
    Serial.print(String(stuff[i]) + unities[i]);
    Serial.print(", ");
  }
  displayGPS();
  
  /*if (logFile)
  {
    logFile.println("Nose");
    logFile.close();
  }*/

  Serial.println();
  //delay(2000);

  package++;
}
