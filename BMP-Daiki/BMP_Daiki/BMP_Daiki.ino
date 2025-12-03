/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

int package;
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  Serial.print(F("Iniciando SD ..."));
  if (!SD.begin(9))
  {
    Serial.println(F("Error al iniciar"));
    return;
  }
  Serial.println(F("Iniciado correctamente"));
 
  // Abrir fichero y mostrar el resultado

  //
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
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
  /*dataFile = SD.open("datalog.txt"); 
  if (dataLine)
  {
  string dataLine;
  
    dataFile.close();
  }
  else 
  {
    Serial.println(F("Error al abrir el archivo"));
  }
  while (dataFile.available())
  {
    dataLine = dataFile.read(); 
    Serial.write(dataLine);  // En un caso real se realizarían las acciones oportunas
  }*/
}
void loop()
{
  /*logFile = SD.open("datalog.txt", FILE_WRITE);

  if (logFile)
  {
    
    float stuff[4] = {package, bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1013.25)};
    for (int i = 0; i < 4; i++)
    {
      Serial.print(stuff[i]);
      logFile.print(", ");
    }
    logFile.println();
  }
  else {
    Serial.println("Error al abrir el archivo");
  }*/


  float stuff[4] = {package, bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1013.25)};
  for (int i = 0; i < 4; i++)
  {
    Serial.print(stuff[i]);
    Serial.print(", ");
  }

  Serial.print("Fatigats");

  Serial.println();
  delay(2000);

  package++;
}
