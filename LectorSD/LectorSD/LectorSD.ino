//GPS
#include <SoftwareSerial.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

int package = 0;
Adafruit_BMP280 bmp; // I2C
File logFile;

int delayMillis = 2000;

int previousMillis;

void setup()
{
  Serial.begin(9600);

  
  Serial.print(F("Iniciando SD ..."));
  if (!SD.begin(9))
  {
    Serial.println(F("Error al iniciar"));
    return;
  }
  Serial.println(F("Iniciado correctamente"));
  WriteSD();

}
void WriteSD()
{
  
  logFile = SD.open("datalog.txt", FILE_WRITE);
  if (logFile)
  {
    logFile.println("Nose");
    logFile.close();
  }
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