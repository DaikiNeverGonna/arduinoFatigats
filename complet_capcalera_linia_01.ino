/*************************************
   BMP280 + ADXL345 + GPS (TinyGPSPlus)
   Imprimir dades en línia CSV un cop/s
   + Capçalera única
*************************************/

// --- Llibreries BMP ---
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// --- Llibreries GPS ---
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// --- Llibreria acceleròmetre ---
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// --------------------- Definicions i objectes ---------------------
#define BMP_CS   10
Adafruit_BMP280 bmp; // Per I2C

static const int RXPin   = 4;  // GPS RX
static const int TXPin   = 3;  // GPS TX
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Comptador de paquets i temps
unsigned long lastPrint = 0;   // per controlar la sortida un cop per segon
int paquet = 0;

// --------------------- Funcions auxiliars acceleròmetre ---------------------
void displaySensorDetails()
{
  sensor_t sensor;
  accel.getSensor(&sensor);

  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------\n");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print("Data Rate: ");
  switch (accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:  Serial.println("3200 Hz");   break;
    case ADXL345_DATARATE_1600_HZ:  Serial.println("1600 Hz");   break;
    case ADXL345_DATARATE_800_HZ:   Serial.println("800 Hz");    break;
    case ADXL345_DATARATE_400_HZ:   Serial.println("400 Hz");    break;
    case ADXL345_DATARATE_200_HZ:   Serial.println("200 Hz");    break;
    case ADXL345_DATARATE_100_HZ:   Serial.println("100 Hz");    break;
    case ADXL345_DATARATE_50_HZ:    Serial.println("50 Hz");     break;
    case ADXL345_DATARATE_25_HZ:    Serial.println("25 Hz");     break;
    case ADXL345_DATARATE_12_5_HZ:  Serial.println("12.5 Hz");   break;
    default:                        Serial.println("Altres");    break;
  }
}

void displayRange(void)
{
  Serial.print("Range: +/- ");
  switch (accel.getRange())
  {
    case ADXL345_RANGE_16_G: Serial.println("16 g"); break;
    case ADXL345_RANGE_8_G:  Serial.println("8 g");  break;
    case ADXL345_RANGE_4_G:  Serial.println("4 g");  break;
    case ADXL345_RANGE_2_G:  Serial.println("2 g");  break;
    default:                 Serial.println("?? g"); break;
  }
}

// --------------------- Setup ---------------------
void setup()
{
  Serial.begin(9600);
  while (!Serial) { /* per a pliques com Leonardo, etc. */ }

  // -- Inicia BMP280 --
  if (!bmp.begin()) {
    Serial.println("No trobo el BMP280. Revisa el cablejat o adreça I2C!");
    while (1) delay(10);
  }
  // Configuració per defecte del BMP280: oversampling, filtre...
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,   // oversampling Temp
                  Adafruit_BMP280::SAMPLING_X16,  // oversampling Pressió
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // -- Inicia GPS --
  ss.begin(GPSBaud);
  Serial.println("Inici TinyGPSPlus");
  Serial.print("Versio llibreria: "); 
  Serial.println(TinyGPSPlus::libraryVersion());

  // -- Inicia acceleròmetre ADXL345 --
  if (!accel.begin()) {
    Serial.println("No trobo l'ADXL345. Revisa cablejat!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  displaySensorDetails();
  displayDataRate();
  displayRange();
  Serial.println("Inicialitzacions finalitzades.\n");

  // -- Imprimeix capçalera només un cop --
  Serial.println("paquete,lat,lon,accelX,accelY,accelZ,temp,pressio,altitud,data,hora");
}

// --------------------- Bucle principal ---------------------
void loop()
{
  // 1) Llegir GPS de manera contínua
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // 2) Comprovar si ha passat 1 segon
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    // 2.1) Llegim acceleròmetre
    sensors_event_t event;
    accel.getEvent(&event);

    // 2.2) Llegim BMP280
    float temperatura = bmp.readTemperature();         // ºC
    float pressio     = bmp.readPressure();            // Pa
    float altitud     = bmp.readAltitude(1025.25);     // Ajust local

    // 2.3) Preparem dades GPS
    String latStr, lonStr;
    if (gps.location.isValid()) {
      latStr = String(gps.location.lat(), 6);
      lonStr = String(gps.location.lng(), 6);
    } else {
      latStr = "INVALID";
      lonStr = "INVALID";
    }

    String dataStr, horaStr;
    if (gps.date.isValid()) {
      dataStr = String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
    } else {
      dataStr = "INVALID";
    }

    if (gps.time.isValid()) {
      // Format: hh:mm:ss
      char buf[16];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
               gps.time.hour(), gps.time.minute(), gps.time.second());
      horaStr = String(buf);
    } else {
      horaStr = "INVALID";
    }

    // 2.4) Imprimir tot en UNA línia CSV
    // Format: paquet, lat, lon, accelX, accelY, accelZ, temperatura, pressio, altitud, data, hora
    Serial.print(paquet);
    Serial.print(",");
    Serial.print(latStr);
    Serial.print(",");
    Serial.print(lonStr);
    Serial.print(",");
    Serial.print(event.acceleration.x, 2);
    Serial.print(",");
    Serial.print(event.acceleration.y, 2);
    Serial.print(",");
    Serial.print(event.acceleration.z, 2);
    Serial.print(",");
    Serial.print(temperatura, 2);
    Serial.print(",");
    Serial.print(pressio, 2);
    Serial.print(",");
    Serial.print(altitud, 2);
    Serial.print(",");
    Serial.print(dataStr);
    Serial.print(",");
    Serial.print(horaStr);
    Serial.print("IES Sonrullan1");
    Serial.println();


    // 2.5) Incrementem comptador
    paquet++;
  }

  // 3) Comprovació: si en 5 segons no hi ha dades GPS, avisem
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected: check wiring.");
    while (true);
  }
}
