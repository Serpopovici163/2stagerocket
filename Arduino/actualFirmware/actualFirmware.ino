#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <SoftwareSerial.h>

#define GPS_RX 10
#define GPS_TX 12
#define LORA_TX 11
#define LORA_RX 13

#define TIMEOUT (1000) //one second timeouts

TinyGPSPlus gpsObject;
Adafruit_BMP280 baro;
Adafruit_MPU6050 mpu;

SoftwareSerial GPS(GPS_RX, GPS_TX);
SoftwareSerial LoRa(LORA_RX, LORA_TX);

long oldTime = 0; //used for timing status update broadcasts
String packet = "";
double initialPressure = 0.0;

void setup() {
  GPS.begin(9600);
  LoRa.begin(9600);

  Wire.begin();
  bmp.begin(0x76); //not sure if address is necessary but why not
  mpu.begin(0x68); //same thing about the addy here

  //set BMP280 settings (copied from create.arduino.cc)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //calibrate bmp
  delay(1000);
  initialPressure = bmp.readPressure();
}

void loop() {
  //read any availabel data from GPS
  while (GPS.available()) {
    gpsObject.encode(GPS.read());
  }

  //broadcast update
  if (millis() > (oldTime + 1000)) {
    broadcastUpdate();
    oldTime = millis();
  }
}

void broadcastUpdate() {
  //add sat count to packet
  packet = "SATCNT:";
  packet += gpsObject.satellites.value();

  //append GPS altitude data
  packet += "|GPSALT:";
  packet += gpsObject.altitude.meters();

  //append BARO altitude data
  packet += "|BAROALT:";
  packet += bmp.readAltitude(initialPressure);

  //append location data
  if (gpsObject.location.isValid()) {
    packet += "|LAT:";
    packet += gpsObject.location.lat();
    packet += "|LON:";
    packet += gpsObject.location.lng();
  } else {
    packet += "|LAT:0|LON:0";
  }
}
