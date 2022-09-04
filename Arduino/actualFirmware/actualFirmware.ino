#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>


#include <BMP280_DEV.h>

#include <Wire.h>
#include <SoftwareSerial.h>

#define GPS_RX 10
#define GPS_TX 12
#define LORA_TX 11
#define LORA_RX 13

#define TIMEOUT (1000) //one second timeouts

TinyGPSPlus gpsObject;
BMP280_DEV bmp;
Adafruit_MPU6050 mpu;

SoftwareSerial GPS(GPS_RX, GPS_TX);
SoftwareSerial LoRa(LORA_RX, LORA_TX);

long oldTime = 0; //used for timing status update broadcasts
String packet = "";
float temperature, pressure, baro_alt;

void setup() {
  //uploading delay
  delay(5000);
  
  //DEBUG
  Serial.begin(115200);
  Serial.println("In setup");
  
  GPS.begin(9600);
  LoRa.begin(9600);

  Serial.println("SoftwareSerial done");

  Wire.begin();
  bmp.begin(); 
  mpu.begin(); 

  Serial.println("I2C done");

  bmp.startNormalConversion();

  //DEBUG
  Serial.println("Done setup!");
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

  delay(10);
}

void broadcastUpdate() {
  //get baro data

  bmp.getMeasurements(temperature, pressure, baro_alt);
  
  //add sat count to packet
  packet = "SATCNT:";
  packet += gpsObject.satellites.value();

  //append GPS altitude data
  packet += "|GPSALT:";
  packet += gpsObject.altitude.meters();

  //append BARO altitude data
  packet += "|BAROALT:";
  packet += baro_alt;

  //append location data
  if (gpsObject.location.isValid()) {
    packet += "|LAT:";
    packet += gpsObject.location.lat();
    packet += "|LON:";
    packet += gpsObject.location.lng();
  } else {
    packet += "|LAT:0|LON:0";
  }

  Serial.println(packet); //DEBUG - replace with LoRa.println
}
