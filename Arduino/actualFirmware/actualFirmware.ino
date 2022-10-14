#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>

#define GPS_RX 10
#define GPS_TX 12
#define LORA_TX 11
#define LORA_RX 13

#define PACKET_DELAY 500

#define TIMEOUT (1000) //one second timeouts

TinyGPSPlus gpsObject;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp; //sensor events for mpu

SoftwareSerial GPS(GPS_RX, GPS_TX);
SoftwareSerial LoRa(LORA_RX, LORA_TX);

long oldTime = 0; //used for timing status update broadcasts
String packet = "";
float groundPressure = 0.0;

void setup() {
  //uploading delay
  delay(5000);
  
  //DEBUG
  Serial.begin(115200);
  Serial.println("In setup");
  
  GPS.begin(9600);
  LoRa.begin(9600);

  Serial.println("SoftwareSerial done");

  Wire.begin(); //TODO: needed?
  mpu.begin(); 
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  bmp.begin(0x76, 0x58);
  groundPressure = bmp.readPressure() / 100;
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("I2C done");
}

void loop() {
  //read any availabel data from GPS
  while (GPS.available()) {
    gpsObject.encode(GPS.read());
  }

  //read accelerometer data
  mpu.getEvent(&a, &g, &temp);

  //broadcast update
  if (millis() > (oldTime + PACKET_DELAY)) {
    broadcastUpdate();
    oldTime = millis();
  }

  delay(10);
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
  packet += bmp.readAltitude(groundPressure);

  //append Y_ACCEL data
  packet += "|Y_ACCEL:";
  packet += -1*(a.acceleration.y - 9.81);

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
