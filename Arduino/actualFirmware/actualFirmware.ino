#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Servo.h>

#define GPS_RX 10
#define GPS_TX 12
#define LORA_TX 13
#define LORA_RX 11

#define STAGE_2_IGNITER 31
#define RECOVERY_IGNITER 30
#define MAIN_CHUTE_SERVO 47

#define PACKET_DELAY 1000

#define TIMEOUT (1000) //one second timeouts

#define MAX_TILT 30 //degrees
#define MIN_ALT_FOR_STAGE_2_IGNITION 250 //meters
#define STAGE_2_IGNITION_DELAY 5000 //milliseconds
#define MAIN_CHUTE_DEPLOY_ALT 250 //meters

TinyGPSPlus gpsObject;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Servo mainChuteServo;

SoftwareSerial GPS(GPS_RX, GPS_TX);
SoftwareSerial LoRa(LORA_RX, LORA_TX);

long oldTime = 0; //used for timing status update broadcasts
String packet = "";
float groundPressure = 0.0;

sensors_event_t a, g, temp; //sensor events for mpu

//state variables relevant to flight state
bool isArmed = false;
float baroAlt = 0;
float yAccel = 0;
float xTilt = 0;
float zTilt = 0;
int flightState = 0 //0 on ground, 1 launch detected, 2 second stage ignited, 3 drogue deployed, 4 main deployed, 5 landed

//time variables
unsigned long long launchTime = 0;
unsigned long long stageTwoIgnitionTime = 0;
unsigned long long recoveryDeployedTime = 0;
unsigned long long mainChuteDeployedTime = 0;

void setup() {
  //uploading delay
  delay(5000);
  
  //DEBUG
  Serial.begin(9600);
  Serial.println("In setup");
  
  GPS.begin(9600);
  LoRa.begin(9600);

  Serial.println("SoftwareSerial done");

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

  pinMode(STAGE_2_IGNITER, OUTPUT);
  pinMode(RECOVERY_IGNITER, OUTPUT);
  mainChuteServo.attach(MAIN_CHUTE_SERVO);
  mainChuteServo.write(0); //set servo to max position

  Serial.println("I/O init done");
}

void loop() {
  //read any availabel data from GPS
  while (GPS.available()) {
    gpsObject.encode(GPS.read());
  }

  while (LoRa.available()) {
    if (LoRa.readString() == "ARM") {
      isArmed = true;
    } else if (LoRa.readString() == "DISARM") {
      isArmed = false;
    }
  }

  //read accelerometer data
  mpu.getEvent(&a, &g, &temp);
  updateStateVars();

  //broadcast update
  if (millis() > (oldTime + PACKET_DELAY)) {
    broadcastUpdate();
    oldTime = millis();
  }

  //manage flight state
  //first check if the rocket is armed
  if (isArmed) {
    //now act according to flight state
    if (flightState == 0) { //on ground
      //check if accel is positive and if so increment state
      if (yAccel > 0) {
        flightState = 1;
        launchTime = millis(); //log launch time
      }
    } else if (flightState == 1) { //launch detected
      if (millis() > (launchTime + STAGE_2_IGNITION_DELAY)) {
        //ignition delay has been reached
        //check if all stage 2 ignition criteria are met, if not deploy recovery
        //check altitude and tilt on both x and z axes
        if (baroAlt > MIN_ALT_FOR_STAGE_2_IGNITION && (xTilt < MAX_TILT && zTilt < MAX_TILT)) {
          //ignite second stage
          flightState = 2;
          stageTwoIgnitionTime = millis();
        } else {
          flightState = 3;
          recoveryDeployedTime = millis();
        }
      }
    } else if (flightState == 2) { //second stage ignited
      //fuck
    } else if (flightState == 3) { //drogue deployed
      //deploy main chute at
      if (baroAlt < MAIN_CHUTE_DEPLOY_ALT) {
        flightState = 4;
        mainChuteServo.write(180);
      }
    } else if (flightState == 4) { //main deployed
      delay(1000);
      if (baroAlt == bmp.readAltitude(groundPressure)) {
        flighState == 5; //touchdown
      }
    }
  }

  delay(10);
}

void updateStateVars() {
  yAccel = -1*a.acceleration.y;
  xTilt = g.orientation.x;
  zTilt = g.orientation.z;
  baroAlt = bmp.readAltitude(groundPressure);
}

void logToBlackbox() {
  //TODO: include this in log: gpsObject.altitude.meters();
  //Serial.println(packet); //DEBUG
}

void broadcastUpdate() {   
  //add sat count to packet
  packet = "SATCNT:";
  packet += gpsObject.satellites.value();

  //append ARM STATE data
  packet += "|ARM_STATE:";
  packet += isArmed;

  //append BARO altitude data
  packet += "|BAROALT:";
  packet += baroAlt;

  //append STATE data
  packet += "|STATE:";
  packet += flightState;

  //append Y_ACCEL data
  packet += "|Y_ACCEL:";
  packet += yAccel;

  //append location data
  if (gpsObject.location.isValid()) {
    packet += "|LAT:";
    packet += gpsObject.location.lat();
    packet += "|LON:";
    packet += gpsObject.location.lng();
  } else {
    packet += "|LAT:0|LON:0";
  }

  LoRa.println(packet);
}