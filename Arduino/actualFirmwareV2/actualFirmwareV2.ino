#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>

#define GPS_RX 10
#define GPS_TX 12
#define LORA_TX 13
#define LORA_RX 11

#define STAGE_2_IGNITER 31
#define RECOVERY_IGNITER 30
#define MAIN_CHUTE_SERVO 47

#define PACKET_DELAY 1000

#define MAX_TILT 30 //degrees
#define ARM_ALT 250 //meters
#define STAGE_2_IGNITION_DELAY 5000 //milliseconds
#define MAIN_CHUTE_DEPLOY_ALT 250 //meters
#define MIN_ALT_FOR_FAILSAFE 150 //meters
#define IGNITE_TIME 1000 //millis

TinyGPSPlus gpsObject;
Adafruit_BMP280 bmp;
Servo mainChuteServo;

SoftwareSerial GPS(GPS_RX, GPS_TX);
SoftwareSerial LoRa(LORA_RX, LORA_TX);

unsigned long oldTime = 0; //used for timing status update broadcasts
String packet = "";
double groundPressure = 0.0;
char loraBuffer;

//state variables relevant to flight state
bool isArmed = false;
double baroAlt = 0;
double latitude = -1.0;
double longitude = -1.0;

//mpu stuff (copy and paste) - https://how2electronics.com/measure-tilt-angle-mpu6050-arduino/#Source_CodeProgram
int16_t AcX,AcY,AcZ;
 
int minVal=265;
int maxVal=402;
 
double xTilt;
double yTilt;
double zTilt;

int flightState = 0 //-1 failsafe, 0 on ground, 1 launch detected, 2 second stage ignition, 3 drogue deployed, 4 main deployed, 5 landed

//time variables
unsigned long launchTime = 0;
unsigned long stageTwoIgnitionTime = 0;
unsigned long recoveryDeployedTime = 0;
unsigned long mainChuteDeployedTime = 0;

void setup() { 
  //DEBUG
  Serial.begin(115200);
  Serial.println("In setup");
  
  GPS.begin(9600);
  LoRa.begin(9600);

  Serial.println("SoftwareSerial done");

  //mpu init
  Wire.begin();
  Wire.beginTransmission(0x68); //TODO: double check IMU addr
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
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
  while (GPS.available() > 0) {
    gpsObject.encode(GPS.read());

    if (gpsObject.location.isUpdated()) {
      latitude = gpsObject.location.lat();
      longitude = gpsObject.location.lng();
    }
  }

  while (LoRa.available()) {
    loraBuffer = LoRa.read();
    if (loraBuffer == 'A') {
      isArmed = true;
    } else if (loraBuffer == 'D' && flightState < 1) { //only accept disarm command if the rocket is on the ground or in failsafe mode
      isArmed = false;
    }
  }

  //read accelerometer data TODO: test
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
   
  xTilt = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  yTilt = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  zTilt = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
  updateStateVars();

  //broadcast update
  if (millis() > (oldTime + PACKET_DELAY)) {
    broadcastUpdate();
    oldTime = millis();
  }

  //disable igniters if they've been on long enough
  if (millis() - stageTwoIgnitionTime > IGNITE_TIME) {
    digitalWrite(STAGE_2_IGNITER, LOW);
  }
  if (millis() - recoveryDeployedTime > IGNITE_TIME) {
    digitalWrite(RECOVERY_IGNITER, LOW);
  }

  //manage flight state
  //first check if the rocket is armed
  if (isArmed) {
    //now act according to flight state
    if (flightState == 0) { //on ground
      //check if accel is positive and if so increment state
      if (baroAlt > ARM_ALT) {
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
          digitalWrite(STAGE_2_IGNITER, HIGH);
          stageTwoIgnitionTime = millis();
        } else { //failsafe
          flightState = -1;
          digitalWrite(RECOVERY_IGNITER, HIGH);
          recoveryDeployedTime = millis();
        }
      }
    } else if (flightState == 2) { //second stage ignited
      //apogee detection
      if (xTilt > 90.0 || (yTilt > 90.0 || zTilt > 90.0)) { //rocket pitchover
        flightSate = 3;
        digitalWrite(RECOVERY_IGNITER, HIGH);
        recoveryDeployedTime = millis();
      }
    } else if (flightState == 3) { //drogue deployed
      //deploy main chute at
      if (baroAlt <= MAIN_CHUTE_DEPLOY_ALT) {
        flightState = 4;
        mainChuteServo.write(180);
      }
    } else if (flightState == 4) { //main deployed
      delay(500); //kinda janky but it should work
      if (baroAlt == bmp.readAltitude(groundPressure)) {
        flighState == 5; //touchdown
      }
    }
  }
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
