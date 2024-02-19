#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>

#define STAGE_2_IGNITER 32
#define RECOVERY_IGNITER 34
#define MAIN_CHUTE_SERVO 47

#define PACKET_DELAY_ON_GROUND 5000
#define PACKET_DELAY_IN_AIR 100
#define PACKET_DELAY_SD_CARD 50 //in effect once vehicle is armed

#define ARM_ALT 250 //meters | min alt for stage 2 ignition
#define STAGE_2_IGNITION_DELAY 5000 //milliseconds | time to stage 2 ignition after launch detection
#define MAIN_CHUTE_DEPLOY_ALT 300 //meters | main chute deployment altitude
#define MIN_FAILSAFE_ALT 50 //meters | min altitude for failsave recovery deployment
#define IGNITE_TIME 1000 //millis | time that ignitor wires stay on

#define ALT_BUFFER_SIZE 20 //MUST BE EVEN

//mpu stuff (copy and paste) - https://how2electronics.com/measure-tilt-angle-mpu6050-arduino/#Source_CodeProgram

TinyGPSPlus gpsObject;
Adafruit_BMP280 bmp;
Servo mainChuteServo;

//buffers
char loraBuffer;
String packet = "";

//timers
unsigned long oldBroadcastTime = 0; //used for timing status update broadcasts
unsigned long oldLogTime = 0; //used for timing SD card logs
unsigned long launchTime = 0;
unsigned long stageTwoIgnitionTime = 0;
unsigned long drogueDeployedTime = 0;
unsigned long mainDeployedTime = 0;
unsigned long touchDownTime = 0;

//ascent tracker variables
double altBuffer[ALT_BUFFER_SIZE];
int altBufferIndex = 0;
double altBufferAverage = 0.0;
bool isAscending = false;

//vehicle status variables
double groundPressure = 0.0;
bool isArmed = false;
double baroAlt = 0;
double latitude = -1.0;
double longitude = -1.0;
int16_t AcX,AcY,AcZ;
 
int minVal=265;
int maxVal=402;
 
double xTilt;
double yTilt;
double zTilt;

int flightState = 0 //-1 failsafe, 0 on ground, 1 launch detected, 2 second stage ignition, 3 drogue deployed, 4 main deployed, 5 landed

void setup() {
  //DEBUG
  Serial.begin(115200);
  Serial.println("In setup");

  //init GPS and LoRa
  Serial2.begin(9600); //LoRa
  Serial3.begin(9600); //GPS
  Serial.println("GPS/LoRa init");
  Serial2.println("Init begin");

  //mpu/baro init
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
  Serial2.println("I2C done");

  pinMode(STAGE_2_IGNITER, OUTPUT);
  pinMode(RECOVERY_IGNITER, OUTPUT);
  digitalWrite(STAGE_2_IGNITER, LOW);
  digitalWrite(RECOVERY_IGNITER, LOW);
  mainChuteServo.attach(MAIN_CHUTE_SERVO);
  mainChuteServo.write(0); //set servo to max position

  Serial.println("I/O done");
  Serial2.println("I/O done");

  //fill alt buffer
  for (int i = 0; i < ALT_BUFFER_SIZE; i++) {
    altBuffer[i] = 0.0;
  }
}

void loop() {
  //update sensors
  updateGPS();
  updateAccel();
  updateGyro();
  updateBaro();

  //check if we have commands to process
  checkLoRaBuffer();

  //check ascent rate
  updateAscentState();

  //update state machine
  updateFlightState();

  //transmit and log vehicle state if needed
  transmitAndLog();

  //disable igniters if they've been on long enough
  disableIgniters();
}

void updateGPS() {
  //update GPS data
  while (Serial3.available() > 0) {
    gpsObject.encode(GPS.read());

    if (gpsObject.location.isUpdated()) {
      latitude = gpsObject.location.lat();
      longitude = gpsObject.location.lng();
    }
  }
}

void checkLoRaBuffer() {
  //check if we got commands
  while (Serial2.available()) {
    loraBuffer = LoRa.read();
    if (loraBuffer == 'A') { //arm
      isArmed = true;
    } else if (loraBuffer == 'C') { //continuity check
      //TODO: figure out how to do this part
    } else if (loraBuffer == 'D' && flightState < 1) { //only accept disarm command if the rocket is on the ground or in failsafe mode
      isArmed = false;
    }
  }
}

void updateAccel() {
  //read accelerometer data 
  //TODO: test
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

  yAccel = -1*a.acceleration.y;
}

void updateGyro() {
  xTilt = g.orientation.x;
  zTilt = g.orientation.z;
}

void updateBaro() {
  baroAlt = bmp.readAltitude(groundPressure);
}

void transmitAndLog() {
  //broadcast update
  if (isArmed) {
    if (millis() > (oldBroadcastTime + PACKET_DELAY_IN_AIR)) {
      broadcastUpdate();
      oldBroadcastTime = millis();
    }
    if (millis() > (oldLoGTime + PACKET_DELAY_SD_CARD)) {
      logToSD();
      oldLogTime = millis();
    }
  } else { //do both at the same time if we're disarmed
    if (millis() > (oldBroadcastTime + PACKET_DELAY_ON_GROUND)) {
      broadcastUpdate();
      logToSD();
      oldBroadcastTime = millis();
    }
  }
}

void updateAscentState() {
  /* add value to buffer
   * increment index
   * compute average
   * assign ascentState variable
   */

   //add value to buffer
   altBuffer[altBufferIndex] = baroAlt;

   //increment buffer
   if (altBufferIndex == (ALT_BUFFER_SIZE - 1)) { //we've reached the end so wrap around
    altBufferIndex = 0;
   } else {
    altBufferIndex++;
   }

   //compute average
   altBufferAverage = 0.0;
   for (int i = 0; i < ALT_BUFFER_SIZE/2; i++) {
    altBufferAverage += (altBuffer[i*2+1] - altBuffer[i*2]);
   }
   altBufferAverage /= (ALT_BUFFER_SIZE / 2);

   //assign ascentState
   if (altBufferAverage > 0) {
    ascentState = true;
   } else {
    ascentState = false;
   }
}

void logToSD() {
  
}

void broadcastUpdate() {
  
}

/*
 * notes
 * 
 * run consistent ascent algorithm
 * 
 * positive accel --> launch detected
 * if exceeded failsafe alt and falling --> failsafe
 * if exceeded arming alt and climbing and stage2ignitiondelay elapsed --> ignite stage 2
 * if falling --> drogue deploy
 * if below main deploy alt --> main deploy
 * if no change for x seconds --> landed (drop transmit interval so probably disarm)
 */

void updateFlightState() {
  //nullify flight state and break if we aren't armed
  if (!isArmed) {
    return;
  }

  if (flightState == 0) { //on ground
    if (yAccel > 0) {
      flightState++;
      launchTime = millis();
    }
  } else if (flightState == 1) { //launch detected
    if (!ascentState && baroAlt > MIN_FAILSAFE_ALT) { 
      flightState == -1;
      failsafe(); //TODO: implement
    } else if (millis() - launchTime > STAGE_2_IGNITION_DELAY && (ascentState && baroAlt > ARM_ALT)) {
      flightState++;
      //ignite stage two
      stageTwoIgnitionTime = millis();
      digitalWrite(STAGE_2_IGNITER, HIGH);
    }
  } else if (flightState == 2) { //stage two ignited
    if (!ascentState) {
      flightState++;
      drogueDeployTime = millis();
      digitalWrite(RECOVERY_IGNITER, HIGH);
    }
  } else if (flightState == 3) { //drogue deployed
    if (baroAlt <= MAIN_CHUTE_DEPLOY_ALT) {
      flightState++;
      mainDeployTime = millis();
      mainChuteServo.write(180);
    }
  } else if (flightState == 4) { //final descent
    //TODO: implement
  }
}

void disableIgniters() {
  //disable igniters if they've been on long enough
  if (millis() - stageTwoIgnitionTime > IGNITE_TIME) {
    digitalWrite(STAGE_2_IGNITER, LOW);
  }
  if (millis() - recoveryDeployedTime > IGNITE_TIME) {
    digitalWrite(RECOVERY_IGNITER, LOW);
  }
}
