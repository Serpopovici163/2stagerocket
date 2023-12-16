#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>

//comment out to remove serial debug output
#define DEBUG

#define STAGE_2_IGNITER 32
#define RECOVERY_IGNITER 34
#define MAIN_CHUTE_SERVO 47
#define RECOVERY_CONTINUITY A0
#define STAGE_2_CONTINUITY A1
#define BATT_SENSE A2

#define PACKET_DELAY_ON_GROUND 5000
#define PACKET_DELAY_IN_AIR 100
#define PACKET_DELAY_SD_CARD_ON_GROUND 250 
#define PACKET_DELAY_SD_CARD_IN_AIR 50 //in effect once vehicle is armed

#define ARM_ALT 250 //meters | min alt for stage 2 ignition
#define STAGE_2_IGNITION_DELAY 5000 //milliseconds | time to stage 2 ignition after launch detection
#define MAIN_CHUTE_DEPLOY_ALT 300 //meters | main chute deployment altitude
#define MIN_FAILSAFE_ALT 50 //meters | min altitude for failsave recovery deployment
#define IGNITE_TIME 1000 //millis | time that ignitor wires stay on

#define ALT_BUFFER_SIZE 10
#define ALT_BUFFER_DELAY 100 //how often alt buffer values are added

//for SD
File logFile;

//baro variables
double groundPressure = 0.0;

//altitude buffer variables
double altBuffer[ALT_BUFFER_SIZE];
int altBufferIndex = 0;
double altBufferAverage = 0.0;
double baroVelocity = 0.0;
unsigned long altitudeBufferTimer = 0;

//vehicle status variables
bool isArmed = false;
double baroAlt = 0;
double latitude = -1.0;
double longitude = -1.0;

//mpu stuff (copy and paste) - https://how2electronics.com/measure-tilt-angle-mpu6050-arduino/#Source_CodeProgram
int16_t AcX,AcY,AcZ; 
int minVal=265;
int maxVal=402;
 
double yAccel = 0.0;
double xTilt = 0.0;
double yTilt = 0.0; //we don't care for roll but log it anyways
double zTilt = 0.0;

int flightState = 0 //-1 failsafe, 0 on ground, 1 launch detected, 2 second stage ignition, 3 drogue deployed, 4 main deployed, 5 landed

//timers
unsigned long oldBroadcastTime = 0; //used for timing status update broadcasts
unsigned long oldLogTime = 0; //used for timing SD card logs

void setup() {

  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("In setup");
  #endif

  //init GPS and LoRa
  Serial2.begin(9600); //LoRa
  Serial3.begin(9600); //GPS
  Serial.println("GPS/LoRa init");
  Serial2.println("INIT_START");

  #ifdef DEBUG
  Serial.println("Serial init done");
  #endif

  //init SD
  if (!SD.begin(SD_CS)) {
    #ifdef DEBUG
    Serial.println("SD_ERROR");
    #endif

    Serial2.println("SD_ERROR");
  }

  logFile = SD.open("log.txt", FILE_WRITE);
  if (logFile) {

    logMessage("INIT_START");

  } else {
    
    #ifdef DEBUG
    Serial.println("FILE_ERROR");
    #endif

    Serial2.println("FILE_ERROR");

  }

  //mpu&baro init
  Wire.begin();
  Wire.beginTransmission(0x68);
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

  #ifdef DEBUG
  Serial.println("I2C_OK");
  #endif

  logMessage("I2C_OK")
  
  //I/O init
  pinMode(STAGE_2_IGNITER, OUTPUT);
  pinMode(RECOVERY_IGNITER, OUTPUT);
  digitalWrite(STAGE_2_IGNITER, LOW);
  digitalWrite(RECOVERY_IGNITER, LOW);
  mainChuteServo.attach(MAIN_CHUTE_SERVO);
  mainChuteServo.write(0); //set servo to max position

  #ifdef DEBUG
  Serial.println("IO_OK");
  #endif

  logMessage("IO_OK")

  //fill alt buffer
  for (int i = 0; i < ALT_BUFFER_SIZE; i++)
    altBuffer[i] = 0.0;
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
  transmitState();
  logState();

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

void updateAccel() {
  //read accelerometer data 
  //TODO: test
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true);
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
  yTilt = g.orientation.y;
  zTilt = g.orientation.z;
}

void updateBaro() {
  baroAlt = bmp.readAltitude(groundPressure);
}

void checkLoRaBuffer() {
  //check if we got commands
  while (Serial2.available()) {

    loraBuffer = LoRa.read();

    if (loraBuffer == 'A') { //arm

      isArmed = true;

    } else if (loraBuffer == 'C') { //continuity check

      if (analogRead(RECOVERY_CONTINUITY) > 500) {
        Serial2.print("1");
      } else { 
        Serial2.print("0");
      }
        
      if (analogRead(STAGE_2_CONTINUITY) > 500) {
        Serial2.println("1");
      } else {
        Serial2.println("0");
      }

    } else if (loraBuffer == 'D' && flightState < 1) { //only accept disarm command if the rocket is on the ground or in failsafe mode
      
      isArmed = false;

    }
  }
}

//custom stack implementation for figuring out whether we're going up or down

/*
we add an altitude value to the buffer
Recompute differences between each set of altitudes within the buffer
average those and save them in altBufferAverage
use altBufferAverage to check if we're ascending or descending
*/

void updateAscentState() {
  if ((millis() - altitudeBufferTimer) > ALT_BUFFER_DELAY) { //add new altitude value to buffer
    addAlt(bmp.readAltitude(groundPressure));
    altitudeBufferTimer = millis();
  }
}

void addAlt(double newAlt) {
  if (altBufferIndex < ALT_BUFFER_SIZE) { //we haven't fully populated the altBuffer array yet
    altBuffer[altBufferIndex] = newAlt;
    altBufferIndex++;
  } else { //altBuffer now functions as stack
    for (int i = 0; i < (ALT_BUFFER_SIZE - 1); i++) 
      altBuffer[i] = altBuffer[i+1];
    altBuffer[ALT_BUFFER_SIZE-1] = newAlt;

    //recompute altitude buffer average
    altBufferAverage = 0.0;
    for (int i = 0; i < ALT_BUFFER_SIZE; i++) 
      altBufferAverage += altBuffer[i+1] - altBuffer[i];
    altBufferAverage = altBufferAverage / (ALT_BUFFER_SIZE - 1);
    baroVelocity = altBufferAverage / ALT_BUFFER_DELAY;
  }
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
  //break if we aren't armed
  if (!isArmed) {
    return;
  }

  if (flightState == 0) { //on ground
    if (yAccel > 0) {
      flightState++;
      launchTime = millis();
    }
  } else if (flightState == 1) { //launch detected
    if ((altBufferAverage < 0) && baroAlt > MIN_FAILSAFE_ALT) { //if we're falling and we've exceeded the minimum failsafe altitude, open chute
      flightState == -1;
      failsafe(); //TODO: implement
    } else if (millis() - launchTime > STAGE_2_IGNITION_DELAY && baroAlt > ARM_ALT) { //if we're ascending, the time to s2 ignition has passed, and we've exceeded minimum stage 2 ignition altitude, ignite stage 2
      flightState++;
      //ignite stage two
      stageTwoIgnitionTime = millis();
      digitalWrite(STAGE_2_IGNITER, HIGH);
    }
  } else if (flightState == 2) { //stage two ignited
    if (altBufferAverage < 0) {
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

/*
notes

packet format for telemetry:
armState | flightState | battSense | baroAlt | baroVelocity | satCount | latitude | longitude
*/

void transmitState() {
  if (flightState > 0 && flightState < 5) { //we're airborne, increase transmit rate
    
    if (millis() - oldBroadcastTime > PACKET_DELAY_IN_AIR) {
      Serial2.println(makeTelemetryPacket);
      oldBroadcastTime = millis();
    }

  } else { //we're on the ground, decrease transmit rate

    if (millis() - oldBroadcastTime > PACKET_DELAY_ON_GROUND) {
      Serial2.println(makeTelemetryPacket);
      oldBroadcastTime = millis();
    }

  }
}

String makeTelemetryPacket() {
  String packet = "";
  packet += armState;
  packet += "|";
  packet += flightState;
  packet += "|";
  packet += analogRead(BATT_SENSE);
  packet += "|";
  packet += baroAlt;
  packet += "|";
  packet += baroVelocity;
  packet += "|";
  packet += gpsObject.satellites.value();
  packet += "|";
  packet += latitude;
  packet += "|";
  packet += longitude;
}

/*
notes

packet format for SD:
armState | flightState | battSense | baroAlt | baroVelocity | satCount | latitude | longitude | AcX | AcY | AcZ | xTilt | yTilt | zTilt | altBufferAverage | yAccel
*/

void logState() {
  if (isArmed) {
    if (millis() - oldLogTime > PACKET_DELAY_SD_CARD_IN_AIR) {
      logMessage(makeSDPacket);
      oldLogTime = millis();
    }
  } else {
    if (millis() - oldLogTime > PACKET_DELAY_SD_CARD_ON_GROUND) {
      logMessage(makeSDPacket);
      oldLogTime = millis();
    }
  }
}

String makeSDPacket() {
  String packet = makeTelemetryPacket();
  packet += "|";
  packet += AcX;
  packet += "|";
  packet += AcY;
  packet += "|";
  packet += AcZ;
  packet += "|";
  packet += xTilt;
  packet += "|";
  packet += yTilt;
  packet += "|";
  packet += zTilt;
  packet += "|";
  packet += altBufferAverage;
  packet += "|";
  packet += yAccel;
}

void logMessage(String message) {
  logFile.println(message);
}

void disableIgniters() {
  if (millis() - stageTwoIgnitionTime > IGNITE_TIME) {
    digitalWrite(STAGE_2_IGNITER, LOW);
  }
  if (millis() - recoveryDeployedTime > IGNITE_TIME) {
    digitalWrite(RECOVERY_IGNITER, LOW);
  }
}