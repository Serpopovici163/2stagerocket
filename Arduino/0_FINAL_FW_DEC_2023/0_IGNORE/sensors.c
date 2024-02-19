void updateGPS(double *latitude, double *longitude) {
  //update GPS data
  while (Serial3.available() > 0) {
    gpsObject.encode(GPS.read());

    if (gpsObject.location.isUpdated()) {
      *latitude = gpsObject.location.lat();
      *longitude = gpsObject.location.lng();
    }
  }
}

void updateAccel(int16_t *AcX, int16_t *AcY, int16_t *AcZ, int minVal, int maxVal, double *xTilt, double *yTilt, double *zTilt, double *yAccel) {
  //read accelerometer data 
  //TODO: test
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true);
  *AcX=Wire.read()<<8|Wire.read();
  *AcY=Wire.read()<<8|Wire.read();
  *AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
   
  *xTilt = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  *yTilt = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  *zTilt = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  *yAccel = -1*a.acceleration.y;
}

void updateGyro(double *xTilt, double *zTilt) {
  *xTilt = g.orientation.x;
  *zTilt = g.orientation.z;
}

void updateBaro(double *baroAlt) {
  baroAlt = bmp.readAltitude(groundPressure);
}