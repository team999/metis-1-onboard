/*
  Test flight basic logging

  This sketch contains logic to read sensor values via I2C and dump data to an SD card
  for further analysis on landing. It also communicates basic data over the 433MHz radio band

  The circuit:
  * Components supplying input to device:
    - 9-Axis motion sensor MPU9250 Shield
  * Components Outputted to:
    - BOB-00544 microSD card SPI breakout
      * SD card attached to SPI bus as follows:
      ** MOSI - pin 11
      ** MISO - pin 12
      ** CLK - 13
      ** CS - 4


  Created 13 May 2016
  By Jamie Sanson

  Modified 7 June 2016
  By Jamie Sanson

*/

#include <i2c_t3.h>
#include <SPI.h>
#include <MPU9250_helper.h>

// Defining the scaling of sensor values
int8_t Ascale = AFS_16G;
int8_t Gscale = GFS_250DPS;
int8_t Mscale = MFS_16BITS;  // Choose either 14-bit or 16-bit magnetometer resolution
int8_t Mmode = 0x02; // 2 for 8Hz, 6 for 100Hz continuous

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

float aRes, gRes, mRes;
float ax,ay,az, gx, gy, gz, mx, my, mz;

// Biases resulting from calibration
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};

MPU9250_helper helper(Ascale, Gscale, Mscale, Mmode);

void setup() {
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  Serial.begin(115200);

  while(!Serial.available()){}

  setupMPU9250();
  setupAK8963();  
}

void loop() {
  helper.readAccelData(accelCount);  // Read the x/y/z adc values
  helper.readGyroData(gyroCount);
  helper.readMagData(magCount);
  
  aRes = helper.getAccelRes();
  gRes = helper.getGyroRes();
  mRes = helper.getMagRes();
  
  ax = (float)accelCount[0]*aRes;
  ay = (float)accelCount[1]*aRes;
  az = (float)accelCount[2]*aRes; 

  gx = (float)gyroCount[0]*gRes;
  gy = (float)gyroCount[1]*gRes;
  gz = (float)gyroCount[2]*gRes; 
  
  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
  my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
  mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   

  Serial.print("X: "); Serial.print(ax, DEC); Serial.print(","); Serial.print(gx, DEC); Serial.print(","); Serial.print(mx,DEC); 
  Serial.print(" Y: "); Serial.print(ay, DEC); Serial.print(","); Serial.print(gy, DEC); Serial.print(","); Serial.print(my,DEC);
  Serial.print(" Z: "); Serial.println(az, DEC); Serial.print(","); Serial.print(gz, DEC); Serial.print(","); Serial.print(mz,DEC);
}

void setupMPU9250() {
  Serial.println("Reading who-am-i byte of MPU9250");
  byte c = helper.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(", I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) {
    Serial.println("MPU9250 online");
    Serial.println("Calibrating...\n");

    helper.calibrateMPU9250(gyroBias, accelBias);

    Serial.println("Accelerometer bias: (mg)"); 
    Serial.println("X: " + (String)(1000*accelBias[0]) + " Y: " + (String)(1000*accelBias[1]) + " Z: " + (String)(1000*accelBias[2]));

    Serial.println("Gyro bias: (o/s)"); 
    Serial.println("X: " + (String)gyroBias[0] + " Y: " + (String)gyroBias[1] + " Z: " + (String)gyroBias[2]);

    helper.initMPU9250(); 
    Serial.println("\nMPU9250 initialized for active data mode....\n");
  }
}

void setupAK8963() {
  Serial.println("Reading who-am-i byte of magnetometer");
  byte d = helper.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(", I should be "); Serial.println(0x48, HEX);

  helper.initAK8963(magCalibration); 

  Serial.print("Calibrating...\n");
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
  Serial.println("\nAK8963 initialized for active data mode....");
}

