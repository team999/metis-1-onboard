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

// Stores the 16-bit signed accelerometer sensor output
int16_t accelCount[3]; 
float aRes;
float ax,ay,az;

MPU9250_helper helper(Ascale, Gscale, Mscale, Mmode);

void setup() {
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  Serial.begin(115200);

  while(!Serial.available()){}
  
  Serial.println("Reading whoami byte (using wire1)");
  byte c = helper.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  helper.initMPU9250(); 
}

void loop() {
  helper.readAccelData(accelCount);  // Read the x/y/z adc values
  aRes = helper.getAccelRes();
  
  ax = (float)accelCount[0]*aRes;
  ay = (float)accelCount[1]*aRes;
  az = (float)accelCount[2]*aRes; 

  Serial.print("X: "); Serial.print(ax, DEC); 
  Serial.print(" Y: "); Serial.print(ay, DEC); 
  Serial.print(" Z: "); Serial.println(az, DEC); 
}


