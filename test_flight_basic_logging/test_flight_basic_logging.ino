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

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float aRes;
float ax,ay,az;

MPU9250_helper sensor_helper;

void setup() {
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  Serial.begin(38400);

  sensor_helper.waitForInput();

  Serial.println("Reading whoami byte (using wire1)");
  byte c = sensor_helper.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  delay(1000);
  sensor_helper.initMPU9250(); 
}

void loop() {
  sensor_helper.readAccelData(accelCount);  // Read the x/y/z adc values
  aRes = sensor_helper.getAccelRes();
  
  // Now we'll calculate the accleration value into actual g's
  ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
  ay = (float)accelCount[1]*aRes; // - accelBias[1];   
  az = (float)accelCount[2]*aRes; // - accelBias[2];  

  Serial.print("X: "); Serial.print(ax, DEC); 
  Serial.print(" Y: "); Serial.print(ay, DEC); 
  Serial.print(" Z: "); Serial.println(az, DEC); 

  delay(100);
}


