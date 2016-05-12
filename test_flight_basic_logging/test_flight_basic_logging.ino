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
      ** MOSI - pin TBD
      ** MISO - pin TBD
      ** CLK - pin TBD
      ** CS - pin TBD
    

  Created 13 May 2016
  By Jamie Sanson
  
  Modified 13 May 2016
  By Jamie Sanson

*/

#include <SPI.h>
#include <SD.h>

const int chipSelect = 0; // Update this with chip select
const String fileName = "flight_log.txt";

void setup() {
  // see if the card is present and can be initialized
  if (!SD.begin(chipSelect)) {
    // don't do anything more
    return;
  }
}

void loop() {

  // Read sensors and build log string here

  File dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile) { // Check to see if file is available
    // Add formatted data here
    dataFile.print(""); 
    // Finalise file write
    dataFile.close(); 
  }
 
}
