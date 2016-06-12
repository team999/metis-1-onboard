/*
  Test flight basic logging

  This sketch contains logic to read sensor values via I2C and dump data to an SD card
  for further analysis on landing. It also communicates basic data over the 433MHz radio band

  This sketch is written for use on a Teensy 3.1/3.2

  The circuit:
  * Components supplying input to device:
    - 9-Axis motion sensor MPU9250 Shield
  * Components Outputted to:
    - BOB-00544 microSD card SPI breakout
      * SD card attached to SPI bus as follows:
      ** MOSI - pin 11
      ** MISO - pin 12
      ** CLK - 13
      ** CS - as defined in sdChipSelect field


  Created 13 May 2016
  By Jamie Sanson

  Modified 12 June 2016
  By Jamie Sanson

*/

#include <i2c_t3.h>
#include <SD.h>
#include <SPI.h>
#include <MPU9250_helper.h>

// REGION CONFIGURATION
uint8_t OSR = ADC_8192;
int8_t Ascale = AFS_16G;
int8_t Gscale = GFS_250DPS;
int8_t Mscale = MFS_16BITS;  // Choose either 14-bit or 16-bit magnetometer resolution
int8_t Mmode = 0x02; // 2 for 8Hz, 6 for 100Hz continuous
boolean serialDebug = true;
const int sdChipSelect = 20;
// END REGION

// REGION SENSOR FIELDS
uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature
float altitudeOffset;

float aRes, gRes, mRes;
float ax,ay,az, gx, gy, gz, mx, my, mz;

// Biases resulting from calibration
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};
// END REGION

// REGION DATA LOGGING
File datafile;
String initFileName = "init/init";
String dataFileName = "data/data";
char fileNameBuffer[20];
boolean isInitState = true;
// END REGION

MPU9250_helper helper(Ascale, Gscale, Mscale, Mmode);

void setup() {
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  
  if (serialDebug) {
    // block until serial sent to micro
    Serial.begin(115200);
    while(!Serial.available()){}
  }

  setupSD();
  setupMPU9250();
  setupAK8963();  
  setupMS5637();

  // Initialise CSV columns
  isInitState = false;
  printData("ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_y,GYRO_Z,MAG_X,MAG_Y,MAG_Z,ALT,TIME,\n");
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

  printData(getLogString(ax, ay, az)+getLogString(gx, gy, gz)+getLogString(mx, my, mz));
  printData(String(getAltitude()) + "," + String(micros()) + ",\n");

}

void setupMPU9250() {
  printData("Reading who-am-i byte of MPU9250\n");
  byte c = helper.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  
  printData("MPU9250 I AM "); printData(String(c, HEX)); printData(", I should be "); printData(String(0x71, HEX) + "\n");

  if (c == 0x71) {
    printData("MPU9250 online\n");
    printData("Calibrating...\n\n");

    helper.calibrateMPU9250(gyroBias, accelBias);

    printData("Accelerometer bias: (mg)\n"); 
    printData("X: " + (String)(1000*accelBias[0]) + " Y: " + (String)(1000*accelBias[1]) + " Z: " + (String)(1000*accelBias[2]) + "\n");

    printData("Gyro bias: (o/s)\n"); 
    printData("X: " + (String)gyroBias[0] + " Y: " + (String)gyroBias[1] + " Z: " + (String)gyroBias[2] + "\n");

    helper.initMPU9250(); 
    printData("\nMPU9250 initialized for active data mode....\n\n");
  }
}

void setupAK8963() {
  printData("Reading who-am-i byte of magnetometer\n");
  byte d = helper.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  printData("AK8963 I AM "); printData(String(d, HEX)); printData(", I should be "); printData(String(0x48, HEX) + "\n");

  helper.initAK8963(magCalibration); 

  printData("Calibrating...\n");
  printData("X-Axis sensitivity adjustment value "); printData(String(magCalibration[0], 2) + "\n");
  printData("Y-Axis sensitivity adjustment value "); printData(String(magCalibration[1], 2) + "\n");
  printData("Z-Axis sensitivity adjustment value "); printData(String(magCalibration[2], 2) + "\n");
  printData("\nAK8963 initialized for active data mode....\n");
}

void setupSD() {
  if (!SD.begin(sdChipSelect)) {
    printData("Card failed to initialise\n");
    return;
  } else {
    // Detect init file on card
    // Same number of init files as there are data files
    int fileCount = 1;
    (initFileName + String(fileCount) + ".log").toCharArray(fileNameBuffer, 20);
    while(SD.exists(fileNameBuffer)) {
      fileCount += 1;
      (initFileName + String(fileCount) + ".log").toCharArray(fileNameBuffer, 20);
    }
    initFileName = initFileName + String(fileCount) + ".log";
    dataFileName = dataFileName + String(fileCount) + ".csv";
  }
}

void setupMS5637(){
  helper.resetMS5637();
  delay(100);
  printData("MS5637 pressure sensor reset...\n");
  // Read PROM data from MS5637 pressure sensor
  helper.readPromMS5637(Pcal);
  printData("PROM data read:\n");
  printData("C0 = "); printData(String(Pcal[0]) + "\n");
  unsigned char refCRC = Pcal[0] >> 12;
  printData("C1 = "); printData(String(Pcal[1]) + "\n");
  printData("C2 = "); printData(String(Pcal[2]) + "\n");
  printData("C3 = "); printData(String(Pcal[3]) + "\n");
  printData("C4 = "); printData(String(Pcal[4]) + "\n");
  printData("C5 = "); printData(String(Pcal[5]) + "\n");
  printData("C6 = "); printData(String(Pcal[6]) + "\n");
  
  nCRC = helper.checkMS5637CRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  printData("Checksum: " + String(nCRC) + ", should be: " + String(refCRC) + "\n");  

  // Calculate offset for relative altitude
  float altitudeTemp = 0;
  for(int i = 0; i < 16; i++) {
    altitudeTemp += getAltitude();
  }

  altitudeOffset = altitudeTemp/16;
}

float getAltitude(){
  D1 = helper.MS5637Read(ADC_D1, OSR);  // get raw pressure value
  D2 = helper.MS5637Read(ADC_D2, OSR);  // get raw temperature value
  dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
  OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
  SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);

  Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;   // First-order Temperature in degrees celsius

  // Second order corrections
  if(Temperature > 20) 
  {
    T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
    OFFSET2 = 0;
    SENS2 = 0;
  }
  if(Temperature < 20)       // correction for low temperature
  {
    T2      = 3*dT*dT/pow(2, 33); 
    OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
    SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
  } 
  if(Temperature < -15)      // correction for very low temperature
  {
    OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
    SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
  }
  // End of second order corrections

  Temperature = Temperature - T2/100;
  OFFSET = OFFSET - OFFSET2;
  SENS = SENS - SENS2;

  Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa

  return ((145366.45*(1.0 - pow((Pressure/1013.25), 0.190284)))/3.2808) - altitudeOffset; // Altitude calculation  
}


// -------------------------------------------------
//                Utility functions
// -------------------------------------------------
void printData(String data) {
  if (serialDebug) {
    Serial.print(data);
  } else {
    if (isInitState) {
      initFileName.toCharArray(fileNameBuffer, 20);
    } else {
      dataFileName.toCharArray(fileNameBuffer, 20);
    }
    datafile = SD.open(fileNameBuffer, FILE_WRITE);      
    datafile.print(data);
    datafile.close();
  }
}

String getLogString(float x, float y, float z) {
  return (String(x, DEC) + "," + String(y, DEC) + "," + String(z, DEC) + ",");
}

