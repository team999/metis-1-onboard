/*
  Test flight basic logging

  This sketch contains logic to read sensor values via I2C and dump data to an SD card
  for further analysis on landing. It also communicates basic data over the 433MHz radio band

  This sketch is written for use on a Teensy 3.1/3.2

  The circuit:
  * Components supplying input:
    - Momentary switch 
      * Measurement - pin 2
    - 9-Axis motion sensor MPU9250 Shield
      * Connected to the SMT pads on the underside of the Teensy
    - Venus GPS logger
      * RX     - pin 7
      * TX     - pin 8
      * Other pins connected to independent power supplies
  * Components Outputted to:
    - BOB-00544 microSD card SPI breakout
      * MOSI   - pin 11
      * MISO   - pin 12
      * CLK    - pin 13
      * CS     - as defined in sdChipSelect field
    - RFM22B-S2 434MHz radio tranciever
      * SDI    - pin 11
      * SDO    - pin 12
      * CLK    - pin 13
      * CS     - as defined in radioChipSelect field
      * SDN    - pin 17
      * RX-ANT - pin 18
      * TX-ANT - pin 19



  Created 13 May 2016
  By Jamie Sanson

  Modified 15 July 2016
  By Jamie Sanson

*/
#include <i2c_t3.h>
#include <SD.h>
#include <SPI.h>
#include <MPU9250_helper.h>
#include <TinyGPS++.h>
#include <RF22.h>

// REGION RFM22B
#define RCS 9
#define radioIntPin 0
#define buttonPin 2
RF22 rf22(RCS, radioIntPin);

uint8_t radioDataBuffer[64];
// END REGION

// REGION GPS
#define gpsSerial Serial3

TinyGPSPlus gps;
boolean gpsLocked = false;
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
boolean initialiseOK = true;

int altitudeCount = 0;
float altitudeBuffer = 0;
boolean altInit = false;

int writeCount = 0;
String dataBuffer = "";
// END REGION

// REGION CONFIGURATION
uint8_t OSR = ADC_8192;
int8_t Ascale = AFS_16G;
int8_t Gscale = GFS_250DPS;
int8_t Mscale = MFS_16BITS;  // Choose either 14-bit or 16-bit magnetometer resolution
int8_t Mmode = 0x02; // 2 for 8Hz, 6 for 100Hz continuous

boolean serialDebug = false;

const int sdChipSelect = 10;
const int radioChipSelect = 9;
// END REGION

MPU9250_helper helper(Ascale, Gscale, Mscale, Mmode);

char data[30];

// GPS sending flag
boolean isPostApogee = false;
long flightStartTime;

void setup() {
  gpsSerial.begin(9600);

  pinMode(RCS, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(radioIntPin, INPUT);
  pinMode(buttonPin, INPUT);
  Wire1.begin(I2C_MASTER, 0x000, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);

  if (serialDebug) {
    // block until serial sent to micro
    Serial.begin(115200);
    
    while(!Serial.available()){}
  } 

  while (!digitalRead(buttonPin));

  setupRFM22B();  
  setupSD();
  delay(100);

  printData("\nBeginning radio test:\n");

  // For loop to send radio signal when SPI devices initialised.
  // Carrier should be detectable if initialisation is successful.
  // Total iteration should take just over 30 seconds.
  for (int it = 0; it < 3; it++) {
    rf22.spiWrite(0x07, 0x08); // turn tx on
    delay(5000);
    String toSend = "$$$$radio_init";
    toSend.toCharArray(data, 30);
    rtty_txstring(data);
    rf22.spiWrite(0x07, 0x01); // turn tx off
    printData("Sent signal " + String(it) + " successfully\n");
  }
                        
  setupMPU9250();
  setupAK8963();
  setupMS5637();

  printData("Sending initOk message:\n");

  // Sends another 6 messages stating whether or not initialisation succeeded
  for (int it = 0; it < 3; it++) {
    rf22.spiWrite(0x07, 0x08); // turn tx on
    delay(5000);
    String temp = "$$$$initOK_";
    temp += initialiseOK ? "true" : "false";
    temp.toCharArray(data, 30);
    rtty_txstring(data);
    rf22.spiWrite(0x07, 0x01); // turn tx off

    printData("Sent signal " + String(it) + " successfully\n");
  }
  
  // Block if not initialised properly (Also set LED state)
  while (!initialiseOK);
  
  printData("\nGPS waiting on lock...\n");
  long timeStarted = millis();
  while (!gpsLocked) {
    if (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    if (gps.sentencesWithFix() > 0) {
      printData("Lock took: " + String((millis() - timeStarted)/1000.0, 2) + " s\n\n");
      gpsLocked = true;
    }
  }

  // finalise init logs
  initFileName.toCharArray(fileNameBuffer, 20);
  datafile = SD.open(fileNameBuffer, O_CREAT | O_WRITE);
  datafile.print(dataBuffer);
  datafile.close();
  dataBuffer = "";

  // Initialise CSV columns
  isInitState = false;
  printData("ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,MAG_X,MAG_Y,MAG_Z,ALT,GPS_LAT,GPS_LONG,GPS_ALT,TIME,\n");

  // Finally, transmit another 6 messages stating if we're good to go
  for (int it = 0; it < 3; it++) {
    rf22.spiWrite(0x07, 0x08); // turn tx on
    delay(5000);
    String toSend = "$$$$launchInitOk";
    toSend.toCharArray(data, 30);
    rtty_txstring(data);
    rf22.spiWrite(0x07, 0x01); // turn tx off
  }
}

void loop() {
  if (isPostApogee) {
    sendGPSData(gps.location.lat(), gps.location.lng());
  } else {
    runPreApogeeIteration();

    isPostApogee = (flightStartTime > 0) && ((millis() - flightStartTime) > 25000); // start sending GPS after 25 seconds
  }
}

void runPreApogeeIteration() {
  if (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  helper.readAccelData(accelCount);
  helper.readGyroData(gyroCount);
  helper.readMagData(magCount);

  aRes = helper.getAccelRes();
  gRes = helper.getGyroRes();
  mRes = helper.getMagRes();

  ax = (float)accelCount[0]*aRes;
  ay = (float)accelCount[1]*aRes;
  az = (float)accelCount[2]*aRes;

  if (ay <= -3.0 && flightStartTime == 0) { // Start timer when pulling more than 3.0G 
    flightStartTime = millis();
  }

  gx = (float)gyroCount[0]*gRes;
  gy = (float)gyroCount[1]*gRes;
  gz = (float)gyroCount[2]*gRes;

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
  my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
  mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];

  printData(getLogString(ax, ay, az)+getLogString(gx, gy, gz)+getLogString(mx, my, mz));\
  printData(String(getAltitude()) + ",");
  printData(getGPSString());
  printData(String(millis()) + ",\n");
}

// -------------------------------------------------
//                 Setup functions
// -------------------------------------------------

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
  } else {
    initialiseOK = false;
    printData("MPU9250 failed to initialise\n");
  }
}

void setupAK8963() {
  printData("Reading who-am-i byte of magnetometer\n");
  byte d = helper.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  printData("AK8963 I AM "); printData(String(d, HEX)); printData(", I should be "); printData(String(0x48, HEX) + "\n");

  if (!d == 0x48) {
    initialiseOK = false;
    printData("AK8963 failed to initialise\n");
  }

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
    initialiseOK = false;
    return;
  } else {
    // Detect init file on card
    // Same number of init files as there are data files
    printData("Initialising card\n");
    int fileCount = 1;
    (initFileName + String(fileCount) + ".txt").toCharArray(fileNameBuffer, 20);
    while(SD.exists(fileNameBuffer)) {
      fileCount += 1;
      (initFileName + String(fileCount) + ".txt").toCharArray(fileNameBuffer, 20);
    }
    initFileName = initFileName + String(fileCount) + ".txt";
    dataFileName = dataFileName + String(fileCount) + ".csv";
    Serial.println(initFileName + " , " + dataFileName);
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

  if (nCRC != refCRC) {
    initialiseOK = false;
    printData("MS5637 checksum integrity failed\n");
  }

  // Calculate offset for relative altitude
  float altitudeTemp = 0;
  for(int i = 0; i < 16; i++) {
    altitudeTemp += getAltitude();
  }

  altitudeOffset = altitudeTemp/16;
  altInit = true;
}

void setupRFM22B() {
  if (!rf22.init()) {
    initialiseOK = false;
    printData("\nRFM22B failed to initialise\n");
  } else {
    rf22.setModeTx(); // Turns off Rx
    rf22.setTxPower(RF22_TXPOW_8DBM);
    rf22.setModemConfig(RF22::UnmodulatedCarrier);
    delay(100);
    printData("\nRFM22B initialisation success\n");
  }
}

// -------------------------------------------------
//              Data processing functions
// -------------------------------------------------

float getAltitude(){
  if (!altInit || altitudeCount == 256) {
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

    altitudeBuffer = ((145366.45*(1.0 - pow((Pressure/1013.25), 0.190284)))/3.2808) - altitudeOffset; // Altitude calculation
    altitudeCount = 0;
  } else {
    altitudeCount++;
  }

  return altitudeBuffer;
}

String getLogString(float x, float y, float z) {
  return (String(x, DEC) + "," + String(y, DEC) + "," + String(z, DEC) + ",");
}

String getGPSString() {
  return (String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," + String(gps.altitude.meters()) + ",");
}

void sendGPSData(float lat, float lng) {
  int diff = (int) lat;
  float newLat = lat - (float) diff;
  diff = (int) lng;
  float newLng = lng - (float) diff;

  rf22.spiWrite(0x07, 0x08); // turn tx on
  delay(500);
  sendViaRadio("$$$$");
  sendViaRadio(String(newLat, 6) + "," + String(newLng, 6));
  rf22.spiWrite(0x07, 0x01); // turn tx off
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

    writeCount++;
    dataBuffer += data;
    if (writeCount == 64) {
      datafile = SD.open(fileNameBuffer, FILE_WRITE);
      datafile.print(dataBuffer);
      datafile.close();

      writeCount = 0;
      dataBuffer = "";
    }
  }
}

boolean sendViaRadio(String dataString) {
  dataString.toCharArray(data, 30);
  rtty_txstring(data);
  return true;
}

// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{

  /* Simple function to sent a char at a time to
  ** rtty_txbyte function.
  ** NB Each char is one byte (8 Bits)
  */
  char c;
  c = *string++;
  while ( c != '\0') {
    rtty_txbyte (c);
    c = *string++;
  }
}

void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to
  ** rtty_txbit function.
  ** NB The bits are sent Least Significant Bit first
  **
  ** All chars should be preceded with a 0 and
  ** proceded with a 1. 0 = Start bit; 1 = Stop bit
  **
  ** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
  */
  int i;
  rtty_txbit (0); // Start bit
  // Send bits for for char LSB first
  for (i=0;i<8;i++)
  {
    if (c & 1) {
      rtty_txbit(1);
    } else {
      rtty_txbit(0);
    }
    c = c >> 1;
  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
    if (bit)
    {
      rf22.setFrequency(434.2010);
    }
    else
    {
       rf22.setFrequency(434.2015);
    }
    delayMicroseconds(10000); // 10000 = 100 BAUD 20150
}
