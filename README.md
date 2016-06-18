# test-flight-software
A repo containing code for the teams initial flight. This has no control over the rocket, and is used to log sensor data and communicate wirelessly over the 433MHz band.

Code used with the MPU-9825 board was found in this repo: https://github.com/kriswiner/MPU-9250
Included is a libraries folder, which must be copied into the Arduino global libraries folder for this code to run.

This code requires the TinyGPS++ library. More information here http://arduiniana.org/libraries/tinygpsplus/

This code requires the now deprecated RF22 library found here http://www.airspayce.com/mikem/arduino/RF22/index.html
Note: the use of a deprecated library is not ideal, however for initial implementation the use of naive radio is required, where the SDR on the ground station does not send back an acknowledge.

To run, build circuit and upload code to Teensy of other Arduino compatable microcontroller, with pin numbers adjusted in code to suit your circuit.
