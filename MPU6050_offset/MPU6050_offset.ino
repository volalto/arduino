/*
* Gyro offset calculator for InvenSense MPU6050
* by Tommaso Falchi Delitala (volalto86@gmail.com)
* https://github.com/volalto/arduino

* The sketch calculates gyro offsets for MPU6050 sensor.
* The algorithms iterates until offset values converge to a stable figure
* (usually after 2-3 iterations).
* Offsets affect raw measurements and must be set after every power-cycle
* Make sure the sensor is still and it has been running for 5-10 min,
* so offset are calculated at the proper operating temperature.
*
* Based on MPU6050_raw example by Jeff Rowberg
* Requires: I2CDev libs for MPU6050 https://github.com/jrowberg/i2cdevlib
*
* (c) Copyright 2013 Tommaso Falchi Delitala
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version. http://www.gnu.org/licenses
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// Number of samples to collect
#define SAMPLE_N 1024

int16_t w[3];
int32_t w_avg[3];
int8_t offset[3], old_offset[3];
unsigned int i = 0;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Init accumulator variables
  w_avg[0] = 0;
  w_avg[1] = 0;
  w_avg[2] = 0;

  // Init serial communication
  Serial.begin(57600);

  // Init device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Reset gyro offset
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  Serial.println("Sensor offsets before calibration (3 x acc, 3 x gyro)...");
  // Save old offsets
  old_offset[0] = accelgyro.getXGyroOffset();
  old_offset[1] = accelgyro.getYGyroOffset();
  old_offset[2] = accelgyro.getZGyroOffset();

  Serial.print(accelgyro.getXAccelOffset());
  Serial.print("\t");
  Serial.print(accelgyro.getYAccelOffset());
  Serial.print("\t");
  Serial.print(accelgyro.getZAccelOffset());
  Serial.print("\t");
  Serial.print(old_offset[0]);
  Serial.print("\t");
  Serial.print(old_offset[1]);
  Serial.print("\t");
  Serial.println(old_offset[2]);

}

void loop() {

  if (i < SAMPLE_N) {
    printProgress();
    // read raw accel/gyro measurements from device
    accelgyro.getRotation(&w[0], &w[1], &w[2]);

    // Add to accumulators
    w_avg[0] += w[0];
    w_avg[1] += w[1];
    w_avg[2] += w[2];

    ++i;
    delay(10);
  }
  else {

    //Calculate average offsets
    w_avg[0] /= SAMPLE_N;
    w_avg[1] /= SAMPLE_N;
    w_avg[2] /= SAMPLE_N;

    // Scale and convert to int8_t
    // Note: scaling factor thanks to luisrodenas
    // (http://www.i2cdevlib.com/forums/topic/91-how-to-decide-gyro-and-accelerometer-offsett/ )
    offset[0] = (int8_t) (- w_avg[0] / 4);
    offset[1] = (int8_t) (- w_avg[1] / 4);
    offset[2] = (int8_t) (- w_avg[2] / 4);

    Serial.print("\nAverage raw gyro readings: ");
    Serial.print(w_avg[0]);
    Serial.print(" ");
    Serial.print(w_avg[1]);
    Serial.print(" ");
    Serial.println(w_avg[2]);

    if (offset[0] == 0 && offset[1] == 0 && offset[2] == 0) {
      Serial.print("Final gyro offset: ");
      Serial.print(old_offset[0]);
      Serial.print(" ");
      Serial.print(old_offset[1]);
      Serial.print(" ");
      Serial.println(old_offset[2]);

      // No need for further adjustments..enter and endless loop to celebrate!
      Serial.println("Done! Stopping...");
      while (true)
        delay(1000);
    }

    // Correct old offset
    accelgyro.setXGyroOffset(old_offset[0] + offset[0]);
    accelgyro.setYGyroOffset(old_offset[1] + offset[1]);
    accelgyro.setZGyroOffset(old_offset[2] + offset[2]);

    old_offset[0] = accelgyro.getXGyroOffset();
    old_offset[1] = accelgyro.getYGyroOffset();
    old_offset[2] = accelgyro.getZGyroOffset();

    Serial.print("New gyro offsets: ");
    Serial.print(old_offset[0]);
    Serial.print(" ");
    Serial.print(old_offset[1]);
    Serial.print(" ");
    Serial.println(old_offset[2]);

    Serial.println("Setting new values and starting over..");

    w_avg[0] = 0;
    w_avg[1] = 0;
    w_avg[2] = 0;
    i = 0;

    delay(1000);
  }
}

void printProgress()
{
  //Print 80 characters per line
  if ((i % 80) == 79)
    Serial.println('.');
  else
    Serial.print('.');
}

