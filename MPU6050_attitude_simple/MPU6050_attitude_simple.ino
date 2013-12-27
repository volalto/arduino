
/*
* Simple attitude integrator for Arduino +  InvenSense MPU-5060
* by Tommaso Falchi Delitala (volalto86@gmail.com)
* based on simple MPU-5060 by Jeff Rowberg
*
* The sketch outputs sensor attitude in deg*100 over serial port at 57600 bps.
* Motion integration is performed at 100 Hz by considering gyrs data only. 
* Serial output rate is adjustable by setting the OUT_INTERVAL constant.
* Initial x,y attitude is estimated from accelerometers readings.
*
* Requires: I2CDev libs for MPU-5060 https://github.com/jrowberg/i2cdevlib
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
#include "math.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// Serial output rate in microseconds
#define OUT_INTERVAL 100000UL // Default: 100 ms

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t w[3]; // omega: angular velocity
long alpha[3]; // alpha: angle
// Time related values
unsigned long t = 0, t_last = 0, last_print = 0, dt = 0;

// Scale factor for gyros ( 131 for range +/- 250 deg/s):
const int scale = 131;

void setup() 
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Clear memory
  memset(w, 0, sizeof(w));
  memset(alpha, 0, sizeof(alpha));

  // initialize serial communication
  Serial.begin(57600);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Set gyro offsets (see my other sketch for offset calculation)
  // Gyro offsets must be set after every power cycle
  // My default values:-786, -277, 840, 0, 0, 0
  accelgyro.setXGyroOffset(79);
  accelgyro.setYGyroOffset(-42);
  accelgyro.setZGyroOffset(-26);

  Serial.print("Internal sensor offsets...");
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");

  // Estimate initial attitude
  float alpha_x_start, alpha_y_start; 
  estimateAttitudeAcc(&alpha_x_start, &alpha_y_start);
  
  Serial.print("Initial attitude estimation: alpha_x = ");
  Serial.print(alpha_x_start);
  Serial.print(" alpha_y = ");
  Serial.println(alpha_y_start);
  
  // Convert float angles to raw values used by the integration algorithm
  alpha[0] = alpha_x_start * 131.0E2;
  alpha[1] = alpha_y_start * 131.0E2;
  
  // Init integration interval timer
  t_last = micros();
}

void loop() 
{
  // Get current time stamp
  t = micros();

  // Run the integration loop at 100 Hz (= 10 ms)
  dt = t - t_last; //us
  if (dt < 10000UL)
    return;

  // Get angular rate measurement from MPU-6050
  accelgyro.getRotation(&w[0], &w[1], &w[2]);
  t_last = t;

  // Integration of raw values (i.e. deg * 131E2)
  alpha[0] += w[0];
  alpha[1] += w[1];
  alpha[2] += w[2];

  //Print at OUT_RATE
  if (t - last_print >= OUT_INTERVAL) {
    printSerial();
    last_print = t;
  }
}

// Estimate attitude from accelerometers reading.
// The sensor must be at rest
// Output: alpha_x/y contains angles in deg
void estimateAttitudeAcc(float *alpha_x, float *alpha_y) 
{
  // Init attitude using gravity vector
  // x-z plane: alpha_y = atan2(ax, az)
  // y-z plane: alpha_x = atan2(ay, az)
  int16_t a[3];
  memset(a, 0, sizeof(a));
  accelgyro.getAcceleration(&a[0], &a[1], &a[2]);
  
  *alpha_x = atan2(a[1], a[2]) * 180/M_PI;  // x
  *alpha_y = -atan2(a[0], a[2]) * 180/M_PI; // y
}

void printSerial()
{
  // Scale and clip to +- 180 deg
  Serial.print(clip180(alpha[0] / scale));
  Serial.print(" ");
  Serial.print(clip180(alpha[1] / scale));
  Serial.print(" ");
  Serial.println(clip180(alpha[2] / scale));
}

// Clip angle to +/- 180 deg 
long clip180(long raw)
{
  if (abs(raw) > 18000) {
    int sign = (raw >= 0) ? -1 : 1;
    return sign * (18000 - (abs(raw) - 18000));
  }
  else
    return raw;
}
