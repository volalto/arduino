arduino
=======

My experiments with Arduino.

MPU6050_attitude_simple
-----------------------
The sketch simply integrates raw gyro readings from MPU6050 to estimate the sensor attitude (pitch / roll / yaw). Initial attitude is determined using accelerometers.

No advanced technique like Complementary Filter, quaternions, etc is used, so the output has a visible drift and the algorithm could explode when starting from "critical" initial conditions (i.e. +/-90 deg).

MPU6050_offset
--------------
Utility to calculate offset values for MPU6050 gyros.
