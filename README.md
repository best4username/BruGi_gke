# BruGi_gke

BruGi is a camera gimble control program the main repostory for which is here:

https://sourceforge.net/projects/brushless-gimbal-brugi/

At this date the last update was BruGi_050_r217 30 May 2014.

In what follows it is assumed you have used BruGi before and are familiar with the Brushless Gimbal Tool (BGT). If not check the repository documentation above.

Many flight controllers are based on the Invensense MPU6xxx but few have temperature compensation for the gyrometers. The gyrometers have a close to linear offset drift with temperature which may be compensated for fairly simply.

The new gyro calibration routine now takes several readings at each of two temperature points and determines the offset at the first temperature and the gradient  of the drift with increasing temperature. The first temperature is usually room temperature and the second is 10C higher. The temperature is obtained from the MPU6xxx on chip temperature sensor.

WARNING: note any tuning settings you may have as the your settings will be overwritten by the new defaults.

The calibration process is:

1. disable the motors (BGT Power Scale checkbox)
2. ensure the camera platform is not moving
3.  start the gyro calibration (BGT Calibration Tab)
4. warm the MPU6xxx sensor on the camera platform usually with a hair dryer or other heat source
5. wait until the calibration completes (BGT message window)

For safety monitor the temperature using the Live View (BGT settings Tab).

You should only need to do this calibration once unless you intend flying at temperatures which are dramatically outside the calibration range because the compensation scheme and the gyro drift is linear with temperature.

The same applies to the accelerometer calibration as it is generally not necessary to recalibrate each flight.
