/*
  This is a library written for the BNO08x
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Adjusted by Pete Lewis @ SparkFun Electronics, June 2023 to incorporate the 
  CEVA Sensor Hub Driver, found here:
  https://github.com/ceva-dsp/sh2

  Also, utilizing code from the Adafruit BNO08x Arduino Library by Bryan Siepert 
  for Adafruit Industries. Found here:
  https://github.com/adafruit/Adafruit_BNO08x

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit 
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  The BNO08x IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO08x and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  Some of this library was based off of the Adafruit BNO08x Arduino Library.
  More specifically, the code layers connecting to the HillCrest/Ceva Driver.
  Their original work can be found here:
  https://github.com/adafruit/Adafruit_BNO08x
  Thank you Adafruit and your developers for all your hard work put into your Library!
*/

/*  This library source code has been modified for STM32F4 using SPI.
    

    Modified by pipipipi2002 (Marvin Pranajaya), February 2024.
*/

#ifndef INC_BNO08X_H
#define INC_BNO08X_H

#include <stdbool.h>
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "main.h"

#if defined (DISABLE_DEBUG)
	#define $INFO(fmt, ...)
	#define $ERROR(fmt, ...) 
	#define $SUCCESS(fmt, ...) 
#elif defined (USE_LOGGER)
	#define $INFO(fmt, ...) log_pInfo(fmt, ##__VA_ARGS__)
	#define $ERROR(fmt, ...) log_pError(fmt, ##__VA_ARGS__)
	#define $SUCCESS(fmt, ...) log_pSuccess(fmt, ##__VA_ARGS__)
#else
	#include <printf.h>
	#define $INFO(fmt, ...)     printf("[BNO08X]"); printf(fmt, ##__VA_ARGS__); printf("\n\r")
	#define $ERROR(fmt, ...)    printf("[BNO08X]"); printf(fmt, ##__VA_ARGS__); printf("\n\r")
	#define $SUCCESS(fmt, ...)  printf("[BNO08X]"); printf(fmt, ##__VA_ARGS__); printf("\n\r")
#endif 



//All the ways we can configure or talk to the BNO08x, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER SH2_ACCELEROMETER
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED SH2_GYROSCOPE_CALIBRATED
#define SENSOR_REPORTID_MAGNETIC_FIELD SH2_MAGNETIC_FIELD_CALIBRATED
#define SENSOR_REPORTID_LINEAR_ACCELERATION SH2_LINEAR_ACCELERATION
#define SENSOR_REPORTID_ROTATION_VECTOR SH2_ROTATION_VECTOR
#define SENSOR_REPORTID_GRAVITY SH2_GRAVITY
#define SENSOR_REPORTID_UNCALIBRATED_GYRO SH2_GYROSCOPE_UNCALIBRATED
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR SH2_GYRO_INTEGRATED_RV
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER SH2_STEP_COUNTER
#define SENSOR_REPORTID_STABILITY_CLASSIFIER SH2_STABILITY_CLASSIFIER
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER SH2_PERSONAL_ACTIVITY_CLASSIFIER
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define TARE_AXIS_ALL 0x07
#define TARE_AXIS_Z   0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

extern sh2_ProductId_t prodIds;            // Product ID returned by the sensor

bool BNO08x_init(SPI_HandleTypeDef* spi, uint16_t intPort, uint16_t intPin, uint16_t rstPort, uint16_t rstPin, uint16_t csPort, uint16_t csPin);
void BNO08x_intSet(bool set);
void BNO08x_hardwareReset(void);
bool BNO08x_wasReset(void);
uint8_t BNO08x_getResetReason(void);
bool BNO08x_enableReport(sh2_SensorId_t sensor, uint32_t interval_us, uint32_t sensorSpecific);
bool BNO08x_getSensorEvent(sh2_SensorValue_t *value);
uint8_t BNO08x_getSensorEventID(void);
void BNO08x_enableDebugging(void);          //Turn on debug printing. If user doesn't specify then Serial will be used.
bool BNO08x_softReset(void);	                //Try to reset the IMU via software
bool BNO08x_serviceBus(void);	
bool BNO08x_modeOn(void);	                    //Use the executable channel to turn the BNO on
bool BNO08x_modeSleep(void);	                //Use the executable channel to put the BNO to sleep
float BNO08x_qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number
bool BNO08x_enableRotationVector(uint16_t timeBetweenReports = 10);
bool BNO08x_enableGeomagneticRotationVector(uint16_t timeBetweenReports = 10);
bool BNO08x_enableGameRotationVector(uint16_t timeBetweenReports = 10);
bool BNO08x_enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
bool BNO08x_enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
bool BNO08x_enableAccelerometer(uint16_t timeBetweenReports = 10);
bool BNO08x_enableLinearAccelerometer(uint16_t timeBetweenReports = 10);
bool BNO08x_enableGravity(uint16_t timeBetweenReports = 10);
bool BNO08x_enableGyro(uint16_t timeBetweenReports = 10);
bool BNO08x_enableUncalibratedGyro(uint16_t timeBetweenReports = 10);
bool BNO08x_enableMagnetometer(uint16_t timeBetweenReports = 10);
bool BNO08x_enableTapDetector(uint16_t timeBetweenReports);
bool BNO08x_enableStepCounter(uint16_t timeBetweenReports = 10);
bool BNO08x_enableStabilityClassifier(uint16_t timeBetweenReports = 10);
bool BNO08x_enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable);
bool BNO08x_enableRawAccelerometer(uint16_t timeBetweenReports = 10);
bool BNO08x_enableRawGyro(uint16_t timeBetweenReports = 10);
bool BNO08x_enableRawMagnetometer(uint16_t timeBetweenReports = 10);
bool BNO08x_enableGyroIntegratedRotationVector(uint16_t timeBetweenReports = 10);
void BNO08x_getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy);
float BNO08x_getQuatI(void);
float BNO08x_getQuatJ(void);
float BNO08x_getQuatK(void);
float BNO08x_getQuatReal(void);
float BNO08x_getQuatRadianAccuracy(void);
uint8_t BNO08x_getQuatAccuracy(void);
float BNO08x_getGameQuatI(void);
float BNO08x_getGameQuatJ(void);
float BNO08x_getGameQuatK(void);
float BNO08x_getGameQuatReal(void);	
void BNO08x_getAccel(float &x, float &y, float &z, uint8_t &accuracy);
float BNO08x_getAccelX(void);
float BNO08x_getAccelY(void);
float BNO08x_getAccelZ(void);
uint8_t BNO08x_getAccelAccuracy(void);
void BNO08x_getLinAccel(float &x, float &y, float &z, uint8_t &accuracy);
float BNO08x_getLinAccelX(void);
float BNO08x_getLinAccelY(void);
float BNO08x_getLinAccelZ(void);
uint8_t BNO08x_getLinAccelAccuracy(void);
void BNO08x_getGyro(float &x, float &y, float &z, uint8_t &accuracy);
float BNO08x_getGyroX(void);
float BNO08x_getGyroY(void);
float BNO08x_getGyroZ(void);
uint8_t BNO08x_getGyroAccuracy(void);
void BNO08x_getUncalibratedGyro(float &x, float &y, float &z, float &bx, float &by, float &bz, uint8_t &accuracy);
float BNO08x_getUncalibratedGyroX(void);
float BNO08x_getUncalibratedGyroY(void);
float BNO08x_getUncalibratedGyroZ(void);
float BNO08x_getUncalibratedGyroBiasX(void);
float BNO08x_getUncalibratedGyroBiasY(void);
float BNO08x_getUncalibratedGyroBiasZ(void);
uint8_t BNO08x_getUncalibratedGyroAccuracy(void);
float BNO08x_getGyroIntegratedRVI(void);
float BNO08x_getGyroIntegratedRVJ(void);
float BNO08x_getGyroIntegratedRVK(void);
float BNO08x_getGyroIntegratedRVReal(void);
float BNO08x_getGyroIntegratedRVangVelX(void);
float BNO08x_getGyroIntegratedRVangVelY(void);
float BNO08x_getGyroIntegratedRVangVelZ(void);
void BNO08x_getMag(float &x, float &y, float &z, uint8_t &accuracy);
float BNO08x_getMagX(void);
float BNO08x_getMagY(void);
float BNO08x_getMagZ(void);
uint8_t BNO08x_getMagAccuracy(void);
void BNO08x_getGravity(float &x, float &y, float &z, uint8_t &accuracy);
float BNO08x_getGravityX(void);
float BNO08x_getGravityY(void);
float BNO08x_getGravityZ(void);
uint8_t BNO08x_getGravityAccuracy(void);
bool BNO08x_setCalibrationConfig(uint8_t sensors);
bool BNO08x_saveCalibration(void);
bool BNO08x_tareNow(bool zAxis=false, sh2_TareBasis_t basis=SH2_TARE_BASIS_ROTATION_VECTOR);
bool BNO08x_saveTare(void);
bool BNO08x_clearTare(void);
uint8_t BNO08x_getTapDetector(void);
uint64_t BNO08x_getTimeStamp(void);
uint16_t BNO08x_getStepCount(void);
uint8_t BNO08x_getStabilityClassifier(void);
uint8_t BNO08x_getActivityClassifier(void);
uint8_t BNO08x_getActivityConfidence(uint8_t activity);
int16_t BNO08x_getRawAccelX(void);
int16_t BNO08x_getRawAccelY(void);
int16_t BNO08x_getRawAccelZ(void);
int16_t BNO08x_getRawGyroX(void);
int16_t BNO08x_getRawGyroY(void);
int16_t BNO08x_getRawGyroZ(void);
int16_t BNO08x_getRawMagX(void);
int16_t BNO08x_getRawMagY(void);
int16_t BNO08x_getRawMagZ(void);
float BNO08x_getRoll(void);
float BNO08x_getPitch(void);
float BNO08x_getYaw(void);

#endif // INC_BNO08X_H