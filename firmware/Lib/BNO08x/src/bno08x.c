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

#include "bno08x.h"

/* Global variables */
sh2_ProductIds_t prodIds;            // Product ID returned by the sensor
sh2_Hal_t _HAL;
sh2_SensorValue_t sensorValue;

/* Private variables */
static bool printDebug = false;
static GPIO_TypeDef *_intPort, *_resetPort, *_csPort, *_wakePort;
static uint16_t _intPin, _resetPin, _csPin, _wakePin;
static SPI_HandleTypeDef* _spi = NULL;
static sh2_SensorValue_t* _sensor_value = NULL; // Used by the sensor callback
static bool _reset_occurred = false;
static bool _int_asserted = false;
static uint8_t tapDetector;

static bool hal_waitForInt(void);
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static void hal_close(sh2_Hal_t *self);
static int hal_open(sh2_Hal_t *self);
static uint32_t hal_getTimeUs(sh2_Hal_t *self);

static void generalCallback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent);

/**
 * @brief Initialisation of the BNO08X sensor. 
 * 
 * @details SPI (and GPIO) needs to be initialised before calling this. 
 *          CS, RESET are active low, set to high before calling this.
 *          INT port is interrupt, set to //TODO
 * @param spi STM32 SPI handler
 * @param intPort Interrupt PORT
 * @param intPin Interrupt PIN
 * @param rstPort Reset PORT
 * @param rstPin Reset PIN
 * @param csPort CS PORT
 * @param csPin CS PIN
 * @return true if init successful
 */
bool BNO08x_init(SPI_HandleTypeDef* spi, GPIO_TypeDef* intPort, uint16_t intPin, GPIO_TypeDef* rstPort, uint16_t rstPin, GPIO_TypeDef* csPort, uint16_t csPin, GPIO_TypeDef* wakePort, uint16_t wakePin)
{
    _intPort = intPort;
    _intPin = intPin;
    _resetPort = rstPort;
    _resetPin = rstPin;
    _csPort = csPort;
    _csPin = csPin;
    _wakePort = wakePort;
    _wakePin = wakePin;

    _HAL.open = hal_open;
    _HAL.close = hal_close;
    _HAL.write = hal_write;
    _HAL.read = hal_read;
    _HAL.getTimeUs = hal_getTimeUs;

    int status;

    $INFO("Start Initialisation.");

    HAL_GPIO_WritePin(wakePort, wakePin, GPIO_PIN_SET); // PS0 set to high for SPI comms
    BNO08x_hardwareReset();

    // Open SH2 interface and register non-sensor event handler.
    status = sh2_open(&_HAL, generalCallback, NULL);
    if (status != SH2_OK) 
    {
        $ERROR("Fail to open SH2. Code: %d", status);
        return false;   
    }
    $INFO("Open SH2 Interface.");

    // Check conection by receiving product id.
    memset(&prodIds, 0, sizeof(prodIds)); // Clear prodIds struct
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) 
    {
        $ERROR("Fail to get product ID. Code: %d", status);
        return false;     
    } 
    $INFO("Receive Product ID.");

    // Register sensor listener
    sh2_setSensorCallback(sensorCallback, NULL);
    $INFO("Registered Sensor Callback.");

    $SUCCESS("Initialisation completed");
    return true;
}

void BNO08x_intSet(bool set)
{
    _int_asserted = set;
}

void BNO08x_hardwareReset(void)
{
    // Active low
    HAL_GPIO_WritePin(_resetPort, _resetPin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(_resetPort, _resetPin, GPIO_PIN_SET);
    HAL_Delay(10);
    $INFO("HW Reset triggered.");
}

bool BNO08x_wasReset(void)
{
    bool x = _reset_occurred;
    _reset_occurred = false;
    return x;
}

/**
 * @brief Reason for last reset
 * 
 * @return uint8_t 1: POR, 2: Internal Reset, 3: Watchdog, 4: External reset, 5: Other
 */
uint8_t BNO08x_getResetReason(void)
{
    return prodIds.entry[0].resetCause;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @param sensorSpecific config settings specific to sensor/reportID.
 * (e.g. enabling/disabling possible activities in personal activity classifier)
 * @return true: success false: failure
 */
bool BNO08x_enableReport(sh2_SensorId_t sensorId, uint32_t interval_us, uint32_t sensorSpecific) 
{
    static sh2_SensorConfig_t config;

    // These sensor options are disabled or not used in most cases
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = sensorSpecific;
    config.reportInterval_us = interval_us;

    if (!hal_waitForInt()) {
        return false;
    }
    
    int status = sh2_setSensorConfig(sensorId, &config);

    if (status != SH2_OK) {
        return false;
    }
    return true;
}

/**
 * @brief Check for new incoming data
 *
 * @return true: There is new data
 */
bool BNO08x_getSensorEvent()
{
  _sensor_value = &sensorValue;

  _sensor_value->timestamp = 0;

  sh2_service(); // fetch new data

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }
  return true;
}

uint8_t BNO08x_getSensorEventID(void)
{
    return _sensor_value->sensorId;
}

void BNO08x_enableDebugguing(bool debug)
{
    // TODO: implementation
    printDebug = debug;
    return;
}

bool BNO08x_softReset(void) 
{
    int status = sh2_devReset();
    if (status != SH2_OK) 
    {
        $ERROR("Soft reset failed");
        return false;
    } 
    
    $INFO("Device reset");
    return true;
}

bool BNO08x_serviceBus(void)
{
    sh2_service();
    return true;
}

/**
 * @brief All configured BNO08X sensors will be enabled
 * 
 * @return true when success
 */
bool BNO08x_modeOn(void)
{
    int status = sh2_devOn();

    if (status != SH2_OK)
    {
        $ERROR("Unable to turn on device.");
        return false;
    }
    $INFO("Device turned on.");
    return true;
}

/**
 * @brief Put BNO08x to sleep. Sensors set to always on 
 *        and wake will not be affected.
 * 
 * @return true when successful
 */
bool BNO08x_modeSleep(void)
{
    int status = sh2_devSleep();
    if (status != SH2_OK)
    {
        $ERROR("Unable to go to sleep.");
        return false;
    }

    $INFO("Device going to sleep");
    return true;
}

float BNO08x_qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    // TODO: Optimise using FPU
    float qFloat = fixedPointValue;
    qFloat *= pow(2, qPoint * -1);
    return qFloat;
}

bool BNO08x_enableRotationVector(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SH2_ROTATION_VECTOR, timeBetweenReports, 0);
}

bool BNO08x_enableGeomagneticRotationVector(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, timeBetweenReports, 0);
}

bool BNO08x_enableGameRotationVector(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SH2_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
}

bool BNO08x_enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports, 0);
}

bool BNO08x_enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
}

bool BNO08x_enableAccelerometer(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SH2_ACCELEROMETER, timeBetweenReports, 0);
}

bool BNO08x_enableLinearAccelerometer(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports, 0);
}

bool BNO08x_enableGravity(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_GRAVITY, timeBetweenReports, 0);
}

bool BNO08x_enableGyro(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, timeBetweenReports, 0);
}

bool BNO08x_enableUncalibratedGyro(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_UNCALIBRATED_GYRO, timeBetweenReports, 0);
}

bool BNO08x_enableMagnetometer(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports, 0);
}

bool BNO08x_enableTapDetector(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_TAP_DETECTOR, timeBetweenReports, 0);
}

bool BNO08x_enableStepCounter(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports, 0);
}

bool BNO08x_enableStabilityClassifier(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports, 0);
}

bool BNO08x_enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToE)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToE);
}

bool BNO08x_enableRawAccelerometer(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports, 0);
}

bool BNO08x_enableRawGyro(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports, 0);
}

bool BNO08x_enableRawMagnetometer(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports, 0);
}

bool BNO08x_enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
    timeBetweenReports = timeBetweenReports * 1000; // ms to us
    return BNO08x_enableReport(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports, 0);
}

bool BNO08x_setCalibrationConfig(uint8_t sensors)
{
    int status = sh2_setCalConfig(sensors);

    if (status != SH2_OK)
    {
        $ERROR("Unable to set calibration config for sensor: %d", sensors);
        return false;
    }

    $INFO("Calibration set for sensor: %d", sensors);
    return true;
}

/**
 * @brief TARE values
 * 
 * @param zAxis def: false
 * @param basis def: SH2_TARE_BASIS_ROTATION_VECTOR
 * @return true if successful
 */
bool BNO08x_tareNow(bool zAxis, sh2_TareBasis_t basis)
{
    int status = sh2_setTareNow(zAxis ? TARE_AXIS_Z : TARE_AXIS_ALL, basis);

    if (status != SH2_OK) 
    {
        $ERROR("Unable to tare");
        return false;
    }
    
    return true;
}

bool BNO08x_saveTare(void)
{
    int status = sh2_persistTare();
    if (status != SH2_OK)
    {
        $ERROR("Unable to save tare");
        return false;
    }
    return true;
}

bool BNO08x_clearTare(void)
{
    int status = sh2_clearTare();

    if (status != SH2_OK) 
    {
        $ERROR("Unable to clear tare");
        return false;
    }
    return true;
}

int16_t BNO08x_getRawAccelX(void)
{
    return _sensor_value->un.rawAccelerometer.x;
}

int16_t BNO08x_getRawAccelY(void)
{
    return _sensor_value->un.rawAccelerometer.y;
}

int16_t BNO08x_getRawAccelZ(void)
{
    return _sensor_value->un.rawAccelerometer.z;
}

int16_t BNO08x_getRawGyroX(void)
{
    return _sensor_value->un.rawGyroscope.x;
}

int16_t BNO08x_getRawGyroY(void)
{
    return _sensor_value->un.rawGyroscope.y;
}

int16_t BNO08x_getRawGyroZ(void)
{
    return _sensor_value->un.rawGyroscope.z;
}

int16_t BNO08x_getRawMagX(void)
{
    return _sensor_value->un.rawMagnetometer.x;
}

int16_t BNO08x_getRawMagY(void)
{
    return _sensor_value->un.rawMagnetometer.y;
}

int16_t BNO08x_getRawMagZ(void)
{
    return _sensor_value->un.rawMagnetometer.z;
}


void BNO08x_getAccel(float *x, float *y, float *z, uint8_t *accuracy)
{
    // *x = BNO08x_qToFloat(rawAccelX, accelerometer_Q1);
    *x = BNO08x_getAccelX();
    // *y = BNO08x_qToFloat(rawAccelY, accelerometer_Q1);
    *y = BNO08x_getAccelY();
    // *z = BNO08x_qToFloat(rawAccelZ, accelerometer_Q1);
    *z = BNO08x_getAccelZ();
    // *accuracy = accelAccuracy;
    *accuracy = BNO08x_getAccelAccuracy();
}

float BNO08x_getAccelX(void)
{
    return _sensor_value->un.accelerometer.x;
}

float BNO08x_getAccelY(void)
{
    return _sensor_value->un.accelerometer.y;
}

float BNO08x_getAccelZ(void)
{
    return _sensor_value->un.accelerometer.z;
}

uint8_t BNO08x_getAccelAccuracy(void)
{
    return _sensor_value->status;
}

void BNO08x_getLinAccel(float *x, float *y, float *z, uint8_t *accuracy) 
{
    // *x = BNO08x_qToFloat(rawLinAccelX, linear_accelerometer_Q1);
    *x = BNO08x_getLinAccelX();
	// *y = BNO08x_qToFloat(rawLinAccelY, linear_accelerometer_Q1);
    *y = BNO08x_getLinAccelY();
	// *z = BNO08x_qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
    *z = BNO08x_getLinAccelZ();
	// *accuracy = accelLinAccuracy;
    *accuracy = BNO08x_getLinAccelAccuracy();
}

float BNO08x_getLinAccelX(void) 
{
    return _sensor_value->un.linearAcceleration.x;
}

float BNO08x_getLinAccelY(void) 
{
    return _sensor_value->un.linearAcceleration.y;
}

float BNO08x_getLinAccelZ(void) 
{
    return _sensor_value->un.linearAcceleration.z;
}

uint8_t BNO08x_getLinAccelAccuracy(void) 
{
    return _sensor_value->status;
}

void BNO08x_getGyro(float *x, float *y, float *z, uint8_t *accuracy)
{
	// *x = BNO08x_qToFloat(rawGyroX, gyro_Q1);
	*x = BNO08x_getGyroX();
	// *y = BNO08x_qToFloat(rawGyroY, gyro_Q1);
    *y = BNO08x_getGyroY();
	// *z = BNO08x_qToFloat(rawGyroZ, gyro_Q1);
    *z = BNO08x_getGyroZ();
	// *accuracy = gyroAccuracy;
    *accuracy = BNO08x_getGyroAccuracy();
}

float BNO08x_getGyroX(void)
{
    return _sensor_value->un.gyroscope.x;
}

float BNO08x_getGyroY(void)
{
    return _sensor_value->un.gyroscope.y;
}

float BNO08x_getGyroZ(void)
{
    return _sensor_value->un.gyroscope.z;
}

uint8_t BNO08x_getGyroAccuracy(void)
{
    return _sensor_value->status;
}


void BNO08x_getUncalibratedGyro(float *x, float *y, float *z, float *bx, float *by, float *bz, uint8_t *accuracy)
{
    // TODO: different from sparkfun library
    // *x = BNO08x_qToFloat(rawUncalibGyroX, gyro_Q1);
    *x = BNO08x_getUncalibratedGyroX(); 
    // *y = BNO08x_qToFloat(rawUncalibGyroY, gyro_Q1);
    *y = BNO08x_getUncalibratedGyroY(); 
    // *z = BNO08x_qToFloat(rawUncalibGyroZ, gyro_Q1);
    *z = BNO08x_getUncalibratedGyroZ(); 
    // *bx = BNO08x_qToFloat(rawBiasX, gyro_Q1);
    *bx = BNO08x_getUncalibratedGyroBiasX(); 
    // *by = BNO08x_qToFloat(rawBiasY, gyro_Q1);
    *by = BNO08x_getUncalibratedGyroBiasY(); 
    // *bz = BNO08x_qToFloat(rawBiasZ, gyro_Q1);
    *bz = BNO08x_getUncalibratedGyroBiasZ(); 
    // *accuracy = UncalibGyroAccuracy;
    *accuracy = BNO08x_getUncalibratedGyroAccuracy();
}

float BNO08x_getUncalibratedGyroX(void) 
{
    return _sensor_value->un.gyroscopeUncal.x;    
}

float BNO08x_getUncalibratedGyroY(void) 
{
    return _sensor_value->un.gyroscopeUncal.y;
}

float BNO08x_getUncalibratedGyroZ(void) 
{
    return _sensor_value->un.gyroscopeUncal.z;  
}

float BNO08x_getUncalibratedGyroBiasX(void) 
{
    return _sensor_value->un.gyroscopeUncal.biasX;  
}

float BNO08x_getUncalibratedGyroBiasY(void) 
{
    return _sensor_value->un.gyroscopeUncal.biasY;
}

float BNO08x_getUncalibratedGyroBiasZ(void) 
{
    return _sensor_value->un.gyroscopeUncal.biasZ;
}

uint8_t BNO08x_getUncalibratedGyroAccuracy(void) 
{
    return _sensor_value->status;
}


void BNO08x_getMag(float *x, float *y, float *z, uint8_t *accuracy)
{
   	// *x = BNO08x_qToFloat(rawMagX, magnetometer_Q1);
    *x = BNO08x_getMagX();
	// *y = BNO08x_qToFloat(rawMagY, magnetometer_Q1);
    *y = BNO08x_getMagY();
	// *z = BNO08x_qToFloat(rawMagZ, magnetometer_Q1);
    *z = BNO08x_getMagZ();
	// *accuracy = magAccuracy; 
    *accuracy = BNO08x_getMagAccuracy();
}

float BNO08x_getMagX(void)
{
    return _sensor_value->un.magneticField.x;
}

float BNO08x_getMagY(void)
{
    return _sensor_value->un.magneticField.y;
}

float BNO08x_getMagZ(void)
{
    return _sensor_value->un.magneticField.z;
}

uint8_t BNO08x_getMagAccuracy(void)
{
    return _sensor_value->status;
}

void BNO08x_getGravity(float *x, float *y, float *z, uint8_t *accuracy)
{
    // *x = BNO08x_qToFloat(gravityX, gravity_Q1);
    *x = BNO08x_getGravityX();
	// *y = BNO08x_qToFloat(gravityY, gravity_Q1);
    *y = BNO08x_getGravityY();
	// *z = BNO08x_qToFloat(gravityZ, gravity_Q1);
    *z = BNO08x_getGravityZ();
	// *accuracy = gravityAccuracy;
    *accuracy = BNO08x_getGravityAccuracy();
}

float BNO08x_getGravityX(void)
{
    return _sensor_value->un.gravity.x;
}

float BNO08x_getGravityY(void)
{
    return _sensor_value->un.gravity.y;
}

float BNO08x_getGravityZ(void)
{
    return _sensor_value->un.gravity.z;
}

uint8_t BNO08x_getGravityAccuracy(void)
{
    return _sensor_value->status;
}


void BNO08x_getQuat(float *i, float *j, float *k, float *real, float *radAccuracy, uint8_t *accuracy)
{
    // *i = BNO08x_qToFloat(rawQuatI, rotationVector_Q1);
    *i = BNO08x_getQuatI();
	// *j = BNO08x_qToFloat(rawQuatJ, rotationVector_Q1);
    *j = BNO08x_getQuatJ();
	// *k = BNO08x_qToFloat(rawQuatK, rotationVector_Q1);
    *k = BNO08x_getQuatK();
	// *real = BNO08x_qToFloat(rawQuatReal, rotationVector_Q1);
	*real = BNO08x_getQuatReal();
	// *radAccuracy = BNO08x_qToFloat(rawQuatRadianAccuracy, rotationVectorAccuracy_Q1);
	*radAccuracy = BNO08x_getQuatRadianAccuracy();
    // *accuracy = quatAccuracy;
    *accuracy = BNO08x_getQuatAccuracy();
}

float BNO08x_getQuatI(void)
{
    return _sensor_value->un.rotationVector.i;
}

float BNO08x_getQuatJ(void) 
{
    return _sensor_value->un.rotationVector.j;
}

float BNO08x_getQuatK(void) 
{
    return _sensor_value->un.rotationVector.k;
}

float BNO08x_getQuatReal(void) 
{
    return _sensor_value->un.rotationVector.real;
}

float BNO08x_getQuatRadianAccuracy(void) 
{
    return _sensor_value->un.rotationVector.accuracy;
}

uint8_t BNO08x_getQuatAccuracy(void) 
{
    return _sensor_value->status;
}


float BNO08x_getGameQuatI(void)
{
    return _sensor_value->un.gameRotationVector.i;
}

float BNO08x_getGameQuatJ(void)
{
    return _sensor_value->un.gameRotationVector.j;
}

float BNO08x_getGameQuatK(void)
{
    return _sensor_value->un.gameRotationVector.k;
}

float BNO08x_getGameQuatReal(void)
{
    return _sensor_value->un.gameRotationVector.real;
}


// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void BNO08x_getRollPitchYaw(float* roll, float* pitch, float* yaw)
{
    float dqw = BNO08x_getQuatReal();
    float dqx = BNO08x_getQuatI();
    float dqy = BNO08x_getQuatJ();
    float dqz = BNO08x_getQuatK();

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // roll (x-axis rotation)
    float t0 = 2.0 * (dqw * dqx + dqy * dqz);
    float t1 = 1.0 - 2.0 * (dqx * dqx + ysqr);
    *roll = atan2(t0, t1);

    // pitch (y-axis rotation)
	float t2 = 2.0 * (dqw * dqy - dqz * dqx);
	t2 = (t2 > 1.0) ? 1.0 : t2;
	t2 = (t2 < -1.0) ? -1.0 : t2;
	*pitch = asin(t2);

    // yaw (z-axis rotation)
	float t3 = 2.0 * (dqw * dqz + dqx * dqy);
	float t4 = 1.0 - 2.0 * (ysqr + dqz * dqz);
	*yaw = atan2(t3, t4);
}

float BNO08x_getRoll(void)
{
    float dqw = BNO08x_getQuatReal();
    float dqx = BNO08x_getQuatI();
    float dqy = BNO08x_getQuatJ();
    float dqz = BNO08x_getQuatK();

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // roll (x-axis rotation)
    float t0 = 2.0 * (dqw * dqx + dqy * dqz);
    float t1 = 1.0 - 2.0 * (dqx * dqx + ysqr);
    float roll = atan2(t0, t1);
    
    return roll;
}

float BNO08x_getPitch(void)
{
    float dqw = BNO08x_getQuatReal();
    float dqx = BNO08x_getQuatI();
    float dqy = BNO08x_getQuatJ();
    float dqz = BNO08x_getQuatK();

	float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
	dqw = dqw / norm;
	dqx = dqx / norm;
	dqy = dqy / norm;
	dqz = dqz / norm;

	// pitch (y-axis rotation)
	float t2 = 2.0 * (dqw * dqy - dqz * dqx);
	t2 = (t2 > 1.0) ? 1.0 : t2;
	t2 = (t2 < -1.0) ? -1.0 : t2;
	float pitch = asin(t2);

	return pitch;
}

float BNO08x_getYaw(void)
{
    float dqw = BNO08x_getQuatReal();
    float dqx = BNO08x_getQuatI();
    float dqy = BNO08x_getQuatJ();
    float dqz = BNO08x_getQuatK();

	float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
	dqw = dqw / norm;
	dqx = dqx / norm;
	dqy = dqy / norm;
	dqz = dqz / norm;

	float ysqr = dqy * dqy;

	// yaw (z-axis rotation)
	float t3 = 2.0 * (dqw * dqz + dqx * dqy);
	float t4 = 1.0 - 2.0 * (ysqr + dqz * dqz);
	float yaw = atan2(t3, t4);

	return yaw;
}

void BNO08x_getGyroIntegratedRV(float *i, float *j, float *k, float *real, float *avx, float *avy, float *avz)
{
    // *i = BNO08x_qToFloat(rawGyroRVI, angular_position_Q1);
    *i = BNO08x_getGyroIntegratedRVI();
	// *j = BNO08x_qToFloat(rawGyroRVJ, angular_position_Q1);
    *j = BNO08x_getGyroIntegratedRVJ();
	// *k = BNO08x_qToFloat(rawGyroRVK, angular_position_Q1);
    *k = BNO08x_getGyroIntegratedRVK();
	// *real = BNO08x_qToFloat(rawGyroRVReal, angular_position_Q1);
    *real = BNO08x_getGyroIntegratedRVReal();
	// *avx = BNO08x_qToFloat(rawGyroRVavx, angular_velocity_Q1);
    *avx = BNO08x_getGyroIntegratedRVangVelX();
	// *avy = BNO08x_qToFloat(rawGyroRVavy, angular_velocity_Q1);
    *avy = BNO08x_getGyroIntegratedRVangVelY();
	// *avz = BNO08x_qToFloat(rawGyroRVavz, angular_velocity_Q1);
    *avz = BNO08x_getGyroIntegratedRVangVelZ();
}


float BNO08x_getGyroIntegratedRVI(void)
{
    return _sensor_value->un.gyroIntegratedRV.i;
}

float BNO08x_getGyroIntegratedRVJ(void)
{
    return _sensor_value->un.gyroIntegratedRV.j;
}

float BNO08x_getGyroIntegratedRVK(void)
{
    return _sensor_value->un.gyroIntegratedRV.k;
}

float BNO08x_getGyroIntegratedRVReal(void)
{
    return _sensor_value->un.gyroIntegratedRV.real;
}

float BNO08x_getGyroIntegratedRVangVelX(void)
{
    return _sensor_value->un.gyroIntegratedRV.angVelX;
}

float BNO08x_getGyroIntegratedRVangVelY(void)
{
    return _sensor_value->un.gyroIntegratedRV.angVelY;
}

float BNO08x_getGyroIntegratedRVangVelZ(void)
{
    return _sensor_value->un.gyroIntegratedRV.angVelZ;
}

uint8_t BNO08x_getTapDetector(void)
{
    uint8_t previousTapDetector = tapDetector;
    tapDetector = 0;
    return previousTapDetector;
}

uint16_t BNO08x_getStepCount(void)
{
    return _sensor_value->un.stepCounter.steps;
}

uint8_t BNO08x_getStabilityClassifier(void)
{
    return _sensor_value->un.stabilityClassifier.classification;
}

uint8_t BNO08x_getActivityClassifier(void)
{
    return _sensor_value->un.personalActivityClassifier.mostLikelyState;
}

uint8_t BNO08x_getActivityConfidence(uint8_t activity)
{
    return _sensor_value->un.personalActivityClassifier.confidence[activity];
}

uint64_t BNO08x_getTimeStamp(void)
{
    return _sensor_value->timestamp;
}

/**
 * @brief check whether interrupt is asserted
 *      Current implementation used polling, support for interrupt will be done later.
 * 
 * @return true if interrupt detected.
 */
static bool hal_waitForInt(void)
{
    for (int i = 0; i < 300; i++)
    {
        if (!HAL_GPIO_ReadPin(_intPort, _intPin)) return true;
        HAL_Delay(1);
    }

    $ERROR("Interrupt not asserted. Timedout.");
    BNO08x_hardwareReset();

    return false;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    if (!hal_waitForInt())
    {
        return 0;
    }

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(_spi, pBuffer, len, 100);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        $ERROR("Unable to transmit on SPI");
        return 0;
    }

    return len;
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    uint16_t packet_size = 0;

    /* Read SHTP Header to detect the payload size */
    memset(pBuffer, 0, 4);

    if (!hal_waitForInt()) 
    {
        return 0;
    }

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Receive(_spi, pBuffer, 4, 100);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    
    if (status != HAL_OK)
    {
        $ERROR("Unable to Receive on SPI. Code: %d", status);
        return 0;
    }

    packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8; // Construct the length 
    packet_size &= ~0x8000;         // Clear the MSB (continue bit)

    /* Read the complete header */
    memset(pBuffer, 0, packet_size);
    if (packet_size > len)
    {
        return 0;
    }

    if (!hal_waitForInt()) 
    {
        return 0;
    }

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    status = HAL_SPI_Receive(_spi, pBuffer, packet_size, 100);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    
    if (status != HAL_OK)
    {
        $ERROR("Unable to Receive on SPI");
        return 0;
    }

    return packet_size;
}

static void hal_close(sh2_Hal_t *self)
{
    return;
}

static int hal_open(sh2_Hal_t *self)
{
    hal_waitForInt();
    return 0;
}

// TODO: Implement this function for I2C
static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    return 0;
}

/**
 * @brief Callback registered in BNO08x_init function
 * 
 * @param cookie 
 * @param pEvent 
 */
static void generalCallback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    // Reset event seen
    if (pEvent->eventId == SH2_RESET)
    {
        _reset_occurred = true;
    }
}

/**
 * @brief Callback registered in BNO08x_init function
 * 
 * @param cookie 
 * @param pEvent 
 */
static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent)
{
    int status;

    $INFO("Got a Sensor Event!");

    status = sh2_decodeSensorEvent(_sensor_value, pEvent);
    if (status != SH2_OK)
    {
        $ERROR("Error decoding sensor event.");
        _sensor_value->timestamp = 0;
        return;
    }
}
