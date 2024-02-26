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
sh2_ProductId_t prodIds;            // Product ID returned by the sensor
sh2_Hal_t _HAL;

/* Private variables */
static bool printDebug = false;
static uint16_t _intPin = NULL, _intPort = NULL, _resetPin = NULL, _resetPort = NULL, _csPin = NULL, _csPort = NULL;
static SPI_HandleTypeDef* _spi = NULL;
static sh2_SensorValue_t* _sensor_value = NULL; // Used currently by the sensor callback
static bool _reset_occurred = false;
static bool _int_asserted = false;

//These are the raw sensor values (without Q applied) pulled from the user requested Input Report
static uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
static uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
static uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
static uint16_t rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ, rawBiasX, rawBiasY, rawBiasZ, UncalibGyroAccuracy;
static uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
static uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
static uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
static uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
static uint8_t tapDetector;
static uint16_t stepCount;
static uint32_t timeStamp;
static uint8_t stabilityClassifier;
static uint8_t activityClassifier;
static uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
static uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
static uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
static uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor

//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
//See the read metadata example for more info
static int16_t rotationVector_Q1 = 14;
static int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
static int16_t accelerometer_Q1 = 8;
static int16_t linear_accelerometer_Q1 = 8;
static int16_t gyro_Q1 = 9;
static int16_t magnetometer_Q1 = 4;
static int16_t angular_velocity_Q1 = 10;
static int16_t gravity_Q1 = 8;

static bool hal_waitForInt(void);
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static void hal_close(sh2_Hal_t *self);
static int hal_open(sh2_Hal_t *self);
static uint32_t hal_getTimeUs(sh2_Hal_t *self);

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
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
bool BNO08x_init(SPI_HandleTypeDef* spi, uint16_t intPort, uint16_t intPin, uint16_t rstPort, uint16_t rstPin, uint16_t csPort, uint16_t csPin)
{
    _intPort = intPort;
    _intPin = intPin;
    _resetPort = rstPort;
    _resetPin = rstPin;
    _csPort = csPort;
    _csPin = csPin;

    _HAL.open = hal_open;
    _HAL.close = hal_close;
    _HAL.write = hal_write;
    _HAL.read = hal_read;
    _HAL.getTimeUs = hal_getTimeUs;

    int status;

    $INFO("Start Initialisation.");

    BNO08x_hardwareReset();

    // Open SH2 interface and register non-sensor event handler.
    status = sh2_open(&_HAL, hal_callback, NULL);
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
    
    if (_resetPin != NULL)
    {
        // Active low
        HAL_GPIO_WritePin(_resetPort, _resetPin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(_resetPort, _resetPin, GPIO_PIN_SET);
        HAL_Delay(10);
        $INFO("HW Reset triggered.");
    }
    else 
    {
        $ERROR("Reset Pin not initialised.");
    }
}

bool BNO08x_wasReset(void)
{
    bool x = _reset_occurred;
    _reset_occurred = false;
    return x;
}

uint8_t BNO08x_getResetReason(void)
{
    return prodIds.resetCause;
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
bool BNO08X_enableReport(sh2_SensorId_t sensorId, uint32_t interval_us, uint32_t sensorSpecific) {
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

    if (!hal_wait_for_int()) {
        return false;
    }
    
    int status = sh2_setSensorConfig(sensorId, &config);

    if (status != SH2_OK) {
        return false;
    }

    return true;
}


/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool BNO08x_getSensorEvent(sh2_SensorValue_t *value)
{
  _sensor_value = value;

  _sensor_value->timestamp = 0;

  sh2_service();

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
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
    HAL_StatusTypeDef status = HAL_SPI_Transmit(_spi, pBuffer, len, 10);
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

    /* Read SHTP Header */
    memset(pBuffer, 0, 4);

    if (!hal_waitForInt()) 
    {
        return 0;
    }

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Receive(_spi, pBuffer, 4, 10);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    
    if (status != HAL_OK)
    {
        $ERROR("Unable to Receive on SPI");
        return 0;
    }

    packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8; // Construct the length 
    packet_size &= ~0x8000;         // Clear the MSB 

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
    HAL_StatusTypeDef status = HAL_SPI_Receive(_spi, pBuffer, packet_size, 10);
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
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    return 0;
}


static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    // Reset event seen
    if (pEvent->eventId == SH2_RESET)
    {
        _reset_occurred = true;
    }
}

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
