/**
 * @file icm_driver.h
 * @author Marvin Pranajaya (pipipipi2002)
 * @brief 
 * @version 0.1
 * @date 2024-03-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef INC_ICM_DRIVER_H
#define INC_ICM_DRIVER_H

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include <stdbool.h>
#include "icm_regs.h"

#define CALIBRATION_SAMPLE_SIZE		(1000U)
#define USE_PRINTF_LIB

#if defined (USE_PRINTF_LIB)
	#include <printf.h>
	#define $INFO(fmt, ...)     printf("[ICM] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
	#define $ERROR(fmt, ...)    printf("[ICM] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
	#define $SUCCESS(fmt, ...)  printf("[ICM] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
#else
    #define $INFO(fmt, ...)
	#define $ERROR(fmt, ...) 
	#define $SUCCESS(fmt, ...) 
#endif 

typedef enum ICM_Status {
    ICM_OK = 0,
    ICM_ERROR
} ICM_Status_t;

typedef enum ICM_SensorID {
    ICM40609D_ID = 0x3B,
    ICM42688P_ID = 0x47
} ICM_SensorID_t;

typedef enum ICM_PWR {
	PWR_OFF = 0,
	PWR_STANDBY = 1,
    PWR_LP = 2,
	PWR_LN = 3
} ICM_PWR_t;

typedef struct ICM_DataPacket {
	float temp;
	float accX;
	float accY;
	float accZ;
	float gyroX;
	float gyroY;
	float gyroZ;
} ICM_DataPacket_t;

typedef struct ICM_RawDataPacket {
	int16_t rawTemp;
	int16_t rawAccX;
	int16_t rawAccY;
	int16_t rawAccZ;
	int16_t rawGyroX;
	int16_t rawGyroY;
	int16_t rawGyroZ;
} ICM_RawDataPacket_t;

typedef struct ICM_HAL {
    SPI_TypeDef* spi;
    GPIO_TypeDef* csPort;
    uint16_t csPin;
    GPIO_TypeDef* int1Port;
    uint16_t int1Pin;
    GPIO_TypeDef* int2Port;
    uint16_t int2Pin;
} ICM_HAL_t;

typedef struct ICM_GyroConfig {
    uint8_t fsr;            // Full Scale Range (reg value)
    uint8_t odr;            // Output Data Rate (reg value)
    float ssf;              // Sensitivity Scale Factor
    float bias[3];          // Bias Offset
    float biasData[3];      // Used for bias calibration purposes
} ICM_GyroConfig_t;

typedef struct ICM_AccConfig {
    uint8_t fsr;            // Full Scale Range (reg value)
    uint8_t odr;            // Output Data Rate (reg value)
    float ssf;              // Sensitivity Scale Factor
    float bias[3];          // Bias Offset
    float biasData[3];      // Used for bias calibration purposes
    float scale[3];         // Scale Offset TODO: Init this to 1.0f 1.0f 1.0f
    float axisMax[3];       // Used for scale calibration purposes
    float axisMin[3];       // Used for scale calibration purposes
} ICM_AccConfig_t;

typedef struct ICM_Config {
	ICM_SensorID_t id;          // Sensor ID
	ICM_HAL_t hal;              // HAL pins
	ICM_DataPacket_t currD;     // Current Data Packet
	ICM_RawDataPacket_t currRD; // Current Raw Data Packet (16-bit ADC values)
	ICM_GyroConfig_t gyroCfg;   // Config for Gyro
	ICM_AccConfig_t accCfg;     // Config for Accel
    uint8_t bank;               // ICM internal bank
} ICM_Config_t;

// ICM_Status_t ICM42688P_init(SPI_TypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin, GPIO_TypeDef* int1Port, uint16_t int1Pin, GPIO_TypeDef* int2Port, uint16_t int2Pin);
ICM_Status_t ICM_init(ICM_Config_t* icm);
ICM_Status_t ICM_reset(ICM_Config_t* icm);
uint8_t ICM_whoami(ICM_Config_t* icm);
ICM_Status_t ICM_enAll(ICM_Config_t* icm);
ICM_Status_t ICM_enIdle(ICM_Config_t* icm, bool en);
ICM_Status_t ICM_enTemp(ICM_Config_t* icm, bool en);
ICM_Status_t ICM_enGyro(ICM_Config_t* icm, ICM_PWR_t mode);
ICM_Status_t ICM_enAccel(ICM_Config_t* icm, ICM_PWR_t mode);

ICM_Status_t ICM_setGyroFSR(ICM_Config_t* icm, uint8_t gyroFsr);
ICM_Status_t ICM_setAccelFSR(ICM_Config_t* icm, uint8_t accelFsr);
ICM_Status_t ICM_setGyroODR(ICM_Config_t* icm, uint8_t gyroOdr);
ICM_Status_t ICM_setAccelODR(ICM_Config_t* icm, uint8_t accelOdr);
ICM_Status_t ICM_setFilters(ICM_Config_t* icm, bool gyroFils, bool accelFils);

ICM_Status_t ICM_enableDataRdyInt(ICM_Config_t* icm);
ICM_Status_t ICM_disableDataRdyInt(ICM_Config_t* icm);

ICM_Status_t ICM_updateAllData(ICM_Config_t* icm);
ICM_Status_t ICM_updateTempData(ICM_Config_t* icm);
ICM_Status_t ICM_updateGyroData(ICM_Config_t* icm);
ICM_Status_t ICM_updateAccelData(ICM_Config_t* icm);

float ICM_getTemp(ICM_Config_t* icm);
float ICM_getAccX(ICM_Config_t* icm);
float ICM_getAccY(ICM_Config_t* icm);
float ICM_getAccZ(ICM_Config_t* icm);
float ICM_getGyroX(ICM_Config_t* icm);
float ICM_getGyroY(ICM_Config_t* icm);
float ICM_getGyroZ(ICM_Config_t* icm);
ICM_DataPacket_t ICM_getData(ICM_Config_t* icm);
ICM_RawDataPacket_t ICM_getRawData(ICM_Config_t* icm);

ICM_Status_t ICM_calibGyro(ICM_Config_t* icm);
ICM_Status_t ICM_calibAccel(ICM_Config_t* icm);
float ICM_getGyroBiasX(ICM_Config_t* icm);
float ICM_getGyroBiasY(ICM_Config_t* icm);
float ICM_getGyroBiasZ(ICM_Config_t* icm);
void ICM_setGyroBiasX(ICM_Config_t* icm, float bias);
void ICM_setGyroBiasY(ICM_Config_t* icm, float bias);
void ICM_setGyroBiasZ(ICM_Config_t* icm, float bias);
float ICM_getAccelBiasX(ICM_Config_t* icm);
float ICM_getAccelScaleX(ICM_Config_t* icm);
float ICM_getAccelBiasY(ICM_Config_t* icm);
float ICM_getAccelScaleY(ICM_Config_t* icm);
float ICM_getAccelBiasZ(ICM_Config_t* icm);
float ICM_getAccelScaleZ(ICM_Config_t* icm);
void ICM_setAccelCalX(ICM_Config_t* icm, float bias, float scale);
void ICM_setAccelCalY(ICM_Config_t* icm, float bias, float scale);
void ICM_setAccelCalZ(ICM_Config_t* icm, float bias, float scale);

ICM_Status_t ICM_enableFifo(ICM_Config_t* icm, bool accel, bool gyro, bool temp);
ICM_Status_t ICM_readFifo(ICM_Config_t* icm);

#endif // INC_ICM_DRIVER_H