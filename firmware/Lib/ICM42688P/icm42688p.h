#ifndef INC_ICM42688P_H
#define INC_ICM42688P_H

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include <stdbool.h>

#define USE_PRINTF_LIB

#if defined (DISABLE_DEBUG)
	#define $INFO(fmt, ...)
	#define $ERROR(fmt, ...) 
	#define $SUCCESS(fmt, ...) 
#elif defined (USE_LOGGER)
	#define $INFO(fmt, ...) log_pInfo(fmt, ##__VA_ARGS__)
	#define $ERROR(fmt, ...) log_pError(fmt, ##__VA_ARGS__)
	#define $SUCCESS(fmt, ...) log_pSuccess(fmt, ##__VA_ARGS__)
#elif defined (USE_PRINTF_LIB)
	#include <printf.h>
	#define $INFO(fmt, ...)     printf("[ICM42688P] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
	#define $ERROR(fmt, ...)    printf("[ICM42688P] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
	#define $SUCCESS(fmt, ...)  printf("[ICM42688P] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
#else
    #define $INFO(fmt, ...)
	#define $ERROR(fmt, ...) 
	#define $SUCCESS(fmt, ...) 
#endif 

typedef enum {
    ICM_OK = 0,
    ICM_ERROR
} ICM_Status_t;

typedef struct ICM_DataPacket_t{
	float temp;
	float accX;
	float accY;
	float accZ;
	float gyroX;
	float gyroY;
	float gyroZ;
} ICM_DataPacket_t;

typedef enum {
	GYRO_DPS_2000 	= 0,
	GYRO_DPS_1000 	= 1,
	GYRO_DPS_500 	= 2,
	GYRO_DPS_250 	= 3,
	GYRO_DPS_125 	= 4,
	GYRO_DPS_62_5 	= 5,
	GYRO_DPS_31_25 	= 6,
	GYRO_DPS_15_625 = 7
} ICM42688P_GYRO_FRS_t;

typedef enum {
	ACCEL_GPM_16 	= 0,
	ACCEL_GPM_8 	= 1,
	ACCEL_GPM_4 	= 2,
	ACCEL_GPM_2 	= 3
} ICM42688P_ACCEL_FRS_t;

typedef enum {
	ODR_32k		= 1,	// LN Mode Only
	ODR_16k		= 2,	// LN Mode Only
	ODR_8k		= 3,	// LN Mode Only
	ODR_4k		= 4,	// LN Mode Only
	ODR_2k		= 5,	// LN Mode Only
	ODR_1k		= 6,	// LN Mode Only
	ODR_200		= 7,
	ODR_100		= 8,
	ODR_50		= 9,
	ODR_25		= 10,
	ODR_12_5	= 11,
	ODR_6_25    = 12,	// LP Mode Only (for accel)
	ODR_3_125	= 13,	// LP Mode Only (for accel)
	ODR_1_5626	= 14,	// LP Mode Only (for accel)
	ODR_500		= 15		
} ICM42688P_ODR_t;

typedef enum {
	GYRO_OFF = 0,
	GYRO_STANDBY = 1,
	GYRO_LN = 3
} ICM42688P_GYRO_PWR_t;

typedef enum {
	ACCEL_OFF = 0,
	ACCEL_LP = 2,
	ACCEL_LN = 3
} ICM42688P_ACCEL_PWR_t;

ICM_Status_t ICM42688P_init(SPI_TypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin, GPIO_TypeDef* int1Port, uint16_t int1Pin, GPIO_TypeDef* int2Port, uint16_t int2Pin);
ICM_Status_t ICM42688P_reset(void);
uint8_t ICM42688P_whoami(void);
ICM_Status_t ICM42688P_enableSensors(bool enTemp, bool enIdle, ICM42688P_GYRO_PWR_t gyroPwr, ICM42688P_ACCEL_PWR_t accelPwr);

ICM_Status_t ICM42688P_setGyroFSR(ICM42688P_GYRO_FRS_t gyroFsr);
ICM_Status_t ICM42688P_setAccelFSR(ICM42688P_ACCEL_FRS_t accelFsr);
ICM_Status_t ICM42688P_setGyroODR(ICM42688P_ODR_t gyroOdr);
ICM_Status_t ICM42688P_setAccelODR(ICM42688P_ODR_t accelOdr);
ICM_Status_t ICM42688P_setFilters(bool gyroFils, bool accelFils);
ICM_Status_t ICM42688P_enableDataRdyInt(void);
ICM_Status_t ICM42688P_disableDataRdyInt(void);

ICM_Status_t ICM42688P_getAccelGyroTempData(void);
float ICM42688_convertRawToTemp(uint16_t raw);
float ICM42688_getTemp(void);
float ICM42688_getAccX(void);
float ICM42688_getAccY(void);
float ICM42688_getAccZ(void);
float ICM42688_getGyroX(void);
float ICM42688_getGyroY(void);
float ICM42688_getGyroZ(void);

ICM_Status_t ICM42688P_enableFifo(bool accel, bool gyro, bool temp);
ICM_Status_t ICM42688P_readFifo(void);



#endif // INC_ICM42688P_H