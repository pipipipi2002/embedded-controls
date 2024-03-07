/**
 * @file icm42688p.h
 * @author Marvin Pranajaya	(pipipipi2002)
 * @brief 
 * @version 0.1
 * @date 2024-03-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef INC_ICM42688P_H
#define INC_ICM42688P_H

typedef enum ICM42688P_GYRO_FSR {
	ICM42688P_GYRO_DPS_2000 	= 0,	// Default
	ICM42688P_GYRO_DPS_1000 	= 1,
	ICM42688P_GYRO_DPS_500 		= 2,
	ICM42688P_GYRO_DPS_250 		= 3,
	ICM42688P_GYRO_DPS_125 		= 4,
	ICM42688P_GYRO_DPS_62_5 	= 5,
	ICM42688P_GYRO_DPS_31_25 	= 6,
	ICM42688P_GYRO_DPS_15_625 	= 7
} ICM42688P_GYRO_FSR_t;

typedef enum ICM42688P_ACCEL_FSR {
	ICM42688P_ACCEL_GPM_16 	= 0,		// default
	ICM42688P_ACCEL_GPM_8 	= 1,
	ICM42688P_ACCEL_GPM_4 	= 2,
	ICM42688P_ACCEL_GPM_2 	= 3
} ICM42688P_ACCEL_FSR_t;

typedef enum ICM42688P_ODR {
	ICM42688P_ODR_32k		= 1,	// LN Mode Only
	ICM42688P_ODR_16k		= 2,	// LN Mode Only
	ICM42688P_ODR_8k		= 3,	// LN Mode Only
	ICM42688P_ODR_4k		= 4,	// LN Mode Only
	ICM42688P_ODR_2k		= 5,	// LN Mode Only
	ICM42688P_ODR_1k		= 6,	// LN Mode Only	Default
	ICM42688P_ODR_200		= 7,
	ICM42688P_ODR_100		= 8,
	ICM42688P_ODR_50		= 9,
	ICM42688P_ODR_25		= 10,
	ICM42688P_ODR_12_5		= 11,
	ICM42688P_ODR_6_25    	= 12,	// LP Mode Only (for accel)
	ICM42688P_ODR_3_125		= 13,	// LP Mode Only (for accel)
	ICM42688P_ODR_1_5626	= 14,	// LP Mode Only (for accel)
	ICM42688P_ODR_500		= 15		
} ICM42688P_ODR_t;

#endif // INC_ICM42688P_H