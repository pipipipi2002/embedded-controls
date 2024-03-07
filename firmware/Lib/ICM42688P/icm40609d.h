/**
 * @file icm40609d.h
 * @author Marvin Pranajaya (pipipipi2002)
 * @brief 
 * @version 0.1
 * @date 2024-03-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef INC_ICM40609D_H
#define INC_ICM40609D_H

typedef enum ICM40609D_GYRO_FSR {
	ICM40609D_GYRO_DPS_2000 	= 0,    // Default
	ICM40609D_GYRO_DPS_1000 	= 1,
	ICM40609D_GYRO_DPS_500 	    = 2,
	ICM40609D_GYRO_DPS_250 	    = 3,
	ICM40609D_GYRO_DPS_125 	    = 4,
	ICM40609D_GYRO_DPS_62_5 	= 5,
	ICM40609D_GYRO_DPS_31_25 	= 6,
	ICM40609D_GYRO_DPS_15_625   = 7
} ICM40609D_GYRO_FSR_t;

typedef enum ICM40609D_ACCEL_FSR {
	ICM40609D_ACCEL_GPM_32 	= 0,    // Default
	ICM40609D_ACCEL_GPM_16 	= 1,
	ICM40609D_ACCEL_GPM_8 	= 2,
	ICM40609D_ACCEL_GPM_4 	= 3
} ICM40609D_ACCEL_FSR_t;

typedef enum ICM40609D_ODR {
	ICM40609D_ODR_32k		= 1,	// LN Mode Only
	ICM40609D_ODR_16k		= 2,	// LN Mode Only
	ICM40609D_ODR_8k		= 3,	// LN Mode Only
	ICM40609D_ODR_4k		= 4,	// LN Mode Only
	ICM40609D_ODR_2k		= 5,	// LN Mode Only
	ICM40609D_ODR_1k		= 6,	// LN Mode Only Default
	ICM40609D_ODR_200		= 7,
	ICM40609D_ODR_100		= 8,
	ICM40609D_ODR_50		= 9,
	ICM40609D_ODR_25		= 10,
	ICM40609D_ODR_12_5	    = 11,
	ICM40609D_ODR_6_25      = 12,	// LP Mode Only (for accel)
	ICM40609D_ODR_3_125	    = 13,	// LP Mode Only (for accel)
	ICM40609D_ODR_1_5626	= 14,	// LP Mode Only (for accel)
	ICM40609D_ODR_500		= 15		
} ICM40609D_ODR_t;

#endif // INC_ICM40609D_H