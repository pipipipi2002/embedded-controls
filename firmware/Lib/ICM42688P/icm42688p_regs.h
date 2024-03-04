/**
 * @file icm42688p_regs.h
 * @author Marvin Pranajaya (pipipipi2002)
 * @brief Registers for ICM IMU range. Checked on datasheets from ICM42688P.
 * @version 0.1
 * @date 2024-03-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef INC_ICM42688P_REGS_H
#define INC_ICM42688P_REGS_H

#define ICM42688P_WHOAMI_BYTE   (0x47)
#define ICM40609D_WHOAMI_BYTE   (0x3B)

#define ICM42688P_TEMP_DATA_REG_SCALE   (132.48f)
#define ICM42688P_TEMP_OFFSET           (25.0f)

/* ===========================================
 * BANK0, 1, 2, 3, 4 ADDRESSES
 * =========================================== */

/* BANK 0 */
#define DEVICE_CONFIG           (0x11)
#define DRIVE_CONFIG            (0x13)
#define INT_CONFIG              (0x14)
#define FIFO_CONFIG             (0x16)
#define TEMP_DATA1              (0x1D)
#define TEMP_DATA0              (0x1E)
#define ACCEL_DATA_X1           (0x1F)
#define ACCEL_DATA_X0           (0x20)            
#define ACCEL_DATA_Y1           (0x21)
#define ACCEL_DATA_Y0           (0x22)
#define ACCEL_DATA_Z1           (0x23)
#define ACCEL_DATA_Z0           (0x24)
#define GYRO_DATA_X1            (0x25)
#define GYRO_DATA_X0            (0x26)
#define GYRO_DATA_Y1            (0x27)
#define GYRO_DATA_Y0            (0x28)
#define GYRO_DATA_Z1            (0x29)
#define GYRO_DATA_Z0            (0x2A)
#define TMST_FSYNCH             (0x2B)
#define TMST_FSYNCL             (0x2C)
#define INT_STATUS              (0x2D)
#define FIFO_COUNTH             (0x2E)
#define FIFO_COUNTL             (0x2F)
#define FIFO_DATA               (0x30)
#define APEX_DATA0              (0x31)
#define APEX_DATA1              (0x32)
#define APEX_DATA2              (0x33)
#define APEX_DATA3              (0x34)
#define APEX_DATA4              (0x35)
#define APEX_DATA5              (0x36)
#define INT_STATUS2             (0x37)
#define INT_STATUS3             (0x38)
#define SIGNAL_PATH_RESET       (0x4B)    
#define INTF_CONFIG0            (0x4C)
#define INTF_CONFIG1            (0x4D)
#define PWR_MGMT0               (0x4E)
#define GYRO_CONFIG0            (0x4F)
#define ACCEL_CONFIG0           (0x50)
#define GYRO_CONFIG1            (0x51)
#define GYRO_ACCEL_CONFIG0      (0x52)
#define ACCEL_CONFIG1           (0x53)
#define TMST_CONFIG             (0x54)
#define APEX_CONFIG0            (0x56)
#define SMD_CONFIG              (0x57)
#define FIFO_CONFIG1            (0x5F)
#define FIFO_CONFIG2            (0x60)
#define FIFO_CONFIG3            (0x61)
#define FYSNC_CONFIG            (0x62)
#define INT_CONFIG0             (0x63)
#define INT_CONFIG1             (0x64)
#define INT_SOURCE0             (0x65)
#define INT_SOURCE1             (0x66)
#define INT_SOURCE3             (0x68)
#define INT_SOURCE4             (0x69)
#define FIFO_LOST_PKT0          (0x6C)
#define FIFO_LOST_PKT1          (0x6D)
#define SELF_TEST_CONFIG        (0x70)
#define WHO_AM_I                (0x75)
#define REG_BANK_SEL            (0x76)


/* BANK1 */
#define SENSOR_CONFIG0          (0x03)
#define GYRO_CONFIG_STATIC2     (0x0B)
#define GYRO_CONFIG_STATIC3     (0x0C)
#define GYRO_CONFIG_STATIC4     (0x0D)
#define GYRO_CONFIG_STATIC5     (0x0E)
#define GYRO_CONFIG_STATIC6     (0x0F)
#define GYRO_CONFIG_STATIC7     (0x10)
#define GYRO_CONFIG_STATIC8     (0x11)
#define GYRO_CONFIG_STATIC9     (0x12)
#define GYRO_CONFIG_STATIC10    (0x13)
#define XG_ST_DATA              (0x5F)
#define YG_ST_DATA              (0x60)
#define ZG_ST_DATA              (0x61)
#define TMSTVAL0                (0x62)
#define TMSTVAL2                (0x63)
#define TMSTVAL3                (0x64)
#define INTF_CONFIG4            (0x7A)    
#define INTF_CONFIG5            (0x7B)    
#define INTF_CONFIG6            (0x7C)    


/* BANK2 */
#define ACCEL_CONFIG_STATIC2    (0x03)
#define ACCEL_CONFIG_STATIC3    (0x04)
#define ACCEL_CONFIG_STATIC4    (0x05)
#define XA_ST_DATA              (0x3B)
#define YA_ST_DATA              (0x3C)
#define ZA_ST_DATA              (0x3D)

/* BANK3 */
#define CLKDIV                  (0x2A)

/* BANK4 */
#define APEX_CONFIG1            (0x40)
#define APEX_CONFIG2            (0x41)
#define APEX_CONFIG3            (0x42)
#define APEX_CONFIG4            (0x43)
#define APEX_CONFIG5            (0x44)
#define APEX_CONFIG6            (0x45)
#define APEX_CONFIG7            (0x46)
#define APEX_CONFIG8            (0x47)
#define APEX_CONFIG9            (0x48)
#define ACCEL_WOM_X_THR         (0x4A)
#define ACCEL_WOM_Y_THR         (0x4B)
#define ACCEL_WOM_Z_THR         (0x4C)
#define INT_SOURCE6             (0x4D)
#define INT_SOURCE7             (0x4E)
#define INT_SOURCE8             (0x4F)
#define INT_SOURCE9             (0x50)
#define INT_SOURCE10            (0x51)    
#define OFFSET_USER0            (0x77)    
#define OFFSET_USER1            (0x78)    
#define OFFSET_USER2            (0x79)    
#define OFFSET_USER3            (0x7A)    
#define OFFSET_USER4            (0x7B)    
#define OFFSET_USER5            (0x7C)    
#define OFFSET_USER6            (0x7D)    
#define OFFSET_USER7            (0x7E)    
#define OFFSET_USER8            (0x7F)    


/* ===========================================
 * Register BANK0
 * =========================================== */
/**
 * SPI_MODE
 *  0: Mode 0 and Mode 3 (default)
 *  1: Mode 1 and Mode 2
*/
#define DEVICE_CONFIG_SPI_MODE_POS              (0x4)
#define DEVICE_CONFIG_SPI_MODE_MASK             (0x1 << DEVICE_CONFIG_SPI_MODE_POS)

/**
 * SOFT_RESET_CONFIG
 *  0: Normal (default)
 *  1: Enable Reset
 */
#define DEVICE_CONFIG_SPI_SOFT_RESET_CONFIG_POS     (0x0)
#define DEVICE_CONFIG_SPI_SOFT_RESET_CONFIG_MASK    (0x1)

#define DRIVE_CONFIG_I2C_SLEW_RATE_POS          (0x3)
#define DRIVE_CONFIG_I2C_SLEW_RATE_MASK         (0x7 << DRIVE_CONFIG_I2C_SLEW_RATE_POS)

#define DRIVE_CONFIG_SPI_SLEW_RATE_POS          (0x0)
#define DRIVE_CONFIG_SPI_SLEW_RATE_MASK         (0x7)

#define INT_CONFIG_INT2_MODE_POS                (0x5)
#define INT_CONFIG_INT2_MODE_MASK               (0x1 << INT_CONFIG_INT2_MODE_POS)

#define INT_CONFIG_INT2_DRIVE_CIRCUIT_POS       (0x4)
#define INT_CONFIG_INT2_DRIVE_CIRCUIT_MASK      (0x1 << INT_CONFIG_INT2_DRIVE_CIRCUIT_POS)

#define INT_CONFIG_INT2_POLARITY_POS            (0x3)
#define INT_CONFIG_INT2_POLARITY_MASK           (0x1 << INT_CONFIG_INT2_POLARITY_POS)

#define INT_CONFIG_INT1_MODE_POS                (0x2)
#define INT_CONFIG_INT1_MODE_MASK               (0x1 << INT_CONFIG_INT1_MODE_POS)

#define INT_CONFIG_INT1_DRIVE_CIRCUIT_POS       (0x1)
#define INT_CONFIG_INT1_DRIVE_CIRCUIT_MASK      (0x1 << INT_CONFIG_INT1_DRIVE_CIRCUIT_POS)

#define INT_CONFIG_INT1_POLARITY_POS            (0x0)
#define INT_CONFIG_INT1_POLARITY_MASK           (0x1)

#define FIFO_CONFIG_FIFO_MODE_POS               (0x6)
#define FIFO_CONFIG_FIFO_MODE_MASK              (0x3 << FIFO_CONFIG_FIFO_MODE_POS)

#define TEMP_DATA1_TEMP_DATA_POS                (0x0)
#define TEMP_DATA1_TEMP_DATA_MASK               (0xFF)

#define TEMP_DATA0_TEMP_DATA_POS                (0x0)
#define TEMP_DATA0_TEMP_DATA_MASK               (0xFF)

#define ACCEL_DATA_X1_ACCEL_DATA_X_POS          (0x0)
#define ACCEL_DATA_X1_ACCEL_DATA_X_MASK         (0xFF)

#define ACCEL_DATA_X0_ACCEL_DATA_X_POS          (0x0)
#define ACCEL_DATA_X0_ACCEL_DATA_X_MASK         (0xFF)

#define ACCEL_DATA_Y1_ACCEL_DATA_Y_POS          (0x0)
#define ACCEL_DATA_Y1_ACCEL_DATA_Y_MASK         (0xFF)

#define ACCEL_DATA_Y0_ACCEL_DATA_Y_POS          (0x0)
#define ACCEL_DATA_Y0_ACCEL_DATA_Y_MASK         (0xFF)

#define ACCEL_DATA_Z1_ACCEL_DATA_Z_POS          (0x0)
#define ACCEL_DATA_Z1_ACCEL_DATA_Z_MASK         (0xFF)

#define ACCEL_DATA_Z0_ACCEL_DATA_Z_POS          (0x0)
#define ACCEL_DATA_Z0_ACCEL_DATA_Z_MASK         (0xFF)

#define GYRO_DATA_X1_GYRO_DATA_X_POS            (0x0)
#define GYRO_DATA_X1_GYRO_DATA_X_MASK           (0xFF)

#define GYRO_DATA_X0_GYRO_DATA_X_POS            (0x0)
#define GYRO_DATA_X0_GYRO_DATA_X_MASK           (0xFF)

#define GYRO_DATA_Y1_GYRO_DATA_Y_POS            (0x0)
#define GYRO_DATA_Y1_GYRO_DATA_Y_MASK           (0xFF)

#define GYRO_DATA_Y0_GYRO_DATA_Y_POS            (0x0)
#define GYRO_DATA_Y0_GYRO_DATA_Y_MASK           (0xFF)

#define GYRO_DATA_Z1_GYRO_DATA_Z_POS            (0x0)
#define GYRO_DATA_Z1_GYRO_DATA_Z_MASK           (0xFF)

#define GYRO_DATA_Z0_GYRO_DATA_Z_POS            (0x0)
#define GYRO_DATA_Z0_GYRO_DATA_Z_MASK           (0xFF)

#define TMST_FSYNCH_TMST_FSYNC_DATA_POS         (0x0)
#define TMST_FSYNCH_TMST_FSYNC_DATA_MASK        (0xFF)

#define TMST_FSYNCL_TMST_FSYNC_DATA_POS         (0x0)
#define TMST_FSYNCL_TMST_FSYNC_DATA_MASK        (0xFF)

#define INT_STATUS_UI_FSYNC_INT_POS             (0x6)
#define INT_STATUS_UI_FSYNC_INT_MASK            (0x1 << INT_STATUS_UI_FSYNC_INT_POS)

#define INT_STATUS_PLL_RDY_INT_POS              (0x5)
#define INT_STATUS_PLL_RDY_INT_MASK             (0x1 << INT_STATUS_PLL_RDY_INT_POS)

#define INT_STATUS_RESET_DONE_INT_POS           (0x4)
#define INT_STATUS_RESET_DONE_INT_MASK          (0x1 << INT_STATUS_RESET_DONE_INT_POS)

#define INT_STATUS_DATA_RDY_INT_POS             (0x3)
#define INT_STATUS_DATA_RDY_INT_MASK            (0x1 << INT_STATUS_DATA_RDY_INT_POS)

#define INT_STATUS_FIFO_THS_INT_POS             (0x2)
#define INT_STATUS_FIFO_THS_INT_MASK            (0x1 << INT_STATUS_FIFO_THS_INT_POS)

#define INT_STATUS_FIFO_FULL_INT_POS            (0x1)
#define INT_STATUS_FIFO_FULL_INT_MASK           (0x1 << INT_STATUS_FIFO_FULL_INT_POS)

#define INT_STATUS_AGC_RDY_INT_POS              (0x0)
#define INT_STATUS_AGC_RDY_INT_MASK             (0x1)

#define FIFO_COUNTH_FIFO_COUNT_POS              (0x0)
#define FIFO_COUNTH_FIFO_COUNT_MASK             (0xFF)

#define FIFO_COUNTL_FIFO_COUNT_POS              (0x0)
#define FIFO_COUNTL_FIFO_COUNT_MASK             (0xFF)

#define FIFO_DATA_FIFO_DATA_POS                 (0x0)
#define FIFO_DATA_FIFO_DATA_MASK                (0xFF)

#define APEX_DATA0_STEP_CNT_POS                 (0x0)
#define APEX_DATA0_STEP_CNT_MASK                (0xFF)

#define APEX_DATA1_STEP_CNT_POS                 (0x0)
#define APEX_DATA1_STEP_CNT_MASK                (0xFF)

#define APEX_DATA2_STEP_CADENCE_POS             (0x0)
#define APEX_DATA2_STEP_CADENCE_MASK            (0xFF)

#define APEX_DATA3_DMP_IDLE_POS                 (0x2)
#define APEX_DATA3_DMP_IDLE_MASK                (0x1 << APEX_DATA3_DMP_IDLE_POS)

#define APEX_DATA3_ACTIVITY_CLASS_POS           (0x0)
#define APEX_DATA3_ACTIVITY_CLASS_MASK          (0x3)

#define APEX_DATA4_TAP_NUM_POS                  (0x3)
#define APEX_DATA4_TAP_NUM_MASK                 (0x3 << APEX_DATA4_TAP_NUM_POS)

#define APEX_DATA4_TAP_AXIS_POS                 (0x1)
#define APEX_DATA4_TAP_AXIS_MASK                (0x3 << APEX_DATA4_TAP_AXIS_POS)

#define APEX_DATA4_TAP_DIR_POS                  (0x0)
#define APEX_DATA4_TAP_DIR_MASK                 (0x1)

#define APEX_DATA5_DOUBLE_TAP_TIMING_POS        (0x0)
#define APEX_DATA5_DOUBLE_TAP_TIMING_MASK       (0x3F << APEX_DATA5_DOUBLE_TAP_TIMING_POS)

#define INT_STATUS2_SMD_INT_POS                 (0x3)
#define INT_STATUS2_SMD_INT_MASK                (0x1 << INT_STATUS2_SMD_INT_POS)

#define INT_STATUS2_WOM_Z_INT_POS               (0x2)
#define INT_STATUS2_WOM_Z_INT_MASK              (0x1 << INT_STATUS2_WOM_Z_INT_POS)

#define INT_STATUS2_WOM_Y_INT_POS               (0x1)
#define INT_STATUS2_WOM_Y_INT_MASK              (0x1 << INT_STATUS2_WOM_Y_INT_POS)

#define INT_STATUS2_WOM_X_INT_POS               (0x0)
#define INT_STATUS2_WOM_X_INT_MASK              (0x1)

#define INT_STATUS3_STEP_DET_INT_POS            (0x5)
#define INT_STATUS3_STEP_DET_INT_MASK           (0x1 << INT_STATUS3_STEP_DET_INT_POS)

#define INT_STATUS3_STEP_CNT_OVF_INT_POS        (0x4)
#define INT_STATUS3_STEP_CNT_OVF_INT_MASK       (0x1 << INT_STATUS3_STEP_CNT_OVF_INT_POS)

#define INT_STATUS3_TILT_DET_INT_POS            (0x3)
#define INT_STATUS3_TILT_DET_INT_MASK           (0x1 << INT_STATUS3_TILT_DET_INT_POS)

#define INT_STATUS3_WAKE_INT_POS                (0x2)
#define INT_STATUS3_WAKE_INT_MASK               (0x1 << INT_STATUS3_WAKE_INT_POS)

#define INT_STATUS3_SLEEP_INT_POS               (0x1)
#define INT_STATUS3_SLEEP_INT_MASK              (0x1 << INT_STATUS3_SLEEP_INT_POS)

#define INT_STATUS3_TAP_DET_INT_POS             (0x0)
#define INT_STATUS3_TAP_DET_INT_MASK            (0x1)

#define SIGNAL_PATH_RESET_DMP_INIT_EN_POS       (0x6)
#define SIGNAL_PATH_RESET_DMP_INIT_EN_MASK      (0x1 << SIGNAL_PATH_RESET_DMP_INIT_EN_POS)

#define SIGNAL_PATH_RESET_DMP_MEM_RESET_EN_POS  (0x5)
#define SIGNAL_PATH_RESET_DMP_MEM_RESET_EN_MASK (0x1 << SIGNAL_PATH_RESET_DMP_MEM_RESET_EN_POS)

#define SIGNAL_PATH_RESET_ABORT_AND_RESET_POS   (0x3)
#define SIGNAL_PATH_RESET_ABORT_AND_RESET_MASK  (0x1 << SIGNAL_PATH_RESET_ABORT_AND_RESET_POS)

#define SIGNAL_PATH_RESET_TMST_STROBE_POS       (0x2)
#define SIGNAL_PATH_RESET_TMST_STROBE_MASK      (0x1 << SIGNAL_PATH_RESET_TMST_STROBE_POS)

#define SIGNAL_PATH_RESET_FIFO_FLUSH_POS        (0x1)
#define SIGNAL_PATH_RESET_FIFO_FLUSH_MASK       (0x1 << SIGNAL_PATH_RESET_FIFO_FLUSH_POS)

#define INTF_CONFIG0_FIFO_HOLD_LAST_DATA_EN_POS     (0x7)
#define INTF_CONFIG0_FIFO_HOLD_LAST_DATA_EN_MASK    (0x1 << INTF_CONFIG0_FIFO_HOLD_LAST_DATA_EN_POS)

#define INTF_CONFIG0_FIFO_COUNT_REC_POS         (0x6)
#define INTF_CONFIG0_FIFO_COUNT_REC_MASK        (0x1 << INTF_CONFIG0_FIFO_COUNT_REC_POS)

#define INTF_CONFIG0_FIFO_COUNT_ENDIAN_POS      (0x5)
#define INTF_CONFIG0_FIFO_COUNT_ENDIAN_MASK     (0x1 << INTF_CONFIG0_FIFO_COUNT_ENDIAN_POS)

#define INTF_CONFIG0_SENSOR_DATA_ENDIAN_POS     (0x4)
#define INTF_CONFIG0_SENSOR_DATA_ENDIAN_MASK    (0x1 << INTF_CONFIG0_SENSOR_DATA_ENDIAN_POS)

#define INTF_CONFIG0_UI_SIFS_CFG_POS            (0x0)
#define INTF_CONFIG0_UI_SIFS_CFG_MASK           (0x3)

#define INTF_CONFIG1_ACCEL_LP_CLK_SEL_POS       (0x3)
#define INTF_CONFIG1_ACCEL_LP_CLK_SEL_MASK      (0x1 << INTF_CONFIG1_ACCEL_LP_CLK_SEL_POS)

#define INTF_CONFIG1_RTC_MODE_POS               (0x2)
#define INTF_CONFIG1_RTC_MODE_MASK              (0x1 << INTF_CONFIG1_RTC_MODE_POS)

#define INTF_CONFIG1_CLKSEL_POS                 (0x0)
#define INTF_CONFIG1_CLKSEL_MASK                (0x3)

/**
 * TEMP_DIS
 *  0: Temp sensor enabled (default)
 *  1: Temp sensor disabled
 */
#define PWR_MGMT0_TEMP_DIS_POS                  (0x5)
#define PWR_MGMT0_TEMP_DIS_MASK                 (0x1 << PWR_MGMT0_TEMP_DIS_POS)
    #define TEMP_ENABLE         (0x0)
    #define TEMP_DISABLE        (0x1 << PWR_MGMT0_TEMP_DIS_POS)

/**
 * IDLE
 *  0: Chip will be off when accel/gyro off (default)
 *  1: Stay on even when accel/gyro off
 */
#define PWR_MGMT0_IDLE_POS                      (0x4)
#define PWR_MGMT0_IDLE_MASK_MASK                (0x1 << PWR_MGMT0_IDLE_POS)
    #define IDLE_DISABLE        (0x0)         
    #define IDLE_ENABLE         (0x1 << PWR_MGMT0_IDLE_POS)

/**
 * GYRO_MODE
 *  00: GYRO OFF (default)
 *  01: GYRO Standby
 *  11: GYRO in Low Noise
 */
#define PWR_MGMT0_GYRO_MODE_POS                 (0x2)
#define PWR_MGMT0_GYRO_MODE_MASK                (0x3 << PWR_MGMT0_GYRO_MODE_POS)
    #define GYRO_MODE_OFF       (0x0)
    #define GYRO_MODE_STANDBY   (0x1 << PWR_MGMT0_GYRO_MODE_POS)
    #define GYRO_MODE_LN        (0x3 << PWR_MGMT0_GYRO_MODE_POS)

/**
 * ACCEL_MODE
 *  00: ACCEL OFF (default)
 *  01: ACCEL OFF 
 *  10: ACCEL in Low Power 
 *  11: ACCEL in Low Noise
 */
#define PWR_MGMT0_ACCEL_MODE_POS                (0x0)
#define PWR_MGMT0_ACCEL_MODE_MASK               (0x3)
    #define ACCEL_MODE_OFF      (0x0)
    #define ACCEL_MODE_LP       (0x2 << PWR_MGMT0_ACCEL_MODE_POS)
    #define ACCEL_MODE_LN       (0x3 << PWR_MGMT0_ACCEL_MODE_POS)

#define GYRO_CONFIG0_GYRO_FS_SEL_POS            (0x5)
#define GYRO_CONFIG0_GYRO_FS_SEL_MASK           (0x7 << GYRO_CONFIG0_GYRO_FS_SEL_POS)

#define GYRO_CONFIG0_GYRO_ODR_POS               (0x0)
#define GYRO_CONFIG0_GYRO_ODR_MASK              (0xF)

#define ACCEL_CONFIG0_ACCEL_FS_SEL_POS          (0x5)
#define ACCEL_CONFIG0_ACCEL_FS_SEL_MASK         (0x7 << ACCEL_CONFIG0_ACCEL_FS_SEL_POS)

#define ACCEL_CONFIG0_ACCEL_ODR_POS             (0x0)
#define ACCEL_CONFIG0_ACCEL_ODR_MASK            (0xF)

#define GYRO_CONFIG1_TEMP_FILT_BW_POS           (0x5)
#define GYRO_CONFIG1_TEMP_FILT_BW_MASK          (0x7 << GYRO_CONFIG1_TEMP_FILT_BW_POS)

#define GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS       (0x2)
#define GYRO_CONFIG1_GYRO_UI_FILT_ORD_MASK      (0x3 << GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS)

#define GYRO_CONFIG1_GYRO_DEC2_M2_ORD_POS       (0x0)
#define GYRO_CONFIG1_GYRO_DEC2_M2_ORD_MASK      (0x3)

#define GYRO_ACCEL_CONFIG0_ACCEL_UI_FILT_BW_POS     (0x4)
#define GYRO_ACCEL_CONFIG0_ACCEL_UI_FILT_BW_MASK    (0xF << GYRO_ACCEL_CONFIG0_ACCEL_UI_FILT_BW_POS)

#define GYRO_ACCEL_CONFIG0_GYRO_UI_FILT_BW_POS      (0x0)
#define GYRO_ACCEL_CONFIG0_GYRO_UI_FILT_BW_MASK     (0xF)

#define ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS     (0x3)
#define ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_MASK    (0x3 << ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS)
    
#define ACCEL_CONFIG1_ACCEL_DEC2_M2_ORD_POS     (0x1)
#define ACCEL_CONFIG1_ACCEL_DEC2_M2_ORD_MASK    (0x3 << ACCEL_CONFIG1_ACCEL_DEC2_M2_ORD_POS)

#define TMST_CONFIG_TMST_TO_REGS_EN_POS         (0x4)
#define TMST_CONFIG_TMST_TO_REGS_EN_MASK        (0x1 << TMST_CONFIG_TMST_TO_REGS_EN_POS)

#define TMST_CONFIG_TMST_RES_POS                (0x3)
#define TMST_CONFIG_TMST_RES_MASK               (0x1 << TMST_CONFIG_TMST_RES_POS)

#define TMST_CONFIG_TMST_DELTA_EN_POS           (0x2)
#define TMST_CONFIG_TMST_DELTA_EN_MASK          (0x1 << TMST_CONFIG_TMST_DELTA_EN_POS)

#define TMST_CONFIG_TMST_FSYNC_EN_POS           (0x1)
#define TMST_CONFIG_TMST_FSYNC_EN_MASK          (0x1 << TMST_CONFIG_TMST_FSYNC_EN_POS)

#define TMST_CONFIG_TMST_EN_POS                 (0x0)
#define TMST_CONFIG_TMST_EN_MASK                (0x1 << TMST_CONFIG_TMST_EN_POS)

#define APEX_CONFIG0_DWP_POWER_SAVE_POS         (0x7)
#define APEX_CONFIG0_DWP_POWER_SAVE_MASK        (0x1 << APEX_CONFIG0_DWP_POWER_SAVE_POS)

#define APEX_CONFIG0_TAP_ENABLE_POS             (0x6)
#define APEX_CONFIG0_TAP_ENABLE_MASK            (0x1 << APEX_CONFIG0_TAP_ENABLE_POS)

#define APEX_CONFIG0_PED_ENABLE_POS             (0x5)
#define APEX_CONFIG0_PED_ENABLE_MASK            (0x1 << APEX_CONFIG0_PED_ENABLE_POS)

#define APEX_CONFIG0_TILT_ENABLE_POS            (0x4)
#define APEX_CONFIG0_TILT_ENABLE_MASK           (0x1 << APEX_CONFIG0_TILT_ENABLE_POS)

#define APEX_CONFIG0_R2W_EN_POS                 (0x3)
#define APEX_CONFIG0_R2W_EN_MASK                (0x1 << APEX_CONFIG0_R2W_EN_POS)

#define APEX_CONFIG0_DMP_ODR_POS                (0x0)
#define APEX_CONFIG0_DMP_ODR_MASK               (0x3)

#define SMD_CONFIG_WOM_INT_MODE_POS             (0x3)
#define SMD_CONFIG_WOM_INT_MODE_MASK            (0x1 << SMD_CONFIG_WOM_INT_MODE_POS)

#define SMD_CONFIG_WOM_MODE_POS                 (0x2)
#define SMD_CONFIG_WOM_MODE_MASK                (0x1 << SMD_CONFIG_WOM_MODE_POS)

#define SMD_CONFIG_SMD_MODE_POS                 (0x0)
#define SMD_CONFIG_SMD_MODE_MASK                (0x3)

#define FIFO_CONFIG1_FIFO_RESUME_PARTIAL_RD_POS     (0x6)
#define FIFO_CONFIG1_FIFO_RESUME_PARTIAL_RD_MASK    (0x1 << FIFO_CONFIG1_FIFO_RESUME_PARTIAL_RD_POS)

#define FIFO_CONFIG1_FIFO_WM_GT_TH_POS          (0x5)
#define FIFO_CONFIG1_FIFO_WM_GT_TH_MASK         (0x1 << FIFO_CONFIG1_FIFO_WM_GT_TH_POS)

#define FIFO_CONFIG1_FIFO_HIRES_EN_POS          (0x4)
#define FIFO_CONFIG1_FIFO_HIRES_EN_MASK         (0x1 << FIFO_CONFIG1_FIFO_HIRES_EN_POS)

#define FIFO_CONFIG1_FIFO_TMST_FSYNC_EN_POS     (0x3)
#define FIFO_CONFIG1_FIFO_TMST_FSYNC_EN_MASK    (0x1 << FIFO_CONFIG1_FIFO_TMST_FSYNC_EN_POS)

#define FIFO_CONFIG1_FIFO_TEMP_EN_POS           (0x2)
#define FIFO_CONFIG1_FIFO_TEMP_EN_MASK          (0x1 << FIFO_CONFIG1_FIFO_TEMP_EN_POS)

#define FIFO_CONFIG1_FIFO_GYRO_EN_POS           (0x1)
#define FIFO_CONFIG1_FIFO_GYRO_EN_MASK          (0x1 << FIFO_CONFIG1_FIFO_GYRO_EN_POS)

#define FIFO_CONFIG1_FIFO_ACCEL_EN_POS          (0x0)
#define FIFO_CONFIG1_FIFO_ACCEL_EN_MASK         (0x1)

#define FIFO_CONFIG2_FIFO_WM_POS                (0x0)
#define FIFO_CONFIG2_FIFO_WM_MASK               (0xFF)

#define FIFO_CONFIG3_FIFO_WM_POS                (0x0)
#define FIFO_CONFIG3_FIFO_WM_MASK               (0xF)

#define FSYNC_CONFIG_FSYNC_UI_SEL_POS           (0x4)
#define FSYNC_CONFIG_FSYNC_UI_SEL_MASK          (0x1 << FSYNC_CONFIG_FSYNC_UI_SEL_POS)

#define FSYNC_CONFIG_FSYNC_UI_FLAG_CLEAR_SEL_POS    (0x1)
#define FSYNC_CONFIG_FSYNC_UI_FLAG_CLEAR_SEL_MASK   (0x1 << FSYNC_CONFIG_FSYNC_UI_FLAG_CLEAR_SEL_POS)

#define FSYNC_CONFIG_FSYNC_POLARITY_POS         (0x0)
#define FSYNC_CONFIG_FSYNC_POLARITY_MASK        (0x1)

#define INT_CONFIG0_UI_DRDY_INT_CLEAR_POS       (0x4)
#define INT_CONFIG0_UI_DRDY_INT_CLEAR_MASK      (0x3 << INT_CONFIG0_UI_DRDY_INT_CLEAR_POS)

#define INT_CONFIG0_FIFO_THS_INT_CLEAR_POS      (0x2)
#define INT_CONFIG0_FIFO_THS_INT_CLEAR_MASK     (0x3 << INT_CONFIG0_FIFO_THS_INT_CLEAR_POS)

#define INT_CONFIG0_FIFO_FULL_INT_CLEAR_POS     (0x0)
#define INT_CONFIG0_FIFO_FULL_INT_CLEAR_MASK    (0x3 << INT_CONFIG0_FIFO_FULL_INT_CLEAR_POS)

#define INT_CONFIG1_INT_TPULSE_DURATION_POS     (0x6)
#define INT_CONFIG1_INT_TPULSE_DURATION_MASK    (0x1 << INT_CONFIG1_INT_TPULSE_DURATION_POS)

#define INT_CONFIG1_INT_TDEASSERT_DISABLE_POS   (0x5)
#define INT_CONFIG1_INT_TDEASSERT_DISABLE_MASK  (0x1 << INT_CONFIG1_INT_TDEASSERT_DISABLE_POS)

#define INT_CONFIG1_INT_ASYNC_RESET_POS         (0x4)
#define INT_CONFIG1_INT_ASYNC_RESET_MASK        (0x1 << INT_CONFIG1_INT_ASYNC_RESET_POS)

#define INT_SOURCE0_UI_FSYNC_INT1_EN_POS        (0x6)
#define INT_SOURCE0_UI_FSYNC_INT1_EN_MASK       (0x1 << INT_SOURCE0_UI_FSYNC_INT1_EN_POS)

#define INT_SOURCE0_PLL_RDY_INT1_EN_POS         (0x5)
#define INT_SOURCE0_PLL_RDY_INT1_EN_MASK        (0x1 << INT_SOURCE0_PLL_RDY_INT1_EN_POS)

#define INT_SOURCE0_RESET_DONE_INT1_EN_POS      (0x4)
#define INT_SOURCE0_RESET_DONE_INT1_EN_MASK     (0x1 << INT_SOURCE0_RESET_DONE_INT1_EN_POS)

#define INT_SOURCE0_UI_DRDY_INT1_EN_POS         (0x3)
#define INT_SOURCE0_UI_DRDY_INT1_EN_MASK        (0x1 << INT_SOURCE0_UI_DRDY_INT1_EN_POS)

#define INT_SOURCE0_FIFO_THS_INT1_EN_POS        (0x2)
#define INT_SOURCE0_FIFO_THS_INT1_EN_MASK       (0x1 << INT_SOURCE0_FIFO_THS_INT1_EN_POS)

#define INT_SOURCE0_FIFO_FULL_INT1_EN_POS       (0x1)
#define INT_SOURCE0_FIFO_FULL_INT1_EN_MASK      (0x1 << INT_SOURCE0_FIFO_FULL_INT1_EN_POS)

#define INT_SOURCE0_UI_AGC_RDY_INT1_EN_POS      (0x0)
#define INT_SOURCE0_UI_AGC_RDY_INT1_EN_MASK     (0x1)

#define INT_SOURCE1_I3C_PROTOCOL_ERROR_INT1_EN_POS      (0x6)
#define INT_SOURCE1_I3C_PROTOCOL_ERROR_INT1_EN_MASK     (0x1 << INT_SOURCE1_I3C_PROTOCOL_ERROR_INT1_EN_POS)

#define INT_SOURCE1_SMD_INT1_EN_POS             (0x3)
#define INT_SOURCE1_SMD_INT1_EN_MASK            (0x1 << INT_SOURCE1_SMD_INT1_EN_POS)

#define INT_SOURCE1_WOM_Z_INT1_EN_POS           (0x2)
#define INT_SOURCE1_WOM_Z_INT1_EN_MASK          (0x1 << INT_SOURCE1_WOM_Z_INT1_EN_POS)

#define INT_SOURCE1_WOM_Y_INT1_EN_POS           (0x1)
#define INT_SOURCE1_WOM_Y_INT1_EN_MASK          (0x1 << INT_SOURCE1_WOM_Y_INT1_EN_POS)

#define INT_SOURCE1_WOM_X_INT1_EN_POS           (0x0)
#define INT_SOURCE1_WOM_X_INT1_EN_MASK          (0x1)

#define INT_SOURCE3_UI_FSYNC_INT2_EN_POS        (0x6)
#define INT_SOURCE3_UI_FSYNC_INT2_EN_MASK       (0x1 << INT_SOURCE3_UI_FSYNC_INT2_EN_POS)

#define INT_SOURCE3_PLL_RDY_INT2_EN_POS         (0x5)
#define INT_SOURCE3_PLL_RDY_INT2_EN_MASK        (0x1 << INT_SOURCE3_PLL_RDY_INT2_EN_POS)

#define INT_SOURCE3_RESET_DONE_INT2_EN_POS      (0x4)
#define INT_SOURCE3_RESET_DONE_INT2_EN_MASK     (0x1 << INT_SOURCE3_RESET_DONE_INT2_EN_POS)

#define INT_SOURCE3_UI_DRDY_INT2_EN_POS         (0x3)
#define INT_SOURCE3_UI_DRDY_INT2_EN_MASK        (0x1 << INT_SOURCE3_UI_DRDY_INT2_EN_POS)

#define INT_SOURCE3_FIFO_THS_INT2_EN_POS        (0x2)
#define INT_SOURCE3_FIFO_THS_INT2_EN_MASK       (0x1 << INT_SOURCE3_FIFO_THS_INT2_EN_POS)

#define INT_SOURCE3_FIFO_FULL_INT2_EN_POS       (0x1)
#define INT_SOURCE3_FIFO_FULL_INT2_EN_MASK      (0x1 << INT_SOURCE3_FIFO_FULL_INT2_EN_POS)

#define INT_SOURCE3_UI_AGC_RDY_INT2_EN_POS      (0x0)
#define INT_SOURCE3_UI_AGC_RDY_INT2_EN_MASK     (0x1)

#define INT_SOURCE4_I3C_PROTOCOL_ERROR_INT2_EN_POS      (0x6)
#define INT_SOURCE4_I3C_PROTOCOL_ERROR_INT2_EN_MASK     (0x1 << INT_SOURCE4_I3C_PROTOCOL_ERROR_INT2_EN_POS)

#define INT_SOURCE4_SMD_INT2_EN_POS             (0x3)
#define INT_SOURCE4_SMD_INT2_EN_MASK            (0x1 << INT_SOURCE4_SMD_INT2_EN_POS)

#define INT_SOURCE4_WOM_Z_INT2_EN_POS           (0x2)
#define INT_SOURCE4_WOM_Z_INT2_EN_MASK          (0x1 << INT_SOURCE4_WOM_Z_INT2_EN_POS)

#define INT_SOURCE4_WOM_Y_INT2_EN_POS           (0x1)
#define INT_SOURCE4_WOM_Y_INT2_EN_MASK          (0x1 << INT_SOURCE4_WOM_Y_INT2_EN_POS)

#define INT_SOURCE4_WOM_X_INT2_EN_POS           (0x0)
#define INT_SOURCE4_WOM_X_INT2_EN_MASK          (0x1)

#define FIFO_LOST_PKT0_FIFO_LOST_PKT_CNT_POS    (0x0)
#define FIFO_LOST_PKT0_FIFO_LOST_PKT_CNT_MASK   (0xFF)

#define FIFO_LOST_PKT1_FIFO_LOST_PKT_CNT_POS    (0x0)
#define FIFO_LOST_PKT1_FIFO_LOST_PKT_CNT_MASK   (0xFF)

#define SELF_TEST_CONFIG_ACCEL_ST_POWER_POS     (0x6)
#define SELF_TEST_CONFIG_ACCEL_ST_POWER_MASK    (0x1 << SELF_TEST_CONFIG_ACCEL_ST_POWER_POS)

#define SELF_TEST_CONFIG_EN_AZ_ST_POS           (0x5)
#define SELF_TEST_CONFIG_EN_AZ_ST_MASK          (0x1 << SELF_TEST_CONFIG_EN_AZ_ST_POS)

#define SELF_TEST_CONFIG_EN_AY_ST_POS           (0x4)
#define SELF_TEST_CONFIG_EN_AY_ST_MASK          (0x1 << SELF_TEST_CONFIG_EN_AY_ST_POS)

#define SELF_TEST_CONFIG_EN_AX_ST_POS           (0x3)
#define SELF_TEST_CONFIG_EN_AX_ST_MASK          (0x1 << SELF_TEST_CONFIG_EN_AX_ST_POS)

#define SELF_TEST_CONFIG_EN_GZ_ST_POS           (0x2)
#define SELF_TEST_CONFIG_EN_GZ_ST_MASK          (0x1 << SELF_TEST_CONFIG_EN_GZ_ST_POS)

#define SELF_TEST_CONFIG_EN_GY_ST_POS           (0x1)
#define SELF_TEST_CONFIG_EN_GY_ST_MASK          (0x1 << SELF_TEST_CONFIG_EN_GY_ST_POS)

#define SELF_TEST_CONFIG_EN_GX_ST_POS           (0x0)
#define SELF_TEST_CONFIG_EN_GX_ST_MASK          (0x1)

#define WHO_AM_I_WHOAMI_POS                     (0x0)
#define WHO_AM_I_WHOAMI_MASK                    (0xFF)

#define REG_BANK_SEL_BANK_SEL_POS               (0x0)
#define REG_BANK_SEL_BANK_SEL_MASK              (0x7)


/* ===========================================
 * Register BANK1
 * =========================================== */
#define SENSOR_CONFIG0_ZG_DISABLE_POS           (0x5)
#define SENSOR_CONFIG0_ZG_DISABLE_MASK          (0x1 << SENSOR_CONFIG0_ZG_DISABLE_POS)

#define SENSOR_CONFIG0_YG_DISABLE_POS           (0x4)
#define SENSOR_CONFIG0_YG_DISABLE_MASK          (0x1 << SENSOR_CONFIG0_YG_DISABLE_POS)
    
#define SENSOR_CONFIG0_XG_DISABLE_POS           (0x3)
#define SENSOR_CONFIG0_XG_DISABLE_MASK          (0x1 << SENSOR_CONFIG0_XG_DISABLE_POS)

#define SENSOR_CONFIG0_ZA_DISABLE_POS           (0x2)
#define SENSOR_CONFIG0_ZA_DISABLE_MASK          (0x1 << SENSOR_CONFIG0_ZA_DISABLE_POS)

#define SENSOR_CONFIG0_YA_DISABLE_POS           (0x1)
#define SENSOR_CONFIG0_YA_DISABLE_MASK          (0x1 << SENSOR_CONFIG0_YA_DISABLE_POS)

#define SENSOR_CONFIG0_XA_DISABLE_POS           (0x0)
#define SENSOR_CONFIG0_XA_DISABLE_MASK          (0x1)

/**
 * GYRO_AAF_DIS
 *  0: Enable Gyro AAF (default)
 *  1: Disable Gyro AAF
 */
#define GYRO_CONFIG_STATIC2_GYRO_AAF_DIS_POS    (0x1)
#define GYRO_CONFIG_STATIC2_GYRO_AAF_DIS_MASK   (0x1 << GYRO_CONFIG_STATIC2_GYRO_AAF_DIS_POS)
    #define GYRO_AAF_ENABLE     (0x0)
    #define GYRO_AAF_DISABLE    (0x1 << GYRO_CONFIG_STATIC2_GYRO_AAF_DIS_POS)

/**
 * GYRO_NF_DIS
 *  0: Enable Gyro NF (default)
 *  1: Disable Gyro NF
 */
#define GYRO_CONFIG_STATIC2_GYRO_NF_DIS_POS     (0x0)
#define GYRO_CONFIG_STATIC2_GYRO_NF_DIS_MASK    (0x1)
    #define GYRO_NF_ENABLE      (0x0)
    #define GYRO_NF_DISABLE     (0x1)

#define GYRO_CONFIG_STATIC3_GYRO_AAF_DELT_POS   (0x0)
#define GYRO_CONFIG_STATIC3_GYRO_AAF_DELT_MASK  (0x3F)

#define GYRO_CONFIG_STATIC4_GYRO_AAF_DELTSQR_POS    (0x0)
#define GYRO_CONFIG_STATIC4_GYRO_AAF_DELTSQR_MASK   (0xFF)

#define GYRO_CONFIG_STATIC5_GYRO_AAF_BITSHIFT_POS   (0x4)
#define GYRO_CONFIG_STATIC5_GYRO_AAF_BITSHIFT_MASK  (0xF << GYRO_CONFIG_STATIC5_GYRO_AAF_BITSHIFT_POS)

#define GYRO_CONFIG_STATIC5_GYRO_AAF_DELTSQR_POS    (0x0)
#define GYRO_CONFIG_STATIC5_GYRO_AAF_DELTSQR_MASK   (0xF)

#define GYRO_CONFIG_STATIC6_GYRO_X_NF_COSWZ_POS     (0x0)
#define GYRO_CONFIG_STATIC6_GYRO_X_NF_COSWZ_MASK    (0xFF)

#define GYRO_CONFIG_STATIC7_GYRO_Y_NF_COSWZ_POS     (0x0)
#define GYRO_CONFIG_STATIC7_GYRO_Y_NF_COSWZ_MASK    (0xFF)

#define GYRO_CONFIG_STATIC8_GYRO_Z_NF_COSWZ_POS     (0x0)
#define GYRO_CONFIG_STATIC8_GYRO_Z_NF_COSWZ_MASK    (0xFF)

#define GYRO_CONFIG_STATIC9_GYRO_Z_NF_COSWZ_SEL_POS     (0x5)
#define GYRO_CONFIG_STATIC9_GYRO_Z_NF_COSWZ_SEL_MASK    (0x1 << GYRO_CONFIG_STATIC9_GYRO_Z_NF_COSWZ_SEL_POS)

#define GYRO_CONFIG_STATIC9_GYRO_Y_NF_COSWZ_SEL_POS     (0x4)
#define GYRO_CONFIG_STATIC9_GYRO_Y_NF_COSWZ_SEL_MASK    (0x1 << GYRO_CONFIG_STATIC9_GYRO_Y_NF_COSWZ_SEL_POS)

#define GYRO_CONFIG_STATIC9_GYRO_X_NF_COSWZ_SEL_POS     (0x3)
#define GYRO_CONFIG_STATIC9_GYRO_X_NF_COSWZ_SEL_MASK    (0x1 << GYRO_CONFIG_STATIC9_GYRO_X_NF_COSWZ_SEL_POS)

#define GYRO_CONFIG_STATIC9_GYRO_Z_NF_COSWZ_POS         (0x2)
#define GYRO_CONFIG_STATIC9_GYRO_Z_NF_COSWZ_MASK        (0x1 << GYRO_CONFIG_STATIC9_GYRO_Z_NF_COSWZ_POS)

#define GYRO_CONFIG_STATIC9_GYRO_Y_NF_COSWZ_POS         (0x1)
#define GYRO_CONFIG_STATIC9_GYRO_Y_NF_COSWZ_MASK        (0x1 << GYRO_CONFIG_STATIC9_GYRO_Y_NF_COSWZ_POS)

#define GYRO_CONFIG_STATIC9_GYRO_X_NF_COSWZ_POS         (0x0)
#define GYRO_CONFIG_STATIC9_GYRO_X_NF_COSWZ_MASK        (0x1)

#define GYRO_CONFIG_STATIC10_GYRO_NF_BW_SEL_POS         (0x4)
#define GYRO_CONFIG_STATIC10_GYRO_NF_BW_SEL_MASK        (0x7 << GYRO_CONFIG_STATIC10_GYRO_NF_BW_SEL_POS)

#define XG_ST_DATA_XG_ST_DATA_POS                   (0x0)
#define XG_ST_DATA_XG_ST_DATA_MASK                  (0xFF)

#define YG_ST_DATA_YG_ST_DATA_POS                   (0x0)
#define YG_ST_DATA_YG_ST_DATA_MASK                  (0xFF)

#define ZG_ST_DATA_ZG_ST_DATA_POS                   (0x0)
#define ZG_ST_DATA_ZG_ST_DATA_MASK                  (0xFF)

#define TMSTVAL0_TMST_VALUE_POS                     (0x0)
#define TMSTVAL0_TMST_VALUE_MASK                    (0xFF)

#define TMSTVAL1_TMST_VALUE_POS                     (0x0)
#define TMSTVAL1_TMST_VALUE_MASK                    (0xFF)

#define TMSTVAL2_TMST_VALUE_POS                     (0x0)
#define TMSTVAL2_TMST_VALUE_MASK                    (0xF)

#define INTF_CONFIG4_I3C_BUS_MODE_POS               (0x6)
#define INTF_CONFIG4_I3C_BUS_MODE_MASK              (0x1 << INTF_CONFIG4_I3C_BUS_MODE_POS)

#define INTF_CONFIG4_SPI_AP_4WIRE_POS               (0x1)
#define INTF_CONFIG4_SPI_AP_4WIRE_MASK              (0x1 << INTF_CONFIG4_SPI_AP_4WIRE_POS)

#define INTF_CONFIG5_PIN9_FUNCTION_POS              (0x1)
#define INTF_CONFIG5_PIN9_FUNCTION_MASK             (0x3 << INTF_CONFIG5_PIN9_FUNCTION_POS)

#define INTF_CONFIG6_ASYNCTIME0_DIS_POS             (0x7)
#define INTF_CONFIG6_ASYNCTIME0_DIS_MASK            (0x1 << INTF_CONFIG6_ASYNCTIME0_DIS_POS)

#define INTF_CONFIG6_I3C_EN_POS                     (0x4)
#define INTF_CONFIG6_I3C_EN_MASK                    (0x1 << INTF_CONFIG6_I3C_EN_POS)

#define INTF_CONFIG6_I3C_IBI_BYTE_EN_POS            (0x3)
#define INTF_CONFIG6_I3C_IBI_BYTE_EN_MASK           (0x1 << INTF_CONFIG6_I3C_IBI_BYTE_EN_POS)

#define INTF_CONFIG6_I3C_IBI_EN_POS                 (0x2)
#define INTF_CONFIG6_I3C_IBI_EN_MASK                (0x1 << INTF_CONFIG6_I3C_IBI_EN_POS)

#define INTF_CONFIG6_I3C_DDR_EN_POS                 (0x1)
#define INTF_CONFIG6_I3C_DDR_EN_MASK                (0x1 << INTF_CONFIG6_I3C_DDR_EN_POS)

#define INTF_CONFIG6_I3C_SDR_EN_POS                 (0x0)
#define INTF_CONFIG6_I3C_SDR_EN_MASK                (0x1)


/* ===========================================
 * Register BANK2
 * =========================================== */
#define ACCEL_CONFIG_STATIC2_ACCEL_AAF_DELT_POS     (0x1)
#define ACCEL_CONFIG_STATIC2_ACCEL_AAF_DELT_MASK    (0x3F << ACCEL_CONFIG_STATIC2_ACCEL_AAF_DELT_POS)

/**
 * ACCEL_AAF
 *  0: Enable Accel AAF (default)
 *  1: Disable Accel AAF
 */
#define ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_POS      (0x0)
#define ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_MASK     (0x1)
    #define ACCEL_AAF_ENABLE    (0x0)
    #define ACCEL_AAF_DISABLE   (0x1)

#define ACCEL_CONFIG_STATIC3_ACCEL_AAF_DELTSQR_POS  (0x0)
#define ACCEL_CONFIG_STATIC3_ACCEL_AAF_DELTSQR_MASK (0xFF)

#define ACCEL_CONFIG_STATIC4_ACCEL_AAF_BITSHIFT_POS     (0x4)
#define ACCEL_CONFIG_STATIC4_ACCEL_AAF_BITSHIFT_MASK    (0xF << ACCEL_CONFIG_STATIC4_ACCEL_AAF_BITSHIFT_POS)

#define ACCEL_CONFIG_STATIC4_ACCEL_AAF_DELTSQR_POS  (0x0)
#define ACCEL_CONFIG_STATIC4_ACCEL_AAF_DELTSQR_MASK (0xF)

#define XA_ST_DATA_XA_ST_DATA_POS                   (0x0)
#define XA_ST_DATA_XA_ST_DATA_MASK                  (0xFF)

#define YA_ST_DATA_YA_ST_DATA_POS                   (0x0)
#define YA_ST_DATA_YA_ST_DATA_MASK                  (0xFF)

#define ZA_ST_DATA_ZA_ST_DATA_POS                   (0x0)
#define ZA_ST_DATA_ZA_ST_DATA_MASK                  (0xFF)

/* ===========================================
 * Register BANK3
 * =========================================== */
#define CLKDIV_CLKDIV_POS                           (0x0)
#define CLKDIV_CLKDIV_MASK                          (0xFF)


/* ===========================================
 * Register BANK4
 * =========================================== */
#define APEX_CONFIG1_LOW_ENERGY_AMP_TH_SEL_POS      (0x4)   
#define APEX_CONFIG1_LOW_ENERGY_AMP_TH_SEL_MASK     (0xF << APEX_CONFIG1_LOW_ENERGY_AMP_TH_SEL_POS)

#define APEX_CONFIG1_DMP_POWER_SAE_TIME_SEL_POS     (0x0)
#define APEX_CONFIG1_DMP_POWER_SAE_TIME_SEL_MASK    (0xF)

#define APEX_CONFIG2_PED_AMP_TH_SEL_POS             (0x4)
#define APEX_CONFIG2_PED_AMP_TH_SEL_MASK            (0xF << APEX_CONFIG2_PED_AMP_TH_SEL_POS)

#define APEX_CONFIG2_PED_STEP_CNT_TH_SEL_POS        (0x0)
#define APEX_CONFIG2_PED_STEP_CNT_TH_SEL_MASK       (0xF)

#define APEX_CONFIG3_PED_STEP_DET_TH_SEL_POS        (0x5)
#define APEX_CONFIG3_PED_STEP_DET_TH_SEL_MASK       (0x7 << APEX_CONFIG3_PED_STEP_DET_TH_SEL_POS)

#define APEX_CONFIG3_PED_SB_TIMER_TH_SEL_POS        (0x2)
#define APEX_CONFIG3_PED_SB_TIMER_TH_SEL_MASK       (0x7 << APEX_CONFIG3_PED_SB_TIMER_TH_SEL_POS)

#define APEX_CONFIG3_PED_HI_EN_TH_SEL_POS           (0x0)
#define APEX_CONFIG3_PED_HI_EN_TH_SEL_MASK          (0x3)

#define APEX_CONFIG4_TILT_WAIT_TIME_SEL_POS         (0x6)
#define APEX_CONFIG4_TILT_WAIT_TIME_SEL_MASK        (0x3 << APEX_CONFIG4_TILT_WAIT_TIME_SEL_POS)

#define APEX_CONFIG4_SLEEP_TIME_OUT_POS             (0x3)
#define APEX_CONFIG4_SLEEP_TIME_OUT_MASK            (0x7 << APEX_CONFIG4_SLEEP_TIME_OUT_POS)

#define APEX_CONFIG5_MOUNTING_MATRIX_POS            (0x0)
#define APEX_CONFIG5_MOUNTING_MATRIX_MASK           (0x7)

#define APEX_CONFIG6_SLEEP_GESTURE_DELAY_POS        (0x0)
#define APEX_CONFIG6_SLEEP_GESTURE_DELAY_MASK       (0x7)

#define APEX_CONFIG7_TAP_MIN_JERK_THR_POS           (0x2)
#define APEX_CONFIG7_TAP_MIN_JERK_THR_MASK          (0x3F << APEX_CONFIG7_TAP_MIN_JERK_THR_POS)

#define APEX_CONFIG7_TAP_MAX_PEAK_TOL_POS           (0x0)
#define APEX_CONFIG7_TAP_MAX_PEAK_TOL_MASK          (0x3)

#define APEX_CONFIG8_TAP_TMAX_POS                   (0x5)
#define APEX_CONFIG8_TAP_TMAX_MASK                  (0x3 << APEX_CONFIG8_TAP_TMAX_POS)

#define APEX_CONFIG8_TAP_TAVG_POS                   (0x3)
#define APEX_CONFIG8_TAP_TAVG_MASK                  (0x3 << APEX_CONFIG8_TAP_TAVG_POS)

#define APEX_CONFIG8_TAP_TMIN_POS                   (0x0)
#define APEX_CONFIG8_TAP_TMIN_MASK                  (0x7)

#define APEX_CONFIG9_SENSITIVITY_MODE_POS           (0x0)
#define APEX_CONFIG9_SENSITIVITY_MODE_MASK          (0x1)

#define ACCEL_WOM_X_THR_WOM_X_TH_POS                (0x0)
#define ACCEL_WOM_X_THR_WOM_X_TH_MASK               (0xFF)

#define ACCEL_WOM_Y_THR_WOM_Y_TH_POS                (0x0)
#define ACCEL_WOM_Y_THR_WOM_Y_TH_MASK               (0xFF)

#define ACCEL_WOM_Z_THR_WOM_Z_TH_POS                (0x0)
#define ACCEL_WOM_Z_THR_WOM_Z_TH_MASK               (0xFF)

#define INT_SOURCE6_STEP_DET_INT1_EN_POS            (0x5)
#define INT_SOURCE6_STEP_DET_INT1_EN_MASK           (0x1 << INT_SOURCE6_STEP_DET_INT1_EN_POS)

#define INT_SOURCE6_STEP_CNT_OFL_INT1_EN_POS        (0x4)
#define INT_SOURCE6_STEP_CNT_OFL_INT1_EN_MASK       (0x1 << INT_SOURCE6_STEP_CNT_OFL_INT1_EN_POS)

#define INT_SOURCE6_TILT_DET_INT1_EN_POS            (0x3)
#define INT_SOURCE6_TILT_DET_INT1_EN_MASK           (0x1 << INT_SOURCE6_TILT_DET_INT1_EN_POS)

#define INT_SOURCE6_WAKE_DET_INT1_EN_POS            (0x2)
#define INT_SOURCE6_WAKE_DET_INT1_EN_MASK           (0x1 << INT_SOURCE6_WAKE_DET_INT1_EN_POS)

#define INT_SOURCE6_SLEEP_DET_INT1_EN_POS           (0x1)
#define INT_SOURCE6_SLEEP_DET_INT1_EN_MASK          (0x1 << INT_SOURCE6_SLEEP_DET_INT1_EN_POS)

#define INT_SOURCE7_TAP_DET_INT2_EN_POS             (0x0)
#define INT_SOURCE7_TAP_DET_INT2_EN_MASK            (0x1)

#define INT_SOURCE7_STEP_DET_INT2_EN_POS            (0x5)
#define INT_SOURCE7_STEP_DET_INT2_EN_MASK           (0x1 << INT_SOURCE7_STEP_DET_INT2_EN_POS)

#define INT_SOURCE7_STEP_CNT_OFL_INT2_EN_POS        (0x4)
#define INT_SOURCE7_STEP_CNT_OFL_INT2_EN_MASK       (0x1 << INT_SOURCE7_STEP_CNT_OFL_INT2_EN_POS)

#define INT_SOURCE7_TILT_DET_INT2_EN_POS            (0x3)
#define INT_SOURCE7_TILT_DET_INT2_EN_MASK           (0x1 << INT_SOURCE7_TILT_DET_INT2_EN_POS)

#define INT_SOURCE7_WAKE_DET_INT2_EN_POS            (0x2)
#define INT_SOURCE7_WAKE_DET_INT2_EN_MASK           (0x1 << INT_SOURCE7_WAKE_DET_INT2_EN_POS)

#define INT_SOURCE7_SLEEP_DET_INT2_EN_POS           (0x1)
#define INT_SOURCE7_SLEEP_DET_INT2_EN_MASK          (0x1 << INT_SOURCE7_SLEEP_DET_INT2_EN_POS)

#define INT_SOURCE7_TAP_DET_INT2_EN_POS             (0x0)
#define INT_SOURCE7_TAP_DET_INT2_EN_MASK            (0x1)

#define INT_SOURCE8_FSYNC_IBI_EN_POS                (0x5)
#define INT_SOURCE8_FSYNC_IBI_EN_MASK               (0x1 << INT_SOURCE8_FSYNC_IBI_EN_POS)

#define INT_SOURCE8_PLL_RDY_IBI_EN_POS              (0x4)
#define INT_SOURCE8_PLL_RDY_IBI_EN_MASK             (0x1 << INT_SOURCE8_PLL_RDY_IBI_EN_POS)

#define INT_SOURCE8_UI_DRDY_IBI_EN_POS              (0x3)
#define INT_SOURCE8_UI_DRDY_IBI_EN_MASK             (0x1 << INT_SOURCE8_UI_DRDY_IBI_EN_POS)

#define INT_SOURCE8_FIFO_THS_IBI_EN_POS             (0x2)
#define INT_SOURCE8_FIFO_THS_IBI_EN_MASK            (0x1 << INT_SOURCE8_FIFO_THS_IBI_EN_POS)

#define INT_SOURCE8_FIFO_FULL_IBI_EN_POS            (0x1)
#define INT_SOURCE8_FIFO_FULL_IBI_EN_MASK           (0x1 << INT_SOURCE8_FIFO_FULL_IBI_EN_POS)

#define INT_SOURCE8_AGC_RDY_IBI_EN_POS              (0x0)
#define INT_SOURCE8_AGC_RDY_IBI_EN_MASK             (0x1)

#define INT_SOURCE9_I3C_PROTOCOL_ERROR_IBI_EN_POS   (0x7)
#define INT_SOURCE9_I3C_PROTOCOL_ERROR_IBI_EN_MASK  (0x1 << INT_SOURCE9_I3C_PROTOCOL_ERROR_IBI_EN_POS)

#define INT_SOURCE9_SMD_IBI_EN_POS                  (0x4)
#define INT_SOURCE9_SMD_IBI_EN_MASK                 (0x1 << INT_SOURCE9_SMD_IBI_EN_POS)

#define INT_SOURCE9_WOM_Z_IBI_EN_POS                (0x3)
#define INT_SOURCE9_WOM_Z_IBI_EN_MASK               (0x1 << INT_SOURCE9_WOM_Z_IBI_EN_POS)

#define INT_SOURCE9_WOM_Y_IBI_EN_POS                (0x2)
#define INT_SOURCE9_WOM_Y_IBI_EN_MASK               (0x1 << INT_SOURCE9_WOM_Y_IBI_EN_POS)

#define INT_SOURCE9_WOM_X_IBI_EN_POS                (0x1)
#define INT_SOURCE9_WOM_X_IBI_EN_MASK               (0x1 << INT_SOURCE9_WOM_X_IBI_EN_POS)

#define INT_SOURCE10_STEP_DET_IBI_EN_POS            (0x5)
#define INT_SOURCE10_STEP_DET_IBI_EN_MASK           (0x1 << INT_SOURCE10_STEP_DET_IBI_EN_POS)

#define INT_SOURCE10_STEP_CNT_OFL_IBI_EN_POS        (0x4)
#define INT_SOURCE10_STEP_CNT_OFL_IBI_EN_MASK       (0x1 << INT_SOURCE10_STEP_CNT_OFL_IBI_EN_POS)

#define INT_SOURCE10_TILT_DET_IBI_EN_POS            (0x3)
#define INT_SOURCE10_TILT_DET_IBI_EN_MASK           (0x1 << INT_SOURCE10_TILT_DET_IBI_EN_POS)

#define INT_SOURCE10_WAKE_DET_IBI_EN_POS            (0x2)
#define INT_SOURCE10_WAKE_DET_IBI_EN_MASK           (0x1 << INT_SOURCE10_WAKE_DET_IBI_EN_POS)

#define INT_SOURCE10_SLEEP_DET_IBI_EN_POS           (0x1)
#define INT_SOURCE10_SLEEP_DET_IBI_EN_MASK          (0x1 << INT_SOURCE10_SLEEP_DET_IBI_EN_POS)

#define INT_SOURCE10_TAP_DET_IBI_EN_POS             (0x0)
#define INT_SOURCE10_TAP_DET_IBI_EN_MASK            (0x1)

#define OFFSET_USER0_GYRO_X_OFFUSER_POS             (0x0)
#define OFFSET_USER0_GYRO_X_OFFUSER_MASK            (0xFF)

#define OFFSET_USER1_GYRO_Y_OFFUSER_POS             (0x4)
#define OFFSET_USER1_GYRO_Y_OFFUSER_MASK            (0xF << OFFSET_USER1_GYRO_Y_OFFUSER_POS)

#define OFFSET_USER1_GYRO_X_OFFUSER_POS             (0x0)
#define OFFSET_USER1_GYRO_X_OFFUSER_MASK            (0xF)

#define OFFSET_USER2_GYRO_Y_OFFUSER_POS             (0x0)
#define OFFSET_USER2_GYRO_Y_OFFUSER_MASK            (0xFF)

#define OFFSET_USER3_GYRO_Z_OFFUSER_POS             (0x0)
#define OFFSET_USER3_GYRO_Z_OFFUSER_MASK            (0xFF)

#define OFFSET_USER4_ACCEL_X_OFFUSER_POS            (0x4)
#define OFFSET_USER4_ACCEL_X_OFFUSER_MASK           (0xF << OFFSET_USER4_ACCEL_X_OFFUSER_POS)

#define OFFSET_USER4_GYRO_Z_OFFUSER_POS             (0x0)
#define OFFSET_USER4_GYRO_Z_OFFUSER_MASK            (0xF)

#define OFFSET_USER5_ACCEL_X_OFFUSER_POS            (0x0)
#define OFFSET_USER5_ACCEL_X_OFFUSER_MASK           (0xF)

#define OFFSET_USER6_ACCEL_Y_OFFUSER_POS            (0x0)
#define OFFSET_USER6_ACCEL_Y_OFFUSER_MASK           (0xFF)

#define OFFSET_USER7_ACCEL_Z_OFFUSER_POS            (0x4)
#define OFFSET_USER7_ACCEL_Z_OFFUSER_MASK           (0xF << OFFSET_USER7_ACCEL_Z_OFFUSER_POS)

#define OFFSET_USER7_ACCEL_Y_OFFUSER_POS            (0x0)
#define OFFSET_USER7_ACCEL_Y_OFFUSER_MASK           (0xF)

#define OFFSET_USER8_ACCEL_Z_OFFUSER_POS            (0x0)
#define OFFSET_USER8_ACCEL_Z_OFFUSER_MASK           (0xFF)

#endif // INC_ICM42688P_REGS_H