#include "icm_driver.h"
#include <math.h>
#include <string.h>

uint8_t _buffer[20] = {0};

const float _gyroSSFArray[8] = {
    0.060976,   // 2000 DPS
    0.030488,   // 1000 DPS
    0.015267,   // 500 DPS
    0.007634,   // 250 DPS
    0.003817,   // 125 DPS
    0.001907,   // 62.5 DPS
    0.000954,   // 31.25 DPS
    0.000477    // 15.625 DPS
};

const float _accelSSFArray[5] = {
    0.000976,   // 32g  (for ICM40609D)
    0.000488,   // 16g
    0.000244,   // 8g
    0.000122,   // 4g
    0.000061    // 2g   (for ICM46288P)
};

static ICM_Status_t setBank(ICM_Config_t* icm, uint8_t bank);
static ICM_Status_t writeReg(ICM_Config_t* icm, uint8_t addr, uint8_t data);
static ICM_Status_t readRegs(ICM_Config_t* icm, uint8_t addr, uint8_t len, uint8_t* data);
static uint8_t readWriteSPI(ICM_Config_t* icm, uint8_t txData);
static inline float convertRawToAccel(float ssf, int16_t raw, float bias, float scale);
static inline float convertRawToGyro(float ssf, int16_t raw, float bias);
static inline float convertRawToTemp(int16_t raw);

ICM_Status_t ICM_init(ICM_Config_t* icm)
{   
    switch (icm->id)
    {
        case ICM40609D_ID:
        {
            icm->gyroCfg.fsr = ICM40609D_GYRO_FSR_DEF;
            icm->gyroCfg.odr = ICM40609D_GYRO_ODR_DEF;
            icm->gyroCfg.ssf = _gyroSSFArray[ICM40609D_GYRO_SSF_DEF];

            icm->accCfg.fsr = ICM40609D_ACC_FSR_DEF;
            icm->accCfg.odr = ICM40609D_ACC_ODR_DEF;
            icm->accCfg.ssf = _accelSSFArray[ICM40609D_ACC_SSF_DEF];
            break;
        }
        
        case ICM42688P_ID:
        {
            icm->gyroCfg.fsr = ICM42688P_GYRO_FSR_DEF;
            icm->gyroCfg.odr = ICM42688P_GYRO_ODR_DEF;
            icm->gyroCfg.ssf = _gyroSSFArray[ICM42688P_GYRO_SSF_DEF];

            icm->accCfg.fsr = ICM42688P_ACC_FSR_DEF;
            icm->accCfg.odr = ICM42688P_ACC_ODR_DEF;
            icm->accCfg.ssf = _accelSSFArray[ICM42688P_ACC_SSF_DEF]; 
            break;   
        }
        default:
        {
            $ERROR("Wrong Sensor ID parameter. Failed to init.");
            return ICM_ERROR;
        }
    }
    
    icm->accCfg.scale[0] = 1.0f;
    icm->accCfg.scale[1] = 1.0f;
    icm->accCfg.scale[2] = 1.0f;

    LL_GPIO_SetOutputPin(icm->hal.csPort, icm->hal.csPin);
    ICM_reset(icm);

    uint8_t whoami = ICM_whoami(icm);
    if (whoami != icm->id)
    {
        $ERROR("Wrong WHOAMI. Exp %d, Recv %d.", icm->id, whoami);
        return ICM_ERROR;
    }

    // Enable Temp, Accel (LN), and Gyro (LN)
    if (ICM_enAll(icm) != ICM_OK)
    {
        $ERROR("Unable to init. Failed to init.");
        return ICM_ERROR;
    }

    HAL_Delay(100);

    $SUCCESS("Init successful");

    return ICM_OK;
}

ICM_Status_t ICM_reset(ICM_Config_t* icm)
{
    setBank(icm, 0);

    writeReg(icm, DEVICE_CONFIG, 0x01);
    HAL_Delay(100);

    return ICM_OK;
}

uint8_t ICM_whoami(ICM_Config_t* icm)
{
    setBank(icm, 0);

    if (readRegs(icm, WHO_AM_I, 1, _buffer) != ICM_OK)
    {
        return 0x00;
    }

    return _buffer[0];
}

ICM_Status_t ICM_enAll(ICM_Config_t* icm)
{
    if (ICM_enTemp(icm, true) != ICM_OK)
    {
        return ICM_ERROR;
    }

    if (ICM_enGyro(icm, PWR_LN) != ICM_OK)
    {
        return ICM_ERROR;
    }

    if (ICM_enAccel(icm, PWR_LN) != ICM_OK)
    {
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM_enTemp(ICM_Config_t* icm, bool en)
{
    setBank(icm, 0);
    uint8_t reg;

    if (readRegs(icm, PWR_MGMT0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read PWR_MGMT0 error in enTemp.");
        return ICM_ERROR;
    }

    if (en)
    {
        reg = (reg & (~PWR_MGMT0_TEMP_DIS_MASK)) | TEMP_ENABLE;
    }
    else 
    {
        reg = (reg & (~PWR_MGMT0_TEMP_DIS_MASK)) | TEMP_DISABLE;
    }

    if (writeReg(icm, PWR_MGMT0, reg) != ICM_OK)
    {
        $ERROR("Write PWR_MGMT0 reg error in enTemp.");
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM_enIdle(ICM_Config_t* icm, bool en)
{
    setBank(icm, 0);
    uint8_t reg;

    if (readRegs(icm, PWR_MGMT0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read PWR_MGMT0 error in enIdle.");
        return ICM_ERROR;
    }

    if (en)
    {
        reg = (reg & (~PWR_MGMT0_IDLE_MASK_MASK)) | IDLE_ENABLE;
    }
    else 
    {
        reg = (reg & (~PWR_MGMT0_IDLE_MASK_MASK)) | IDLE_DISABLE;
    }

    if (writeReg(icm, PWR_MGMT0, reg) != ICM_OK)
    {
        $ERROR("Write PWR_MGMT0 reg error in enIdle.");
        return ICM_ERROR;
    }

    return ICM_OK;
}


ICM_Status_t ICM_enGyro(ICM_Config_t* icm, ICM_PWR_t mode)
{
    setBank(icm, 0);

    uint8_t reg;

    if (readRegs(icm, PWR_MGMT0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read PWR_MGMT0 error in enGyro.");
        return ICM_ERROR;
    }

    switch (mode)
    {
        case (PWR_OFF):
        {
            reg = (reg & (~PWR_MGMT0_GYRO_MODE_MASK)) | GYRO_MODE_OFF;
            break;
        }
        case (PWR_STANDBY):
        {
            reg = (reg & (~PWR_MGMT0_GYRO_MODE_MASK)) | GYRO_MODE_STANDBY;
            break;
        }
        case (PWR_LN):
        {
            reg = (reg & (~PWR_MGMT0_GYRO_MODE_MASK)) | GYRO_MODE_LN;
            break;
        }
        default:
            break;
    }

    if (writeReg(icm, PWR_MGMT0, reg) != ICM_OK)
    {
        $ERROR("Write PWR_MGMT0 reg error in enGyro.");
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM_enAccel(ICM_Config_t* icm, ICM_PWR_t mode)
{
    setBank(icm, 0);
    uint8_t reg;

    if (readRegs(icm, PWR_MGMT0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read PWR_MGMT0 error in enAccel.");
        return ICM_ERROR;
    }

    switch (mode)
    {
        case (PWR_OFF):
        {
            reg = (reg & (~PWR_MGMT0_ACCEL_MODE_MASK)) | ACCEL_MODE_OFF;
            break;
        }
        case (PWR_LP):
        {
            reg = (reg & (~PWR_MGMT0_ACCEL_MODE_MASK)) | ACCEL_MODE_LP;
            break;
        }
        case (PWR_LN):
        {
            reg = (reg & (~PWR_MGMT0_ACCEL_MODE_MASK)) | ACCEL_MODE_LN;
            break;
        }
        default:
            break;
    }

    if (writeReg(icm, PWR_MGMT0, reg) != ICM_OK)
    {
        $ERROR("Write PWR_MGMT0 reg error in enAccel.");
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM_setGyroFSR(ICM_Config_t* icm, uint8_t gyroFsr)
{
    if (gyroFsr ==  icm->gyroCfg.fsr)
    {
        return ICM_OK;
    }

    setBank(icm, 0);

    uint8_t reg;
    // Read current reg
    if(readRegs(icm, GYRO_CONFIG0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read GYRO_CONFIG0 reg error in setGyroFSR.");
        return ICM_ERROR;
    }
    
    // Write only the gyro FS bits
    reg = (reg & (~GYRO_CONFIG0_GYRO_FS_SEL_MASK)) | (uint8_t)(gyroFsr << GYRO_CONFIG0_GYRO_FS_SEL_POS);

    if (writeReg(icm, GYRO_CONFIG0, reg) != ICM_OK)
    {
        $ERROR("Write GYRO_CONFIG0 reg error in setGyroFSR.");
        return ICM_ERROR;
    }

    // Store Full Scale Range
    icm->gyroCfg.fsr = gyroFsr;
    // Store current dps per 1 bit change
    icm->gyroCfg.ssf = _gyroSSFArray[gyroFsr];

    return ICM_OK;
}

ICM_Status_t ICM_setAccelFSR(ICM_Config_t* icm, uint8_t accelFsr)
{
    if (accelFsr == icm->accCfg.fsr)
    {
        return ICM_OK;
    }

    setBank(icm, 0);

    uint8_t reg;
    // Read current reg
    if(readRegs(icm, ACCEL_CONFIG0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read ACCEL_CONFIG0 reg error in setAccelFSR.");
        return ICM_ERROR;
    }
    
    // Write only the accel FS bits
    reg = (reg & (~ACCEL_CONFIG0_ACCEL_FS_SEL_MASK)) | (uint8_t)(accelFsr << ACCEL_CONFIG0_ACCEL_FS_SEL_POS);

    if (writeReg(icm, ACCEL_CONFIG0, reg) != ICM_OK)
    {
        $ERROR("Write ACCEL_CONFIG0 reg error in setAccelFSR.");
        return ICM_ERROR;
    }

    // Store Full Scale Range
    icm->accCfg.fsr = accelFsr;
    // Store current g per 1 bit change
    icm->accCfg.ssf = (icm->id == ICM42688P_ID) ? _accelSSFArray[accelFsr+1] : _accelSSFArray[accelFsr];

    return ICM_OK;
}

ICM_Status_t ICM_setGyroODR(ICM_Config_t* icm, uint8_t gyroOdr)
{
    if (icm->gyroCfg.odr == gyroOdr)
    {
        return ICM_OK;
    }

    setBank(icm, 0);

    // read current reg 
    uint8_t reg;
    if (readRegs(icm, GYRO_CONFIG0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read GYRO_CONFIG0 reg error in setGyroODR.");
        return ICM_ERROR;
    }

    // write only ODR bits
    reg = (reg & (~GYRO_CONFIG0_GYRO_ODR_MASK)) | (uint8_t)(gyroOdr);

    if (writeReg(icm, GYRO_CONFIG0, reg) != ICM_OK)
    {
        $ERROR("Write GYRO_CONFIG0 reg error in setGyroODR.");
        return ICM_ERROR;
    }

    icm->gyroCfg.odr = gyroOdr;

    return ICM_OK;
}

ICM_Status_t ICM_setAccelODR(ICM_Config_t* icm, uint8_t accelOdr)
{
    if (accelOdr == icm->accCfg.odr)
    {
        return ICM_OK;
    }

    setBank(icm, 0);

    // read current reg 
    uint8_t reg;
    if (readRegs(icm, ACCEL_CONFIG0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read ACCEL_CONFIG0 reg error in setAccelODR.");
        return ICM_ERROR;
    }

    // write only ODR bits
    reg = (reg & (~ACCEL_CONFIG0_ACCEL_ODR_MASK)) | (uint8_t)(accelOdr);

    if (writeReg(icm, ACCEL_CONFIG0, reg) != ICM_OK)
    {
        $ERROR("Write ACCEL_CONFIG0 reg error in setAccelODR.");
        return ICM_ERROR;
    }

    icm->accCfg.odr = accelOdr;

    return ICM_OK;
}

ICM_Status_t ICM_setFilters(ICM_Config_t* icm, bool gyroFils, bool accelFils)
{
    setBank(icm, 1);

    /* Set GYRO AAF and NF */
    if (gyroFils)
    {   
        // Direct write possible since no other fields in the registers
        if (writeReg(icm, GYRO_CONFIG_STATIC2, GYRO_AAF_ENABLE | GYRO_NF_ENABLE) != ICM_OK)
        {
            $ERROR("Write GYRO_CONFIG_STATIC2 reg error in setFilters");
            return ICM_ERROR;
        }
    }
    else 
    {
        // Direct write possible since no other fields in the registers
        if (writeReg(icm, GYRO_CONFIG_STATIC2, GYRO_AAF_DISABLE | GYRO_NF_DISABLE) != ICM_OK)
        {
            $ERROR("Write GYRO_CONFIG_STATIC2 reg error in setFilters");
            return ICM_ERROR;
        }
    }

    uint8_t reg;
    // Read the registers first to persist existing data
    if (readRegs(icm, ACCEL_CONFIG_STATIC2, 1, &reg) != ICM_OK)
    {
        $ERROR("Read ACCEL_CONFIG_STATIC2 reg error in setFilters");
        return ICM_ERROR;
    }

    if (accelFils)
    {
        // Change only the DIS fields
        reg = (reg & (~ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_MASK)) | (ACCEL_AAF_ENABLE);
        if (writeReg(icm, ACCEL_CONFIG_STATIC2, reg) != ICM_OK)
        {
            $ERROR("Write ACCEL_CONFIG_STATIC2 reg error in setFilters");
            return ICM_ERROR;
        }
    }
    else
    {
        // Change only the DIS fields
        reg = (reg & (~ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_MASK)) | (ACCEL_AAF_DISABLE);
        if (writeReg(icm, ACCEL_CONFIG_STATIC2, reg) != ICM_OK)
        {
            $ERROR("Write ACCEL_CONFIG_STATIC2 reg error in setFilters");
            return ICM_ERROR;
        }
    }

    return ICM_OK;
}

ICM_Status_t ICM_calibGyro(ICM_Config_t* icm)
{
    const uint8_t currFSR = icm->gyroCfg.fsr;

    // Set to 250 DPS
    if (ICM_setGyroFSR(icm, 3) != ICM_OK)
    {
        $ERROR("Unable to calib gyro.");
        return ICM_ERROR;
    }
    
    icm->gyroCfg.biasData[0] = 0;
    icm->gyroCfg.biasData[1] = 0;
    icm->gyroCfg.biasData[2] = 0;

    for (uint32_t i = 0; i < CALIBRATION_SAMPLE_SIZE; i++)
    {
        ICM_updateGyroData(icm);
        icm->gyroCfg.biasData[0] += (ICM_getGyroX(icm) + icm->gyroCfg.bias[0]) / CALIBRATION_SAMPLE_SIZE;
        icm->gyroCfg.biasData[1] += (ICM_getGyroY(icm) + icm->gyroCfg.bias[1]) / CALIBRATION_SAMPLE_SIZE;
        icm->gyroCfg.biasData[2] += (ICM_getGyroZ(icm) + icm->gyroCfg.bias[2]) / CALIBRATION_SAMPLE_SIZE;
        HAL_Delay(1);
    }

    icm->gyroCfg.bias[0] = icm->gyroCfg.biasData[0];
    icm->gyroCfg.bias[1] = icm->gyroCfg.biasData[1];
    icm->gyroCfg.bias[2] = icm->gyroCfg.biasData[2];

    if (ICM_setGyroFSR(icm, currFSR) != ICM_OK)
    {
        return ICM_ERROR;
    }

    $SUCCESS("Calibration of Gyro OK");
    return ICM_OK;
}

ICM_Status_t ICM_calibAccel(ICM_Config_t* icm)
{
    const uint8_t currFSR = icm->accCfg.fsr;

    // Set to 2g FSR for ICM42688P, 4g FSR for ICM40609D
    if (ICM_setAccelFSR(icm, 3) != ICM_OK)
    {
        $ERROR("Unable to calibrate accel");
        return ICM_ERROR;
    }

    icm->accCfg.biasData[0] = 0;
    icm->accCfg.biasData[1] = 0;
    icm->accCfg.biasData[2] = 0;

    for (uint32_t i = 0; i < CALIBRATION_SAMPLE_SIZE; i++)
    {
        ICM_updateAccelData(icm);
        icm->accCfg.biasData[0] += (ICM_getAccX(icm) / icm->accCfg.scale[0] + icm->accCfg.bias[0]) / CALIBRATION_SAMPLE_SIZE;
        icm->accCfg.biasData[1] += (ICM_getAccY(icm) / icm->accCfg.scale[1] + icm->accCfg.bias[1]) / CALIBRATION_SAMPLE_SIZE;
        icm->accCfg.biasData[2] += (ICM_getAccZ(icm) / icm->accCfg.scale[2] + icm->accCfg.bias[2]) / CALIBRATION_SAMPLE_SIZE;
        HAL_Delay(1);
    }

    if (icm->accCfg.biasData[0] > 0.9f) icm->accCfg.axisMax[0] = icm->accCfg.biasData[0];
    if (icm->accCfg.biasData[1] > 0.9f) icm->accCfg.axisMax[1] = icm->accCfg.biasData[1];
    if (icm->accCfg.biasData[2] > 0.9f) icm->accCfg.axisMax[2] = icm->accCfg.biasData[2];
    if (icm->accCfg.biasData[0] < -0.9f) icm->accCfg.axisMin[0] = icm->accCfg.biasData[0];
    if (icm->accCfg.biasData[1] < -0.9f) icm->accCfg.axisMin[1] = icm->accCfg.biasData[1];
    if (icm->accCfg.biasData[2] < -0.9f) icm->accCfg.axisMin[2] = icm->accCfg.biasData[2];

    if ((fabs(icm->accCfg.axisMin[0]) > 0.9f) && (fabs(icm->accCfg.axisMax[0]) > 0.9f))
    {
        icm->accCfg.bias[0] = (icm->accCfg.axisMin[0] + icm->accCfg.axisMax[0]) / 2.0f;
        icm->accCfg.scale[0] = 1 / ((fabs(icm->accCfg.axisMin[0]) + fabs(icm->accCfg.axisMax[0])) / 2.0f);
    }
    
    if ((fabs(icm->accCfg.axisMin[1]) > 0.9f) && (fabs(icm->accCfg.axisMax[1]) > 0.9f))
    {
        icm->accCfg.bias[1] = (icm->accCfg.axisMin[1] + icm->accCfg.axisMax[1]) / 2.0f;
        icm->accCfg.scale[1] = 1 / ((fabs(icm->accCfg.axisMin[1]) + fabs(icm->accCfg.axisMax[1])) / 2.0f);
    }
    
    if ((fabs(icm->accCfg.axisMin[2]) > 0.9f) && (fabs(icm->accCfg.axisMax[2]) > 0.9f))
    {
        icm->accCfg.bias[2] = (icm->accCfg.axisMin[2] + icm->accCfg.axisMax[2]) / 2.0f;
        icm->accCfg.scale[2] = 1 / ((fabs(icm->accCfg.axisMin[2]) + fabs(icm->accCfg.axisMax[2])) / 2.0f);
    }

    if (ICM_setAccelFSR(icm, currFSR) != ICM_OK)
    {
        return ICM_ERROR;
    }

    $SUCCESS("Calibration of Accel OK");
    return ICM_OK;
}

float ICM_getGyroBiasX(ICM_Config_t* icm)
{
    return icm->gyroCfg.bias[0];
}

float ICM_getGyroBiasY(ICM_Config_t* icm)
{
    return icm->gyroCfg.bias[1];
}

float ICM_getGyroBiasZ(ICM_Config_t* icm)
{
    return icm->gyroCfg.bias[2];
}

void ICM_setGyroBiasX(ICM_Config_t* icm, float bias)
{
    icm->gyroCfg.bias[0] = bias;
}

void ICM_setGyroBiasY(ICM_Config_t* icm, float bias)
{
    icm->gyroCfg.bias[1] = bias;
}

void ICM_setGyroBiasZ(ICM_Config_t* icm, float bias)
{
    icm->gyroCfg.bias[2] = bias;
}

float ICM_getAccelBiasX(ICM_Config_t* icm)
{
    return icm->accCfg.bias[0];
}

float ICM_getAccelScaleX(ICM_Config_t* icm)
{
    return icm->accCfg.scale[0];
}

float ICM_getAccelBiasY(ICM_Config_t* icm)
{
    return icm->accCfg.bias[1];
}

float ICM_getAccelScaleY(ICM_Config_t* icm)
{
    return icm->accCfg.scale[1];
}

float ICM_getAccelBiasZ(ICM_Config_t* icm)
{
    return icm->accCfg.bias[2];
}

float ICM_getAccelScaleZ(ICM_Config_t* icm)
{
    return icm->accCfg.scale[2];
}

void ICM_setAccelCalX(ICM_Config_t* icm, float bias, float scale)
{
    icm->accCfg.bias[0] = bias;
    icm->accCfg.scale[0] = scale;
}

void ICM_setAccelCalY(ICM_Config_t* icm, float bias, float scale)
{
    icm->accCfg.bias[1] = bias;
    icm->accCfg.scale[1] = scale;
}

void ICM_setAccelCalZ(ICM_Config_t* icm, float bias, float scale)
{
    icm->accCfg.bias[2] = bias;
    icm->accCfg.scale[2] = scale;
}


ICM_Status_t ICM_enableDataRdyInt(ICM_Config_t* icm)
{
    // Set ACTIVE HIGH, Push Pull and Pulsed Mode Interrupt on INT1 and INT2
    if (writeReg(icm, INT_CONFIG, 0x18 | 0x03) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG error in enDataRdyInt");
        return ICM_ERROR;
    }

    uint8_t reg;
    if (readRegs(icm, INT_CONFIG1, 1, &reg) != ICM_OK)
    {
        $ERROR("Read INT_CONFIG1 error in enDataRdyInt");
        return ICM_ERROR;
    }
    // Clear bit 4 for proper INT1 and INT2 operation
    reg = reg & ~(0x10);
    if (writeReg(icm, INT_CONFIG1, reg) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG1 error in enDataRdyInt");
        return ICM_ERROR;
    }

    // Route UI_DRDY_INT (and reset) to INT1
    if (writeReg(icm, INT_SOURCE0, 0x18) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG1 error in enDataRdyInt");
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM_disableDataRdyInt(ICM_Config_t* icm)
{
    uint8_t reg;
    if (readRegs(icm, INT_CONFIG1, 1, &reg) != ICM_OK)
    {
        $ERROR("Read INT_CONFIG1 error in disableDataRdyInt");
        return ICM_ERROR;
    }
    // Set bit 4 to default value
    reg = reg | 0x10; 
    if (writeReg(icm, INT_CONFIG1, reg) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG1 error in disableDataRdyInt");
        return ICM_ERROR;
    }

    // Unroute UI_DRDY_INT 
    if (writeReg(icm, INT_SOURCE0, 0x10) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG1 error in enDataRdyInt");
        return ICM_ERROR;
    }

    return ICM_OK;
}


ICM_Status_t ICM_updateAllData(ICM_Config_t* icm)
{
    // burst read all the sensor data
    if (readRegs(icm, TEMP_DATA1, 14, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in updateAllData.");
        return ICM_ERROR;
    }

    icm->currRD.rawTemp = ((int16_t) _buffer[0] << 8) | _buffer[1];
    icm->currRD.rawAccX = ((int16_t) _buffer[2] << 8) | _buffer[3];
    icm->currRD.rawAccY = ((int16_t) _buffer[4] << 8) | _buffer[5];
    icm->currRD.rawAccZ = ((int16_t) _buffer[6] << 8) | _buffer[7];
    icm->currRD.rawGyroX = ((int16_t) _buffer[8] << 8) | _buffer[9];
    icm->currRD.rawGyroY = ((int16_t) _buffer[10] << 8) | _buffer[11];
    icm->currRD.rawGyroZ = ((int16_t) _buffer[12] << 8) | _buffer[13];


    icm->currD.temp = convertRawToTemp(icm->currRD.rawTemp);

    icm->currD.accX = convertRawToAccel(icm->accCfg.ssf, icm->currRD.rawAccX, icm->accCfg.bias[0], icm->accCfg.scale[0]);
    icm->currD.accY = convertRawToAccel(icm->accCfg.ssf, icm->currRD.rawAccY, icm->accCfg.bias[1], icm->accCfg.scale[1]);
    icm->currD.accZ = convertRawToAccel(icm->accCfg.ssf, icm->currRD.rawAccZ, icm->accCfg.bias[2], icm->accCfg.scale[2]);
    
    icm->currD.gyroX = convertRawToGyro(icm->gyroCfg.ssf, icm->currRD.rawGyroX, icm->gyroCfg.bias[0]);
    icm->currD.gyroY = convertRawToGyro(icm->gyroCfg.ssf, icm->currRD.rawGyroY, icm->gyroCfg.bias[1]);
    icm->currD.gyroZ = convertRawToGyro(icm->gyroCfg.ssf, icm->currRD.rawGyroZ, icm->gyroCfg.bias[2]);

    return ICM_OK;
}

ICM_Status_t ICM_updateTempData(ICM_Config_t* icm)
{
    if (readRegs(icm, TEMP_DATA1, 2, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in updateTempData");
        return ICM_ERROR;
    }

    icm->currRD.rawTemp = ((int16_t) _buffer[0] << 8) | _buffer[1];
    icm->currD.temp = convertRawToTemp(icm->currRD.rawTemp);

    return ICM_OK;
}

ICM_Status_t ICM_updateAccelData(ICM_Config_t* icm)
{
    if (readRegs(icm, ACCEL_DATA_X1, 6, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in updateTempData");
        return ICM_ERROR;
    }

    icm->currRD.rawAccX = ((int16_t) _buffer[0] << 8) | _buffer[1];
    icm->currRD.rawAccY = ((int16_t) _buffer[2] << 8) | _buffer[3];
    icm->currRD.rawAccZ = ((int16_t) _buffer[4] << 8) | _buffer[5];

    icm->currD.accX = convertRawToAccel(icm->accCfg.ssf, icm->currRD.rawAccX, icm->accCfg.bias[0], icm->accCfg.scale[0]);
    icm->currD.accY = convertRawToAccel(icm->accCfg.ssf, icm->currRD.rawAccY, icm->accCfg.bias[1], icm->accCfg.scale[1]);
    icm->currD.accZ = convertRawToAccel(icm->accCfg.ssf, icm->currRD.rawAccZ, icm->accCfg.bias[2], icm->accCfg.scale[2]);

    return ICM_OK;
}

ICM_Status_t ICM_updateGyroData(ICM_Config_t* icm)
{
    if (readRegs(icm, GYRO_DATA_X1, 6, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in updateTempData");
        return ICM_ERROR;
    }

    icm->currRD.rawGyroX = ((int16_t) _buffer[0] << 8) | _buffer[1];
    icm->currRD.rawGyroY = ((int16_t) _buffer[2] << 8) | _buffer[3];
    icm->currRD.rawGyroZ = ((int16_t) _buffer[4] << 8) | _buffer[5];

    icm->currD.gyroX = convertRawToGyro(icm->gyroCfg.ssf, icm->currRD.rawGyroX, icm->gyroCfg.bias[0]);
    icm->currD.gyroY = convertRawToGyro(icm->gyroCfg.ssf, icm->currRD.rawGyroY, icm->gyroCfg.bias[1]);
    icm->currD.gyroZ = convertRawToGyro(icm->gyroCfg.ssf, icm->currRD.rawGyroZ, icm->gyroCfg.bias[2]);

    return ICM_OK;
}

inline float ICM_getTemp(ICM_Config_t* icm)
{
    return icm->currD.temp;
}

inline float ICM_getAccX(ICM_Config_t* icm)
{
    return icm->currD.accX;
}

inline float ICM_getAccY(ICM_Config_t* icm)
{
    return icm->currD.accY;
}

inline float ICM_getAccZ(ICM_Config_t* icm)
{
    return icm->currD.accZ;
}

inline float ICM_getGyroX(ICM_Config_t* icm)
{
    return icm->currD.gyroX;
}

inline float ICM_getGyroY(ICM_Config_t* icm)
{
    return icm->currD.gyroY;
}

inline float ICM_getGyroZ(ICM_Config_t* icm)
{
    return icm->currD.gyroZ;
}

inline ICM_DataPacket_t ICM_getData(ICM_Config_t* icm) 
{
    return icm->currD;
}

inline ICM_RawDataPacket_t ICM_getRawData(ICM_Config_t* icm) 
{
    return icm->currRD;
}

static inline float convertRawToGyro(float ssf, int16_t raw, float bias)
{
    return (raw * ssf) - bias;
}

static inline float convertRawToAccel(float ssf, int16_t raw, float bias, float scale)
{
    return ((raw * ssf) - bias) * scale;
}

static inline float convertRawToTemp(int16_t raw)
{
    return ((float)(raw) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;
}

/**
 * @brief Set the Bank of the internal register. 
 * 
 * @param bank 
 * @return ICM_Status_t 
 */
static ICM_Status_t setBank(ICM_Config_t* icm, uint8_t bank)
{
    ICM_Status_t ret = ICM_OK;
    if (bank > 4)
    {
        ret = ICM_ERROR;
    }
    else if (bank == icm->bank)
    {
        ret = ICM_OK;
    }
    else 
    {
        ret = writeReg(icm, REG_BANK_SEL, bank);
        icm->bank = bank;
    }
    
    return ret;
}

/**
 * @brief Write one byte to the internal sensor register.
 * 
 * @param addr Address of the register to write to
 * @param data Byte to write
 * @return ICM_Status_t ICM_OK if successful
 */
static ICM_Status_t writeReg(ICM_Config_t* icm, uint8_t addr,  uint8_t data)
{
    ICM_Status_t ret = ICM_OK;
    
    // Clear CS pin
    LL_GPIO_ResetOutputPin(icm->hal.csPort, icm->hal.csPin);
    // Send the address first and discard the return value
    (void)readWriteSPI(icm, addr & 0x7F); // MSB 0: write instruction
    // Send the data and discard the return value
    (void)readWriteSPI(icm, data); 
    // Set CS Pin
    LL_GPIO_SetOutputPin(icm->hal.csPort, icm->hal.csPin);

    return ret;
}

/**
 * @brief Read one or more bytes from internal sensor register.
 * 
 * @param addr Address of the register to read from
 * @param len Number of bytes to read
 * @param data pointer to the buffer to store the read
 * @return ICM_Status_t ICM_OK if successful
 */
static ICM_Status_t readRegs(ICM_Config_t* icm, uint8_t addr, uint8_t len, uint8_t* data)
{
    ICM_Status_t ret = ICM_OK;

    if (len <= 0)
    {
        $ERROR("Read length requested less than 0.");
        return ICM_ERROR;
    }

    // Clear CS pin
    LL_GPIO_ResetOutputPin(icm->hal.csPort, icm->hal.csPin);
    // Send address and discard return value
    (void)readWriteSPI(icm, addr | 0x80); // MSB 1: read instruction

    for (uint8_t i = 0; i < len; i++)
    {
        data[i] = readWriteSPI(icm, 0x00); // Send dummy byte to read
    }

    // Set CS Pin
    LL_GPIO_SetOutputPin(icm->hal.csPort, icm->hal.csPin);

    return ret;
}

/**
 * @brief Hardware implementation to read/write a byte on SPI using LL.
 * 
 * @param txData Byte to send via the SPI line
 * @return uint8_t Byte received via the SPI line
 */
static uint8_t readWriteSPI(ICM_Config_t* icm, uint8_t txData)
{
    uint8_t rxByte;

    // Wait for TX buffer to be empty
    while (!LL_SPI_IsActiveFlag_TXE(icm->hal.spi));
    // Transmit a byte
    LL_SPI_TransmitData8(icm->hal.spi, txData);
    // Wait until the RX buffer is not empty
    while (!LL_SPI_IsActiveFlag_RXNE(icm->hal.spi));
    // Receive the byte
    rxByte = LL_SPI_ReceiveData8(icm->hal.spi);

    return rxByte;
}

