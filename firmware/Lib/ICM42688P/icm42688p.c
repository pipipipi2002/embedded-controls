#include "icm42688p.h"
#include "icm42688p_regs.h"
#include <math.h>

SPI_TypeDef *_spi = NULL;
GPIO_TypeDef *_csPort = NULL, *_int1Port = NULL, *_int2Port = NULL;
uint16_t _csPin, _int1Pin, _int2Pin;

uint8_t _buffer[20] = {0};                  
ICM_DataPacket_t _currData = {0};
ICM_RawDataPacket_t _currRawData = {0};

ICM42688P_GYRO_FRS_t _gyroFSR;
ICM42688P_ACCEL_FRS_t _accelFSR;
ICM42688P_ODR_t _gyroODR, _accelODR;
float _gyroScaleFactor = 0.0f;
float _accelScaleFactor = 0.0f;

float _gyroBias[3] = {0};
float _gyroBiasData[3] = {0};

float _accBias[3] = {0};
float _accBiasData[3] = {0};
float _accScale[3] = {1.0f, 1.0f, 1.0f};
float _accMax[3] = {0};
float _accMin[3] = {0};

const float _gyroScaleFactorArray[8] = {
    0.060976,   // 2000 DPS
    0.030488,   // 1000 DPS
    0.015267,   // 500 DPS
    0.007634,   // 250 DPS
    0.003817,   // 125 DPS
    0.001907,   // 62.5 DPS
    0.000954,   // 31.25 DPS
    0.000477    // 15.625 DPS
};

const float _accelScaleFactorArray[4] = {
    0.000488,   // 16g
    0.000244,   // 8g
    0.000122,   // 4g
    0.000061    // 2g
};

uint8_t _bank = 0;
static ICM_Status_t setBank(uint8_t bank);
static ICM_Status_t writeReg(uint8_t addr,  uint8_t data);
static ICM_Status_t readRegs(uint8_t addr, uint8_t len, uint8_t* data);
static uint8_t readWriteSPI(uint8_t txData);
static inline float convertRawToAccel(int16_t raw, float bias, float scale);
static inline float convertRawToGyro(int16_t raw, float bias);
static inline float convertRawToTemp(int16_t raw);

ICM_Status_t ICM42688P_init(SPI_TypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin, GPIO_TypeDef* int1Port, uint16_t int1Pin, GPIO_TypeDef* int2Port, uint16_t int2Pin)
{
    if (spi == NULL || csPort == NULL || int1Port == NULL || int2Port == NULL)
    {
        return ICM_ERROR;
    }

    _spi = spi;
    _csPort = csPort;
    _csPin = csPin;
    _int1Port = int1Port;
    _int1Pin = int1Pin;
    _int2Port = int2Port;
    _int2Pin = int2Pin;

    // Default values for ICM42688P
    _gyroFSR = GYRO_DPS_2000;
    _gyroScaleFactor = _gyroScaleFactorArray[_gyroFSR];
    _gyroODR = ODR_1k;
    
    _accelFSR = ACCEL_GPM_16;
    _accelScaleFactor = _accelScaleFactorArray[_accelFSR];
    _accelODR = ODR_1k;

    LL_GPIO_SetOutputPin(_csPort, _csPin);
    ICM42688P_reset();

    // Check for device ID
    uint8_t whoami = ICM42688P_whoami();
    if (whoami != ICM42688P_WHOAMI_BYTE)
    {
        $ERROR("Wrong WHOAMI. Exp %d, Recv %d.", ICM42688P_WHOAMI_BYTE, whoami);
        return ICM_ERROR;
    }

    self_test();

    // ICM42688P_setFilters(true, true);

    // Enable Temp, Accel (LN), and Gyro (LN)
    if (ICM42688P_enAll() != ICM_OK)
    {
        $ERROR("Unable to init, failed at enabling sensors.");
        return ICM_ERROR;
    }

    HAL_Delay(100);

    $SUCCESS("Init successful");

    return ICM_OK;
}

ICM_Status_t ICM42688P_reset(void)
{
    setBank(0);

    writeReg(DEVICE_CONFIG, 0x01);
    HAL_Delay(100);

    return ICM_OK;
}

uint8_t ICM42688P_whoami(void)
{
    setBank(0);

    if (readRegs(WHO_AM_I, 1, _buffer) != ICM_OK)
    {
        return 0x00;
    }

    return _buffer[0];
}

ICM_Status_t ICM42688P_enAll(void)
{
    if (ICM42688P_enTemp(true) != ICM_OK)
    {
        return ICM_ERROR;
    }

    if (ICM42688P_enGyro(GYRO_LN) != ICM_OK)
    {
        return ICM_ERROR;
    }

    if (ICM42688P_enAccel(ACCEL_LN) != ICM_OK)
    {
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM42688P_enTemp(bool en)
{
    setBank(0);
    uint8_t reg;

    if (readRegs(PWR_MGMT0, 1, &reg) != ICM_OK)
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

    if (writeReg(PWR_MGMT0, reg) != ICM_OK)
    {
        $ERROR("Write PWR_MGMT0 reg error in enTemp.");
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM42688P_enIdle(bool en)
{
    setBank(0);
    uint8_t reg;

    if (readRegs(PWR_MGMT0, 1, &reg) != ICM_OK)
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

    if (writeReg(PWR_MGMT0, reg) != ICM_OK)
    {
        $ERROR("Write PWR_MGMT0 reg error in enIdle.");
        return ICM_ERROR;
    }

    return ICM_OK;
}


ICM_Status_t ICM42688P_enGyro(ICM42688P_GYRO_PWR_t mode)
{
    setBank(0);

    uint8_t reg;

    if (readRegs(PWR_MGMT0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read PWR_MGMT0 error in enGyro.");
        return ICM_ERROR;
    }

    switch (mode)
    {
        case (GYRO_OFF):
        {
            reg = (reg & (~PWR_MGMT0_GYRO_MODE_MASK)) | GYRO_MODE_OFF;
            break;
        }
        case (GYRO_STANDBY):
        {
            reg = (reg & (~PWR_MGMT0_GYRO_MODE_MASK)) | GYRO_MODE_STANDBY;
            break;
        }
        case (GYRO_LN):
        {
            reg = (reg & (~PWR_MGMT0_GYRO_MODE_MASK)) | GYRO_MODE_LN;
            break;
        }
        default:
            break;
    }

    if (writeReg(PWR_MGMT0, reg) != ICM_OK)
    {
        $ERROR("Write PWR_MGMT0 reg error in enGyro.");
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM42688P_enAccel(ICM42688P_ACCEL_PWR_t mode)
{
    setBank(0);
    uint8_t reg;

    if (readRegs(PWR_MGMT0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read PWR_MGMT0 error in enAccel.");
        return ICM_ERROR;
    }

    switch (mode)
    {
        case (ACCEL_OFF):
        {
            reg = (reg & (~PWR_MGMT0_ACCEL_MODE_MASK)) | ACCEL_MODE_OFF;
            break;
        }
        case (ACCEL_LP):
        {
            reg = (reg & (~PWR_MGMT0_ACCEL_MODE_MASK)) | ACCEL_MODE_LP;
            break;
        }
        case (ACCEL_LN):
        {
            reg = (reg & (~PWR_MGMT0_ACCEL_MODE_MASK)) | ACCEL_MODE_LN;
            break;
        }
        default:
            break;
    }

    if (writeReg(PWR_MGMT0, reg) != ICM_OK)
    {
        $ERROR("Write PWR_MGMT0 reg error in enAccel.");
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM42688P_setGyroFSR(ICM42688P_GYRO_FRS_t gyroFsr)
{
    if (gyroFsr ==  _gyroFSR)
    {
        return ICM_OK;
    }

    setBank(0);

    uint8_t reg;
    // Read current reg
    if(readRegs(GYRO_CONFIG0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read GYRO_CONFIG0 reg error in setGyroFSR.");
        return ICM_ERROR;
    }
    
    // Write only the gyro FS bits
    reg = (reg & (~GYRO_CONFIG0_GYRO_FS_SEL_MASK)) | (uint8_t)(gyroFsr << GYRO_CONFIG0_GYRO_FS_SEL_POS);

    if (writeReg(GYRO_CONFIG0, reg) != ICM_OK)
    {
        $ERROR("Write GYRO_CONFIG0 reg error in setGyroFSR.");
        return ICM_ERROR;
    }

    // Store Full Scale Range
    _gyroFSR = gyroFsr;
    // Store current dps per 1 bit change
    _gyroScaleFactor = _gyroScaleFactorArray[gyroFsr];

    return ICM_OK;
}

ICM_Status_t ICM42688P_setAccelFSR(ICM42688P_ACCEL_FRS_t accelFsr)
{
    if (accelFsr == _accelFSR)
    {
        return ICM_OK;
    }

    setBank(0);

    uint8_t reg;
    // Read current reg
    if(readRegs(ACCEL_CONFIG0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read ACCEL_CONFIG0 reg error in setAccelFSR.");
        return ICM_ERROR;
    }
    
    // Write only the accel FS bits
    reg = (reg & (~ACCEL_CONFIG0_ACCEL_FS_SEL_MASK)) | (uint8_t)(accelFsr << ACCEL_CONFIG0_ACCEL_FS_SEL_POS);

    if (writeReg(ACCEL_CONFIG0, reg) != ICM_OK)
    {
        $ERROR("Write ACCEL_CONFIG0 reg error in setAccelFSR.");
        return ICM_ERROR;
    }

    // Store Full Scale Range
    _accelFSR = accelFsr;
    // Store current g per 1 bit change
    _accelScaleFactor = _accelScaleFactorArray[accelFsr];

    return ICM_OK;
}

ICM_Status_t ICM42688P_setGyroODR(ICM42688P_ODR_t gyroOdr)
{
    if (_gyroODR == gyroOdr)
    {
        return ICM_OK;
    }

    setBank(0);

    // read current reg 
    uint8_t reg;
    if (readRegs(GYRO_CONFIG0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read GYRO_CONFIG0 reg error in setGyroODR.");
        return ICM_ERROR;
    }

    // write only ODR bits
    reg = (reg & (~GYRO_CONFIG0_GYRO_ODR_MASK)) | (uint8_t)(gyroOdr);

    if (writeReg(GYRO_CONFIG0, reg) != ICM_OK)
    {
        $ERROR("Write GYRO_CONFIG0 reg error in setGyroODR.");
        return ICM_ERROR;
    }

    _gyroODR = gyroOdr;

    return ICM_OK;
}

ICM_Status_t ICM42688P_setAccelODR(ICM42688P_ODR_t accelOdr)
{
    if (accelOdr == _accelODR)
    {
        return ICM_OK;
    }

    setBank(0);

    // read current reg 
    uint8_t reg;
    if (readRegs(ACCEL_CONFIG0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read ACCEL_CONFIG0 reg error in setAccelODR.");
        return ICM_ERROR;
    }

    // write only ODR bits
    reg = (reg & (~ACCEL_CONFIG0_ACCEL_ODR_MASK)) | (uint8_t)(accelOdr);

    if (writeReg(ACCEL_CONFIG0, reg) != ICM_OK)
    {
        $ERROR("Write ACCEL_CONFIG0 reg error in setAccelODR.");
        return ICM_ERROR;
    }

    _accelODR = accelOdr;

    return ICM_OK;
}

ICM_Status_t ICM42688P_setFilters(bool gyroFils, bool accelFils)
{
    setBank(1);

    /* Set GYRO AAF and NF */
    if (gyroFils)
    {   
        // Direct write possible since no other fields in the registers
        if (writeReg(GYRO_CONFIG_STATIC2, GYRO_AAF_ENABLE | GYRO_NF_ENABLE) != ICM_OK)
        {
            $ERROR("Write GYRO_CONFIG_STATIC2 reg error in setFilters");
            return ICM_ERROR;
        }
    }
    else 
    {
        // Direct write possible since no other fields in the registers
        if (writeReg(GYRO_CONFIG_STATIC2, GYRO_AAF_DISABLE | GYRO_NF_DISABLE) != ICM_OK)
        {
            $ERROR("Write GYRO_CONFIG_STATIC2 reg error in setFilters");
            return ICM_ERROR;
        }
    }

    uint8_t reg;
    // Read the registers first to persist existing data
    if (readRegs(ACCEL_CONFIG_STATIC2, 1, &reg) != ICM_OK)
    {
        $ERROR("Read ACCEL_CONFIG_STATIC2 reg error in setFilters");
        return ICM_ERROR;
    }

    if (accelFils)
    {
        // Change only the DIS fields
        reg = (reg & (~ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_MASK)) | (ACCEL_AAF_ENABLE);
        if (writeReg(ACCEL_CONFIG_STATIC2, reg) != ICM_OK)
        {
            $ERROR("Write ACCEL_CONFIG_STATIC2 reg error in setFilters");
            return ICM_ERROR;
        }
    }
    else
    {
        // Change only the DIS fields
        reg = (reg & (~ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_MASK)) | (ACCEL_AAF_DISABLE);
        if (writeReg(ACCEL_CONFIG_STATIC2, reg) != ICM_OK)
        {
            $ERROR("Write ACCEL_CONFIG_STATIC2 reg error in setFilters");
            return ICM_ERROR;
        }
    }

    return ICM_OK;
}

ICM_Status_t ICM42688P_calibGyro(void)
{
    const ICM42688P_GYRO_FRS_t currFSR = _gyroFSR;

    if (ICM42688P_setGyroFSR(GYRO_DPS_250) != ICM_OK)
    {
        $ERROR("Unable to calib gyro.");
        return ICM_ERROR;
    }
    
    _gyroBiasData[0] = 0;
    _gyroBiasData[1] = 0;
    _gyroBiasData[2] = 0;

    for (uint32_t i = 0; i < CALIBRATION_SAMPLE_SIZE; i++)
    {
        ICM42688P_updateGyroData();
        _gyroBiasData[0] += (ICM42688_getGyroX() + _gyroBias[0]) / CALIBRATION_SAMPLE_SIZE;
        _gyroBiasData[1] += (ICM42688_getGyroY() + _gyroBias[1]) / CALIBRATION_SAMPLE_SIZE;
        _gyroBiasData[2] += (ICM42688_getGyroZ() + _gyroBias[2]) / CALIBRATION_SAMPLE_SIZE;
        HAL_Delay(1);
    }

    _gyroBias[0] = _gyroBiasData[0];
    _gyroBias[1] = _gyroBiasData[1];
    _gyroBias[2] = _gyroBiasData[2];

    if (ICM42688P_setGyroFSR(currFSR) != ICM_OK)
    {
        return ICM_ERROR;
    }

    $SUCCESS("Calibration of Gyro OK");
    return ICM_OK;
}

ICM_Status_t ICM42688P_calibAccel(void)
{
    const ICM42688P_ACCEL_FRS_t currFSR = _accelFSR;

    if (ICM42688P_setAccelFSR(ACCEL_GPM_2) != ICM_OK)
    {
        $ERROR("Unable to calibrate accel");
        return ICM_ERROR;
    }

    _accBiasData[0] = 0;
    _accBiasData[1] = 0;
    _accBiasData[2] = 0;

    for (uint32_t i = 0; i < CALIBRATION_SAMPLE_SIZE; i++)
    {
        ICM42688P_updateAccelData();
        _accBiasData[0] += (ICM42688_getAccX() / _accScale[0] + _accBias[0]) / CALIBRATION_SAMPLE_SIZE;
        _accBiasData[1] += (ICM42688_getAccY() / _accScale[1] + _accBias[1]) / CALIBRATION_SAMPLE_SIZE;
        _accBiasData[2] += (ICM42688_getAccZ() / _accScale[2] + _accBias[2]) / CALIBRATION_SAMPLE_SIZE;
        HAL_Delay(1);
    }

    if (_accBiasData[0] > 0.9f) _accMax[0] = _accBiasData[0];
    if (_accBiasData[1] > 0.9f) _accMax[1] = _accBiasData[1];
    if (_accBiasData[2] > 0.9f) _accMax[2] = _accBiasData[2];
    if (_accBiasData[0] < -0.9f) _accMin[0] = _accBiasData[0];
    if (_accBiasData[1] < -0.9f) _accMin[1] = _accBiasData[1];
    if (_accBiasData[2] < -0.9f) _accMin[2] = _accBiasData[2];

    if ((fabs(_accMin[0]) > 0.9f) && (fabs(_accMax[0]) > 0.9f))
    {
        _accBias[0] = (_accMin[0] + _accMax[0]) / 2.0f;
        _accScale[0] = 1 / ((fabs(_accMin[0]) + fabs(_accMax[0])) / 2.0f);
    }
    
    if ((fabs(_accMin[1]) > 0.9f) && (fabs(_accMax[1]) > 0.9f))
    {
        _accBias[1] = (_accMin[1] + _accMax[1]) / 2.0f;
        _accScale[1] = 1 / ((fabs(_accMin[1]) + fabs(_accMax[1])) / 2.0f);
    }
    
    if ((fabs(_accMin[2]) > 0.9f) && (fabs(_accMax[2]) > 0.9f))
    {
        _accBias[2] = (_accMin[2] + _accMax[2]) / 2.0f;
        _accScale[2] = 1 / ((fabs(_accMin[2]) + fabs(_accMax[2])) / 2.0f);
    }

    if (ICM42688P_setAccelFSR(currFSR) != ICM_OK)
    {
        return ICM_ERROR;
    }

    $SUCCESS("Calibration of Accel OK");
    return ICM_OK;
}

float ICM42688P_getGyroBiasX(void)
{
    return _gyroBias[0];
}

float ICM42688P_getGyroBiasY(void)
{
    return _gyroBias[1];
}

float ICM42688P_getGyroBiasZ(void)
{
    return _gyroBias[2];
}

void ICM42688P_setGyroBiasX(float bias)
{
    _gyroBias[0] = bias;
}

void ICM42688P_setGyroBiasY(float bias)
{
    _gyroBias[1] = bias;
}

void ICM42688P_setGyroBiasZ(float bias)
{
    _gyroBias[2] = bias;
}

float ICM42688P_getAccelBiasX(void)
{
    return _accBias[0];
}

float ICM42688P_getAccelScaleX(void)
{
    return _accScale[0];
}

float ICM42688P_getAccelBiasY(void)
{
    return _accBias[1];
}

float ICM42688P_getAccelScaleY(void)
{
    return _accScale[1];
}

float ICM42688P_getAccelBiasZ(void)
{
    return _accBias[2];
}

float ICM42688P_getAccelScaleZ(void)
{
    return _accScale[2];
}

void ICM42688P_setAccelCalX(float bias, float scale)
{
    _accBias[0] = bias;
    _accScale[0] = scale;
}

void ICM42688P_setAccelCalY(float bias, float scale)
{
    _accBias[1] = bias;
    _accScale[1] = scale;
}

void ICM42688P_setAccelCalZ(float bias, float scale)
{
    _accBias[2] = bias;
    _accScale[2] = scale;
}


ICM_Status_t ICM42688P_enableDataRdyInt(void)
{
    // Set ACTIVE HIGH, Push Pull and Pulsed Mode Interrupt on INT1 and INT2
    if (writeReg(INT_CONFIG, 0x18 | 0x03) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG error in enDataRdyInt");
        return ICM_ERROR;
    }

    uint8_t reg;
    if (readRegs(INT_CONFIG1, 1, &reg) != ICM_OK)
    {
        $ERROR("Read INT_CONFIG1 error in enDataRdyInt");
        return ICM_ERROR;
    }
    // Clear bit 4 for proper INT1 and INT2 operation
    reg = reg & ~(0x10);
    if (writeReg(INT_CONFIG1, reg) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG1 error in enDataRdyInt");
        return ICM_ERROR;
    }

    // Route UI_DRDY_INT (and reset) to INT1
    if (writeReg(INT_SOURCE0, 0x18) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG1 error in enDataRdyInt");
        return ICM_ERROR;
    }

    return ICM_OK;
}

ICM_Status_t ICM42688P_disableDataRdyInt(void)
{
    uint8_t reg;
    if (readRegs(INT_CONFIG1, 1, &reg) != ICM_OK)
    {
        $ERROR("Read INT_CONFIG1 error in disableDataRdyInt");
        return ICM_ERROR;
    }
    // Set bit 4 to default value
    reg = reg | 0x10; 
    if (writeReg(INT_CONFIG1, reg) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG1 error in disableDataRdyInt");
        return ICM_ERROR;
    }

    // Unroute UI_DRDY_INT 
    if (writeReg(INT_SOURCE0, 0x10) != ICM_OK)
    {
        $ERROR("Write INT_CONFIG1 error in enDataRdyInt");
        return ICM_ERROR;
    }

    return ICM_OK;
}


ICM_Status_t ICM42688P_updateAllData(void)
{
    // burst read all the sensor data
    if (readRegs(TEMP_DATA1, 14, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in updateAllData.");
        return ICM_ERROR;
    }

    _currRawData.rawTemp = ((int16_t) _buffer[0] << 8) | _buffer[1];
    _currRawData.rawAccX = ((int16_t) _buffer[2] << 8) | _buffer[3];
    _currRawData.rawAccY = ((int16_t) _buffer[4] << 8) | _buffer[5];
    _currRawData.rawAccZ = ((int16_t) _buffer[6] << 8) | _buffer[7];
    _currRawData.rawGyroX = ((int16_t) _buffer[8] << 8) | _buffer[9];
    _currRawData.rawGyroY = ((int16_t) _buffer[10] << 8) | _buffer[11];
    _currRawData.rawGyroZ = ((int16_t) _buffer[12] << 8) | _buffer[13];


    _currData.temp = convertRawToTemp(_currRawData.rawTemp);

    _currData.accX = convertRawToAccel(_currRawData.rawAccX, _accBias[0], _accScale[0]);
    _currData.accY = convertRawToAccel(_currRawData.rawAccY, _accBias[1], _accScale[1]);
    _currData.accZ = convertRawToAccel(_currRawData.rawAccZ, _accBias[2], _accScale[2]);

    _currData.gyroX = convertRawToGyro(_currRawData.rawGyroX, _gyroBias[0]);
    _currData.gyroY = convertRawToGyro(_currRawData.rawGyroY, _gyroBias[1]);
    _currData.gyroZ = convertRawToGyro(_currRawData.rawGyroZ, _gyroBias[2]);

    return ICM_OK;
}

ICM_Status_t ICM42688P_updateTempData(void)
{
    if (readRegs(TEMP_DATA1, 2, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in updateTempData");
        return ICM_ERROR;
    }

    _currRawData.rawTemp = ((int16_t) _buffer[0] << 8) | _buffer[1];
    _currData.temp = convertRawToTemp(_currRawData.rawTemp);

    return ICM_OK;
}

ICM_Status_t ICM42688P_updateAccelData(void)
{
    if (readRegs(ACCEL_DATA_X1, 6, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in updateTempData");
        return ICM_ERROR;
    }

    _currRawData.rawAccX = ((int16_t) _buffer[0] << 8) | _buffer[1];
    _currRawData.rawAccY = ((int16_t) _buffer[2] << 8) | _buffer[3];
    _currRawData.rawAccZ = ((int16_t) _buffer[4] << 8) | _buffer[5];

    _currData.accX = convertRawToAccel(_currRawData.rawAccX, _accBias[0], _accScale[0]);
    _currData.accY = convertRawToAccel(_currRawData.rawAccY, _accBias[1], _accScale[1]);
    _currData.accZ = convertRawToAccel(_currRawData.rawAccZ, _accBias[2], _accScale[2]);

    return ICM_OK;
}

ICM_Status_t ICM42688P_updateGyroData(void)
{
    if (readRegs(GYRO_DATA_X1, 6, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in updateTempData");
        return ICM_ERROR;
    }

    _currRawData.rawGyroX = ((int16_t) _buffer[0] << 8) | _buffer[1];
    _currRawData.rawGyroY = ((int16_t) _buffer[2] << 8) | _buffer[3];
    _currRawData.rawGyroZ = ((int16_t) _buffer[4] << 8) | _buffer[5];

    _currData.gyroX = convertRawToGyro(_currRawData.rawGyroX, _gyroBias[0]);
    _currData.gyroY = convertRawToGyro(_currRawData.rawGyroY, _gyroBias[1]);
    _currData.gyroZ = convertRawToGyro(_currRawData.rawGyroZ, _gyroBias[2]);

    return ICM_OK;
}

float ICM42688_getTemp(void)
{
    return _currData.temp;
}

float ICM42688_getAccX(void)
{
    return _currData.accX;
}

float ICM42688_getAccY(void)
{
    return _currData.accY;
}

float ICM42688_getAccZ(void)
{
    return _currData.accZ;
}

float ICM42688_getGyroX(void)
{
    return _currData.gyroX;
}

float ICM42688_getGyroY(void)
{
    return _currData.gyroY;
}

float ICM42688_getGyroZ(void)
{
    return _currData.gyroZ;
}

ICM_DataPacket_t ICM42688_getData(void) 
{
    return _currData;
}

ICM_RawDataPacket_t ICM42688_getRawData(void) 
{
    return _currRawData;
}

static inline float convertRawToGyro(int16_t raw, float bias)
{
    return (raw * _gyroScaleFactor) - bias;
}

static inline float convertRawToAccel(int16_t raw, float bias, float scale)
{
    return ((raw * _accelScaleFactor) - bias) * scale;
}

static inline float convertRawToTemp(int16_t raw)
{
    return ((float)(raw) / ICM42688P_TEMP_DATA_REG_SCALE) + ICM42688P_TEMP_OFFSET;
}

/**
 * @brief Set the Bank of the internal register. 
 * 
 * @param bank 
 * @return ICM_Status_t 
 */
static ICM_Status_t setBank(uint8_t bank)
{
    ICM_Status_t ret = ICM_OK;
    if (bank > 4)
    {
        ret = ICM_ERROR;
    }
    else if (bank == _bank)
    {
        ret = ICM_OK;
    }
    else 
    {
        ret = writeReg(REG_BANK_SEL, bank);
        _bank = bank;
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
static ICM_Status_t writeReg(uint8_t addr,  uint8_t data)
{
    ICM_Status_t ret = ICM_OK;
    
    // Clear CS pin
    LL_GPIO_ResetOutputPin(_csPort, _csPin);
    // Send the address first and discard the return value
    (void)readWriteSPI(addr & 0x7F); // MSB 0: write instruction
    // Send the data and discard the return value
    (void)readWriteSPI(data); 
    // Set CS Pin
    LL_GPIO_SetOutputPin(_csPort, _csPin);

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
static ICM_Status_t readRegs(uint8_t addr, uint8_t len, uint8_t* data)
{
    ICM_Status_t ret = ICM_OK;

    if (len <= 0)
    {
        $ERROR("Read length requested less than 0.");
        return ICM_ERROR;
    }

    // Clear CS pin
    LL_GPIO_ResetOutputPin(_csPort, _csPin);
    // Send address and discard return value
    (void)readWriteSPI(addr | 0x80); // MSB 1: read instruction

    for (uint8_t i = 0; i < len; i++)
    {
        data[i] = readWriteSPI(0x00); // Send dummy byte to read
    }

    // Set CS Pin
    LL_GPIO_SetOutputPin(_csPort, _csPin);

    return ret;
}

/**
 * @brief Hardware implementation to read/write a byte on SPI using LL.
 * 
 * @param txData Byte to send via the SPI line
 * @return uint8_t Byte received via the SPI line
 */
static uint8_t readWriteSPI(uint8_t txData)
{
    uint8_t rxByte;

    // Wait for TX buffer to be empty
    while (!LL_SPI_IsActiveFlag_TXE(_spi));
    // Transmit a byte
    LL_SPI_TransmitData8(_spi, txData);
    // Wait until the RX buffer is not empty
    while (!LL_SPI_IsActiveFlag_RXNE(_spi));
    // Receive the byte
    rxByte = LL_SPI_ReceiveData8(_spi);

    return rxByte;
}

