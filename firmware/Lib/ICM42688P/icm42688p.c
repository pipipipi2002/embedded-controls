#include "icm42688p.h"
#include "icm42688p_regs.h"

SPI_TypeDef *_spi = NULL;
GPIO_TypeDef *_csPort = NULL, *_int1Port = NULL, *_int2Port = NULL;
uint16_t _csPin, _int1Pin, _int2Pin;

uint8_t _buffer[20] = {0};                  // Store 8bits read from ICM
uint16_t _rawMeasure[7] = {0};              // Store concatenated data from buffer

ICM_DataPacket_t _currData = {0};
ICM42688P_GYRO_FRS_t _gyroFSR;
ICM42688P_ACCEL_FRS_t _accelFSR;
ICM42688P_ODR_t _gyroODR, _accelODR;
float _gyroFloatScale = 0.0f;
float _accelFloatScale = 0.0f;

uint8_t _bank = 0;
static ICM_Status_t setBank(uint8_t bank);
static ICM_Status_t writeReg(uint8_t addr,  uint8_t data);
static ICM_Status_t readRegs(uint8_t addr, uint8_t len, uint8_t* data);
static uint8_t readWriteSPI(uint8_t txData);

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
    _gyroODR = ODR_1k;
    _accelFSR = ACCEL_GPM_16;
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

    // Enable power to Temp, Accel (LN), and Gyro (LN)
    if (ICM42688P_enableSensors(true, false, GYRO_LN, ACCEL_LN) != ICM_OK)
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

ICM_Status_t ICM42688P_enableSensors(bool enTemp, bool enIdle, ICM42688P_GYRO_PWR_t gyroPwr, ICM42688P_ACCEL_PWR_t accelPwr)
{
    uint8_t reg;

    if (readRegs(PWR_MGMT0, 1, &reg) != ICM_OK)
    {
        $ERROR("Read PWR_MGMT0 error in enableSensors.");
        return ICM_ERROR;
    }

    if (enTemp)
    {
        reg = (reg & (~PWR_MGMT0_TEMP_DIS_MASK)) | TEMP_ENABLE;
    }
    else 
    {
        reg = (reg & (~PWR_MGMT0_TEMP_DIS_MASK)) | TEMP_DISABLE;
    }

    if (enIdle)
    {
        reg = (reg & (~PWR_MGMT0_IDLE_MASK_MASK)) | IDLE_ENABLE;
    }
    else 
    {
        reg = (reg & (~PWR_MGMT0_IDLE_MASK_MASK)) | IDLE_DISABLE;
    }

    switch (gyroPwr)
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

    switch (accelPwr)
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
        $ERROR("Write PWR_MGMT0 reg error in enableSensors.");
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
    _gyroFloatScale = (2000.0f / (float)(1 << gyroFsr)) / 32768.0f;

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
    _accelFloatScale = (float)(1 << (4 - accelFsr)) / 32768.0f;

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

ICM_Status_t ICM42688P_getAccelGyroTempData(void)
{
    if (readRegs(TEMP_DATA1, 14, _buffer) != ICM_OK)
    {
        $ERROR("Read TEMP_DATA1 error in getAccelGyroTempData.");
        return ICM_ERROR;
    }

    for (uint8_t i = 0; i < 7; i++)
    {
        _rawMeasure[i] = ((uint16_t) _buffer[i*2] << 8) | _buffer[i*2+1];
    }

    _currData.temp = ICM42688_convertRawToTemp(_rawMeasure[0]);
    _currData.accX = _rawMeasure[1];
    _currData.accY = _rawMeasure[2];
    _currData.accZ = _rawMeasure[3];
    _currData.gyroX = _rawMeasure[4];
    _currData.gyroY = _rawMeasure[5];
    _currData.gyroZ = _rawMeasure[6];

    return ICM_OK;
}

float ICM42688_convertRawToTemp(uint16_t raw)
{
    return ((float)(raw) / ICM42688P_TEMP_DATA_REG_SCALE) + ICM42688P_TEMP_OFFSET;
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

/**
 * @brief Set the Bank of the internal register. 
 * 
 * @param bank 
 * @return ICM_Status_t 
 */
static ICM_Status_t setBank(uint8_t bank)
{
    ICM_Status_t ret = ICM_OK;
    if (bank < 0 || bank > 4)
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

