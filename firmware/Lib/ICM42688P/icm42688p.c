#include "icm42688p.h"
#include "icm42688p_regs.h"

SPI_TypeDef *_spi = NULL;
GPIO_TypeDef *_csPort = NULL, *_int1Port = NULL, *_int2Port = NULL;
uint16_t _csPin, _int1Pin, _int2Pin;

ICM42688P_GYRO_FRS_t _gyroFRS;
ICM42688P_ACCEL_FRS_t _accelFRS;
float _gyroFloatScale = 0.0f;
float _accelFloatScale = 0.0f;

uint8_t _bank = 0;
static ICM_Status_t setBank(uint8_t bank);
static ICM_Status_t writeReg(uint8_t addr,  uint8_t data);
static ICM_Status_t readRegs(uint8_t addr, uint8_t len, uint8_t* data);
static uint8_t readWriteSPI(uint8_t txData);


static ICM_Status_t setBank(uint8_t bank)
{
    ICM_Status_t ret = ICM_OK;
    if (bank < 0 || bank > 4)
    {
        ret = ICM_ERROR;
    }
    else 
    {
        ret = writeReg(REG_BANK_SEL, bank);
    }
    
    return ret;
}

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

static ICM_Status_t readRegs(uint8_t addr, uint8_t len, uint8_t* data)
{
    ICM_Status_t ret = ICM_OK;

    if (len <= 0)
    {
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

