# The Hardware
- [Interactive BOM](https://pipipipi2002.github.io/embedded-controls/hardware/controls-carier-board/output/interactive_bom.html)

## MCU
I will be using the STM32F446VETx Module from [stm32f446vetx-dev-module](https://github.com/pipipipi2002/stm32f446vetx-dev-module)

## Sensor
No | Sensor | Description | Connection
---| --- |--- | ---
1  | ICM-40609-D | 6-axis IMU | SPI
2  | ICM-42688-P | 6-axis IMU | SPI
3  | BNO085 | 9-axis IMU | SPI
4  | BMM150 | Magnetometer | SPI
5  | BMP390 | Barometer | SPI

Item 1 and 2 will share the same SPI line, the rest will have its own SPI master connection.

