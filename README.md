# BMP581 STM32 library
This repository contains the library for the BMP581 sensor, designed to facilitate easy integration into STM32 projects. The BMP581 is a high-precision, low-power 24 bit digital barometer that offers superior accuracy and performance. This library enables the sensor to communicate with a STM32 only via I2C communication protocol.

## Functions
Detail function's parameters and returns are commented in .C file.

### Sensor control
- Sets i2c stuct and sensor address: BMP5_Init(BMP5 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t addr);
- Starts sensor: BMP5_Start_Mode(BMP5 *dev, uint8_t mode, uint8_t dataRate, uint8_t osr_p, uint8_t osr_t);
- Sensors stops and goes to sleep: BMP5_Sleep_Mode(BMP5 *dev);

### DataReady flag
- Sets DataReady by checking sensor's register if data is ready: BMP5_SOFT_DataReady(BMP5 *dev);
- Sets DataReady when this function is in EXTI callback: BMP5_INT_EXTI_DataReady(BMP5 *dev, uint16_t GPIO_Pin);
> Make sure to set after Init: BMP5_INT_EXTI_Enable(BMP5 *dev, GPIO_TypeDef *INT_GPIOx, uint16_t INT_GPIO_Pin);
### Data read
- Get data from sensor (reading sensor registers if DataReady): BMP5_SaveConvData(BMP5 *dev);
- Get temperature and pressure data (returns values from struct): BMP5_GetConvData(BMP5 *dev, float *pressure, float *temperature);

## Getting Started
Initialize the sensor and then read the temperature and pressure data.

### Example

    // Include the BMP581 driver header
    #include "BMP581.h"
    // Create stucture
    BMP5 sensor;

    // Initialize the BMP581
    BMP5_Init(&sensor, &hi2c1, BMP5_I2C_ADDR_FIRST);
    BMP5_INT_EXTI_Enable(&sensor, BMP_INT_GPIO_Port, BMP_INT_Pin);

    // Read temperature and pressure
    float temperature, pressure;
    BMP5_SaveConvData(&sensor);
    BMP5_GetConvData(&sensor, &pressure, &temperature);

## Notes
- Depending on your STM32 set different include in .H file
