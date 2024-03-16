# BMP581-driver
This repository contains the driver for the BMP581 sensor, designed to facilitate easy integration into STM32 projects. The BMP581 is a high-precision, low-power 24 bit digital barometer that offers superior accuracy and performance. This driver enables the sensor to communicate with a microcontroller only via I2C communication protocol.

### Features

    Functions for reading temperature and pressure data
    Low power consumption mode support
    Easy to integrate with various microcontroller platforms

Getting Started
Prerequisites

    Microcontroller compatible with the I2C or SPI protocol
    BMP581 sensor module

Installation

    Clone the repository to your local machine:

bash

git clone https://github.com/KrabasChuliganas/BMP581-driver.git

    Include the driver in your project.

Usage

To use this driver, initialize the sensor and then read the temperature and pressure data. Example code snippets are provided below for both I2C and SPI configurations.
I2C Example

c

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


License

This project is licensed under the MIT License - see the LICENSE file for details.
