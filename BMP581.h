/*
 * BMP581 pressure sensor I2C driver
 *
 * Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf
 *
 * NOTE: FIFO and NVM is not implemented!
 *
 *  Created on: Feb 23, 2024
 *      Author: Krabas
 */

#ifndef INC_BMP581_H_
#define INC_BMP581_H_

#include <stm32WBAxx_hal.h>

// Register addresses
#define BMP5_REG_CHIP_ID			0x01
#define BMP5_REG_REV_ID				0x02
#define BMP5_REG_CHIP_STATUS		0x11
#define BMP5_REG_DRIVE_CONFIG 		0x13
#define BMP5_REG_INT_CONFIG			0x14
#define BMP5_REG_INT_SOURCE			0x15
#define BMP5_REG_FIFO_CONFIG		0x16
#define BMP5_REG_FIFO_COUNT			0x17
#define BMP5_REG_FIFO_SEL			0x18
#define BMP5_REG_TEMP_DATA_XLSB		0x1D
#define BMP5_REG_TEMP_DATA_LSB		0x1E
#define BMP5_REG_TEMP_DATA_MSB		0x1F
#define BMP5_REG_PRESS_DATA_XLSB	0x20
#define BMP5_REG_PRESS_DATA_LSB		0x21
#define BMP5_REG_PRESS_DATA_MSB		0x22
#define BMP5_REG_INT_STATUS			0x27
#define BMP5_REG_STATUS				0x28
#define BMP5_REG_FIFO_DATA			0x29
#define BMP5_REG_NVM_ADDR			0x2B
#define BMP5_REG_NVM_DATA_LSB		0x2C
#define BMP5_REG_NVM_DATA_MSB		0x2D
#define BMP5_REG_DSP_CONFIG			0x30
#define BMP5_REG_DSP_IIR			0x31
#define BMP5_REG_OOR_THR_P_LSB		0x32
#define BMP5_REG_OOR_THR_P_MSB		0x33
#define BMP5_REG_OOR_RANGE			0x34
#define BMP5_REG_OOR_CONFIG			0x35
#define BMP5_REG_OSR_CONFIG			0x36
#define BMP5_REG_ODR_CONFIG			0x37
#define BMP5_REG_OSR_EFF			0x38
#define BMP5_REG_CMD				0x7E

// I2C addresses
#define BMP5_I2C_ADDR_FIRST			0x46 // (SDO pulled down)
#define BMP5_I2C_ADDR_SECOND		0x47 // (SDO pulled up)

// CMD commands
#define BMP5_SOFT_RESET_CMD			0xB6
#define BMP5_NVM_FIRST_CMD			0x5D
#define BMP5_NVM_READ_ENABLE_CMD	0xA5
#define BMP5_NVM_WRITE_ENABLE_CMD	0xA0

// I2C communication timeout
#define BMP5_TIMEOUT       			10

typedef struct {
	// pointer to i2c handler
	I2C_HandleTypeDef *i2cHandle;
	// interrupt pin (rising edge (pulse))
	GPIO_TypeDef *INT_GPIOx;
	uint16_t INT_GPIO_Pin;
	// 1 - enabled; 0 - disabled
	uint8_t INT_enabled;
	// device I2C address
	uint8_t i2c_addr;
	// 1 - ready ; 0 - NOT ready
	uint8_t dataReady;
	// raw temperature
	float temperature;
	float pressure;

} BMP5;

// user can use these functions
uint8_t BMP5_Init(BMP5 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t addr);
uint8_t BMP5_INT_EXTI_Enable(BMP5 *dev, GPIO_TypeDef *INT_GPIOx, uint16_t INT_GPIO_Pin);
void BMP5_INT_EXTI_DataReady(BMP5 *dev, uint16_t GPIO_Pin);
uint8_t BMP5_SOFT_DataReady(BMP5 *dev);
uint8_t BMP5_SaveConvData(BMP5 *dev);
void BMP5_GetConvData(BMP5 *dev, float *pressure, float *temperature);
uint8_t BMP5_Start_Mode(BMP5 *dev, uint8_t mode, uint8_t dataRate, uint8_t osr_p, uint8_t osr_t);
uint8_t BMP5_Sleep_Mode(BMP5 *dev);
uint8_t BMP5_SOFT_Reset(BMP5 *dev);

// private I2C communication functions
// write command 8 bit (register) + 8 bit data X size
uint8_t BMP5_WriteReg(BMP5 *dev, uint8_t reg, uint8_t size, uint8_t *data);
// read command 8 bit (register) + 8 bit data X size
uint8_t BMP5_ReadReg(BMP5 *dev, uint8_t reg, uint8_t size, uint8_t *data);

#endif
