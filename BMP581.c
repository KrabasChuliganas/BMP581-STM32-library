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

#include <BMP581.h>

// -------------------------------------------------------
// -------------------- User functions -------------------
// -------------------------------------------------------

/**
 * @brief BMP5 status check. CALL FIRST !!! In setup.
 * @param dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @param *i2cHandle: pointer to a I2C_HandleTypeDef structure that contains
 *               the configuration information for I2C module.
 * @param addr: address I2C.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR; 2 = PARAM_ERROR; DEV_ERROR.
 */
uint8_t BMP5_Init(BMP5 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t addr) {
	// set data to struct
	dev->i2cHandle = i2cHandle;
	// verify that address in correct
	if (addr == BMP5_I2C_ADDR_FIRST || addr == BMP5_I2C_ADDR_SECOND) {
		dev->i2c_addr = (addr << 1);
	} else {
		return 2;
	}
	// preset data
	dev->dataReady = 0;
	dev->INT_enabled = 0;

	//wait for sensor to startup (2 ms min)
	while (HAL_GetTick() < 3) {
	}
	// reset ADC
	if (BMP5_SOFT_Reset(dev) == 1) {
		return 1;
	}
	// 2ms after startup
	HAL_Delay(3);

	// chip ID register check
	uint8_t dataReg;
	if (BMP5_ReadReg(dev, BMP5_REG_CHIP_ID, 1, &dataReg) == 1) {
		return 1;
	}
	// chip ID check
	if (dataReg == 0) {
		return 3;
	}

	// status register check
	if (BMP5_ReadReg(dev, BMP5_REG_STATUS, 1, &dataReg) == 1) {
		return 1;
	}
	// nvm_rdy and nvm_err check
	if ((dataReg & 0b110) != 0b010) {
		return 3;
	}

	// int_status register check
	if (BMP5_ReadReg(dev, BMP5_REG_INT_STATUS, 1, &dataReg) == 1) {
		return 1;
	}
	// POR or software reset complete check
	if ((dataReg & 0b10000) == 0b10000) {
		return 3;
	}

	return 0;
}

/**
 * @brief Configuration of BMP5 interrupt pin. In setup. Pulse mode. Active high.
 * @param dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @param *INT_GPIOx: INT pin port.
 * @param INT_GPIO_Pin: INT pin.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR.
 */
uint8_t BMP5_INT_EXTI_Enable(BMP5 *dev, GPIO_TypeDef *INT_GPIOx, uint16_t INT_GPIO_Pin) {
	// set data to struct about HW pins
	dev->INT_GPIOx = INT_GPIOx;
	dev->INT_GPIO_Pin = INT_GPIO_Pin;
	// INT_CONFIG and INT_SOURCE
	uint8_t dataReg[] = { 0b00111010, 0b00000001 };
	if (BMP5_WriteReg(dev, BMP5_REG_INT_CONFIG, 2, dataReg) == 1) {
		return 1;
	}
	dev->INT_enabled = 1;
	return 0;
}

/**
 * @brief Check if data is ready from INT pin. Put this function in EXTI callback.
 * @param dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @param GPIO_Pin: EXTI callback GPIO_Pin parameter
 */
void BMP5_INT_EXTI_DataReady(BMP5 *dev, uint16_t GPIO_Pin) {
	if (dev->INT_enabled == 1 && dev->dataReady == 0) {
		if (GPIO_Pin == dev->INT_GPIO_Pin) {
			dev->dataReady = 1;
		}
	}
}

/**
 * @brief Check if data is ready from INT pin. Software check. Don't spam requests.
 * @param dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR.
 */
uint8_t BMP5_SOFT_DataReady(BMP5 *dev) {
	if (dev->dataReady == 0) {
		uint8_t dataReg;
		// int_status.drdy_data_reg
		if (BMP5_ReadReg(dev, BMP5_REG_INT_STATUS, 1, &dataReg) == 1) {
			return 1;
		}
		// is flag set on data ready
		if ((dataReg & 0b1) == 0b1) {
			dev->dataReady = 1;
		}
	}
	return 0;
}

/**
 * @brief Update data of temperature and pressure variables. In loop.
 * @param dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR.
 */
uint8_t BMP5_SaveConvData(BMP5 *dev) {
	if (dev->dataReady == 1) {
		// read 3(temperature) + 3(pressure) bytes
		uint8_t dataReg[6];
		if (BMP5_ReadReg(dev, BMP5_REG_TEMP_DATA_XLSB, 6, dataReg) == 1) {
			return 1;
		}
		dev->temperature = (float) ((dataReg[2] << 16) + (dataReg[1] << 8) + dataReg[0]) / (1 << 16);
		dev->pressure = (float) ((dataReg[5] << 16) + (dataReg[4] << 8) + dataReg[3]) / (1 << 6);
		dev->dataReady = 2;
	}
	return 0;
}

/**
 * @brief Get data of temperature and pressure variables. In loop.
 * @param dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @param pressure: measured pressure.
 * @param pressure: measured temperature.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR.
 */
void BMP5_GetConvData(BMP5 *dev, float *pressure, float *temperature) {
	*pressure = dev->pressure;
	*temperature = dev->temperature;
	dev->dataReady = 0;
}

/**
 * @brief Change measure mode of sensor. Initiates start.
 * @param dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @param mode: 0 - sleep; 1 - normal; 2 - forced; 3 - continuous.
 * @param dataRate: output date rate. Check datasheet 65 page.
 * @param osr_p: oversampling pressure. Check datasheet 64 page.
 * @param osr_t: oversampling temperature. Check datasheet 64 page.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR; 2 = PARAM_ERROR.
 */
uint8_t BMP5_Start_Mode(BMP5 *dev, uint8_t mode, uint8_t dataRate, uint8_t osr_p, uint8_t osr_t) {
	// check if correct mode is requested
	if (mode > 3 || mode == 0) {
		return 2;
	}
	if (dataRate > 0x1F) {
		return 2;
	}
	// check if correct oversample is requested
	if (osr_p > 7) {
		return 2;
	}
	if (osr_t > 7) {
		return 2;
	}

	uint8_t dataReg[] = { 0b01000000 + (osr_p << 3) + osr_t, (dataRate << 2) + mode };
	if (BMP5_WriteReg(dev, BMP5_REG_OSR_CONFIG, 2, dataReg) == 1) {
		return 1;
	}

	return 0;
}

/**
 * @brief Change mode of sensor to sleep. Sensor will go to deep sleep. Initiates sleep.
 * @param dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR.
 */
uint8_t BMP5_Sleep_Mode(BMP5 *dev) {
	uint8_t dataReg = 0b01110000;
	if (BMP5_WriteReg(dev, BMP5_REG_ODR_CONFIG, 1, &dataReg) == 1) {
		return 1;
	}
	return 0;
}

/**
 * @brief  Reset BMP5 sensor.
 * @param  dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR.
 */
uint8_t BMP5_SOFT_Reset(BMP5 *dev) {
	uint8_t dataReg = BMP5_SOFT_RESET_CMD;
	return BMP5_WriteReg(dev, BMP5_REG_CMD, 1, &dataReg);
}

// -------------------------------------------------------
// ------------------ Private functions ------------------
// -------------------------------------------------------

/**
 * @brief  Write BMP5 register(s).
 * @param  dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @param  reg: start register to write.
 * @param  size: amount of registers to write.
 * @param  *data: pointer of data to transmit.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR.
 */
uint8_t BMP5_WriteReg(BMP5 *dev, uint8_t reg, uint8_t size, uint8_t *data) {
	// write byte(s) data to device over I2C
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->i2cHandle, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, size, BMP5_TIMEOUT);
	if (status != HAL_OK) {
		return 1;
	}
	return 0;
}

/**
 * @brief  Read BMP5 register(s).
 * @param  dev: pointer to a BMP5 structure that contains
 *               the configuration information of BMP5.
 * @param  reg: start register to read.
 * @param  size: amount of registers to read.
 * @param  *data: pointer of data to receive.
 * @retval error codes: 0 = OK; 1 = I2C_ERROR.
 */
uint8_t BMP5_ReadReg(BMP5 *dev, uint8_t reg, uint8_t size, uint8_t *data) {
	// read byte(s) data from device over I2C
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->i2cHandle, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, size, BMP5_TIMEOUT);
	if (status != HAL_OK) {
		return 1;
	}
	return 0;
}
