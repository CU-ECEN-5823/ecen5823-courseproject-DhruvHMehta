/*
 * i2c.h
 *
 *  Created on: Sep 14, 2021
 *      Author: Dhruv
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include "sl_i2cspm.h"
#include "em_i2c.h"
#include "gpio.h"
#include "timers.h"

/***************************************************************************//**
 * @name I2C_Init
 *
 * @brief
 *   Initializes the I2C peripheral for the Si7021 temperature sensor.
 *
 * @param[in] none
 *
 * @return void
 ******************************************************************************/
void I2CInit();

/***************************************************************************//**
 * @name I2CTransfer
 *
 * @brief
 *   Sets up the transfer sequence structure for the I2CSPM_Transfer call
 *   based on the parameters.
 *
 * @param[in/out] Data - I2C command or pointer to buffer to send/receive.
 * @param[in/out] ReadWrite - Read or Write I2C bit. Refer to I2CSPM_Transfer flags.
 * @param[in/out] DataLen - sizeof(Data)
 *
 * @return status - Transfer successful/unsuccesful
 ******************************************************************************/
uint8_t I2CTransfer(uint8_t* Data, uint8_t ReadWrite, uint8_t DataLen);

/***************************************************************************//**
 * @name getTemperatureSi7021
 *
 * @brief
 *   Gets the Temperature value from the Si7021 sensor.
 *
 * @param[in] none
 *
 * @return void
 ******************************************************************************/
void getTemperatureSi7021();

#endif /* SRC_I2C_H_ */
