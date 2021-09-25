/*
 * irq.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 */

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_

#include "em_letimer.h"

#include "gpio.h"
#include "scheduler.h"
#include "timers.h"
#include "em_i2c.h"

/***************************************************************************//**
 * @name letimerMilliseconds
 *
 * @brief
 *   Returns the number of seconds since the system startup.
 *
 * @param[in] osc
 *   none
 *
 * @return uint32_t - number of seconds since the system startup.
 ******************************************************************************/
uint32_t letimerMilliseconds();

/***************************************************************************//**
 * @name getI2CTransferReturn
 *
 * @brief
 *   Gets the Transfer Return Status of the I2C0 peripheral
 *
 * @param[in] osc
 *   none
 *
 * @return I2C_TransferReturn_TypeDef - I2C Transfer Return Status
 ******************************************************************************/
I2C_TransferReturn_TypeDef getI2CTransferReturn();

#endif /* SRC_IRQ_H_ */
