/*
 * irq.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 */

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_

#include "em_letimer.h"

// Remove this?
#include "gpio.h"
#include "scheduler.h"
#include "timers.h"
#include "em_i2c.h"

uint32_t letimerMilliseconds();
I2C_TransferReturn_TypeDef getI2CTransferReturn();

#endif /* SRC_IRQ_H_ */
