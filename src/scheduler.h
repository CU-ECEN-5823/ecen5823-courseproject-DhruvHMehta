/*
 * scheduler.h
 *
 *  Created on: Sep 14, 2021
 *      Author: Dhruv
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include "em_core.h"
#include "i2c.h"
#include "app.h"
#include "irq.h"

/* Function Prototypes */

/***************************************************************************//**
 * @name app_process_action
 *
 * @brief Contains the scheduler for the events which gets the temperature
 *        when UF event occurs.
 *
 * @param[in] none
 *
 *
 * @return void
 ******************************************************************************/

void app_process_action();

/***************************************************************************//**
 * @name schedulerSetEvent_UF
 *
 * @brief Sets the UF event in the scheduler event.
 *
 * @param[in] none
 *
 *
 * @return void
 ******************************************************************************/
void schedulerSetEvent_UF();

/***************************************************************************//**
 * @name schedulerSetEvent_COMP1
 *
 * @brief Sets the COMP1 event in the scheduler event.
 *
 * @param[in] none
 *
 *
 * @return void
 ******************************************************************************/
void schedulerSetEvent_COMP1();

/***************************************************************************//**
 * @name schedulerSetEvent_I2Cdone
 *
 * @brief Sets the I2C done event in the scheduler event.
 *
 * @param[in] none
 *
 *
 * @return void
 ******************************************************************************/
void schedulerSetEvent_I2Cdone();

#endif /* SRC_SCHEDULER_H_ */
