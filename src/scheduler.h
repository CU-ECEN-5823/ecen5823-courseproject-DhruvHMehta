/*
 * scheduler.h
 *
 *  Created on: Sep 14, 2021
 *      Author: Dhruv
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

// Include in .c ?
#include "em_core.h"
#include "i2c.h"
#include "app.h"

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

#endif /* SRC_SCHEDULER_H_ */
