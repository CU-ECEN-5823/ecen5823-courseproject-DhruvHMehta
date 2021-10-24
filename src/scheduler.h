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
#include "irq.h"
#include "sl_bt_api.h"

/* Function Prototypes */

/***************************************************************************//**
 * @name temperatureStateMachine
 *
 * @brief Contains the scheduler for the events in the Server which gets
 *        the temperature when UF event occurs.
 *
 * @param[in] sl_bt_msg_t *evt
 *
 *
 * @return void
 ******************************************************************************/
void temperatureStateMachine(sl_bt_msg_t *evt);

/***************************************************************************//**
 * @name discovery_state_machine
 *
 * @brief Contains the scheduler for the events in the Client which connect
 *        to the Server and acquire temperature data.
 *
 * @param[in] sl_bt_msg_t *evt
 *
 *
 * @return void
 ******************************************************************************/
void discovery_state_machine(sl_bt_msg_t *evt);

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

/***************************************************************************//**
 * @name schedulerSetEvent_ButtonPressed
 *
 * @brief Sets the Button Pressed in the scheduler event.
 *
 * @param[in] none
 *
 *
 * @return void
 ******************************************************************************/
void schedulerSetEvent_ButtonPressed();

#endif /* SRC_SCHEDULER_H_ */
