/*
 * timers.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "app.h"
#include "em_letimer.h"


/* Function Prototypes */

/***************************************************************************//**
 * @name LETIMER0Init
 *
 * @brief
 *   Initialize the LETIMER0 peripheral to run in continous operation with
 *   CNT set to LETIMER_CTR_VAL and COMP0 as the reload register.
 *   COMP1 is set to LETIMER_COMP1_VAL.
 *
 * @param[in] osc
 *   none
 *
 * @return void
 ******************************************************************************/
void LETIMER0Init();

/***************************************************************************//**
 * @name LETIMER0InterruptEn
 *
 * @brief
 *   Enable interrupts in the LETIMER0 Interrupt register for UF and COMP1
 *   Also enables interrupts in the NVIC.
 *
 * @param[in] osc
 *   none
 *
 * @return void
 ******************************************************************************/
void LETIMER0InterruptEn();

/***************************************************************************//**
 * @name timerWaitUs_polled
 *
 * @brief
 *   Delays/Spins for the time specified in us_wait. Has upper and lower limit
 *   checks and a timer rollover check as well.
 *
 * @param[in] us_wait - Time to spin/delay
 *
 * @return void
 ******************************************************************************/
void timerWaitUs_polled(uint32_t us_wait);

/***************************************************************************//**
 * @name timerWaitUs_irq
 *
 * @brief
 *   Non-blocking implementation of wait routine.
 *
 * @param[in] us_wait - Time to delay/wait
 *
 * @return void
 ******************************************************************************/
void timerWaitUs_irq(uint32_t us_wait);

#endif /* SRC_TIMERS_H_ */
