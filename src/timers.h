/*
 * timers.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "app.h"
#include "EFR32BG13P632F512GM48.h"
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

#endif /* SRC_TIMERS_H_ */
