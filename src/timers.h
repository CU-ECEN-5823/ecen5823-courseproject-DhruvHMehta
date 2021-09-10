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

/* Use LFXO and prescalar for high energy modes */
#if ((LOWEST_ENERGY_MODE == EM0) || (LOWEST_ENERGY_MODE == EM1) || (LOWEST_ENERGY_MODE == EM2))
#define PRESCALAR_VALUE   4
#define ACTUAL_CLOCK_FREQ (32768/PRESCALAR_VALUE)

/* Use ULFRCO and prescalar for low energy modes */
#else
#define PRESCALAR_VALUE   1
#define ACTUAL_CLOCK_FREQ (1000/PRESCALAR_VALUE)
#endif

#define LETIMER_CTR_VAL   ((LETIMER_PERIOD_MS * ACTUAL_CLOCK_FREQ)/1000)
#define LETIMER_COMP1_VAL ((LETIMER_ON_TIME_MS * ACTUAL_CLOCK_FREQ)/1000)

/* Function Prototypes */
void LETIMER0Init();
void LETIMER0InterruptEn();

#endif /* SRC_TIMERS_H_ */
