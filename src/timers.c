/*
 * timers.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 *      Brief : Contains functions for initializing the LETIMER0 and setting up
 *              interrupts.
 */

#include "timers.h"

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

void LETIMER0Init()
{
  LETIMER_Init_TypeDef LETIMER0_Init_Struct;

  /* Reload value of COMP0 into CNT when UF occurs */
  LETIMER0_Init_Struct.enable   = false;
  LETIMER0_Init_Struct.debugRun = false;
  LETIMER0_Init_Struct.comp0Top = true;
  LETIMER0_Init_Struct.bufTop   = false;
  LETIMER0_Init_Struct.out0Pol  = false;
  LETIMER0_Init_Struct.out1Pol  = false;
  LETIMER0_Init_Struct.ufoa0    = letimerUFOANone;
  LETIMER0_Init_Struct.ufoa1    = letimerUFOANone;
  LETIMER0_Init_Struct.repMode  = letimerRepeatFree;
  LETIMER0_Init_Struct.topValue = false;

  /* Initialize LETIMER0 peripheral with Init Struct values */
  LETIMER_Init(LETIMER0, &LETIMER0_Init_Struct);

  /* Set CNT value to required value */
  LETIMER_CounterSet(LETIMER0, LETIMER_CTR_VAL);

  /* Set COMP0 value to required value (same as CNT for reload) */
  LETIMER_CompareSet(LETIMER0, 0, LETIMER_CTR_VAL);

  /* Set COMP1 value to required value */
  LETIMER_CompareSet(LETIMER0, 1, LETIMER_COMP1_VAL);
}

void LETIMER0InterruptEn()
{
  /* Enable interrupts for UF and COMP1 */
  LETIMER_IntEnable(LETIMER0, (LETIMER_IEN_UF | LETIMER_IEN_COMP1));

  /* Clear any pending IRQ */
  NVIC_ClearPendingIRQ(LETIMER0_IRQn);

  /* Enable Interrupts in the NVIC for LETIMER0 */
  NVIC_EnableIRQ(LETIMER0_IRQn);

  /* Enable the LETIMER0 peripheral */
  LETIMER_Enable(LETIMER0, ENABLE);
}
