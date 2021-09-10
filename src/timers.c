/*
 * timers.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 */

#include "timers.h"

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
