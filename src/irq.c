/*
 * irq.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 *      Brief : Contains all IRQ Handlers.
 */
#include "irq.h"

/***************************************************************************//**
 * @name LETIMER0_IRQHandler
 *
 * @brief
 *   Interrupt handler which turns on LED0 on COMP1 interrupt and turns off
 *   LED0 on UF interrupt of LETIMER0.
 *
 * @param[in] osc
 *   none
 *
 * @return void
 ******************************************************************************/
void LETIMER0_IRQHandler()
{
  /* Check which IF is set */
  uint32_t flags = LETIMER_IntGetEnabled(LETIMER0);

  /* Clear the interrupt */
  LETIMER_IntClear(LETIMER0, flags);

  /* Turn on the LED if COMP1 interrupt is set */
  if(flags == LETIMER_IEN_COMP1)
    gpioLed0SetOn();

  /* Turn off the LED if UF interrupt is set */
  else if(flags == LETIMER_IEN_UF)
    gpioLed0SetOff();
}
