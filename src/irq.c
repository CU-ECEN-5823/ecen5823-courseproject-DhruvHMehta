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
 *   Interrupt handler which sets events based on the interrupt.
 *
 * @param[in] osc
 *   none
 *
 * @return void
 ******************************************************************************/
void LETIMER0_IRQHandler()
{
  //sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);

  /* Check which IF is set */
  uint32_t flags = LETIMER_IntGetEnabled(LETIMER0);

  /* Clear the interrupt */
  LETIMER_IntClear(LETIMER0, flags);

  /* Set the UF Event */
  if(flags == LETIMER_IEN_UF)
    schedulerSetEvent_UF();

  //sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM2);
}
