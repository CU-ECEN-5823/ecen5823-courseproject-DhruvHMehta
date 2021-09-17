/*
 * scheduler.c
 *
 *  Created on: Sep 14, 2021
 *      Author: Dhruv
 *      Brief : Contains the scheduler to handle events such as scheduling a temperature
 *              sensor acquisition from the Si7021 sensor every 3 seconds.
 */

#include "scheduler.h"

enum Events{evtNone, evtLETIMER0_UF};
enum Events currentevt;

uint32_t getNextEvent()
{
  uint32_t evt;

  /* Get the current event status */
  evt = currentevt;

  /* Clear the current event */
  CORE_CRITICAL_SECTION(currentevt = evtNone;);

  return evt;
}

void app_process_action()
{
  uint32_t evt;

  /* Get the next event */
  evt = getNextEvent();

  /* Event handling */
  switch(evt)
  {
    /* LETIMER0 has underflowed, acquire temperature value */
    case evtLETIMER0_UF:
      getTemperatureSi7021();
      break;

    /* No events, set the MCU to a low power state */
    case evtNone:
      /* Go to sleep */
      break;
  }

}

void schedulerSetEvent_UF()
{
  CORE_CRITICAL_SECTION(currentevt = evtLETIMER0_UF;);
}



