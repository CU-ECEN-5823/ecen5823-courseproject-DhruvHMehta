/*
 * scheduler.c
 *
 *  Created on: Sep 14, 2021
 *      Author: Dhruv
 *      Brief : Contains the scheduler to handle events such as scheduling a temperature
 *              sensor acquisition from the Si7021 sensor every 3 seconds.
 */

#include "scheduler.h"

enum States{IDLE, POWERUP, MEASURE, WAIT, REPORT};
enum Events{evtNone, evtLETIMER0_UF, evtLETIMER0_COMP1, evtI2C0_Complete};

enum Events currentevt = evtNone;
enum States currentste = IDLE;

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
  I2C_TransferReturn_TypeDef I2CTransferReturnStatus;

  /* Get the next event */
  evt = getNextEvent();

  /* Event handling */
  switch(currentste)
  {
    /* IDLE State, check if UF event has occurred, power on Si7021 */
    case IDLE:
      if(evt == evtLETIMER0_UF)
        {
          currentste = POWERUP;
          powerOnSi7021();
        }
      /* Go to sleep */
      break;

    /* POWERUP State, check if the COMP1 event has occurred, send command */
    case POWERUP:
      if(evt == evtLETIMER0_COMP1)
        {
          currentste = MEASURE;
          getTemperatureSi7021();
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
        }
      break;

    /* MEASURE State, check if the I2C0 event has occurred, wait for conversion */
    case MEASURE:
      if(evt == evtI2C0_Complete)
        {
          I2CTransferReturnStatus = getI2CTransferReturn();
          if(I2CTransferReturnStatus == i2cTransferDone)
            NVIC_DisableIRQ(I2C0_IRQn);

          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

          currentste = WAIT;
          waitConversionTimeSi7021();
        }
      break;

    /* MEASURE State, check if the I2C0 event has occurred, wait for conversion */
    case WAIT:
      if(evt == evtLETIMER0_COMP1)
        {
          currentste = REPORT;
          readTemperatureSi7021();
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
        }
      break;

      /* MEASURE State, check if the I2C0 event has occurred, wait for conversion */
      case REPORT:
        if(evt == evtI2C0_Complete)
          {
            I2CTransferReturnStatus = getI2CTransferReturn();
            if(I2CTransferReturnStatus == i2cTransferDone)
              NVIC_DisableIRQ(I2C0_IRQn);

            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

            currentste = IDLE;
            reportTemperatureSi7021();
          }
        break;

  }

}

void schedulerSetEvent_UF()
{
  CORE_CRITICAL_SECTION(currentevt = evtLETIMER0_UF;);
}

void schedulerSetEvent_COMP1()
{
  CORE_CRITICAL_SECTION(currentevt = evtLETIMER0_COMP1;);
}

void schedulerSetEvent_I2Cdone()
{
  CORE_CRITICAL_SECTION(currentevt = evtI2C0_Complete;);
}
