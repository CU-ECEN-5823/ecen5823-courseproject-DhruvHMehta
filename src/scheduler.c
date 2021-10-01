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

enum States currentste = IDLE;

void temperatureStateMachine(sl_bt_msg_t *evt)
{
  I2C_TransferReturn_TypeDef I2CTransferReturnStatus;

  if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id)
    {
      /* Event handling */
      switch(currentste)
      {
        /* IDLE State, check if UF event has occurred, power on Si7021 */
        case IDLE:
          if(evt->data.evt_system_external_signal.extsignals == evtLETIMER0_UF)
            {
              currentste = POWERUP;
              powerOnSi7021();
            }
          /* Go to sleep */
          break;

        /* POWERUP State, check if the COMP1 event has occurred, send command and change to EM1 */
        case POWERUP:
          if(evt->data.evt_system_external_signal.extsignals == evtLETIMER0_COMP1)
            {
              currentste = MEASURE;
              sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
              getTemperatureSi7021();
            }
          break;

        /* MEASURE State, check if the I2C0 event has occurred, go to EM3 and wait for conversion */
        case MEASURE:
          if(evt->data.evt_system_external_signal.extsignals == evtI2C0_Complete)
            {
              /* Disable IRQ on successful transfer */
              I2CTransferReturnStatus = getI2CTransferReturn();
              if(I2CTransferReturnStatus == i2cTransferDone)
                NVIC_DisableIRQ(I2C0_IRQn);

              sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

              currentste = WAIT;
              waitConversionTimeSi7021();
            }
          break;

        /* WAIT State, check if the COMP1 event has occurred, go to EM1 and send read command */
        case WAIT:
          if(evt->data.evt_system_external_signal.extsignals == evtLETIMER0_COMP1)
            {
              currentste = REPORT;
              sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
              readTemperatureSi7021();
            }
          break;

          /* REPORT State, check if the I2C0 event has occurred, go to EM3 and LOG Temperature */
          case REPORT:
            if(evt->data.evt_system_external_signal.extsignals == evtI2C0_Complete)
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
}

void schedulerSetEvent_UF()
{
  CORE_CRITICAL_SECTION(sl_bt_external_signal(evtLETIMER0_UF););
}

void schedulerSetEvent_COMP1()
{
  CORE_CRITICAL_SECTION(sl_bt_external_signal(evtLETIMER0_COMP1););
}

void schedulerSetEvent_I2Cdone()
{
  CORE_CRITICAL_SECTION(sl_bt_external_signal(evtI2C0_Complete););
}
