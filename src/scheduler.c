/*
 * scheduler.c
 *
 *  Created on: Sep 14, 2021
 *      Author: Dhruv
 *      Brief : Contains the scheduler to handle events such as scheduling a temperature
 *              sensor acquisition from the Si7021 sensor every 3 seconds.
 */

#include "scheduler.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


#if DEVICE_IS_BLE_SERVER
enum States{IDLE, POWERUP, MEASURE, WAIT, REPORT};
enum Events{evtNone, evtLETIMER0_UF, evtLETIMER0_COMP1, evtI2C0_Complete, evtButtonPressed};
enum States currentste = IDLE;
#else
enum States{OPEN, CHARACTERISTICS, NOTIFY, CLOSE};
enum Events{evtNone, evtOpenConnection, evtGATTComplete, evtConnectionClosed};
enum States currentste = OPEN;
#endif

#if DEVICE_IS_BLE_SERVER
void temperatureStateMachine(sl_bt_msg_t *evt)
{
  /* Variables for I2C Return Check and BLE Data */
  I2C_TransferReturn_TypeDef I2CTransferReturnStatus;
  ble_data_struct_t* ble_data = getBleDataPtr();

  /* Only run the state machine if a connection is established and indications are enabled
   * There is an approx 100ms time period during which a bug could arise if the connection
   * is lost when the below state machine runs. However, this was not produceable in testing */

  if((ble_data->gatt_server_connection != 0 && ble_data->htm_indications_enabled != 0) && \
      (SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id))
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

                  reportTemperatureSi7021();

                currentste = IDLE;
              }
            break;
      }
    }
}

#else
void discovery_state_machine(sl_bt_msg_t *evt)
{
  /* Get the ble data */
  ble_data_struct_t* ble_data = getBleDataPtr();
  int sc;

    /* Event handling */
    switch(currentste)
    {
      /* OPEN State, check if Connection open event has occurred, get the Services */
      case OPEN:
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_opened_id) &&
            ble_data->discoveryEvt == evtOpenConnection)
          {

            /* Discover the primary services */
            sc = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->gatt_server_connection,
                                                              sizeof(ble_data->thermo_service),
                                                              ble_data->thermo_service);

            if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
              }

            currentste = CHARACTERISTICS;
          }
        break;

        /* Characteristics State, check if Services discovery is complete, get the discover characteristics API */
      case CHARACTERISTICS:
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) &&
            ble_data->discoveryEvt == evtGATTComplete)
            {
              /* Discover the characteristics in the HTM Service */
              sc = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->gatt_server_connection,
                                                          ble_data->serviceHandle,
                                                          sizeof(ble_data->thermo_char),
                                                          ble_data->thermo_char);
              if (sc != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                }

              currentste = NOTIFY;
            }
        break;

        /* Notify State, check if Characteristics discovery is complete, send the characteristic notification */
      case NOTIFY:
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) &&
            ble_data->discoveryEvt == evtGATTComplete)
            {
              /* Enable indications for the HTM Thermometer Measurement Characteristic */
              sc = sl_bt_gatt_set_characteristic_notification(ble_data->gatt_server_connection,
                                                              ble_data->characteristicHandle,
                                                              sl_bt_gatt_indication);

              if (sc != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                }

              currentste = CLOSE;
            }
        break;

        /* Close State, if connection closes, start over */
      case CLOSE:
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id) &&
            ble_data->discoveryEvt == evtConnectionClosed)
          {
            currentste = OPEN;
          }
        break;
    }

    /* Make sure the event is cleared */
    ble_data->discoveryEvt = evtNone;
}
#endif

#if DEVICE_IS_BLE_SERVER
/* Events for Temperature State Machine */
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

void schedulerSetEvent_ButtonPressed()
{
  CORE_CRITICAL_SECTION(sl_bt_external_signal(evtButtonPressed););
}
#endif
