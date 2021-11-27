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
enum States{POWERUP, START_CONV, MEASURE};
enum Events{evtNone = 1, evtLETIMER0_UF, evtLETIMER0_COMP1, evtADC0_SINGLE};
enum States currentste = POWERUP;
enum Events evt = evtNone;
#else
enum States{OPEN_S1, OPEN_S2, CHARACTERISTICS_S1, CHARACTERISTICS_S2, NOTIFY_S1, NOTIFY_S2, CLOSE};
enum Events{evtNone, evtOpenConnection, evtGATTComplete, evtConnectionClosed, evtButtonPressed_PB0, evtButtonPressed_PB1};
enum States currentste = OPEN_S1;
#endif

#if DEVICE_IS_BLE_SERVER

void ambientLightStateMachine(sl_bt_msg_t *evt)
{
  uint32_t ambient_analog_val;

  switch(currentste)
  {
    case POWERUP:
      /* Wait for UF event, power up sensor */
      if(evt->data.evt_system_external_signal.extsignals == evtLETIMER0_UF)
      //if(evt == evtLETIMER0_UF)
        {
          /* Power up Sensor */
          gpioAMBSensor(true);
          timerWaitUs_irq(1*1000);
          currentste = START_CONV;
        }
      break;

    case START_CONV:
      /* Wait for 1mS before starting ADC conversion */
      if(evt->data.evt_system_external_signal.extsignals == evtLETIMER0_COMP1)
      //if(evt == evtLETIMER0_COMP1)
        {
          /* Start a Conversion */
          ADC_Start(ADC0, adcStartSingle);
          currentste = MEASURE;
        }
      break;

    case MEASURE:
      /* Start ADC Conversion */
      if(evt->data.evt_system_external_signal.extsignals == evtADC0_SINGLE)
      //if(evt == evtADC0_SINGLE)
        {
          /* Get the converted value */
          ambient_analog_val = ADC_DataSingleGet(ADC0);
          LOG_INFO("ADCval = %d\r\n", ambient_analog_val);

          /* Stub the SendTemperature function with this analog val */
          SendTemperature(ambient_analog_val);

          /* Power down sensor */
          gpioAMBSensor(false);
          currentste = POWERUP;
        }
      break;
  }

  //evt = evtNone;
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
      case OPEN_S1:
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

            currentste = OPEN_S2;
          }
        break;

        /* OPEN State, check if Connection open event has occurred, get the Services */
        case OPEN_S2:
          if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) &&
              ble_data->discoveryEvt == evtGATTComplete)
            {

              /* Discover the primary services */
              sc = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->gatt_server_connection,
                                                                sizeof(ble_data->encrypted_service),
                                                                ble_data->encrypted_service);

              if (sc != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                }

              currentste = CHARACTERISTICS_S1;
            }
          break;

        /* Characteristics State, check if Services discovery is complete, get the discover characteristics API */
      case CHARACTERISTICS_S1:
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) &&
            ble_data->discoveryEvt == evtGATTComplete)
            {
              /* Discover the characteristics in the HTM Service */
              sc = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->gatt_server_connection,
                                                          ble_data->serviceHandle[0],
                                                          sizeof(ble_data->thermo_char),
                                                          ble_data->thermo_char);
              if (sc != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                }

              currentste = CHARACTERISTICS_S2;
            }
        break;

        /* Characteristics State, check if Services discovery is complete, get the discover characteristics API */
      case CHARACTERISTICS_S2:
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) &&
            ble_data->discoveryEvt == evtGATTComplete)
            {
              /* Discover the characteristics in the Btn Service */
              sc = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->gatt_server_connection,
                                                          ble_data->serviceHandle[1],
                                                          sizeof(ble_data->encrypted_char),
                                                          ble_data->encrypted_char);

              if (sc != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                }

              currentste = NOTIFY_S1;
            }
        break;

        /* Notify State, check if Characteristics discovery is complete, send the characteristic notification */
      case NOTIFY_S1:
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) &&
            ble_data->discoveryEvt == evtGATTComplete)
            {
              /* Enable indications for the HTM Thermometer Measurement Characteristic */
              sc = sl_bt_gatt_set_characteristic_notification(ble_data->gatt_server_connection,
                                                              ble_data->characteristicHandle[0],
                                                              sl_bt_gatt_indication);

              displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications");
              if (sc != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                }

              currentste = NOTIFY_S2;
            }
        break;

        /* Notify State, check if Characteristics discovery is complete, send the characteristic notification */
      case NOTIFY_S2:
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) &&
            ble_data->discoveryEvt == evtGATTComplete)
            {
              /* Enable indications for the Btn_State Characteristic */
              sc = sl_bt_gatt_set_characteristic_notification(ble_data->gatt_server_connection,
                                                              ble_data->characteristicHandle[1],
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
            currentste = OPEN_S1;
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
  //evt = evtLETIMER0_UF;
}

void schedulerSetEvent_COMP1()
{
  CORE_CRITICAL_SECTION(sl_bt_external_signal(evtLETIMER0_COMP1););
  //evt = evtLETIMER0_COMP1;
}

void schedulerSetEvent_ADC0_Single()
{
  CORE_CRITICAL_SECTION(sl_bt_external_signal(evtADC0_SINGLE););
  //evt = evtADC0_SINGLE;
}

void schedulerSetEvent_I2Cdone()
{
  ;
}

void schedulerSetEvent_ButtonPressed()
{
  ;
}
#else
void schedulerSetEvent_ButtonPressed_PB0()
{
  CORE_CRITICAL_SECTION(sl_bt_external_signal(evtButtonPressed_PB0););
}
void schedulerSetEvent_ButtonPressed_PB1()
{
  CORE_CRITICAL_SECTION(sl_bt_external_signal(evtButtonPressed_PB1););
}
#endif
