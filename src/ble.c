/*
 * ble.c
 *
 *  Created on: Sep 30, 2021
 *      Author: Dhruv
 *      Reference : Used Bluetooth (SoC) Thermometer Example Code
 *      for handle_ble_events.
 *      Professor David Sluiter - Used gattFloat32ToInt function
 *      to convert from IEEE-11073 32-bit float to integer.
 *
 */
#include "src/ble.h"
#include "app_log.h"
#include "app_assert.h"
#include <math.h>
#include <stdint.h>

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


#if DEVICE_IS_BLE_SERVER
#define SIZE (16)           /* Size of Buffer */
#define EXTSIGEVENT 5

struct buffer_data{
  uint16_t charHandle;      /* Characteristic handle from GATTdb */
  size_t   bufferLength;    /* Length of buffer in bytes to send */
  uint8_t  buffer[5];       /* Actual buffer size. 5 bytes for htm and 2 for btn */
};

typedef struct{
struct buffer_data Data[SIZE];    /* Buffer */
uint8_t wrloc;                    /* Write Location (where to write next) */
uint8_t rdloc;                    /* Read Location (where to read from next) */
uint8_t isFull;                   /* Flag to indicate buffer full */
}Buffer;

Buffer queue = {.isFull = 0, .rdloc = 0, .wrloc = 0};

// The advertising set handle allocated from Bluetooth stack.
static ble_data_struct_t ble_data = {.advertisingSetHandle = 0xff, .amb_indications_enabled = 0,
                                     .btn_indications_enabled = 0, .gatt_server_connection = 0,
                                     .bonding_state = 0};
struct buffer_data indication_data;
#else
static ble_data_struct_t ble_data = {.amb_indications_enabled = 0, .gatt_server_connection = 0,

                                     .ambient_service = {0x34, 0x23, 0x41, 0xed, 0xd7, 0xd7,
                                     0xea, 0x87, 0x3e, 0x41, 0x9b, 0x0f, 0xf2, 0x67, 0x21, 0x39},

                                     .ambient_char = {0xf8, 0x63, 0x5e, 0x5c, 0x2c, 0x42, 0x5e,
                                     0xbc, 0xba, 0x45, 0x1f, 0xa5, 0x84, 0xdb, 0x15, 0xd9},

                                     .gesture_service = {0xfa, 0xb2, 0x71, 0x76, 0x21, 0x00,
                                     0x8f, 0x91, 0xf2, 0x4b, 0x4b, 0x9a, 0x02, 0x7d, 0x9d, 0xb7},

                                     .gesture_char = {0xa5, 0xd1, 0xab, 0x3f, 0xb9, 0xe7,
                                     0x95, 0xbd, 0x81, 0x4f, 0x99, 0xe5, 0x4f, 0x05, 0x91, 0x57},

                                     .bonding_state = 0};

/* Initial sensor value to not send the client into low power */
uint16_t ambient_lt_val = 4095;
uint8_t gesture_val = 0;

#define PASSIVE_SCANNING 0
#define SCAN_INTERVAL    80
#define SCAN_WINDOW      40
#define CONNECTION_OPEN  1
#define GATT_COMPLETE    2
#define CONNECTION_CLOSED 3
#define SERVICE          0
#define CHARACTERISTIC   1
#endif

#define EXTSIGEVENT_PB0 4
#define EXTSIGEVENT_PB1 5
#define SM_CONFIG_FLAGS 0x0F
#define MIN_MAX_INTERVAL  60
#define SLAVE_LATENCY     3
#define SLAVE_TIMEOUT     75

ble_data_struct_t* getBleDataPtr()
{
  return(&ble_data);
}

#if DEVICE_IS_BLE_SERVER
int cbfifo_enqueue(Buffer* Cbfifo, struct buffer_data *buf)
{
    if(buf == NULL || Cbfifo == NULL)
        return -1;                              /* Input buffer is NULL */

    if((Cbfifo->wrloc == Cbfifo->rdloc) && (Cbfifo->isFull == 1))
      return -1;

    Cbfifo->Data[Cbfifo->wrloc] = *buf;        /* Write to buffer */
    Cbfifo->wrloc = (Cbfifo->wrloc + 1) & (SIZE - 1); /* Wrapped addition of wrloc */

    /* If ptrs match, buffer is full */
    if(Cbfifo->wrloc == Cbfifo->rdloc)
        Cbfifo->isFull = 1;

    return 0;
}

int cbfifo_dequeue(Buffer* Cbfifo, struct buffer_data *buf)
{
    if(buf == NULL || Cbfifo == NULL)
        return -1;                                /* Input buffer is NULL */

    if((Cbfifo->rdloc == Cbfifo->wrloc) && (Cbfifo->isFull == 0))
      return -1;

    *buf = Cbfifo->Data[Cbfifo->rdloc];        /* Read from buffer */
    Cbfifo->rdloc = (Cbfifo->rdloc + 1) & (SIZE - 1);  /* Wrapped addition of rdloc */

    if(Cbfifo->isFull == 1)
      Cbfifo->isFull = 0;

    return 0;
}

size_t cbfifo_length(Buffer* Cbfifo)
{
  if(Cbfifo->isFull == 0)
      return ((Cbfifo->wrloc - Cbfifo->rdloc) & (SIZE - 1));

  else return 1;
}
#else
/*******************************************************************************
 * @name check_slave_addr
 * @brief Checks if the address obtained from the current advertising packet
 *        is from a recognized server.
 *
 * @param sl_bt_msg_t *evt - Current BLE Stack event
 * @return uint8_t Address match result = 1 if address matches, 0 if not
 *
 *******************************************************************************/
static uint8_t check_slave_addr(sl_bt_msg_t *evt)
{
  /* Get the address from the macro and save it */
  uint8_t slave_addr_check = 0;
  bd_addr ActualSlaveAddr = SERVER_BT_ADDRESS;
  ble_data.serverAddress = ActualSlaveAddr;

  /* Check each address byte */
  for(int i = 0; i < 6; i++)
    {
      if(evt->data.evt_scanner_scan_report.address.addr[i] == ActualSlaveAddr.addr[i])
        {
          slave_addr_check++;
        }

    }

  /* Address matched, return 1 = success */
  if(slave_addr_check == 6)
    {
      return 1;
    }

  /* Address did not match, return 0 = fail */
  return 0;
}

static uint8_t UUID_Compare(sl_bt_msg_t *evt, uint8_t evttype, uint8_t sensortype)
{
  /* Get the address from the macro and save it */
  uint8_t uuid_check_counter = 0;

  /* Check each address byte */
  for(int i = 0; i < 16; i++)
    {
      if(evttype == SERVICE)
        {
          if(sensortype == AMBIENT)
            {
              if(evt->data.evt_gatt_service.uuid.data[i] == ble_data.ambient_service[i])
                {
                  uuid_check_counter++;
                }
            }

          else if(sensortype == GESTURE_SNSR)
            {
              if(evt->data.evt_gatt_service.uuid.data[i] == ble_data.gesture_service[i])
                {
                  uuid_check_counter++;
                }
            }
        }
      else if(evttype == CHARACTERISTIC)
        {
          if(sensortype == AMBIENT)
            {
              if(evt->data.evt_gatt_characteristic.uuid.data[i] == ble_data.ambient_char[i])
              {
                uuid_check_counter++;
              }
            }

          else if(sensortype == GESTURE_SNSR)
            {
              if(evt->data.evt_gatt_characteristic.uuid.data[i] == ble_data.gesture_char[i])
                {
                  uuid_check_counter++;
                }
            }
        }
    }

  /* Address matched, return 1 = success */
  if(uuid_check_counter == 16)
    {
      return 1;
    }

  /* Address did not match, return 0 = fail */
  return 0;
}

uint16_t getSensorValue(uint8_t sensortype)
{
  uint8_t returnval;

  if(sensortype == AMBIENT)
      return ambient_lt_val;

  else if(sensortype == GESTURE_SNSR)
    {
      returnval = gesture_val;
      gesture_val = 0;
    }

  return returnval;
}

#endif

void handle_ble_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;
    uint8_t address_type;
#if DEVICE_IS_BLE_SERVER
#else
    uint8_t slave_addr_match = 0;
#endif
    // Handle stack events
    switch (SL_BT_MSG_ID(evt->header)) {

/*******************************************************************************
 * Events Common to Client and Server
 *
 ******************************************************************************/

      // This event indicates the device has started and the radio is ready.
      // Do not call any stack command before receiving this boot event!
      /* Boot Event */
      case sl_bt_evt_system_boot_id:

        // Extract unique ID from BT Address.
        sc = sl_bt_system_get_identity_address(&ble_data.myAddress, &address_type);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

#if DEVICE_IS_BLE_SERVER
        // SERVER
        // Create an advertising set.
        sc = sl_bt_advertiser_create_set(&(ble_data.advertisingSetHandle));

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_create_set() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        // Set advertising interval to 250ms.
        sc = sl_bt_advertiser_set_timing(
          ble_data.advertisingSetHandle, // advertising set handle
          400,                           // min. adv. interval (milliseconds * 1.6)
          400,                           // max. adv. interval (milliseconds * 1.6)
          0,                             // adv. duration
          0);                            // max. num. adv. events

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_set_timing() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        // Start general advertising and enable connections.
        sc = sl_bt_advertiser_start(
          ble_data.advertisingSetHandle,
          sl_bt_advertiser_general_discoverable,
          sl_bt_advertiser_connectable_scannable);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_set_timing() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        //LOG_INFO("Started advertising\r\n");

        sc = sl_bt_system_set_soft_timer(4096, 1, 0);

        if (sc != SL_STATUS_OK)
         {
            LOG_ERROR("sl_bt_system_set_soft_timer() returned != 0 status=0x%04x", (unsigned int) sc);
         }

        gpioLed0SetOff();
        gpioLed1SetOff();
#else
        // CLIENT

        /* Use Passive Scanning */
        sc = sl_bt_scanner_set_mode(sl_bt_gap_phy_1m, PASSIVE_SCANNING);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_set_mode() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /* Scan Interval = 50mS
         * Scan Window = 25mS
         */
        sc = sl_bt_scanner_set_timing(sl_bt_gap_phy_1m, SCAN_INTERVAL, SCAN_WINDOW);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_set_timing() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /* Set default connection parameters */
        sc = sl_bt_connection_set_default_parameters(MIN_MAX_INTERVAL, MIN_MAX_INTERVAL, SLAVE_LATENCY,
                                                     SLAVE_TIMEOUT, 0, 0xFFFF);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_connection_set_default_parameters() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /* Start scanning for all devices */
        sc = sl_bt_scanner_start(sl_bt_gap_phy_1m, sl_bt_scanner_discover_observation);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x", (unsigned int) sc);
          }

#endif
        /* Configure the Security Manager for MITM protection and I/O displayyesno capability */
        sc = sl_bt_sm_configure(SM_CONFIG_FLAGS, sl_bt_sm_io_capability_displayyesno);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_sm_configure() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        /* Delete previous bonding information */
        sc = sl_bt_sm_delete_bondings();

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        // Initializing the display and printing my BT Address
        displayInit();
        displayPrintf(DISPLAY_ROW_NAME, BLE_DEVICE_TYPE_STRING);
        displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X", ble_data.myAddress.addr[0], \
                      ble_data.myAddress.addr[1], ble_data.myAddress.addr[2], \
                      ble_data.myAddress.addr[3], ble_data.myAddress.addr[4], \
                      ble_data.myAddress.addr[5]);

        displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Course Project");
#if DEVICE_IS_BLE_SERVER
        displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
#else
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
#endif
        break;

      // -------------------------------
      // This event indicates that a new connection was opened.
      /* Connection Opened Event */
      case sl_bt_evt_connection_opened_id:

#if DEVICE_IS_BLE_SERVER
        // SERVER
        //LOG_INFO("Connection opened\r\n");
        /* Stop advertising */
        sc = sl_bt_advertiser_stop(ble_data.advertisingSetHandle);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_stop() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        /* Get the connection handle */
        ble_data.gatt_server_connection = evt->data.evt_connection_opened.connection;

        /* Set the connection parameters */
        sc = sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection, MIN_MAX_INTERVAL,
                                             MIN_MAX_INTERVAL, SLAVE_LATENCY, SLAVE_TIMEOUT, 0, 0xFFFF);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_connection_set_parameters() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        /* Display Connected on the LCD */
        displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");

#else
        // CLIENT
        /* Get the connection handle and save it, set connection opened event  */
        ble_data.gatt_server_connection = evt->data.evt_connection_opened.connection;
        ble_data.discoveryEvt = CONNECTION_OPEN;

        /* Display Connected on the LCD and Server Address */
        displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
        displayPrintf(DISPLAY_ROW_BTADDR2, "%02X:%02X:%02X:%02X:%02X:%02X", ble_data.serverAddress.addr[0], \
                      ble_data.serverAddress.addr[1], ble_data.serverAddress.addr[2], \
                      ble_data.serverAddress.addr[3], ble_data.serverAddress.addr[4], \
                      ble_data.serverAddress.addr[5]);

#endif
        break;
      // -------------------------------
      // This event indicates that a connection was closed.
      /* Connection Close Event */
      case sl_bt_evt_connection_closed_id:

#if DEVICE_IS_BLE_SERVER
        // SERVER
        /* Reset the connection handle and the indication bool */
        ble_data.gatt_server_connection = 0;
        ble_data.amb_indications_enabled = 0;
        ble_data.gesture_indications_enabled = 0;
        ble_data.in_flight = 0;

        //LOG_INFO("Connection closed\r\n");

        // Restart advertising after client has disconnected.
        sc = sl_bt_advertiser_start(
          ble_data.advertisingSetHandle,
          sl_bt_advertiser_general_discoverable,
          sl_bt_advertiser_connectable_scannable);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        //LOG_INFO("Started advertising\r\n");

        displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");

        /* Display Advertising on the LCD */
        displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");

        gpioLed0SetOff();
        gpioLed1SetOff();
#else
        //CLIENT
        /* Start scanning for new devices */
        sc = sl_bt_scanner_start(sl_bt_gap_phy_1m, sl_bt_scanner_discover_observation);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /* Set the connection closed event */
        ble_data.discoveryEvt = CONNECTION_CLOSED;

        /* Display Discovering on the LCD */
        displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
        displayPrintf(DISPLAY_ROW_BTADDR2, " ");
        displayPrintf(DISPLAY_ROW_9, " ");
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
#endif

        /* Delete any existing bondings */
        sc = sl_bt_sm_delete_bondings();

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }

        ble_data.bonding_state = NOT_BONDED;
        break;

      /* Connnection Parameters Event */
      case sl_bt_evt_connection_parameters_id:

        /* Common to SERVER and CLIENT */
        LOG_INFO("Interval = %d\r\n Latency = %d\r\n Timeout = %d\r\n",
                  (int)((evt->data.evt_connection_parameters.interval)*1.25),
                  (int)(evt->data.evt_connection_parameters.latency),
                  (int)((evt->data.evt_connection_parameters.timeout)*10));

        break;

      /* 1s Timer event for LCD EXTcomin */
      case sl_bt_evt_system_soft_timer_id:

        /* Common to SERVER and CLIENT */
        if(evt->data.evt_system_soft_timer.handle == 0)
          displayUpdate();

#if DEVICE_IS_BLE_SERVER
    // SERVER
        else if(evt->data.evt_system_soft_timer.handle == 1)
          {
            /* No indications in flight */
            if((cbfifo_length(&queue) > 0) && ble_data.in_flight == 0)
              {
                if((cbfifo_dequeue(&queue, &indication_data)) == 0)
                  {
                    // -------------------------------// Write our local GATT DB// -------------------------------
                    sc = sl_bt_gatt_server_write_attribute_value(
                        indication_data.charHandle, // handle from gatt_db.h
                        0,                              // offset
                        indication_data.bufferLength,   // length
                        indication_data.buffer      // pointer to buffer where data is
                    );

                    if (sc != SL_STATUS_OK)
                      {
                        LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                      }

                    sc = sl_bt_gatt_server_send_indication(ble_data.gatt_server_connection,
                                                           indication_data.charHandle,
                                                           indication_data.bufferLength,
                                                           indication_data.buffer);

                    ble_data.in_flight = 1;
                    if (sc != SL_STATUS_OK)
                      {
                        LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                        ble_data.in_flight = 0;
                      }
                  }
              }
          }
#endif
        break;

      /* GATT Server Indication Timeout Event */
      case sl_bt_evt_gatt_server_indication_timeout_id:
        break;

/*******************************************************************************
 *
 * Events only for SERVER
 *
 *******************************************************************************/
#if DEVICE_IS_BLE_SERVER
      /* GATT Server Characteristic Status ID Event */
      case sl_bt_evt_gatt_server_characteristic_status_id:

       if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_light_analog_value)
        {

           /* Indications have been turned on */
          if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config &&
              evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
            {
              /* Start reading temperature */
              ble_data.amb_indications_enabled = 1;
              gpioLed0SetOn();
            }

          /* Received confirmation from client */
          if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation &&
              evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
            {
              ble_data.in_flight = 0;
            }

          /* Indications have been turned off */
          else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)
            {
                displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
                ble_data.amb_indications_enabled = 0;
                gpioLed0SetOff();
            }
        }

       else if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_gesture)
         {

           /* Indications for button state have been turned on */
           if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config &&
               evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
             {
               /* Read button state */
               ble_data.gesture_indications_enabled = 1;
               gpioLed1SetOn();
             }

           /* Received confirmation from client */
           if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation &&
               evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
             {
               ble_data.in_flight = 0;
             }

           /* Indications have been turned off */
           else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)
             {
                 ble_data.gesture_indications_enabled = 0;
                 gpioLed1SetOff();
             }
         }
       break;

       /* Button Press Event */
      case sl_bt_evt_system_external_signal_id:

        /* Button pressed */
        if(evt->data.evt_system_external_signal.extsignals == EXTSIGEVENT)
          {
            /* Pairing request received */
            if(ble_data.bonding_state == BONDING)
              {
                displayPrintf(DISPLAY_ROW_PASSKEY, " ");
                displayPrintf(DISPLAY_ROW_ACTION, " ");
                sc = sl_bt_sm_passkey_confirm(ble_data.gatt_server_connection, 1);

                if (sc != SL_STATUS_OK)
                  {
                    LOG_ERROR("sl_bt_sm_passkey_confirm() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                  }
              }
          }
        break;

        /* Bonding Confirm Event */
      case sl_bt_evt_sm_confirm_bonding_id:

        /* Accept the bonding request from the client */
        sc = sl_bt_sm_bonding_confirm(ble_data.gatt_server_connection, 1);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_sm_bonding_confirm() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
          }
        break;

        /* Bonding Failed. Error message logged */
      case sl_bt_evt_sm_bonding_failed_id:

        /* Log error if bonding failed */
        LOG_ERROR("Bonding request for connection %d failed, reason = %d\r\n", evt->data.evt_sm_bonding_failed.connection,
                  evt->data.evt_sm_bonding_failed.reason);
        break;

        /* Confirm passkey event. Wait for user-button press */
      case sl_bt_evt_sm_confirm_passkey_id:

        if(ble_data.gatt_server_connection == evt->data.evt_sm_confirm_passkey.connection)
          {
            displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey %d", evt->data.evt_sm_confirm_passkey.passkey);
            displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");

            /* Pairing in process */
            ble_data.bonding_state = BONDING;
          }
        break;

        /* Bonded with client */
      case sl_bt_evt_sm_bonded_id:
        /* Bonded */
        ble_data.bonding_state = BONDED;
        displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
        break;

#else
/*******************************************************************************
 *
 * Events only for CLIENT
 *
 *******************************************************************************/

     /* Event Scanner Scan Report Event */
      case sl_bt_evt_scanner_scan_report_id:
        // CLIENT

        slave_addr_match = check_slave_addr(evt);

        if(slave_addr_match)
          {
            /* Server found, stop scanning for more advertising packets */
            sc = sl_bt_scanner_stop();

            if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_scanner_stop() returned != 0 status=0x%04x", (unsigned int) sc);
              }

            /* Connect to the Server */
            sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                       evt->data.evt_scanner_scan_report.address_type,
                                       sl_bt_gap_phy_1m, NULL);

            if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_connection_open() returned != 0 status=0x%04x", (unsigned int) sc);
              }

          }
        break;

      case sl_bt_evt_gatt_service_id:
        // CLIENT
        /* Save the newly discovered service's handle */
        if(UUID_Compare(evt, SERVICE, AMBIENT))
            ble_data.serviceHandle[0] = evt->data.evt_gatt_service.service;

        else if(UUID_Compare(evt, SERVICE, GESTURE_SNSR))
            ble_data.serviceHandle[1] = evt->data.evt_gatt_service.service;

        break;

      case sl_bt_evt_gatt_characteristic_id:
        // CLIENT
        /* Save the newly discovered characteristics' handle */
        if(UUID_Compare(evt, CHARACTERISTIC, AMBIENT))
            ble_data.characteristicHandle[0] = evt->data.evt_gatt_characteristic.characteristic;

        else if(UUID_Compare(evt, CHARACTERISTIC, GESTURE_SNSR))
            ble_data.characteristicHandle[1] = evt->data.evt_gatt_characteristic.characteristic;

        break;

      case sl_bt_evt_gatt_characteristic_value_id:
        //CLIENT
        /* Check if the att_opcode and gatt characteristic handle match */
        if(evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication &&
            evt->data.evt_gatt_characteristic_value.characteristic == ble_data.characteristicHandle[0])
          {
            sc = sl_bt_gatt_send_characteristic_confirmation(ble_data.gatt_server_connection);

            if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() returned != 0 status=0x%04x", (unsigned int) sc);
              }

            ambient_lt_val = (evt->data.evt_gatt_characteristic_value.value.data[0] |
                             (evt->data.evt_gatt_characteristic_value.value.data[1] << 8));

            /* Display Received Ambient Light Value on the LCD for the Client */
            //displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Light=%d", ambient_lt_val);

            LOG_INFO("Received Light value = %d\r\n", ambient_lt_val);
          }

        /* If it is an indication or a read response for btn state, display it */
        else if((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication ||
                 evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_read_response) &&
                 evt->data.evt_gatt_characteristic_value.characteristic == ble_data.characteristicHandle[1])
          {
            /* Send confirmation only if it is an indication */
            if(evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
              {
                sc = sl_bt_gatt_send_characteristic_confirmation(ble_data.gatt_server_connection);

                if (sc != SL_STATUS_OK)
                  {
                    LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() returned != 0 status=0x%04x", (unsigned int) sc);
                  }
              }
            gesture_val = evt->data.evt_gatt_characteristic_value.value.data[0];
            LOG_INFO("Gesture received = %d\r\n", gesture_val);
          }
        displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Light=%d Gesture=%d", ambient_lt_val, gesture_val);
        break;

      case sl_bt_evt_gatt_procedure_completed_id:
        // CLIENT
        /* Check which GATT procedure was completed */
        /* If Discover Services by UUID was completed, check return status and set evtGattComplete */
        if(evt->data.evt_gatt_procedure_completed.result == 0)
          {
            /* Set the GATT completed event */
            ble_data.discoveryEvt = GATT_COMPLETE;
          }

        else
          {
            LOG_ERROR("sl_bt_evt_gatt_procedure_completed_id() returned != 0 status=0x%04x\r\n",
                       (unsigned int) evt->data.evt_gatt_procedure_completed.result);

            /* Increase the security level and initiate passkey bonding */
            if(evt->data.evt_gatt_procedure_completed.result == ((sl_status_t) SL_STATUS_BT_ATT_INSUFFICIENT_ENCRYPTION))
              {
                sc = sl_bt_sm_increase_security(ble_data.gatt_server_connection);

                if (sc != SL_STATUS_OK)
                  {
                    LOG_ERROR("sl_bt_sm_increase_security() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                  }
              }
          }
        break;

        /* Button Press Event */
       case sl_bt_evt_system_external_signal_id:
         if(evt->data.evt_system_external_signal.extsignals == EXTSIGEVENT_PB1)
           {
             /* PB0 is not pressed, send a read characteristic value */
             if(GPIO_PinInGet(gpioPortF, PB0_pin) == 1)
               {
                 sc = sl_bt_gatt_read_characteristic_value(ble_data.gatt_server_connection, ble_data.characteristicHandle[1]);

                 if (sc != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_read_characteristic_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                   }
               }
           }


         else if(evt->data.evt_system_external_signal.extsignals == EXTSIGEVENT_PB0)
           {
             /* Pairing request received */
             if(ble_data.bonding_state == BONDING)
               {
                 displayPrintf(DISPLAY_ROW_PASSKEY, " ");
                 displayPrintf(DISPLAY_ROW_ACTION, " ");
                 sc = sl_bt_sm_passkey_confirm(ble_data.gatt_server_connection, 1);

                 if (sc != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_sm_passkey_confirm() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                   }

                 else ble_data.bonding_state = BONDED;
               }
           }
         break;

         /* Bonding Failed. Error message logged */
       case sl_bt_evt_sm_bonding_failed_id:

         ble_data.bonding_state = NOT_BONDED;
         /* Log error if bonding failed */
         LOG_ERROR("Bonding request for connection %d failed, reason = %d\r\n", evt->data.evt_sm_bonding_failed.connection,
                   evt->data.evt_sm_bonding_failed.reason);
         break;

         /* Confirm passkey event. Wait for user-button press */
       case sl_bt_evt_sm_confirm_passkey_id:

         if(ble_data.gatt_server_connection == evt->data.evt_sm_confirm_passkey.connection)
           {
             displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey %d", evt->data.evt_sm_confirm_passkey.passkey);
             displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");

             /* Pairing in process */
             ble_data.bonding_state = BONDING;
           }
         break;

         /* Bonded with client */
       case sl_bt_evt_sm_bonded_id:
         /* Bonded */

         //displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
         break;
#endif

      // -------------------------------
      // Default event handler.
      default:
        break;
    }
}

#if DEVICE_IS_BLE_SERVER
void SendLightValue(uint16_t analog_val)
{

  // -------------------------------------------------------------------
  // Update our local GATT DB and send indication if enabled for the characteristic
  // -------------------------------------------------------------------

  /* Write attribute and send indications only if indications are enabled and connection is maintained */
  if(ble_data.amb_indications_enabled == 1 && ble_data.gatt_server_connection != 0 && ble_data.in_flight == 0)
    {

      // -------------------------------// Write our local GATT DB// -------------------------------
      uint32_t sc = sl_bt_gatt_server_write_attribute_value(
          gattdb_light_analog_value, // handle from gatt_db.h
          0,                              // offset
          2,                              // length
          &analog_val                     // pointer to buffer where data is
      );

        {
        /*
          if (sc != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
            }
        */
          sc = sl_bt_gatt_server_send_indication(ble_data.gatt_server_connection,
                                                 gattdb_light_analog_value,
                                                 2,
                                                 &analog_val);

          ble_data.in_flight = 1;
          if (sc != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
              ble_data.in_flight = 0;
            }

          /* Update Temperature on the LCD */
          displayPrintf(DISPLAY_ROW_TEMPVALUE, "Ambient Light=%d", analog_val);
        }

    }

  else if ((ble_data.gatt_server_connection != 0) && (ble_data.amb_indications_enabled == 1) &&
          (ble_data.in_flight == 1))
    {
      indication_data.charHandle   = gattdb_light_analog_value;
      indication_data.bufferLength = 2;
      memcpy(indication_data.buffer, &analog_val, 2);

      if((cbfifo_enqueue(&queue, &indication_data)) == -1)
        LOG_ERROR("Queue full, discarding event\r\n");
    }

}

/*Send Gesture value to the Client*/
void send_gesture_value(uint8_t gesture_val)
{

  // -------------------------------------------------------------------
  // Update our local GATT DB and send indication if enabled for the characteristic
  // -------------------------------------------------------------------

  /* Write attribute and send indications only if indications are enabled and connection is maintained */
  if(ble_data.gesture_indications_enabled == 1 && ble_data.gatt_server_connection != 0 && ble_data.in_flight == 0)
    {

      // -------------------------------// Write our local GATT DB// -------------------------------
      uint32_t sc = sl_bt_gatt_server_write_attribute_value(
          gattdb_gesture, // handle from gatt_db.h
          0,                              // offset
          1,                              // length
          &gesture_val                     // pointer to buffer where data is
      );

        {

          if (sc != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
            }

          sc = sl_bt_gatt_server_send_indication(ble_data.gatt_server_connection,
                                                 gattdb_gesture,
                                                 1,
                                                 &gesture_val);

          ble_data.in_flight = 1;
          if (sc != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
              ble_data.in_flight = 0;
            }

          /* Update Temperature on the LCD */
          displayPrintf(DISPLAY_ROW_8, "Gesture_Value = %d", gesture_val);
        }

    }

  else if ((ble_data.gatt_server_connection != 0) && (ble_data.gesture_indications_enabled == 1) &&
          (ble_data.in_flight == 1))
    {
      indication_data.charHandle   = gattdb_gesture;
      indication_data.bufferLength = 1;
      memcpy(indication_data.buffer, &gesture_val, 1);

      if((cbfifo_enqueue(&queue, &indication_data)) == -1)
        LOG_ERROR("Queue full, discarding event\r\n");
    }

}
#endif
