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
#define EXTSIGEVENT 4

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
static ble_data_struct_t ble_data = {.advertisingSetHandle = 0xff, .htm_indications_enabled = 0,
                                     .btn_indications_enabled = 0, .gatt_server_connection = 0,
                                     .bonding_state = 0};
struct buffer_data indication_data;
#else
static ble_data_struct_t ble_data = {.amb_indications_enabled = 0, .gatt_server_connection = 0,

                                     .ambient_service = {0x34, 0x23, 0x41, 0xed, 0xd7, 0xd7,
                                     0xea, 0x87, 0x3e, 0x41, 0x9b, 0x0f, 0xf2, 0x67, 0x21, 0x39},

                                     .ambient_char = {0xf8, 0x63, 0x5e, 0x5c, 0x2c, 0x42, 0x5e,
                                     0xbc, 0xba, 0x45, 0x1f, 0xa5, 0x84, 0xdb, 0x15, 0xd9},

                                     .encrypted_service = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65,
                                     0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00, 0x00},

                                     .encrypted_char = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65,
                                     0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00, 0x00},

                                     .bonding_state = 0, .btn_indications_enabled = sl_bt_gatt_indication};

/* Initial sensor value to not send the client into low power */
uint16_t ambient_lt_val = 4095;

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
#define NOT_BONDED  0
#define BONDING     1
#define BONDED      2
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

static uint8_t UUID_Compare(sl_bt_msg_t *evt, uint8_t evttype)
{
  /* Get the address from the macro and save it */
  uint8_t uuid_check_counter = 0;

  /* Check each address byte */
  for(int i = 0; i < 16; i++)
    {
      if(evttype == SERVICE)
        {
          if(evt->data.evt_gatt_service.uuid.data[i] == ble_data.ambient_service[i])
          {
            uuid_check_counter++;
          }
        }
      else if(evttype == CHARACTERISTIC)
        {
          if(evt->data.evt_gatt_characteristic.uuid.data[i] == ble_data.ambient_char[i])
          {
            uuid_check_counter++;
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

// Original code from Dan Walkes. I (Sluiter) fixed a sign extension bug with the mantissa.
// convert IEEE-11073 32-bit float to integer
static int32_t gattFloat32ToInt(const uint8_t *value_start_little_endian)
{
  uint8_t     signByte = 0;
  int32_t     mantissa;
  // data format pointed at by value_start_little_endian is:
  // [0]       = contains the flags byte
  // [3][2][1] = mantissa (24-bit 2’s complement)
  // [4]       = exponent (8-bit 2’s complement)
  int8_t exponent = (int8_t)value_start_little_endian[4];

  // sign extend the mantissa value if the mantissa is negative
  if (value_start_little_endian[3] & 0x80)
    { // msb of [3] is the sign of the mantissa
      signByte = 0xFF;
    }

  mantissa = (int32_t) (value_start_little_endian[1]  << 0)  |
                       (value_start_little_endian[2]  << 8)  |
                       (value_start_little_endian[3]  << 16) |
                       (signByte                      << 24) ;

  // value = 10^exponent * mantissa, pow() returns a double type
  return (int32_t) (pow(10, exponent) * mantissa);

} // gattFloat32ToInt

uint16_t getSensorValue(uint8_t sensortype)
{
  if(sensortype == AMBIENT)
    {
      return ambient_lt_val;
    }
}

#endif

void handle_ble_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;
    uint8_t address_type;
#if DEVICE_IS_BLE_SERVER
    uint8_t button_val = 0;
#else
    uint8_t slave_addr_match = 0;
    uint8_t client_btn_state;
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

        displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A9");
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
        ble_data.htm_indications_enabled = 0;
        ble_data.btn_indications_enabled = 0;
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

       if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
        {

           /* Indications have been turned on */
          if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config &&
              evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
            {
              /* Start reading temperature */
              ble_data.htm_indications_enabled = 1;
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
                ble_data.htm_indications_enabled = 0;
                gpioLed0SetOff();
            }
        }

       else if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_button_state)
         {

           /* Indications for button state have been turned on */
           if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config &&
               evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
             {
               /* Read button state */
               ble_data.btn_indications_enabled = 1;
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
                 ble_data.btn_indications_enabled = 0;
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

            /* Check the button state */
            button_val = GPIO_PinInGet(gpioPortF, PB0_pin);

            /* 0 means that the button is pressed since the button is connected to GND */
            if(button_val == 1)
                displayPrintf(DISPLAY_ROW_9, "Button Released");
            else if(button_val == 0)
              displayPrintf(DISPLAY_ROW_9, "Button Pressed");

            /* To interpret button press as 1 and release as 0 */
            button_val = !button_val;

            // -------------------------------// Write our local GATT DB// -------------------------------
            sc = sl_bt_gatt_server_write_attribute_value(
                gattdb_button_state, // handle from gatt_db.h
                0,                              // offset
                1,                              // length
                &button_val      // pointer to buffer where data is
            );

            if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
              }

            /* Send indications only if connection is established, client is bonded and indications are enabled */
            if((ble_data.gatt_server_connection != 0) && (ble_data.btn_indications_enabled == 1) &&
                (ble_data.bonding_state == BONDED) && (ble_data.in_flight == 0))
              {

                sc = sl_bt_gatt_server_send_indication(ble_data.gatt_server_connection,
                                                       gattdb_button_state,
                                                       1,
                                                       &button_val);

                ble_data.in_flight = 1;

                if (sc != SL_STATUS_OK)
                  {
                    LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                    ble_data.in_flight = 0;
                  }

              }

            else if ((ble_data.gatt_server_connection != 0) && (ble_data.btn_indications_enabled == 1) &&
                (ble_data.bonding_state == BONDED) && (ble_data.in_flight == 1))
              {
                indication_data.charHandle   = gattdb_button_state;
                indication_data.bufferLength = 1;
                memcpy(indication_data.buffer, &button_val, 1);

                if((cbfifo_enqueue(&queue, &indication_data)) == -1)
                  LOG_ERROR("Queue full, discarding event\r\n");
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
        if(UUID_Compare(evt, SERVICE))
            ble_data.serviceHandle[0] = evt->data.evt_gatt_service.service;

        /*
        else if(UUID_Compare(evt, SERVICE))
            ble_data.serviceHandle[1] = evt->data.evt_gatt_service.service;
        */
        break;

      case sl_bt_evt_gatt_characteristic_id:
        // CLIENT
        /* Save the newly discovered characteristics' handle */
        if(UUID_Compare(evt, CHARACTERISTIC))
            ble_data.characteristicHandle[0] = evt->data.evt_gatt_characteristic.characteristic;

        /*
        else if(UUID_Compare(evt, CHARACTERISTIC))
            ble_data.characteristicHandle[1] = evt->data.evt_gatt_characteristic.characteristic;
        */
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

            ambient_lt_val = evt->data.evt_gatt_characteristic_value.value.data[0];

            /* Display Received Ambient Light Value on the LCD for the Client */
            displayPrintf(DISPLAY_ROW_TEMPVALUE, "Light=%d", ambient_lt_val);

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
            client_btn_state = evt->data.evt_gatt_characteristic_value.value.data[0];

            if(client_btn_state == 1)
                displayPrintf(DISPLAY_ROW_9, "Button Pressed");

            else if(client_btn_state == 0)
              displayPrintf(DISPLAY_ROW_9, "Button Released");

            LOG_INFO("Button State = %s\r\n", client_btn_state ? "Button Pressed" : "Button Released");
          }
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

             /* PB0 Button is pressed, toggle indications */
             else if(GPIO_PinInGet(gpioPortF, PB0_pin) == 0)
               {

                 /* Flip indications for the Btn_State Characteristic */

                 ble_data.btn_indications_enabled ^= 2; // Second bit is flipped to turn on/off indications */
                 sc = sl_bt_gatt_set_characteristic_notification(ble_data.gatt_server_connection,
                                                                 ble_data.characteristicHandle[1],
                                                                 ble_data.btn_indications_enabled);

                 if (sc != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
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

         displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
         break;
#endif

      // -------------------------------
      // Default event handler.
      default:
        break;
    }
}

#if DEVICE_IS_BLE_SERVER
void SendTemperature(float Temperature)
{

  // -------------------------------------------------------------------
  // Update our local GATT DB and send indication if enabled for the characteristic
  // -------------------------------------------------------------------

  uint8_t   htm_temperature_buffer[5];   // Stores the temperature data in the Health Thermometer (HTM) format.
                                         // format of the buffer is: flags_byte + 4-bytes of IEEE-11073 32-bit float
  uint8_t   *p = htm_temperature_buffer; // Pointer to HTM temperature buffer needed for converting values to bitstream.
  uint32_t  htm_temperature_flt;         // Stores the temperature data read from the sensor in the IEEE-11073 32-bit float format
  uint8_t   flags = 0x00;                // HTM flags set as 0 for Celsius, no time stamp and no temperature type.

  // "bitstream" refers to the order of bytes and bits sent. byte[0] is sent first, followed by byte[1]...
  UINT8_TO_BITSTREAM(p, flags); // put the flags byte in first, "convert" is a strong word, it places the byte into the buffer

  // Convert sensor data to IEEE-11073 32-bit floating point format.
  htm_temperature_flt = UINT32_TO_FLOAT(Temperature*1000, -3); // Convert temperature to bitstream and place it in the htm_temperature_buffer
  UINT32_TO_BITSTREAM(p, htm_temperature_flt);

  /* Write attribute and send indications only if indications are enabled and connection is maintained */
  if(ble_data.htm_indications_enabled == 1 && ble_data.gatt_server_connection != 0 && ble_data.in_flight == 0)
    {

      // -------------------------------// Write our local GATT DB// -------------------------------
      uint32_t sc = sl_bt_gatt_server_write_attribute_value(
          gattdb_temperature_measurement, // handle from gatt_db.h
          0,                              // offset
          5,                              // length
          &htm_temperature_buffer[0]      // pointer to buffer where data is
      );

        {
          if (sc != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
            }

          sc = sl_bt_gatt_server_send_indication(ble_data.gatt_server_connection,
                                                 gattdb_temperature_measurement,
                                                 5,
                                                 htm_temperature_buffer);

          ble_data.in_flight = 1;
          if (sc != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
              ble_data.in_flight = 0;
            }

          /* Update Temperature on the LCD */
          displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp=%d", (int) Temperature);
        }

    }

  else if ((ble_data.gatt_server_connection != 0) && (ble_data.htm_indications_enabled == 1) &&
          (ble_data.in_flight == 1))
    {
      indication_data.charHandle   = gattdb_temperature_measurement;
      indication_data.bufferLength = 5;
      memcpy(indication_data.buffer, htm_temperature_buffer, 5);

      if((cbfifo_enqueue(&queue, &indication_data)) == -1)
        LOG_ERROR("Queue full, discarding event\r\n");
    }

}
#endif
