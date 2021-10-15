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

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


#if DEVICE_IS_BLE_SERVER
// The advertising set handle allocated from Bluetooth stack.
static ble_data_struct_t ble_data = {.advertisingSetHandle = 0xff, .htm_indications_enabled = 0,
                                     .gatt_server_connection = 0};
#else
static ble_data_struct_t ble_data = {.htm_indications_enabled = 0, .gatt_server_connection = 0,
                                      .thermo_service = {0x09, 0x18},
                                     .thermo_char = {0x1c, 0x2a}};
#endif

ble_data_struct_t* getBleDataPtr()
{
  return(&ble_data);
}

static uint8_t check_slave_addr(sl_bt_msg_t *evt)
{
  uint8_t slave_addr_check = 0;
  bd_addr ActualSlaveAddr = SERVER_BT_ADDRESS;

  for(int i = 0; i < 6; i++)
    {
      if(evt->data.evt_scanner_scan_report.address.addr[i] == ActualSlaveAddr.addr[i])
        {
          slave_addr_check++;
        }

    }

  if(slave_addr_check == 6)
    {
      return 1;
    }

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

void handle_ble_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;
    uint8_t address_type;
    uint8_t slave_addr_match = 0;
    uint8_t *float_tempval;

    // Handle stack events
    switch (SL_BT_MSG_ID(evt->header)) {
      // -------------------------------
      // This event indicates the device has started and the radio is ready.
      // Do not call any stack command before receiving this boot event!
      /* Boot Event */
      case sl_bt_evt_system_boot_id:

#if DEVICE_IS_BLE_SERVER
        // SERVER

        // Extract unique ID from BT Address.
        sc = sl_bt_system_get_identity_address(&ble_data.myAddress, &address_type);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        // Create an advertising set.
        sc = sl_bt_advertiser_create_set(&(ble_data.advertisingSetHandle));

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_create_set() returned != 0 status=0x%04x", (unsigned int) sc);
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
            LOG_ERROR("sl_bt_advertiser_set_timing() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        // Start general advertising and enable connections.
        sc = sl_bt_advertiser_start(
          ble_data.advertisingSetHandle,
          sl_bt_advertiser_general_discoverable,
          sl_bt_advertiser_connectable_scannable);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_set_timing() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        //LOG_INFO("Started advertising\r\n");

        // Initializing the display and printing my BT Address
        displayInit();
        displayPrintf(DISPLAY_ROW_NAME, BLE_DEVICE_TYPE_STRING);
        displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X", ble_data.myAddress.addr[0], \
                      ble_data.myAddress.addr[1], ble_data.myAddress.addr[2], \
                      ble_data.myAddress.addr[3], ble_data.myAddress.addr[4], \
                      ble_data.myAddress.addr[5]);

        displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A6");
        displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");

#else
        // CLIENT

        /* Use Passive Scanning */
        sc = sl_bt_scanner_set_mode(sl_bt_gap_phy_1m, 0);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_set_mode() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /* Scan Interval = 50mS
         * Scan Window = 25mS
         */
        sc = sl_bt_scanner_set_timing(sl_bt_gap_phy_1m, 80, 40);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_set_timing() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /* Set default connection parameters */
        sc = sl_bt_connection_set_default_parameters(60, 60, 3, 75, 0, 0xFFFF);

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
            LOG_ERROR("sl_bt_advertiser_stop() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /* Get the connection handle */
        ble_data.gatt_server_connection = evt->data.evt_connection_opened.connection;

        /* Set the connection parameters */
        sc = sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection, 60, 60, 3, 75, 0, 0xFFFF);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_connection_set_parameters() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /* Display Connected on the LCD */
        displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");

#else
        // CLIENT
        /* Get the connection handle and save it */
        ble_data.gatt_server_connection = evt->data.evt_connection_opened.connection;
        ble_data.discoveryEvt = 1;
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

        //LOG_INFO("Connection closed\r\n");

        // Restart advertising after client has disconnected.
        sc = sl_bt_advertiser_start(
          ble_data.advertisingSetHandle,
          sl_bt_advertiser_general_discoverable,
          sl_bt_advertiser_connectable_scannable);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        //LOG_INFO("Started advertising\r\n");

        displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");

        /* Display Advertising on the LCD */
        displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
#else
        //CLIENT
        /* Start scanning for new devices */
        sc = sl_bt_scanner_start(sl_bt_gap_phy_1m, sl_bt_scanner_discover_observation);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        ble_data.discoveryEvt = 3;
#endif
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
        displayUpdate();

        break;



      /* GATT Server Characteristic Status ID Event */
      case sl_bt_evt_gatt_server_characteristic_status_id:
/*
        LOG_INFO("status flag = %d\r\n client conf flags = %d\r\n",
                 evt->data.evt_gatt_server_characteristic_status.status_flags,
                 evt->data.evt_gatt_server_characteristic_status.client_config_flags);
*/
   if  (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
    {
      //ble_data.gatt_server_connection = evt->data.evt_gatt_server_characteristic_status.connection;

      if( evt->data.evt_gatt_server_characteristic_status.status_flags == 1 &&
          evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
        {
          /* Start reading temperature */
          ble_data.htm_indications_enabled = 1;
        }

      /* Indications have been turned off */
      else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)
        {
            displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
            ble_data.htm_indications_enabled = 0;
        }
    }
        break;

      /* GATT Server Indication Timeout Event */
      case sl_bt_evt_gatt_server_indication_timeout_id:
        break;

     /* Event Scanner Scan Report Event */
      case sl_bt_evt_scanner_scan_report_id:
#if DEVICE_IS_BLE_SERVER
        //SERVER
        // should not happen
#else
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

#endif
        break;

      case sl_bt_evt_gatt_service_id:
#if DEVICE_IS_BLE_SERVER
        //SERVER
        // should not happen
#else
        // CLIENT
        /* Save the newly discovered service's handle */
        if(evt->data.evt_gatt_service.uuid.data[0] == ble_data.thermo_service[0] &&
           evt->data.evt_gatt_service.uuid.data[1] == ble_data.thermo_service[1])
            ble_data.serviceHandle = evt->data.evt_gatt_service.service;
#endif
        break;

      case sl_bt_evt_gatt_characteristic_id:
#if DEVICE_IS_BLE_SERVER
        //SERVER
        // should not happen
#else
        // CLIENT
        /* Save the newly discovered characteristics' handle */
        if(evt->data.evt_gatt_characteristic.uuid.data[0] == ble_data.thermo_char[0] &&
           evt->data.evt_gatt_characteristic.uuid.data[1] == ble_data.thermo_char[1])
            ble_data.characteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
#endif
        break;

      case sl_bt_evt_gatt_characteristic_value_id:
#if DEVICE_IS_BLE_SERVER
        //SERVER
        // should not happen
#else
        //CLIENT
        /* Check if the att_opcode and gatt characteristic handle match */
        if(evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication &&
            evt->data.evt_gatt_characteristic_value.characteristic == ble_data.characteristicHandle)
          {
            sc = sl_bt_gatt_send_characteristic_confirmation(ble_data.gatt_server_connection);

            if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() returned != 0 status=0x%04x", (unsigned int) sc);
              }

            float_tempval = &(evt->data.evt_gatt_characteristic_value.value.data);

            LOG_INFO("Received Temperature = %d\r\n", gattFloat32ToInt(float_tempval));
          }
#endif
        break;

      case sl_bt_evt_gatt_procedure_completed_id:
#if DEVICE_IS_BLE_SERVER
        //SERVER
        // should not happen
#else
        // CLIENT
        /* Check which GATT procedure was completed */
        /* If Discover Services by UUID was completed, check return status and set evtGattComplete */
        if(evt->data.evt_gatt_procedure_completed.result == 0)
          {
            ble_data.discoveryEvt = 2;
          }

        /* If Discover Ch by UUID was completed, check return status and set GattComplete */

#endif
        break;


      // -------------------------------
      // Default event handler.
      default:

        break;
    }
}


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

  /* Write attribute and send indications only if indications are enabled and connection is maintained */
  if(ble_data.htm_indications_enabled == 1 && ble_data.gatt_server_connection != 0)
    {
      UINT8_TO_BITSTREAM(p, flags); // put the flags byte in first, "convert" is a strong word, it places the byte into the buffer

      // Convert sensor data to IEEE-11073 32-bit floating point format.
      htm_temperature_flt = UINT32_TO_FLOAT(Temperature*1000, -3); // Convert temperature to bitstream and place it in the htm_temperature_buffer
      UINT32_TO_BITSTREAM(p, htm_temperature_flt);

      // -------------------------------// Write our local GATT DB// -------------------------------
      uint32_t sc = sl_bt_gatt_server_write_attribute_value(
          gattdb_temperature_measurement, // handle from gatt_db.h
          0,                              // offset
          5,                              // length
          &htm_temperature_buffer[0]      // pointer to buffer where data is
      );

      if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
        }

      sc = sl_bt_gatt_server_send_indication(ble_data.gatt_server_connection,
                                             gattdb_temperature_measurement,
                                             5,
                                             htm_temperature_buffer);

      if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x", (unsigned int) sc);
        }

      /* Update Temperature on the LCD */
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp=%d", (int) Temperature);

    }
}
