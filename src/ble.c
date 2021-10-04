/*
 * ble.c
 *
 *  Created on: Sep 30, 2021
 *      Author: Dhruv
 *      Reference : Used Bluetooth (SoC) Thermometer Example Code
 *      for handle_ble_events.
 *
 */
#include "src/ble.h"
#include "app_log.h"
#include "app_assert.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

// The advertising set handle allocated from Bluetooth stack.
static ble_data_struct_t ble_data = {.advertisingSetHandle = 0xff, .readTemperature = 0, \
                                     .gatt_server_connection = 0};

ble_data_struct_t* getBleDataPtr()
{
  return(&ble_data);
}

void handle_ble_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;
    uint8_t address_type;

    // Handle stack events
    switch (SL_BT_MSG_ID(evt->header)) {
      // -------------------------------
      // This event indicates the device has started and the radio is ready.
      // Do not call any stack command before receiving this boot event!
      /* Boot Event */
      case sl_bt_evt_system_boot_id:
        // Print boot message.

        /*
        LOG_INFO("Bluetooth stack booted: v%d.%d.%d-b%d\r\n",
                     evt->data.evt_system_boot.major,
                     evt->data.evt_system_boot.minor,
                     evt->data.evt_system_boot.patch,
                     evt->data.evt_system_boot.build);
        */
        // Extract unique ID from BT Address.
        sc = sl_bt_system_get_identity_address(&ble_data.myAddress, &address_type);

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        /*
        LOG_INFO("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                     address_type ? "static random" : "public device",
                     ble_data.myAddress.addr[5],
                     ble_data.myAddress.addr[4],
                     ble_data.myAddress.addr[3],
                     ble_data.myAddress.addr[2],
                     ble_data.myAddress.addr[1],
                     ble_data.myAddress.addr[0]);
         */

        // Create an advertising set.
        sc = sl_bt_advertiser_create_set(&(ble_data.advertisingSetHandle));

        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_create_set() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        // Set advertising interval to 250ms.
        sc = sl_bt_advertiser_set_timing(
          ble_data.advertisingSetHandle, // advertising set handle
          400, // min. adv. interval (milliseconds * 1.6)
          400, // max. adv. interval (milliseconds * 1.6)
          0,   // adv. duration
          0);  // max. num. adv. events

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
        break;

      // -------------------------------
      // This event indicates that a new connection was opened.
      /* Connection Opened Event */
      case sl_bt_evt_connection_opened_id:
        //LOG_INFO("Connection opened\r\n");
        sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
        sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection, 60, 60, 4, 75, 0, 0xFFFF);
        break;

      // -------------------------------
      // This event indicates that a connection was closed.
      /* Connection Close Event */
      case sl_bt_evt_connection_closed_id:

        ble_data.gatt_server_connection = 0;

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
        break;

      /* Connnection Parameters Event */
      case sl_bt_evt_connection_parameters_id:

        LOG_INFO("Interval = %d\r\n Latency = %d\r\n Timeout = %d\r\n",
                  evt->data.evt_connection_parameters.interval,
                  evt->data.evt_connection_parameters.latency,
                  evt->data.evt_connection_parameters.timeout);

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
      ble_data.gatt_server_connection = evt->data.evt_gatt_server_characteristic_status.connection;

      if( evt->data.evt_gatt_server_characteristic_status.status_flags == 1 && \
          evt->data.evt_gatt_server_characteristic_status.client_config_flags == 2)
        {
          /* Start reading temperature */
          ble_data.readTemperature = 1;
        }

      /* Indications have been turned off */
      else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0)
                ble_data.readTemperature = 0;
    }
        break;

      /* GATT Server Indication Timeout Event */
      case sl_bt_evt_gatt_server_indication_timeout_id:
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
  if(ble_data.readTemperature == 1 && ble_data.gatt_server_connection != 0)
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

    }
}
