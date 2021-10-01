/*
 * ble.c
 *
 *  Created on: Sep 30, 2021
 *      Author: Dhruv
 */
#include "src/ble.h"
#include "app_log.h"
#include "app_assert.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// -------------------------------------------------------------------
// Update our local GATT DB and send indication if enabled for the characteristic
// -------------------------------------------------------------------

uint8_t   htm_temperature_buffer[5];   // Stores the temperature data in the Health Thermometer (HTM) format.
                                       // format of the buffer is: flags_byte + 4-bytes of IEEE-11073 32-bit float
uint8_t   *p = htm_temperature_buffer; // Pointer to HTM temperature buffer needed for converting values to bitstream.
uint32_t  htm_temperature_flt;         // Stores the temperature data read from the sensor in the IEEE-11073 32-bit float format
uint8_t   flags = 0x00;                // HTM flags set as 0 for Celsius, no time stamp and no temperature type.

// "bitstream" refers to the order of bytes and bits sent. byte[0] is sent first, followed by byte[1]...

void handle_ble_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
    bd_addr address;
    uint8_t address_type;
    uint8_t system_id[8];

    // Handle stack events
    switch (SL_BT_MSG_ID(evt->header)) {
      // -------------------------------
      // This event indicates the device has started and the radio is ready.
      // Do not call any stack command before receiving this boot event!
      case sl_bt_evt_system_boot_id:
        // Print boot message.
        app_log_info("Bluetooth stack booted: v%d.%d.%d-b%d\n",
                     evt->data.evt_system_boot.major,
                     evt->data.evt_system_boot.minor,
                     evt->data.evt_system_boot.patch,
                     evt->data.evt_system_boot.build);

        // Extract unique ID from BT Address.
        sc = sl_bt_system_get_identity_address(&address, &address_type);
        app_assert_status(sc);

        // Pad and reverse unique ID to get System ID.
        system_id[0] = address.addr[5];
        system_id[1] = address.addr[4];
        system_id[2] = address.addr[3];
        system_id[3] = 0xFF;
        system_id[4] = 0xFE;
        system_id[5] = address.addr[2];
        system_id[6] = address.addr[1];
        system_id[7] = address.addr[0];

        sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                     0,
                                                     sizeof(system_id),
                                                     system_id);
        app_assert_status(sc);

        app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                     address_type ? "static random" : "public device",
                     address.addr[5],
                     address.addr[4],
                     address.addr[3],
                     address.addr[2],
                     address.addr[1],
                     address.addr[0]);

        // Create an advertising set.
        sc = sl_bt_advertiser_create_set(&advertising_set_handle);
        app_assert_status(sc);

        // Set advertising interval to 250ms.
        sc = sl_bt_advertiser_set_timing(
          advertising_set_handle, // advertising set handle
          400, // min. adv. interval (milliseconds * 1.6)
          400, // max. adv. interval (milliseconds * 1.6)
          0,   // adv. duration
          0);  // max. num. adv. events
        app_assert_status(sc);
        // Start general advertising and enable connections.
        sc = sl_bt_advertiser_start(
          advertising_set_handle,
          sl_bt_advertiser_general_discoverable,
          sl_bt_advertiser_connectable_scannable);
        app_assert_status(sc);
        app_log_info("Started advertising\n");
        break;

      // -------------------------------
      // This event indicates that a new connection was opened.
      case sl_bt_evt_connection_opened_id:
        app_log_info("Connection opened\n");
        sl_bt_advertiser_stop(advertising_set_handle);
        sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection, 60, 60, 4, 600, 0xFFFF, 0xFFFF);

  #ifdef SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT
        // Set remote connection power reporting - needed for Power Control
        sc = sl_bt_connection_set_remote_power_reporting(
          evt->data.evt_connection_opened.connection,
          sl_bt_connection_power_reporting_enable);
        app_assert_status(sc);
  #endif // SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT

        break;

      // -------------------------------
      // This event indicates that a connection was closed.
      case sl_bt_evt_connection_closed_id:
        app_log_info("Connection closed\n");
        // Restart advertising after client has disconnected.
        sc = sl_bt_advertiser_start(
          advertising_set_handle,
          sl_bt_advertiser_general_discoverable,
          sl_bt_advertiser_connectable_scannable);
        app_assert_status(sc);
        app_log_info("Started advertising\n");
        break;

      case sl_bt_evt_connection_parameters_id:
        break;

      case sl_bt_evt_gatt_server_characteristic_status_id:

        ///////////////////////////////////////////////////////////////////////////
        // Add additional event handlers here as your application requires!      //
        ///////////////////////////////////////////////////////////////////////////
          UINT8_TO_BITSTREAM(p, flags); // put the flags byte in first, "convert" is a strong word, it places the byte into the buffer

          // Convert sensor data to IEEE-11073 32-bit floating point format.
          htm_temperature_flt = UINT32_TO_FLOAT(25*1000, -3); // Convert temperature to bitstream and place it in the htm_temperature_buffer
          UINT32_TO_BITSTREAM(p, htm_temperature_flt);

          // -------------------------------// Write our local GATT DB// -------------------------------
          sc = sl_bt_gatt_server_write_attribute_value(
              gattdb_temperature_measurement, // handle from gatt_db.h
              0,                              // offset
              5,                              // length
              &htm_temperature_buffer[0]      // pointer to buffer where data is
          );
          if (sc != SL_STATUS_OK)
            {
              //LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
            }
        break;

      case sl_bt_evt_gatt_server_indication_timeout_id:
        break;


      // -------------------------------
      // Default event handler.
      default:
        break;
    }
}
