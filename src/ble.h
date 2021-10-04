/*
 * ble.h
 *
 *  Created on: Sep 30, 2021
 *      Author: Dhruv
 */

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include "sl_bt_api.h"
#include "gatt_db.h"

/* Macro Definitions */
#define UINT8_TO_BITSTREAM(p, n)      { *(p)++ = (uint8_t)(n); }

#define UINT32_TO_BITSTREAM(p, n)     { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
                                        *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }

#define UINT32_TO_FLOAT(m, e)         (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

typedef struct
{
   bd_addr myAddress;
   uint8_t advertisingSetHandle;
   uint8_t readTemperature;
   uint8_t gatt_server_connection;
}ble_data_struct_t;

/* Function Prototypes */

/***************************************************************************//**
 * @name handle_ble_event
 *
 * @brief Contains the BLE State machine which handles all BLE events:
 *        Boot
 *        Connection Opened
 *        Connection Closed
 *        Connection Parameters
 *        Characteristic Status
 *        Indication Timeout
 *
 * @param[in] sl_bt_msg_t *evt - BLE Event
 *
 * @return void
 ******************************************************************************/
void handle_ble_event(sl_bt_msg_t *evt);

/***************************************************************************//**
 * @name SendTemperature
 *
 * @brief Sends the temperature from I2C to BLE for writing and sending
 *        indications to the Client when indications are enabled.
 *
 * @param[in] float Temperature - Temperature in C
 *
 * @return void
 ******************************************************************************/
void SendTemperature(float Temperature);

/***************************************************************************//**
 * @name getBleDataPtr
 *
 * @brief Gets the BLE Data pointer. Used to obtain the following:
 *        BLE Address
 *        Advertising Handle
 *        Indication Enabled (Read Temperature)
 *        Connection Handle
 *
 * @param[in] float Temperature - Temperature in C
 *
 * @return ble_data_struct_t* - Pointer to instance of ble_data
 ******************************************************************************/
ble_data_struct_t* getBleDataPtr();

#endif /* SRC_BLE_H_ */
