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

#define UINT8_TO_BITSTREAM(p, n)      { *(p)++ = (uint8_t)(n); }

#define UINT32_TO_BITSTREAM(p, n)     { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
                                        *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }


#define UINT32_TO_FLOAT(m, e)         (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

void handle_ble_event(sl_bt_msg_t *evt);

#endif /* SRC_BLE_H_ */
