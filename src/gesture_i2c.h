/*
 * gesture_i2c.h
 *
 *  Created on: 19-Nov-2021
 *      Author: Pradyumna
 */

#ifndef SRC_GESTURE_I2C_H_
#define SRC_GESTURE_I2C_H_

#include "em_device.h"
#include "sl_i2cspm.h"

/***************************************************************************/
/***************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/
typedef struct {
    I2C_TypeDef *i2c;
    uint8_t addr;
  } si114x_i2c_t;

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/
uint32_t gesture_Write_Register (I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t data);
uint32_t gesture_Write_Block_Register (I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t length, uint8_t const *data);
uint32_t gesture_Read_Block_Register (I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t length, uint8_t  *data);
uint32_t gesture_Read_Register (I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t *data);

#endif /* SRC_GESTURE_I2C_H_ */
