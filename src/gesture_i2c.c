/*
 * gesture_i2c.c
 *
 *  Created on: 19-Nov-2021
 *      Author: Pradyumna
 *   Reference : 1) https://github.com/sparkfun/SparkFun_APDS-9960_Sensor_Arduino_Library
 *               2) https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/si1147__i2c_8c_source.html
 *
 */

/***************************************************************************/

#include "em_i2c.h"
#include "gesture_i2c.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

  /*******************************************************************************
      * @name gesture_Read_Register
      * @brief Driver function gesture application for reading a register.
      *
      * @param I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t *data
      * @return True if the data is read completely , else with the error value
      *
      *******************************************************************************/
  /**************************************************************************/
  uint32_t gesture_Read_Register(I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t *data)
  {
    I2C_TransferSeq_TypeDef    seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t i2c_write_data[1];

    seq.addr  = (addr << 1);
    seq.flags = I2C_FLAG_WRITE_READ;
    /* Select register to start reading from */
    i2c_write_data[0] = reg;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len  = 1;
    /* Select length of data to be read */
    seq.buf[1].data = data;
    seq.buf[1].len  = 1;

    ret = I2CSPM_Transfer(i2c, &seq);
    if(ret == 0)
         {
          //LOG_INFO (" bytes read\r\n");
         }
       else
         {
           LOG_ERROR (" bytes is not written due to returned value was %d: gesture_Read_Register\r\n",ret);
         }

    if (ret != i2cTransferDone)
    {
      *data = 0xff;
      return (uint32_t)ret;
    }
    return (uint32_t)1;
  }
  /*******************************************************************************
      * @name gesture_Write_Register
      * @brief Driver function gesture application for writing a register.
      *
      * @param I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t data
      * @return True if the data is written completely , else with the error value
      *
      *******************************************************************************/
  /**************************************************************************/
  uint32_t gesture_Write_Register(I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t data)
  {
    I2C_TransferSeq_TypeDef    seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t i2c_write_data[2];
    uint8_t i2c_read_data[1];

    seq.addr  = (addr << 1);
    seq.flags = I2C_FLAG_WRITE;
    /* Select register and data to write */
    i2c_write_data[0] = reg;
    i2c_write_data[1] = data;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len  = 2;
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len  = 0;

    ret = I2CSPM_Transfer(i2c, &seq);

    if(ret == 0)
             {
              //LOG_INFO (" bytes read\r\n");
             }
           else
             {
               LOG_ERROR (" bytes is not written due to returned value was %d: gesture_Write_Register\r\n",ret);
             }


    if (ret != i2cTransferDone)
    {
      return (uint32_t)ret;
    }
    return (uint32_t)1;
  }
  /*******************************************************************************
    * @name gesture_Write_Block_Register
    * @brief Driver function gesture application for writing a block of register data.
    *
    * @param I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t length, uint8_t const *data
    * @return True if the data is written completely , else with the error value
    *
    *******************************************************************************/
  /**************************************************************************/
  uint32_t gesture_Write_Block_Register(I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t length, uint8_t const *data)
  {
    I2C_TransferSeq_TypeDef    seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t i2c_write_data[10];
    uint8_t i2c_read_data[1];
    int i;

    seq.addr  = addr << 1;
    seq.flags = I2C_FLAG_WRITE;
    /* Select register to start writing to*/
    i2c_write_data[0] = reg;
    for (i=0; i<length;i++)
    {
      i2c_write_data[i+1] = data[i];
    }
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len  = 1+length;
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len  = 0;

    ret = I2CSPM_Transfer(i2c, &seq);

    if(ret == 0)
    {
    // LOG_INFO (" bytes read\r\n");
    }
    else
    {
     LOG_ERROR (" bytes is not written due to returned value was %d : gesture_Write_Block_Register\r\n",ret);
    }


    if (ret != i2cTransferDone)
    {
      return (uint32_t)ret;
    }
    return (uint32_t)1;
  }
  /*******************************************************************************
   * @name gesture_Read_Block_Register
   * @brief Driver function gesture application for reading a block of register data.
   *
   * @param I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data
   * @return length if the data is read completely , else with the error value
   *
   *******************************************************************************/
  /**************************************************************************/
  uint32_t gesture_Read_Block_Register(I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
  {
    I2C_TransferSeq_TypeDef    seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t i2c_write_data[1];

    seq.addr  = addr << 1;
    seq.flags = I2C_FLAG_WRITE_READ;
    /* Select register to start reading from */
    i2c_write_data[0] = reg;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len  = 1;
    /* Select length of data to be read */
    seq.buf[1].data = data;
    seq.buf[1].len  = length;

    ret = I2CSPM_Transfer(i2c, &seq);

    if(ret == 0)
        {
        // LOG_INFO (" bytes read\r\n");
        }
        else
        {
         LOG_ERROR (" bytes is not written due to returned value was %d : gesture_Read_Block_Register\r\n",ret);
        }

    if (ret != i2cTransferDone)
    {
      return (uint32_t)ret;
    }
    return (uint32_t)length;
  }

