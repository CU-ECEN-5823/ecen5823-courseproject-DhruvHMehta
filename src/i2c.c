/*
 * i2c.c
 *
 *  Created on: Sep 14, 2021
 *      Author: Dhruv
 *      Brief : Contains the I2C initialization and communication functions to
 *              get Temperature data from the Si7021 on-board sensor.
 */


#include "i2c.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


#define SI_7021_I2CADDR 0x40
#define SI_7021_TREAD   0xF3

I2CSPM_Init_TypeDef init;
I2C_TransferSeq_TypeDef TransferSeq;
uint8_t cmd_Tread = SI_7021_TREAD;
uint8_t rx_Temp[2];


void I2CInit()
{

  init.port       = I2C0;
  init.sclPort    = gpioPortC;
  init.sclPin     = 10;
  init.sdaPort    = gpioPortC;
  init.sdaPin     = 11;
  init.portLocationScl = 14;
  init.portLocationSda = 16;
  init.i2cRefFreq = 0;
  init.i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
  init.i2cClhr    = i2cClockHLRStandard;

  I2CSPM_Init(&init);
}

uint8_t I2CTransferInitWrapper(uint8_t* Data, uint8_t ReadWrite, uint8_t DataLen)
{
  I2C_TransferReturn_TypeDef I2CTransferReturn;

  TransferSeq.addr    = (SI_7021_I2CADDR << 1);
  TransferSeq.flags   = ReadWrite;

  TransferSeq.buf[0].data  = Data;
  TransferSeq.buf[0].len   = DataLen;

  NVIC_EnableIRQ(I2C0_IRQn);

  I2CTransferReturn = I2C_TransferInit(I2C0, &TransferSeq);

  if(I2CTransferReturn < 0)
    {
      LOG_ERROR("I2C_TransferInit Failed. Error = %d\n\r", I2CTransferReturn);
      return 1;
    }

  return 0;
}

/*
static void sendReadCommandSi7021()
{
  I2CTransfer(&cmd_Tread, I2C_FLAG_WRITE, sizeof(cmd_Tread));
}

static void recvTempSi7021()
{
  I2CTransfer(&rx_Temp, I2C_FLAG_READ, sizeof(rx_Temp));
}
*/

void powerOnSi7021()
{
  /* Turn on power to the sensor and wait for 100ms */
  sensorLPMControl(true);
  timerWaitUs_irq(100*1000);
}

void getTemperatureSi7021()
{
  uint8_t ret_status;

  /* Send the Temperature read command and check for error */
  ret_status = I2CTransferInitWrapper(&cmd_Tread, I2C_FLAG_WRITE, sizeof(cmd_Tread));
  if(ret_status)
    {
      sensorLPMControl(false);
      return;
    }
}

void waitConversionTimeSi7021()
{
  /* Wait for 15ms for the sensor to get the data */
  timerWaitUs_irq(15*1000);
}

void readTemperatureSi7021()
{
  uint8_t ret_status;

  /* Receive the temperature data and check for error */
  ret_status = I2CTransferInitWrapper(&rx_Temp, I2C_FLAG_READ, sizeof(rx_Temp));
  if(ret_status)
    {
      sensorLPMControl(false);
      return;
    }
}
void reportTemperatureSi7021()
{
  int Calc_Temp;
  uint16_t Read_Temp;

  /* Arrange MSB and LSB into Read_Temp variable */
  Read_Temp = ((rx_Temp[0] << 8) | rx_Temp[1]);

  /* Convert received value to degrees Celsius and log */
  Calc_Temp = (175 * Read_Temp)/65536 - 47;
  LOG_INFO("Temperature = %dC\n\r", Calc_Temp);

  /* Turn off power to the sensor */
  sensorLPMControl(false);

}
