/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

 */

#include "gpio.h"

// Set GPIO drive strengths and modes of operation
void gpioInit()
{

  // Student Edit:


	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

  //GPIO_PinModeSet(I2C0_port, I2C0_SCL, gpioModePushPull, false);
  //GPIO_PinModeSet(I2C0_port, I2C0_SDA, gpioModePushPull, false);
  GPIO_PinModeSet(SENS_port, SENS_EN, gpioModePushPull, false);
  GPIO_PinModeSet(DISP_port, DISP_EN, gpioModePushPull, false);
  GPIO_PinModeSet(PB0_port, PB0_pin, gpioModeInputPullFilter, true);
  GPIO_ExtIntConfig(PB0_port, PB0_pin, PB0_pin, true, true, true);

  /* Clear any pending IRQ */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);

  /* Enable Interrupts in the NVIC for GPIO */
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

} // gpioInit()


void gpioSensorEnSetOn()
{
  GPIO_PinOutSet(SENS_port, SENS_EN);
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

void  gpioSetDisplayExtcomin(bool value)
{
  if(value == true)
    {
      GPIO_PinOutSet(DISP_port, DISP_EN);
    }

  else if(value == false)
    {
      GPIO_PinOutClear(DISP_port, DISP_EN);
    }
}




