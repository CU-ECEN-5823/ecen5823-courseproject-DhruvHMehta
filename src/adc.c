/*
 * adc.c
 *
 *  Created on: Nov 12, 2021
 *      Author: Dhruv
 */

#include "adc.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

ADC_Init_TypeDef ADCinit = ADC_INIT_DEFAULT;
ADC_InitSingle_TypeDef ADCSingle;

void ADCInit()
{
  /* Enable the Clock gating to the ADC0 peripheral */
  CMU_ClockEnable(cmuClock_ADC0, true);
  ADC_Init(ADC0, &ADCinit);

  /* Set the configurations for ADC0
   * Single Cycle Conversion
   * 5V Reference
   * 12-bit Resolution
   * APORT4XCH11 = PA3
   */
  ADCSingle.acqTime     = adcAcqTime4;
  ADCSingle.reference   = adcRef5V;
  ADCSingle.resolution  = adcRes12Bit;
  ADCSingle.posSel      = adcPosSelAPORT4XCH11;
  ADCSingle.negSel      = adcNegSelVSS;
  ADCSingle.diff        = false;
  ADCSingle.prsEnable   = false;
  ADCSingle.leftAdjust  = false;
  ADCSingle.rep         = false;

  ADC_InitSingle(ADC0, &ADCSingle);
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);

  uint32_t retval;

#if 0
  while(1)
    {
      gpioAMBSensor(true);
      for(long i = 0; i < 100000; i++);

      /* Start a Conversion */
      ADC_Start(ADC0, adcStartSingle);

      /* Wait till conversion is complete */
      while(!(ADC0->STATUS && ADC_STATUS_SINGLEDV));

      /* Get the converted value */
      //retval = ADC_DataSingleGet(ADC0);
      //LOG_INFO("ADCval = %d\r\n", retval);

      //gpioAMBSensor(false);
      for(long i = 0; i < 1000000; i++);
    }
#endif
}


