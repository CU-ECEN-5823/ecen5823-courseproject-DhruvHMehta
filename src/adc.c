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
  CMU_ClockEnable(cmuClock_ADC0, true);
  ADC_Init(ADC0, &ADCinit);

  ADCSingle.acqTime     = _ADC_SINGLECTRL_AT_1CYCLE;
  ADCSingle.reference   = adcRef5V;
  ADCSingle.resolution  = _ADC_SINGLECTRL_RES_12BIT;
  ADCSingle.posSel      = adcPosSelAPORT4XCH11;
  ADCSingle.diff        = false;
  ADCSingle.prsEnable   = false;
  ADCSingle.leftAdjust  = false;
  ADCSingle.rep         = false;

  ADC_InitSingle(ADC0, &ADCSingle);

  while(1)
    {
      ADC_Start(ADC0, adcStartSingle);
      LOG_INFO("ADCval = %d\r\n", ADC_DataSingleGet(ADC0));
      for(long i = 0; i < 10000000; i++);
    }
}


