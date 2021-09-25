/*
 * irq.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 *      Brief : Contains all IRQ Handlers.
 */
#include "irq.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

/* Use LFXO and prescalar for high energy modes */
#if ((LOWEST_ENERGY_MODE == EM0) || (LOWEST_ENERGY_MODE == EM1) || (LOWEST_ENERGY_MODE == EM2))
#define PRESCALAR_VALUE   4
#define ACTUAL_CLOCK_FREQ (32768/PRESCALAR_VALUE)

/* Use ULFRCO and prescalar for low energy modes */
#else
#define PRESCALAR_VALUE   1
#define ACTUAL_CLOCK_FREQ (1000/PRESCALAR_VALUE)
#endif

#define LETIMER_CTR_VAL   ((LETIMER_PERIOD_MS * ACTUAL_CLOCK_FREQ)/1000)
#define LETIMER_COMP1_VAL ((LETIMER_ON_TIME_MS * ACTUAL_CLOCK_FREQ)/1000)

I2C_TransferReturn_TypeDef I2CTransferReturn;
/* Keeps count of time passed since the system startup */
static uint32_t milliseconds;

/***************************************************************************//**
 * @name LETIMER0_IRQHandler
 *
 * @brief
 *   Interrupt handler which sets events based on the interrupt.
 *
 * @param[in] osc
 *   none
 *
 * @return void
 ******************************************************************************/
void LETIMER0_IRQHandler()
{
  //sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);

  /* Check which IF is set */
  uint32_t flags = LETIMER_IntGetEnabled(LETIMER0);

  /* Clear the interrupt */
  LETIMER_IntClear(LETIMER0, flags);

  /* Set the UF Event */
  if(flags == LETIMER_IEN_UF)
    {
      schedulerSetEvent_UF();

      /* 3 seconds have passed */
      CORE_CRITICAL_SECTION(milliseconds += LETIMER_PERIOD_MS;);
    }

  /* Set the COMP1 Event */
  if(flags == LETIMER_IEN_COMP1)
    {
      LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
      schedulerSetEvent_COMP1();
    }

  //sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM2);
}

/***************************************************************************//**
 * @name I2C0_IRQHandler
 *
 * @brief
 *   Interrupt handler which sets events based on the interrupt.
 *
 * @param[in] osc
 *   none
 *
 * @return void
 ******************************************************************************/
void I2C0_IRQHandler()
{

  I2CTransferReturn = I2C_Transfer(I2C0);

  if(I2CTransferReturn == i2cTransferDone)
    schedulerSetEvent_I2Cdone();

  if(I2CTransferReturn < 0)
    LOG_ERROR("I2C Transfer Failed with status = %d\n", I2CTransferReturn);
}

I2C_TransferReturn_TypeDef getI2CTransferReturn()
{
  return I2CTransferReturn;
}

uint32_t letimerMilliseconds()
{
  uint32_t ctrmilliseconds;

  ctrmilliseconds = (LETIMER_CTR_VAL - LETIMER_CounterGet(LETIMER0));
  ctrmilliseconds = (ctrmilliseconds*1000)/ACTUAL_CLOCK_FREQ;


  return (milliseconds + ctrmilliseconds);
}
