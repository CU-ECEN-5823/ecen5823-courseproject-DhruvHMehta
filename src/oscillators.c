/*
 * oscillators.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 *      Brief : Initialization functions for oscillators.
 */
#include "oscillators.h"

void OscillatorInit()
{
#if ((LOWEST_ENERGY_MODE == EM0) || (LOWEST_ENERGY_MODE == EM1) || (LOWEST_ENERGY_MODE ==  EM2))

  /* Enable LFXO Oscillator, wait for clock to stabilize */
  CMU_OscillatorEnable(cmuOsc_LFXO, ENABLE, ENABLE);

  /* Select LFA Clock branch and LFXO Clock reference */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

  /* Set Clock divider to 4 */
  CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_4);

  /* Enable the LETIMER0 */
  CMU_ClockEnable(cmuClock_LETIMER0, ENABLE);

#else

  /* Enable ULFRCO Oscillator, wait for clock to stabilize */
  CMU_OscillatorEnable(cmuOsc_ULFRCO, ENABLE, ENABLE);

  /* Select LFA Clock branch and ULFRCO Clock reference */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

  /* Set Clock divider to 1 */
  CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_1);

  /* Enable the LETIMER0 */
  CMU_ClockEnable(cmuClock_LETIMER0, ENABLE);

#endif
}
