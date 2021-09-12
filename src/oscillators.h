/*
 * oscillators.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 */

#ifndef SRC_OSCILLATORS_H_
#define SRC_OSCILLATORS_H_

#include "app.h"
#include "em_cmu.h"

/* Function Prototypes */

/***************************************************************************//**
 * @name OscillatorInit
 *
 * @brief
 *   Initializes LFXO for EM0/1/2 and ULFRCO for EM3.
 *
 * @param[in] osc
 *   none
 *
 * @return void
 ******************************************************************************/
void OscillatorInit();

#endif /* SRC_OSCILLATORS_H_ */
