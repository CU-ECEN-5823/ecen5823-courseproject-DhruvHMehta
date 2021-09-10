/*
 * timers.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Dhruv
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "app.h"
#include "EFR32BG13P632F512GM48.h"
#include "em_letimer.h"


/* Function Prototypes */
void LETIMER0Init();
void LETIMER0InterruptEn();

#endif /* SRC_TIMERS_H_ */
