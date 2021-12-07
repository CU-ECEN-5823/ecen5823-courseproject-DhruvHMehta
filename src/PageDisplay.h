/*
 * PageDisplay.h
 *
 *  Created on: Nov 28, 2021
 *      Author: Dhruv
 */

#ifndef SRC_PAGEDISPLAY_H_
#define SRC_PAGEDISPLAY_H_

#include "lcd.h"
#include "string.h"
#include "stdlib.h"

void PrintDisplay();
void scrollUp();
void scrollDown();
void nextPage();
void prevPage();
void clearDisplay();

#endif /* SRC_PAGEDISPLAY_H_ */
