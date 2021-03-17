/*
 * time_utils.h
 * Ported from: https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/time_utils.h
 */

#ifndef TIME_UTILS_H_
#define TIME_UTILS_H_

#include "stm32h7xx_hal.h"

/**
 * Function implementing delay() function in milliseconds
 * - blocking function
 * - hardware specific
 * @param ms number of milliseconds to wait
 */
void _delay(unsigned long ms);

/**
 * Function implementing timestamp getting function in microseconds
 * hardware specific
 */
unsigned long _micros();

#endif /* TIME_UTILS_H_ */
