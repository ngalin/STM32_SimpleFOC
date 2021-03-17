/*
 * time_utils.cpp
 * Ported from: https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/time_utils.cpp
 */

#include "time_utils.h"

// function buffering delay()
void _delay(unsigned long ms){
	HAL_Delay(ms);

}

// function buffering _micros()
unsigned long _micros(){
	return HAL_GetTick()*1000; //get microseconds
}
