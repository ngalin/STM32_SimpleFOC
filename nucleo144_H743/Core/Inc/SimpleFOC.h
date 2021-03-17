/*
 * SimpleFOC.h
 * Ported from: https://github.com/simplefoc/Arduino-FOC/blob/master/src/SimpleFOC.h
 */

#ifndef SIMPLEFOC_H_
#define SIMPLEFOC_H_

/* Only supports the following setup:
 * BLDC motor with 15 pole pairs.
 * 3PWM driver on pins: 9, 5,6
 * Enable driver pin: 8
 * Encoder: pins(2,3), quadrature, no pullups, 800 PPR
 * Microcontroller: STM32H743ZI, see: https://www.st.com/resource/en/user_manual/dm00499160-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf
 */

#include "BLDCMotor.h"
#include "../src/motor_drivers/BLDCDriver3PWM.h"
#include "../src/sensors/Encoder.h"

#endif /* SIMPLEFOC_H_ */
