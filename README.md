# STM32_SimpleFOC
Port of SimpleFOC (https://github.com/simplefoc/Arduino-FOC/tree/v2.0.2) to STM32 environment.

Current hardware used:
* SimpleFOCShield v1.3.3
* Nucleo-H743ZI (caution board number is: MB1364, which means this is the correct user manual: 
https://www.st.com/resource/en/user_manual/dm00499160-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf

Things to fix/change/add/general thoughts on future improvements/work:

* change PWM pins on driver board to all use TIM1. 
	Advantage of this is avoiding timer synchronisation issues. 
	At the moment use default pins 9/6/5 for PWM outputs. These pins map to PD15/PE9/PE11 on the STM32 board respectively, 
	and these pins have the following timer connections: TIM4(CH4) / TIM1(CH1) / TIM1(CH2). 
	Suggested change:
	
	| Signal    | Arduino pin name | STM32 pin name | Function    |
	| --------- | ---------------- | -------------- | ----------- |
	| PWM A     | 3                | PE13           | TIM1(CH3)   |
	| PWM B     | 6                | PE9            | TIM1(CH1)   |
	| PWM C     | 5                | P11            | TIM1(CH2)   |
	| Enable    | 8                | PF3            | GPIO_Output |
	| Encoder A | A2               | PC3            | GPIO_EXTI3  |
| Encoder B | A1               | PC0            | GPIO_EXTI0  |
	
	
	
* Avoid pulse_counter overflow. Count number of rotations, and keep pulse_counter between 0 - CPR

* Switch to dedicated timer hardware for encoder pulse count. Requires hardware signal re-routing. 
				

Unresolved questions:
* Operating in FOC position control, encoder pulse counter does not increment properly when motor is externally
pushed out of position. Hence, motor can be successfully pushed away from target position set point. Would expect this
not to happen. Something to do with electrical angle normalisation?
* When switch to SimpleFOCShield v2.0.2 (using inline phase current sensing) make sure to align encoder 0 count to Iq = 0.

