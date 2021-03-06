/*
 * BLDCDriver3PWM.h
 * Ported from: https://github.com/simplefoc/Arduino-FOC/blob/master/src/drivers/BLDCDriver3PWM.h
 */

#ifndef BLDCDRIVER3PWM_H_
#define BLDCDRIVER3PWM_H_

#include "../common/base_classes/BLDCDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"

/**
 3 pwm bldc driver class
*/
class BLDCDriver3PWM: public BLDCDriver
{
  public:
    /**
      BLDCDriver class constructor
      @param phA A phase pwm pin
      @param phB B phase pwm pin
      @param phC C phase pwm pin
      @param en1 enable pin
    */
    BLDCDriver3PWM(int phA,int phB,int phC, int en = NOT_SET);

    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // hardware variables
  	int pwmA; //!< phase A pwm pin number
  	int pwmB; //!< phase B pwm pin number
  	int pwmC; //!< phase C pwm pin number
    int enable_pin; //!< enable pin number

    /**
     * Set phase voltages to the harware
     *
     * @param Ua - phase A voltage
     * @param Ub - phase B voltage
     * @param Uc - phase C voltage
    */
    void setPwm(float Ua, float Ub, float Uc) override;

  private:

};

#endif /* BLDCDRIVER3PWM_H_ */
