/*
 * BLDCDriver.h
 * Ported from: https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/base_classes/BLDCDriver.h
 */

#ifndef BLDCDRIVER_H_
#define BLDCDRIVER_H_

class BLDCDriver{
    public:

        /** Initialise hardware */
        virtual int init(){};
        /** Enable hardware */
        virtual void enable(){};
        /** Disable hardware */
        virtual void disable(){};

        long pwm_frequency; //!< pwm frequency value in hertz
        float voltage_power_supply; //!< power supply voltage
        float voltage_limit; //!< limiting voltage set to the motor

        /**
         * Set phase voltages to the harware
         *
         * @param Ua - phase A voltage
         * @param Ub - phase B voltage
         * @param Uc - phase C voltage
        */
        virtual void setPwm(float Ua, float Ub, float Uc){};
};

#endif /* BLDCDRIVER_H_ */
