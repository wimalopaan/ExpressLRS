#pragma once
#include "gyro_types.h"

class Mode_Base
{
    public:
        virtual void    initialize(gyro_mode_t mode);
        virtual void    calculate_pid(float input_rpt[], float acc_rpy[], float ang_rpy[]);
        virtual void    printState();
        virtual uint16_t applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, bool inverted);
    protected:
        gyro_stick_priority_t stick_priority = STICK_PRIORITY_HALF;
        float corr[3];
};