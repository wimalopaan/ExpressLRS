#pragma once
#include "mixer.h"
#include "gyro.h"
#include "mode_rate.h"

class SafeController: public RateController
{
    public:
        void    initialize(gyro_mode_t mode);
        void    calculate_pid(float roll_in, float pitch_in, float yaw_in);
        void    printState();

    protected:
        rx_config_gyro_fmode_t fm_angle_settings;
        PID pid_angle_roll, pid_angle_pitch;
};


