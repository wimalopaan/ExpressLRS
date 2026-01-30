#pragma once
#if defined(HAS_GYRO)

#include "mixer.h"
#include "gyro.h"
#include "mode_rate.h"


class HoverController: public RateController
{
    public:
        void    initialize(gyro_mode_t mode);
        void    calculate_pid(float roll_in, float pitch_in, float yaw_in);
};

#endif