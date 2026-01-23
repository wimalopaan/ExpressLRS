#pragma once
#include "mixer.h"
#include "gyro.h"
#include "mode_level.h"

class SafeController: public LevelController
{
    public:
        void    initialize();
        void    calculate_pid();
        float   out(gyro_output_channel_function_t channel_function, float command);
        virtual uint16_t applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, float correction);
};
