#pragma once
#include "gyro.h"


class RateController: public ModeController
{
    public:
        void    initialize();
        void    applyFModeSettings(gyro_mode_t fm);
        void    calculate_pid();
        float   out(gyro_output_channel_function_t channel_function, float command);
        virtual uint16_t applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, float correction);
        bool    isInverted();
    protected:
        rx_config_gyro_fmode_t fm_settings;
};
