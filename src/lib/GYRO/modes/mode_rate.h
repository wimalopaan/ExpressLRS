#pragma once
#include "gyro.h"


class RateController: public ModeController
{
    public:
        RateController();
        void    initialize(gyro_mode_t mode);
       
        void    applyFModeSettings(gyro_mode_t fm);
        void    calculate_pid(float roll_in, float pitch_in, float yaw_in);
        void    calculate_stick_pri(float roll_in, float pitch_in, float yaw_in);
        virtual uint16_t applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command,bool inverted);
        bool    isInverted();
        void    printState();
    protected:
        void    configure_pids(float roll_limit, float pitch_limit, float yaw_limit, const rx_config_gyro_fmode_t *fm);
        void    configure_pid_gains(PID *pid, const rx_config_gyro_PID_t *pid_params, int8_t gain, float max, float min);
        
        gyro_mode_t  mode;
        rx_config_gyro_fmode_t fm_settings;
        float roll_stick_pri,pitch_stick_pri,yaw_stick_pri;
        bool  roll_ignore_command,pitch_ignore_command;
        PID pid_roll, pid_pitch, pid_yaw;
        float roll_in, pitch_in, yaw_in;
};
