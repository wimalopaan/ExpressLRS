#include "gyro.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "crsf_protocol.h"

#include "mode_level.h"
#include "mixer.h"
#include "pid.h"
#include "gyro_types.h"

/**
 * Airplane Level/Stable Mode
 *
 * This mode tries to keep the plane flying level (pitch/roll) when there is no
 * stick input.
 *
 */



void LevelController::initialize()
{
    applyFModeSettings(GYRO_MODE_LEVEL);
    configure_pids(1.0, 1.0, 1.0, &fm_settings);
}

static float channel_command(uint8_t ch)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
    const unsigned crsfVal = ChannelData[chConfig->val.inputChannel];
    uint16_t us = CRSF_to_US(crsfVal);
    return us_command_to_float(ch, us);
}


/**
 * Return the first channel matching input `mode` or -1 if not found.
*/
static int8_t GetGyroInputChannelNumber(gyro_input_channel_function_t mode)
{
    for (int8_t i = 0; i < GYRO_MAX_CHANNELS; i++) {
        auto info =  config.GetGyroChannel(i);
        if (info->val.input_mode == mode)
            return i;
    }
    return -1;
}

/**
 * Return the first channel matching output `mode` or -1 if not found.
*/
static int8_t GetGyroOutputChannelNumber(gyro_output_channel_function_t mode)
{
    for (uint8_t i = 0; i < GYRO_MAX_CHANNELS; i++) {
        auto info =  config.GetGyroChannel(i);
        if (info->val.output_mode == mode)
            return i;
    }
    return -1;
}



void LevelController::calculate_pid()
{
    int8_t channel = GetGyroInputChannelNumber(FN_IN_ROLL);
    if (channel != -1) {
        pid_roll.calculate(
            channel_command(channel) * degToRad(fm_settings.val.trimRoll),
            gyro.rpy[GYRO_AXIS_ROLL]
        );
    }

    channel = GetGyroInputChannelNumber(FN_IN_PITCH);
    if (channel != -1) {
        pid_pitch.calculate(
                channel_command(channel) * degToRad(fm_settings.val.trimPitch),
                // For the pitch access in launch mode (pitch_offset != 0)
                // we change what the PID controllers sees as level
                degToRad(fm_settings.val.trimPitch) - gyro.rpy[GYRO_AXIS_PITCH]
        );
    }

    pid_yaw.calculate(0, -gyro.f_gyro[2]);
}

float LevelController::out(
    gyro_output_channel_function_t channel_function,
    float command
) {
    // TODO: Invert elevator if craft is inverted

    if (channel_function == FN_AILERON) {
        return pid_roll.output;
    } else if (channel_function == FN_ELEVATOR) {
        if (isInverted()) pid_pitch.reset();
        return pid_pitch.output; 
    } else if (channel_function == FN_RUDDER)
        return pid_yaw.output;

    return 0.0;
}

uint16_t LevelController::applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, float correction) {

    //Limit correction as set from gain input channel
    //correction *= gyro.master_gain;

    // In this mode, the correction is the command

    // Modulate the correction depending on how much axis stick command
    float stick_gain_div = (1 << stick_priority); // 1,2,4

    float percent = (1/stick_gain_div)-fabs(command);
    if (percent < 0) percent = 0.0;

    // At Center, Gyro has 100%, and command has 0
    // At Stick Pri, Gyro has 0, command has 100

    correction *= percent;   
    command *= (1-percent);

     return float_to_us(ch, command + correction);
}


void LaunchController::initialize()
{
    applyFModeSettings(GYRO_MODE_LAUNCH); // Same logic as Level, but different angles
    configure_pids(1.0, 1.0, 1.0, &fm_settings);
}


#endif
