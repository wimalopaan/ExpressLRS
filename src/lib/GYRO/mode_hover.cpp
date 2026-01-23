#include "mode_hover.h"
#include "config.h"

#if defined(HAS_GYRO)
/**
 * Airplane Hover Mode
 *
 * For hover mode we care about two angles
 *
 * 1. The "pitch" angle of the horizon.
 *    This is the amount of error.
 *
 * 2. Rotation around the "roll" axis.
 *    This is the modulation between elevator and rudder to correct the error.
 *
 * With these angles we can command elevator and rudder to correct towards a nose
 * directly up attitude.
 */

static int16_t hoverStrength ;

void HoverController::initialize() {
    applyFModeSettings(GYRO_MODE_HOVER);
    
    configure_pids(1.0, 1.0, 1.0, &fm_settings);
    hoverStrength = 10; // config.GetGyroHoverStrength();
}

void HoverController::calculate_pid()
{
    pid_roll.calculate(0, gyro.f_gyro[0]);
    pid_pitch.calculate(0, gyro.f_gyro[1]);
    pid_yaw.calculate(0, -gyro.f_gyro[2]);
}

float HoverController::out(
    gyro_output_channel_function_t channel_function,
    float command
)
{
    float correction = 0.0;
    float error = gyro.ypr[1] - M_PI_2; // Pi/2 = 90degrees
    error *= (float) hoverStrength / 16;

    switch (channel_function)
    {
    case FN_AILERON:
        return pid_roll.output;

    case FN_ELEVATOR:
        return pid_pitch.output + (error * cos(gyro.ypr[2]));

    case FN_RUDDER:
        return pid_yaw.output + (error * sin(gyro.ypr[2]));

    default: ;
    }

    return correction;
}

uint16_t HoverController::applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, float correction) {
    // Limit correction as set from gain input channel
    correction *= gyro.master_gain;

    // Modulate the correction depending on how much axis stick command
    correction *= 1 - fabs(command);

    // Limit of min and max µS values is done in devServoOutput
    return float_to_us(ch, command + correction);
}
#endif
