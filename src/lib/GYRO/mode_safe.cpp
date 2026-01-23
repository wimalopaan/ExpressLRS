#include "gyro.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "mode_safe.h"
#include "mixer.h"
#include "pid.h"
#include "gyro_types.h"
#include "gyro.h"
#include "mode_level.h"

/**
 * Airplane Safe Mode
 *
 * This allows normal flying, but tries to stop the plane going past set angles.
*/
 
void SafeController::initialize()
{
    applyFModeSettings(GYRO_MODE_SAFE);

    // Set limits to two to be able to fully override a full stick input command
    // on roll and pitch axes
    configure_pids(2.0, 2.0, 1.0, &fm_settings);
}

static void _calculate_pid(PID *pid, float angle, float max_angle)
{
    if (abs(angle) < max_angle) {
        pid->reset();
    } else {
        float setpoint = angle > 0 ? max_angle : - max_angle;
        pid->calculate(setpoint, angle);
    }
}

void SafeController::calculate_pid()
{
    _calculate_pid(&pid_pitch, -gyro.rpy[GYRO_AXIS_PITCH], degToRad(fm_settings.val.trimPitch));
    _calculate_pid(&pid_roll, gyro.rpy[GYRO_AXIS_ROLL], degToRad(fm_settings.val.trimRoll));

    pid_yaw.calculate(0, -gyro.f_gyro[GYRO_AXIS_YAW]);
}

float SafeController::out(
    gyro_output_channel_function_t channel_function,
    float command
) {
    // TODO: Invert elevator if craft is inverted
    // TODO: Modulate elevator and rudder when needed (knife edge nose down)
    switch (channel_function)
    {
    case FN_AILERON:
        return pid_roll.output;

    case FN_ELEVATOR:
        if (isInverted()) pid_pitch.reset();
        return pid_pitch.output;

    case FN_RUDDER:
        return pid_yaw.output;

    default: ;
    }
    return 0.0;
}

uint16_t SafeController::applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, float correction) {
     // Limit of min and max µS values is done in devServoOutput
    return float_to_us(ch, command + correction);
}



#endif
