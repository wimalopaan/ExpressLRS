#include "gyro.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "crsf_protocol.h"

#include "mode_level.h"
#include "mixer.h"
#include "pid.h"
#include "gyro_types.h"
#include "logging.h"

/**
 * Airplane Level/Stable Mode
 *
 * This mode tries to keep the plane flying level (pitch/roll) when there is no
 * stick input.
 *
 */

void LevelController::initialize(gyro_mode_t mode)
{
    RateController::initialize(mode);

    fm_angle_settings.raw =  config.GetGyroFMode(mode)->raw; // Default settings for ALL

    const rx_config_gyro_PID_t *roll_pid_params     = config.GetGyroPID(GYRO_AXIS_ROLL);
    const rx_config_gyro_PID_t *pitch_pid_params    = config.GetGyroPID(GYRO_AXIS_PITCH);
   
    float roll_limit = 1.0;
    float pitch_limit = 1.0;
    float yaw_limit = 1.0;

    configure_pid_gains(&pid_angle_roll,  roll_pid_params,    fm_angle_settings.val.gainRoll,  roll_limit, -1.0 * roll_limit);
    configure_pid_gains(&pid_angle_pitch, pitch_pid_params,   fm_angle_settings.val.gainPitch,  pitch_limit, -1.0 * pitch_limit);
}

void LevelController::calculate_pid(float roll_in, float pitch_in, float yaw_in)
{
    RateController::calculate_pid(roll_in, pitch_in, yaw_in);

    // Get Pitch/Roll angles adjusted to the Level trims  (global + current senttings)
    float pitch_angle = - gyro.rpy[GYRO_AXIS_PITCH] + degToRad(fm_settings.val.trimPitch);
    float roll_angle  = gyro.rpy[GYRO_AXIS_ROLL] + degToRad(fm_settings.val.trimRoll);

    // Angle Demand
    // The stick tell the percentage of the max angle where we want the plane to be
    float setpoint_pitch = (pitch_in * degToRad(fm_angle_settings.val.angleLimitPitch));
    float setpoint_roll  = -roll_in  * degToRad(fm_angle_settings.val.angleLimitRoll);

    pid_angle_pitch.calculate(setpoint_pitch,pitch_angle);
    pid_angle_roll.calculate(setpoint_roll,roll_angle);
    if (isInverted()) pid_angle_pitch.reset();

    // Add angle correction to rate corrections ajusted to angle Gains
    roll_cor  += pid_angle_roll.output;
    pitch_cor += pid_angle_pitch.output;

    // In Level mode, the Gyro has full control, not the command
    roll_ignore_command = pitch_ignore_command = true; 
}

void LevelController::printState() {
    RateController::printState();

    DBGLN("IgnoreCmd:  Roll:%d Pitch:%d ", roll_ignore_command, pitch_ignore_command);
    DBGLN("Ang Corr:   Roll:%f Pitch:%f", pid_angle_roll.output, pid_angle_pitch.output);

}

#endif
