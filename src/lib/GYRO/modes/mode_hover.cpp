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

void HoverController::initialize(gyro_mode_t mode) {
    RateController::initialize(mode);

    hoverStrength = 10; // config.GetGyroHoverStrength();
}

void HoverController::calculate_pid(float input_rpy[], float acc_rpy[], float ang_rpy[])
{
    RateController::calculate_pid(input_rpy, acc_rpy, ang_rpy);
    
    float pitchRad = ang_rpy[GYRO_AXIS_PITCH];
    float error = pitchRad - M_PI_2; // Pi/2 = 90degrees
    error *= (float) hoverStrength / 16;

    corr[GYRO_AXIS_PITCH] += (error * cos(pitchRad));
    corr[GYRO_AXIS_YAW]   += (error * sin(pitchRad));
}

#endif
