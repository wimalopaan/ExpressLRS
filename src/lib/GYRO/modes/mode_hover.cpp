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

void HoverController::calculate_pid(float roll_in, float pitch_in, float yaw_in)
{
    RateController::calculate_pid(roll_in,pitch_in, yaw_in);
    
    float pitchRad = gyro.rpy[GYRO_AXIS_PITCH];
    float error = pitchRad - M_PI_2; // Pi/2 = 90degrees
    error *= (float) hoverStrength / 16;

    pitch_cor += (error * cos(pitchRad));
    yaw_cor   += (error * sin(pitchRad));
}

#endif
