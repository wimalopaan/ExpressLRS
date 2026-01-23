#if defined(HAS_GYRO)

#include "gyro.h"
#include "mixer.h"
#include "mode_rate.h"
#include "logging.h"

/**
 * Airplane Normal Mode / Rate Mode
 *
 * This is a basic "wind rejection mode" and counteracts roll and pitch changes.
 *
 * As the channel command increases the correction decreases allowing unlimited
 * angular rates.
 */


void  RateController::applyFModeSettings(gyro_mode_t fm) {
    fm_settings.raw =  config.GetGyroFMode(GYRO_MODE_OFF)->raw; // Default settings for ALL

    auto rate_settings = config.GetGyroFMode(fm); // Get settings for Specific flight Mode

    if (rate_settings->val.angleLimitEnable) { // Override
        fm_settings.val.angleLimitPitch = rate_settings->val.angleLimitPitch;
        fm_settings.val.angleLimitRoll = rate_settings->val.angleLimitRoll;
    } else {
         fm_settings.val.angleLimitPitch=0;
         fm_settings.val.angleLimitRoll=0;
    }

    if (rate_settings->val.trimEnable) { // Add to Defaults, that is Level Flight
        fm_settings.val.trimPitch += rate_settings->val.trimPitch;
        fm_settings.val.trimRoll += rate_settings->val.trimRoll;
    }

    if (rate_settings->val.gainEnable) { // Override
        fm_settings.val.gainPitch = rate_settings->val.gainPitch;
        fm_settings.val.gainRoll = rate_settings->val.gainRoll;
        fm_settings.val.gainYaw = rate_settings->val.gainYaw;
    }

    if (fm_settings.val.angleLimitPitch+fm_settings.val.angleLimitRoll > 0) {
        DBGLN("Angle Limits: Pitch=%d Roll=%d",(int8_t)fm_settings.val.angleLimitPitch, (int8_t)fm_settings.val.angleLimitRoll);
    }

    if (fm_settings.val.trimPitch+fm_settings.val.trimRoll != 0) {
       DBGLN("Trims: Pitch=%d Poll=%d",(int8_t)fm_settings.val.trimPitch, (int8_t)fm_settings.val.trimRoll);
    }

    if (rate_settings->val.gainEnable) {
      DBGLN("Gains Override!"); 
    }

}

bool RateController::isInverted() {
    float angleDeg = fabs(radToDeg(gyro.rpy[GYRO_AXIS_ROLL]));
    return angleDeg > 100;
}


void RateController::initialize()
{
    applyFModeSettings(GYRO_MODE_RATE);
    configure_pids(1.0, 1.0, 1.0, &fm_settings);
    

    // For rate mode we have a basic derivative from the gyro which is the
    // angular velocity. Therefor we turn of any derivitive term.
    // pid_pitch._Kd = 0;
    // pid_roll._Kd = 0;
    // pid_yaw._Kd = 0;
}

void RateController::calculate_pid()
{
    // Desired angular rate is zero
    pid_roll.calculate(0, gyro.f_gyro[0]);
    pid_pitch.calculate(0, gyro.f_gyro[1]);
    pid_yaw.calculate(0, -gyro.f_gyro[2]);
}

float RateController::out(
    gyro_output_channel_function_t channel_function,
    float command
) {
    switch (channel_function)
    {
    case FN_AILERON:
        return pid_roll.output;

    case FN_ELEVATOR:
        return pid_pitch.output;

    case FN_RUDDER:
        return pid_yaw.output;

    default: ;
    }
    return 0.0;
}

uint16_t RateController::applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, float correction) {
    // Limit correction as set from gain input channel
    correction *= gyro.master_gain;

    // Modulate the correction depending on how much axis stick command
    float stick_gain_div = (1 << stick_priority); // 1,2,4

    float percent = (1/stick_gain_div)-fabs(command);
    if (percent < 0) percent = 0.0;

    correction *= percent;


    // Limit of min and max µS values is done in devServoOutput
    return float_to_us(ch, command + correction);
}

#endif
