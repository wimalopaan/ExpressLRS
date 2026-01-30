#if defined(HAS_GYRO)

#include "gyro.h"
#include "mixer.h"
#include "mode_rate.h"
#include "pid.h"
#include "logging.h"

/**
 * Airplane Normal Mode / Rate Mode
 *
 * This is a basic "wind rejection mode" and counteracts roll and pitch changes.
 *
 * As the channel command increases the correction decreases allowing unlimited
 * angular rates.
 */

void _make_gyro_debug_string(PID *pid, char *str) {
    sprintf(str, "Setpoint: %5.2f PV: %5.2f I:%5.2f D:%5.2f Error: %5.2f Out: %5.2f",
        pid->setpoint, pid->pv, pid->Iout, pid->Dout, pid->error, pid->output);

}

 RateController::RateController() :
    pid_pitch(),
    pid_roll(),
    pid_yaw()
 {
 }

void RateController::configure_pid_gains(PID *pid, const rx_config_gyro_PID_t *pid_params, int8_t gain,
                         float max, float min)
{
    DBG("Config gains: [P=%d I=%d D=%d G=%d] ", pid_params->p, pid_params->i, pid_params->d, (int8_t) gain);
    if (max == 0.0 && min == 0.0) {
        // not gyro correction on this axis
        pid->configure(0.0, 0.0, 0.0, 0.0, 0.0);
    } else {
        float p = gain * pid_params->p / 1000.0;
        float i = gain * pid_params->i / 1000.0;
        float d = gain * pid_params->d / 1000.0;
        DBG("PID: [P=%f I=%f D=%f]", p, i, d);

        pid->configure(p, i, d, max, min);
    }
    DBGLN("");
    pid->reset();
}

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

    if (fm_settings.val.angleLimitPitch+fm_settings.val.angleLimitRoll > 0) {
        DBGLN("Angle Limits: Pitch=%d Roll=%d",(int8_t)fm_settings.val.angleLimitPitch, (int8_t)fm_settings.val.angleLimitRoll);
    }

    if (fm_settings.val.trimPitch+fm_settings.val.trimRoll != 0) {
       DBGLN("Trims: Pitch=%d Poll=%d",(int8_t)fm_settings.val.trimPitch, (int8_t)fm_settings.val.trimRoll);
    }
}


bool RateController::isInverted() {
    float angleDeg = fabs(radToDeg(gyro.rpy[GYRO_AXIS_ROLL]));
    return angleDeg > 100;
}


void RateController::initialize(gyro_mode_t mode)
{
    RateController::mode = mode;

    float roll_limit = 1.0;
    float pitch_limit = 1.0;
    float yaw_limit = 1.0;

    applyFModeSettings(mode);

    const rx_config_gyro_PID_t *roll_pid_params     = config.GetGyroPID(GYRO_AXIS_ROLL);
    const rx_config_gyro_PID_t *pitch_pid_params    = config.GetGyroPID(GYRO_AXIS_PITCH);
    const rx_config_gyro_PID_t *yaw_pid_params      = config.GetGyroPID(GYRO_AXIS_YAW);

    configure_pid_gains(&pid_roll,  roll_pid_params,    fm_settings.val.gainRoll,   roll_limit, -1.0 * roll_limit);
    configure_pid_gains(&pid_pitch, pitch_pid_params,   fm_settings.val.gainPitch,  pitch_limit, -1.0 * pitch_limit);
    configure_pid_gains(&pid_yaw,   yaw_pid_params,     fm_settings.val.gainYaw,    yaw_limit, -1.0 * yaw_limit);
}

void RateController::calculate_stick_pri(float roll_in, float pitch_in, float yaw_in) {
    // Modulate the correction depending on how much axis stick command
    //float stick_gain_div = (1 << stick_priority); // 1,2,4
    //float percent = (1/stick_gain_div)-fabs(roll_in);
    //if (percent < 0) percent = 0.0;

    // Modulate the correction depending on how much axis stick command
    roll_stick_pri  = 1 - fabs(roll_in);
    pitch_stick_pri = 1 - fabs(pitch_in);
    yaw_stick_pri   = 1 - fabs(yaw_in);
}

void RateController::calculate_pid(float roll_in, float pitch_in, float yaw_in)
{
    pitch_ignore_command = false;
    roll_ignore_command = false;
    RateController::roll_in = roll_in;
    RateController::pitch_in = pitch_in;
    RateController::yaw_in = yaw_in;

    calculate_stick_pri(roll_in,pitch_in, yaw_in);

    // Desired angular rate is zero
    pid_roll.calculate(0, gyro.f_gyro[GYRO_AXIS_ROLL]);
    pid_pitch.calculate(0, gyro.f_gyro[GYRO_AXIS_PITCH]);
    pid_yaw.calculate(0, -gyro.f_gyro[GYRO_AXIS_YAW]);

    roll_cor  = pid_roll.output  * roll_stick_pri  * gyro.master_gain;
    pitch_cor = pid_pitch.output * pitch_stick_pri * gyro.master_gain;
    yaw_cor   = pid_yaw.output   * yaw_stick_pri   * gyro.master_gain;
}

void RateController::printState() {
        char piddebug[128];
        DBGLN("MASTER GAIN %f", gyro.master_gain);
        sprintf(piddebug,"Roll:%5.2f Pitch:%5.2f Yaw:%5.2f", radToDeg(gyro.rpy[0]), radToDeg(gyro.rpy[1]), radToDeg(gyro.rpy[2])); 
        DBGLN("Angles:  %s",piddebug);
        sprintf(piddebug,"Roll:%5.2f Pitch:%5.2f Yaw:%5.2f", roll_in, pitch_in, yaw_in);    
        DBGLN("Cmds:    %s",piddebug);
        sprintf(piddebug,"Roll:%5.2f Pitch:%5.2f Yaw:%5.2f", roll_stick_pri, pitch_stick_pri, yaw_stick_pri);
        DBGLN("StickPri:%s",piddebug);
        sprintf(piddebug,"Roll:%5.2f Pitch:%5.2f Yaw:%5.2f",roll_cor, pitch_cor, yaw_cor);
        DBGLN("Corr    :%s",piddebug);

        _make_gyro_debug_string(&pid_pitch, piddebug);
        DBGLN("PID Pitch %s", piddebug);
        _make_gyro_debug_string(&pid_roll, piddebug);
        DBGLN("PID Roll  %s", piddebug);
        _make_gyro_debug_string(&pid_yaw, piddebug);
        DBGLN("PID Yaw   %s", piddebug);
}

uint16_t RateController::applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, bool inverted) {
    float correction = 0.0;
    if (channel_function==FN_AILERON) {
        correction = (inverted)?-roll_cor:roll_cor;
        if (roll_ignore_command) {
            command = 0;
        }
    } else if (channel_function==FN_ELEVATOR) {
        correction = (inverted)?-pitch_cor:pitch_cor;
        if (pitch_ignore_command) {
            command = 0;
        }
    } 
    else if (channel_function==FN_ELEVON_L || channel_function==FN_ELEVON_R) {
        correction = (inverted)?-pitch_cor:pitch_cor; // Invert elevator if needed
        auto cor2 = (channel_function==FN_ELEVON_L)?roll_cor:-roll_cor; // Invert Aileron depending of Left/Right 
        correction = (correction + cor2)/2;
        if (pitch_ignore_command) {
            command = 0;
        }
    }
    else if (channel_function==FN_VTAIL_L || channel_function==FN_VTAIL_R) {
        correction = (inverted)?-pitch_cor:pitch_cor; // Invert elevator if needed
        auto cor2 = (channel_function==FN_VTAIL_L)?yaw_cor:-yaw_cor; // Invert Rud depending of Left/Right 
        correction = (correction + cor2)/2;
        if (pitch_ignore_command) {
            command = 0;
        }
    }
    else if (channel_function==FN_RUDDER) {
        correction = (inverted)?-yaw_cor:yaw_cor; // Invert rudder if needed
    }

    return float_to_us(ch, command + correction);
}

#endif
