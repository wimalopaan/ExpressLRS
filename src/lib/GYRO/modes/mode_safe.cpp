#include "gyro.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "crsf_protocol.h"

#include "mode_safe.h"
#include "mixer.h"
#include "pid.h"
#include "gyro_types.h"
#include "logging.h"

/**
 * Airplane SAFE Mode
 *
 * This mode will prevent the plane from exceding the MAX angles.
 *
 */

 typedef enum {
    ANGLE_STATE_OFF=0,
    ANGLE_STATE_AT_MAX,
    ANGLE_STATE_REVERSING
 } AngleLockState;


 /**
 * Manage a state machine when we reach an angle.
 * We should be able to get to the desire max angle, but after that, disable the stick 
 * action (more angle), until the stick direction changes.. Then the stick becomes effective again.
 * Similar Behaviour to Spektrum AS3X/SAFE 
 */
 class AngleLock {
    public:
        AngleLockState state = ANGLE_STATE_OFF;
        int8_t cmd_dir = 0;
        bool   ignore_cmd = false;
        
        void reset() {
            state = ANGLE_STATE_OFF;
            cmd_dir = 0;
            ignore_cmd = false;
        }

        int8_t ignoreCommand() { return ignore_cmd; }
        
        /* manage gyro at different states */
        void compute_pid (PID *pid, float angle, float max_angle, float cmd_in) {
            if (state==ANGLE_STATE_OFF) { // Still below angle limits
                if (abs(angle) > max_angle) { // Past angle limit??
                    float setpoint = angle > 0 ? max_angle : - max_angle;
                    pid->calculate(setpoint,angle);
                    cmd_dir = (cmd_in<0?-1:+1); // Save the direction of the stick input
                    state = ANGLE_STATE_AT_MAX; // At Angle Limit
                    DBGLN("Safe(): MAX Angle Locked");
                } else {
                    // Normal operation, below angle limit
                    pid->reset();
                }
            } else
            if (state==ANGLE_STATE_AT_MAX) { // Above angle limit
                 ignore_cmd = true; // Ignore the stick input
                 float setpoint = angle > 0 ? max_angle : - max_angle;
                 pid->calculate(setpoint,angle);
                 int8_t stick_dir = (cmd_in<0?-1:+1); // Sign/direction of stick
                 if (stick_dir!=cmd_dir) { // Stick change direction past middle??
                    // Stick trying to degrease bank angle
                    DBGLN("Safe(): Dectected Stick Reversal");
                    state=ANGLE_STATE_REVERSING; // Rolling back
                 }
            } 
            if (state==ANGLE_STATE_REVERSING) { // Reversing Dir, wait until we decrease angle below max to reset state
                 ignore_cmd = false; // stick is effective again, decreasing angle 
                 pid->reset();
                 if (abs(angle) < max_angle) { // Below max angle??
                    DBGLN("Safe(): Back to normal Angle");
                    state = ANGLE_STATE_OFF;  // back to normal
                 } 
            } 
        };
 };


static AngleLock AngleLockPitch;
static AngleLock AngleLockRoll;

void SafeController::initialize(gyro_mode_t mode)
{
    RateController::initialize(mode);

    fm_angle_settings.raw =  config.GetGyroFMode(mode)->raw; // Default settings for ALL

    const rx_config_gyro_PID_t *roll_pid_params     = config.GetGyroPID(GYRO_AXIS_ROLL);
    const rx_config_gyro_PID_t *pitch_pid_params    = config.GetGyroPID(GYRO_AXIS_PITCH);
   
    float roll_limit = 1.0;
    float pitch_limit = 1.0;

    configure_pid_gains(&pid_angle_roll,  roll_pid_params,    fm_angle_settings.val.gainRoll,  roll_limit,  -1.0 * roll_limit);
    configure_pid_gains(&pid_angle_pitch, pitch_pid_params,   fm_angle_settings.val.gainPitch, pitch_limit, -1.0 * pitch_limit);

    // Reset angle lock state
    AngleLockPitch.reset();
    AngleLockRoll.reset();
}


void SafeController::calculate_pid(float roll_in, float pitch_in, float yaw_in)
{
    RateController::calculate_pid(roll_in, pitch_in, yaw_in);

    // Adjust angle with Level Trims
    float pitch_angle = - gyro.rpy[GYRO_AXIS_PITCH] + degToRad(fm_settings.val.trimPitch);
    float roll_angle  = gyro.rpy[GYRO_AXIS_ROLL] + degToRad(fm_settings.val.trimRoll);

    AngleLockPitch.compute_pid(&pid_angle_pitch, pitch_angle, degToRad(fm_angle_settings.val.angleLimitPitch), pitch_in);
    AngleLockRoll.compute_pid(&pid_angle_roll, roll_angle, degToRad(fm_angle_settings.val.angleLimitRoll), roll_in);

    if (isInverted()) pid_angle_pitch.reset();
    
    // Add angle correction to rate corrections
    roll_cor  += pid_angle_roll.output;
    pitch_cor += pid_angle_pitch.output;

    pitch_ignore_command = AngleLockPitch.ignoreCommand();
    roll_ignore_command = AngleLockRoll.ignoreCommand();

}

void SafeController::printState() {
    RateController::printState();

    DBGLN("IgnoreCmd:  Roll:%d Pitch:%d ", roll_ignore_command, pitch_ignore_command);
    DBGLN("Ang Corr:   Roll:%f Pitch:%f", pid_angle_roll.output, pid_angle_pitch.output);

}

#endif
