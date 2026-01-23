#include "targets.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "gyro.h"
#include "gyro_types.h"
#include "mixer.h"
//#include "device.h"
#include "mpu_mpu6050.h"
#include "mode_level.h"
#include "mode_hover.h"
#include "mode_rate.h"
#include "mode_safe.h"
#include "CRSFRouter.h"
#include "logging.h"

// Channel Data
extern uint32_t ChannelData[CRSF_NUM_CHANNELS];


PID pid_roll  = PID(1, -1, 0.0, 0.0, 0.0);
PID pid_pitch = PID(1, -1, 0.0, 0.0, 0.0);
PID pid_yaw   = PID(1, -1, 0.0, 0.0, 0.0);

#define GYRO_SUBTRIM_INIT_SAMPLES 10
static uint8_t stick_subtrim_cycles = 0;
static rx_config_pwm_limits_t temp_limits[PWM_MAX_CHANNELS] = {};

// Must match mixer.h: gyro_input_channel_function_t
static const char* STR_gyroInputChannelMode[] = {"None","Roll","Pitch","Yaw","Mode","Gain"};
// Must match mixer.h: gyro_output_channel_function_t
static const char* STR_gyroOutputChannelMode[] = {"None","Aileron","Elevator","Rudder","Elevon","V Tail"};
// Must match gyro.h gyro_mode_t
static const char* STR_gyroMode[] = {"Off","Rate","SAFE","Level","Launch","Hover"};
// Must match gyro_axis_t
static const char* STR_gyroAxis[] = {"Roll","Pitch","Yaw"}; 

volatile gyro_event_t gyro_event = GYRO_EVENT_NONE;

static ModeController*  mode_controllers [6] = { };

#ifdef GYRO_BOOT_JITTER
static uint8_t boot_jitter_times = 0;
static uint32_t boot_jitter_time = 0;
static int8_t boot_jitter_offset = GYRO_BOOT_JITTER_US;

static bool boot_jitter(uint16_t *us)
{
    if (boot_jitter_times > GYRO_BOOT_JITTER_TIMES)
        return false;

    if ((millis() - boot_jitter_time) > GYRO_BOOT_JITTER_MS)
    {
        boot_jitter_times++;
        boot_jitter_time = millis();
        boot_jitter_offset *= -1;
    }

    *us = *us + boot_jitter_offset;
    return true;
}
#endif

void Gyro::init()
{
    DBGLN("Gyro:Init()");
    mode_controllers[GYRO_MODE_OFF] = nullptr;
    mode_controllers[GYRO_MODE_RATE] = new RateController();
    mode_controllers[GYRO_MODE_LEVEL] = new LevelController();
    mode_controllers[GYRO_MODE_LAUNCH] = new LevelController();
    mode_controllers[GYRO_MODE_SAFE] = new SafeController();
    mode_controllers[GYRO_MODE_HOVER] = new HoverController();

    mode_controller = nullptr;
}

gyro_status_t Gyro::getStatus() 
{
    if (!config.GetGyroEnabled()) return GYRO_STATUS_OFF;
    if (mpuDev== nullptr) return GYRO_STATUS_NOT_DETECTED;
    if (!mpuDev->isRunning()) return GYRO_STATUS_NEED_CALIBRATION; 
    return GYRO_STATUS_OK;
}

void Gyro::calibrate()
{
    //mpuDev->calibrate();
    #ifdef GYRO_BOOT_JITTER
    boot_jitter_times = 0;
    boot_jitter_time = 0;
    #endif
}

void Gyro::detect_mode(uint16_t us)
{
    const rx_config_gyro_mode_pos_t *modes = config.GetGyroModePos();
    const uint16_t width = (GYRO_US_MAX - GYRO_US_MIN) / 5;
    uint8_t channel_position = (us - GYRO_US_MIN) / width;
    channel_position = channel_position > 4 ? 4 : channel_position;
    gyro_mode_t selected_mode;
    switch (channel_position)
    {
        case 0: selected_mode = (gyro_mode_t) modes->val.pos1; break;
        case 1: selected_mode = (gyro_mode_t) modes->val.pos2; break;
        case 2: selected_mode = (gyro_mode_t) modes->val.pos3; break;
        case 3: selected_mode = (gyro_mode_t) modes->val.pos4; break;
        case 4: selected_mode = (gyro_mode_t) modes->val.pos5; break;
        default: selected_mode = GYRO_MODE_OFF; break;
    }
    if (gyro_mode != selected_mode)
        switch_mode(selected_mode);
}

/**
 * Trigger a gyro re-initialization of the current gyro mode
*/
void Gyro::reload()
{
    DBGLN("Gyro:Reload()");
    initialized = false;
    if (!config.GetGyroEnabled()) return; //not enabled
    if (mpuDev== nullptr) return; // No Gyro Detected
    
    gyro_mode = GYRO_MODE_OFF;
    learn_state = GYRO_LEARN_OFF;
    mpuDev->start();
    initialized = mpuDev->isRunning();
}

void Gyro::switch_mode(gyro_mode_t mode)
{
    DBGLN("Gyro: Switching mode=[%s]", STR_gyroMode[mode]);
    DBGLN("Gyro: Master Gain=[%f]", master_gain);

    gyro_mode = mode;
    mode_controller = mode_controllers[mode];

    if (mode_controller != nullptr) {
        mode_controller->initialize();
    }
}

void Gyro::detect_gain(uint16_t us)
{
    master_gain = (us_command_to_float(us) + 1) / 2;
    //master_gain = (float(us - GYRO_US_MIN) / (GYRO_US_MAX - GYRO_US_MIN)) * 500;
}


/**
 * Apply gyro servo output mixing and detect gyro mode
 */
void Gyro::mixer(uint8_t ch, uint16_t *us)
{
    // We get called before the gyro configuration is initialized
    if (!initialized) return;

    if (learn_state != GYRO_LEARN_OFF) {
        learn_sticks(ch,*us);
        return;
    }


    auto ch_info = config.GetGyroChannel(ch);
    auto output_mode = (gyro_output_channel_function_t) ch_info->val.output_mode;
    auto input_mode = (gyro_input_channel_function_t) ch_info->val.input_mode;

    if (input_mode == FN_IN_NONE && output_mode == FN_NONE)
        return;

    #ifdef GYRO_BOOT_JITTER
    if (boot_jitter(us))
        return;
    #endif

    switch (input_mode)
    {
    case FN_IN_GYRO_MODE:
        detect_mode(*us);
        return;
    case FN_IN_GYRO_GAIN:
        detect_gain(*us);
        return;
    default:
        break;
    }

    if (output_mode == FN_NONE)
        return;

    // Normalize the µs value to a +-1.0 keeping in mind subtrim and max throws
    float command = us_command_to_float(ch, *us);
    float correction = 0.0;

     if (mode_controller != nullptr) {
        correction = mode_controller->out(output_mode, command); 
        
        // Gyro specific channel inversion. We might remove this later if not needed.
        if (ch_info->val.inverted) {
            correction *= -1;
        }

        *us = mode_controller->applyCorrection(ch, output_mode, command, correction);
    }
}

static int16_t decidegrees2Radians10000(int16_t angle_decidegree)
{
    while (angle_decidegree > 1800)
    {
        angle_decidegree -= 3600;
    }
    while (angle_decidegree < -1800)
    {
        angle_decidegree += 3600;
    }
    return (int16_t)((M_PI / 180.0f) * 1000.0f * angle_decidegree);
}

void Gyro::send_telemetry()
{
    // Get yaw/pitch/roll in decidegrees and convert to uint16_t
    uint16_t ypr16[3] = {0};
    ypr16[0] = (uint16_t)(gyro.ypr[0] * 1800 / M_PI);
    ypr16[1] = (uint16_t)(gyro.ypr[1] * 1800 / M_PI);
    ypr16[2] = (uint16_t)(gyro.ypr[2] * 1800 / M_PI);

    CRSF_MK_FRAME_T(crsf_sensor_attitude_t)
    crsfAttitude = {0};
    crsfAttitude.p.pitch = htobe16(decidegrees2Radians10000(ypr16[1]));
    crsfAttitude.p.roll = htobe16(decidegrees2Radians10000(ypr16[2]));
    crsfAttitude.p.yaw = htobe16(decidegrees2Radians10000(ypr16[0]));

    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfAttitude, CRSF_FRAMETYPE_ATTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfAttitude.h);

    CRSF_MK_FRAME_T(crsf_flight_mode_t)
    crsfFlightMode = {0};

    strcpy(crsfFlightMode.p.flight_mode, STR_gyroMode[gyro_mode]);

    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfFlightMode, CRSF_FRAMETYPE_FLIGHT_MODE, CRSF_FRAME_SIZE(sizeof(crsf_flight_mode_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_CRSF_TRANSMITTER, &crsfFlightMode.h);
}

/*
bool Gyro::read_device()
{
    if (dev->read())
    {
        // Calculate gyro update rate in HZ
        update_rate = 1.0 /
                      ((micros() - last_update) / 1000000.0);

        last_update = micros();
        send_telemetry();
    }
    return DURATION_IMMEDIATELY;
}
*/

#define GYRO_PID_DEBUG_TIME 3000  // Time im Ms



#ifdef GYRO_PID_DEBUG_TIME
unsigned long gyro_debug_time = 0;

void _make_gyro_debug_string(PID *pid, char *str) {
    sprintf(str, "Setpoint: %5.2f PV: %5.2f I:%5.2f D:%5.2f Error: %5.2f Out: %5.2f",
        pid->setpoint, pid->pv, pid->Iout, pid->Dout, pid->error, pid->output);

}
#endif

void Gyro::tick()
{
    if (!initialized) return;

    if (gyro.mpuDev->read()) {
        gyro.last_update = micros();
        gyro.send_telemetry();
    }
    
    if ((micros() - pid_delay) < 1000 ) return; // ~1k PID loop
    pid_delay = micros();

    if (mode_controller != nullptr) {
        mode_controller->calculate_pid();
    }


    #ifdef GYRO_PID_DEBUG_TIME
    if (gyro_mode != GYRO_MODE_OFF &&
        micros() - gyro_debug_time > GYRO_PID_DEBUG_TIME * 1000
    ) {
        DBGLN("MASTER GAIN %f", master_gain);
        DBGLN("Angles:  Roll:%f Pitch:%f Yaw:%f", radToDeg(gyro.rpy[0]), radToDeg(gyro.rpy[1]), radToDeg(gyro.rpy[2]));

        char piddebug[128];
        _make_gyro_debug_string(&pid_pitch, piddebug);
        DBGLN("PID Pitch %s", piddebug);
        _make_gyro_debug_string(&pid_roll, piddebug);
        DBGLN("PID Roll  %s", piddebug);
        _make_gyro_debug_string(&pid_yaw, piddebug);
        DBGLN("PID Yaw   %s", piddebug);
        gyro_debug_time = micros();
    }
    #endif
}

void Gyro::learn_sticks(uint8_t ch, uint16_t us) {
    if (learn_state== GYRO_LEARN_SUBTRIMS) {
        // Set midpoint (subtrim) from an average of a set of samples
        if (ch == 0 && ++stick_subtrim_cycles > GYRO_SUBTRIM_INIT_SAMPLES) {
            // Completed
            learn_state = GYRO_LEARN_OFF;
            return;
        }

        // Average over 10 cycles
        if (stick_subtrim_cycles < GYRO_SUBTRIM_INIT_SAMPLES) {
            auto ch_limit = &temp_limits[ch];
            ch_limit->val.mid = (((ch_limit->val.mid * stick_subtrim_cycles) / stick_subtrim_cycles) + us) / 2;
        }
    }
    else 
    if (learn_state== GYRO_LEARN_LIMIT_START) {
        auto ch_limit = &temp_limits[ch];

        if (us < ch_limit->val.min) {
            ch_limit->val.min = us;
        }
        if (us > ch_limit->val.max) {
            ch_limit->val.max = us;
        }
    }
}

void Gyro::StickCenterCalibration() {

    DBGLN("Gyro(): Stick Center Calibration (Init)");
    stick_subtrim_cycles = 0;

    //initialize min,max, mid
    for (int ch=0;ch<PWM_MAX_CHANNELS;ch++) {
        auto ch_limit = &temp_limits[ch];
        ch_limit->val.mid = 1500;
        ch_limit->val.min = 1500;
        ch_limit->val.max = 1500;
    }

    learn_state = GYRO_LEARN_SUBTRIMS;
}

void Gyro::StickLimitCalibration(bool done)
{
   DBGLN("Gyro(): Stick Range Calibration (%s)",done?"Complete":"Started");

   if (done) {
        learn_state = GYRO_LEARN_LIMIT_DONE;
        // save the Range
        for (int ch=0;ch<PWM_MAX_CHANNELS;ch++) {
            auto pwm_limits =  &temp_limits[ch];
            DBGLN("Ch%d: Min: %d Max: %d Center: %d", 
                ch, (uint16_t) pwm_limits->val.min, (uint16_t) pwm_limits->val.max, (uint16_t) pwm_limits->val.mid);
            config.SetPwmChannelLimitsRaw(ch,pwm_limits->raw);
        }
        config.Commit();
   } else {
        learn_state = GYRO_LEARN_LIMIT_START;
   }
}



void configure_pids(float roll_limit, float pitch_limit, float yaw_limit, const rx_config_gyro_fmode_t *fm)
{
    const rx_config_gyro_PID_t *roll_pid_params     = config.GetGyroPID(GYRO_AXIS_ROLL);
    const rx_config_gyro_PID_t *pitch_pid_params    = config.GetGyroPID(GYRO_AXIS_PITCH);
    const rx_config_gyro_PID_t *yaw_pid_params      = config.GetGyroPID(GYRO_AXIS_YAW);

    configure_pid_gains(&pid_roll,  roll_pid_params,    fm->val.gainRoll,   roll_limit, -1.0 * roll_limit);
    configure_pid_gains(&pid_pitch, pitch_pid_params,   fm->val.gainPitch,  pitch_limit, -1.0 * pitch_limit);
    configure_pid_gains(&pid_yaw,   yaw_pid_params,     fm->val.gainYaw,    yaw_limit, -1.0 * yaw_limit);
}

void configure_pid_gains(PID *pid, const rx_config_gyro_PID_t *pid_params, int8_t gain,
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

#endif
