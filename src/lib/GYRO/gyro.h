#pragma once
#include "targets.h"

#if defined(HAS_GYRO)
#include "device.h"
#include "config.h"
#include "pid.h"
#include "gyro_types.h"
#include <math.h>
#include "helper_3dmath.h"

#define GYRO_US_MIN 988
#define GYRO_US_MID 1500
#define GYRO_US_MAX 2012

#define radToDeg(angleInRadians) ((angleInRadians) * RAD_TO_DEG)
#define degToRad(angleInDegrees) ((angleInDegrees) * DEG_TO_RAD)

/**
 * Add some servo jitter feedback to the pilot after the gyro has initialized.
 */
#define GYRO_BOOT_JITTER
#ifdef GYRO_BOOT_JITTER
#define GYRO_BOOT_JITTER_US 45
#define GYRO_BOOT_JITTER_MS 175
#define GYRO_BOOT_JITTER_TIMES 4
#endif

class  MPUDevice;

class ModeController
{
    public:
        virtual void    initialize(gyro_mode_t mode);
        virtual void    calculate_pid(float roll_in, float pitch_in, float yaw_in);
        virtual void    printState();
        virtual uint16_t applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, bool inverted);
    protected:
        gyro_stick_priority_t stick_priority = STICK_PRIORITY_HALF;
        float roll_cor, pitch_cor, yaw_cor;
};



class Gyro
{
public:
    void init();
    gyro_status_t getStatus();
    gyro_mode_t getMode(void);
    void mixerInput();
    void mixerOutput(uint8_t ch, uint16_t *us);
    void send_telemetry();
    //bool read_device();
    void tick();
    void calibrate();
    void reload();
    void StickCenterCalibration();
    void StickLimitCalibration(bool done);

    MPUDevice *mpuDev = nullptr;

    float master_gain = 1.0;
     gyro_mode_t gyro_mode;
// protected:

    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorInt16 aa;      // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorInt16 v_gyro;
    VectorFloat gravity; // [x, y, z]            gravity vector    
    float euler[3];      // [psi, theta, phi]    Euler angle container
    float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container

    float f_gyro[3];     // roll/pitch/yaw
    float rpy[3];        // [roll, pitch, yaw] 

    uint16_t update_rate;
    unsigned long last_update;
    bool initialized;
    gyro_learn_state_t learn_state = GYRO_LEARN_OFF;

private:
   
    ModeController* mode_controller;

    int8_t  mode_ch   = -1;
    int8_t  gain_ch   = -1;
    int8_t  roll_ch   = -1;
    int8_t  pitch_ch  = -1;
    int8_t  yaw_ch    = -1;

    void detect_gain(uint16_t us);
    void detect_mode(uint16_t us);
    void switch_mode(gyro_mode_t mode);
    void learn_sticks(uint8_t ch, uint16_t us);

    unsigned long pid_delay=0;
};

extern Gyro gyro;

// configure PID controllers from LUA gains for each axis with the specified limit
// (typically 1.0). Set a limit to 0.0 to disable PID control on an axis.
void configure_pids(float roll_limit, float pitch_limit, float yaw_limit, const rx_config_gyro_fmode_t *fm);

// Helper method to configure a PID controller instance use the rx config values
void configure_pid_gains(PID* pid, const rx_config_gyro_PID_t* pid_params, int8_t gain, float max, float min);
#endif