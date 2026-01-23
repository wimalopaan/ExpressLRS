#pragma once
#include <stdint.h>

typedef enum {
    GYRO_STATUS_OFF,
    GYRO_STATUS_NOT_DETECTED,
    GYRO_STATUS_NEED_CALIBRATION,
    GYRO_STATUS_OK
} gyro_status_t;

typedef enum
{
    GYRO_EVENT_NONE,
    GYRO_EVENT_CALIBRATE,
    GYRO_EVENT_HORIZONTAL_CALIBRATE,
    GYRO_EVENT_VERTICAL_CALIBRATE,
    GYRO_EVENT_SUBTRIMS
} gyro_event_t;

#define GYRO_MAX_FMODES 6 

typedef enum
{
    GYRO_MODE_OFF = 0,
    GYRO_MODE_RATE,
    GYRO_MODE_SAFE,
    GYRO_MODE_LEVEL,
    GYRO_MODE_LAUNCH,
    GYRO_MODE_HOVER,
} gyro_mode_t;

typedef enum {
    FN_IN_NONE,
    FN_IN_ROLL,
    FN_IN_PITCH,
    FN_IN_YAW,
    FN_IN_GYRO_MODE,
    FN_IN_GYRO_GAIN
} gyro_input_channel_function_t;


#define GYRO_N_AXES 3

typedef enum {
    GYRO_AXIS_ROLL,
    GYRO_AXIS_PITCH,
    GYRO_AXIS_YAW
} gyro_axis_t;

typedef enum {
    GYRO_RATE_VARIABLE_P,
    GYRO_RATE_VARIABLE_I,
    GYRO_RATE_VARIABLE_D
} gyro_rate_variable_t;


typedef enum {
    FN_NONE,
    FN_AILERON,
    FN_ELEVATOR,
    FN_RUDDER,
    FN_ELEVON,
    FN_VTAIL
} gyro_output_channel_function_t;


typedef enum { // values are important
    STICK_PRIORITY_FULL=0, 
    STICK_PRIORITY_HALF=1, 
    STICK_PRIORITY_QUARTER=2 
} gyro_stick_priority_t;

typedef enum { 
    GYRO_LEARN_OFF, 
    GYRO_LEARN_SUBTRIMS, 
    GYRO_LEARN_LIMIT_START,
    GYRO_LEARN_LIMIT_DONE 
} gyro_learn_state_t;



typedef struct __attribute__((packed)) {
    uint8_t p;
    uint8_t i;
    uint8_t d;
    uint8_t gain;
} rx_config_gyro_PID_t;

/*
typedef struct {
    uint16_t min;
    uint16_t mid;
    uint16_t max;
} rx_config_gyro_timings_t;
*/

typedef union __attribute__((packed)) {
    struct {
        uint32_t input_mode:5,
                 output_mode:5,
                 inverted:1,     // invert gyro output
                 auto_subtrim:1, // Set subtrim at first connection
                 unused:20;
    } val;
    uint32_t raw;
} rx_config_gyro_channel_t;

typedef union __attribute__((packed)) {
    struct {
        uint64_t 
                 angleLimitEnable:1,
                 trimEnable:1,
                 gainEnable:1,

                 angleLimitPitch:8,
                 angleLimitRoll:8,

                 trimPitch:8,
                 trimRoll:8,

                 gainRoll:7,
                 gainPitch:7,
                 gainYaw:7,
                 
                 unused:1;
    } val;
    uint64_t raw;
} rx_config_gyro_fmode_t;

typedef union __attribute__((packed)) {
    struct {
        uint32_t pos1: 4,
                 pos2: 4,
                 pos3: 4,
                 pos4: 4,
                 pos5: 4,
                 unused: 12;
    } val;
    uint32_t raw;
} rx_config_gyro_mode_pos_t;

typedef struct __attribute__((packed)) {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} rx_config_gyro_calibration_t;

constexpr uint8_t GYRO_MAX_CHANNELS = 16;
