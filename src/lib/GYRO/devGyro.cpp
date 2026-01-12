#include "targets.h"

#if defined(HAS_GYRO)
#include "gyro.h"
#include "mpu_mpu6050.h"
#include "mixer.h"
#include "logging.h"
#include "elrs_eeprom.h" // only needed to satisfy PIO
#include "config.h"
#include "MPU6050.h"

extern boolean i2c_enabled;

Gyro gyro = Gyro();

static bool initialize()
{
    gyro.initialized=false;
    if (!i2c_enabled)
    {
        return false;
    }
#ifdef GYRO_DEVICE_MPU6050
    gyro.mpuDev = new MPUDev_MPU6050();
    bool ret = gyro.mpuDev->initialize();
    if (ret) {
        DBGLN("Detected MPU6050 Gyro");
    } else {
        DBGLN("MPU6050 Gyro Not Detected");
        gyro.mpuDev=nullptr;
    }
#endif
    return ret;
}

static bool gyro_detect() {
    return gyro.mpuDev != nullptr;
}

static int start()
{
    if (!mixer_initialize() || !gyro_detect()) {
        DBGLN("Gyro initialization failed");
        return DURATION_NEVER;
    }
    gyro.mpuDev->start();
    gyro.initialized = gyro.mpuDev->isRunning();
    return DURATION_IMMEDIATELY; // Call timeout() immediately;
}

static int timeout()
{
    if (gyro.mpuDev->read()) {
        gyro.last_update = micros();
        gyro.send_telemetry();
    }
    gyro.tick();
    return DURATION_IMMEDIATELY;
}

extern bool auto_subtrim_complete;
extern uint8_t subtrim_init;
extern gyro_event_t gyro_event;

static int event()
{
    switch (gyro_event)
    {
    /*    
    case GYRO_EVENT_CALIBRATE:
        gyro.dev->calibrate();
        config.SetCalibrateGyro(false);
        gyro_event = GYRO_EVENT_NONE;
        break;
    */

    case GYRO_EVENT_HORIZONTAL_CALIBRATE:
        gyro.mpuDev->calibrate();
        gyro.mpuDev->OrientationHorizontalExecute();
        gyro_event = GYRO_EVENT_NONE;
        break;
    case GYRO_EVENT_VERTICAL_CALIBRATE:
        gyro.mpuDev->OrientationVerticalExecute();
        gyro.reload();
        gyro_event = GYRO_EVENT_NONE;
        break;

    case GYRO_EVENT_SUBTRIMS:
        auto_subtrim_complete = false;
        subtrim_init = 0;
        gyro_event = GYRO_EVENT_NONE;
        break;

    default: ;
    }

    return DURATION_IGNORE;
}

device_t Gyro_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONFIG_GYRO_CHANGED,
};

#endif