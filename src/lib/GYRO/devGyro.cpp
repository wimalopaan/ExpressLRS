#include "targets.h"

#if defined(HAS_GYRO)
#include "gyro.h"
#include "mpu/mpu_mpu6050.h"
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
        DBGLN("devGyro.init(): Detected MPU6050 Gyro");
    } else {
        DBGLN("devGyro.init(): MPU6050 Gyro Not Detected");
        gyro.mpuDev=nullptr;
    }
#endif
    gyro.init();
    return ret;
}

static bool gyro_detect() {
    return gyro.mpuDev != nullptr;
}

static int start()
{
    DBGLN("deGyro.start()");
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
    gyro.tick();
    return DURATION_IMMEDIATELY;
}

extern gyro_event_t gyro_event;

static int event()
{
    DBGLN("deGyro.Event()");
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