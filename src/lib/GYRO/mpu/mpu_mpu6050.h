#pragma once

#include "stdint.h"
//#include "gyro.h"
#include "mpu.h"
#include "MPU6050_6Axis_MotionApps612.h"

// See betaflight src/main/drivers/accgyro/accgyro_mpu6050.c

class MPUDev_MPU6050 : public MPU_Base
{
    using MPU_Base::MPU_Base;
    public:
        bool initialize();
        void start();
        uint8_t event();
        bool read(float accel_rpy[], float angle_rpy[]);
        void calibrate(bool save);
        void OrientationHorizontalExecute();
        void OrientationVerticalExecute();
        bool isRunning();
    private:
        MPU6050 *mpu;

        void dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
        #ifdef DEBUG_GYRO_STATS
        void print_gyro_stats();
        unsigned long last_gyro_stats_time;
        #endif

        void setupOrientation();
        uint8_t readAndGetGravity();
};
