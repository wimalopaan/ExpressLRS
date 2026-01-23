#pragma once

#include "stdint.h"
#include "gyro.h"

// See betaflight src/main/drivers/accgyro/accgyro_mpu6050.c

//class Gyro;

class MPUDevice
{
    public:
        virtual bool initialize();
        virtual void start();
        virtual uint8_t event();
        virtual bool read();
        virtual void calibrate();
        virtual void OrientationHorizontalExecute();
        virtual void OrientationVerticalExecute();
        virtual bool isRunning();
};

class MPUDev_MPU6050 : public MPUDevice
{
    using MPUDevice::MPUDevice;
    public:
        bool initialize();
        void start();
        uint8_t event();
        bool read();
        void calibrate();
        void OrientationHorizontalExecute();
        void OrientationVerticalExecute();
        bool isRunning();
    private:
        void dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
        #ifdef DEBUG_GYRO_STATS
        void print_gyro_stats();
        unsigned long last_gyro_stats_time;
        #endif

        void setupOrientation();
        uint8_t readAndGetGravity();
};
