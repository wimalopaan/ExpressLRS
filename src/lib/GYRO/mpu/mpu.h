#pragma once

class MPU_Base
{
    public:
        virtual bool initialize();
        virtual void start();
        virtual uint8_t event();
        virtual bool read(float accel_rpy[], float angle_rpy[]);
        virtual void calibrate();
        virtual void OrientationHorizontalExecute();
        virtual void OrientationVerticalExecute();
        virtual bool isRunning();
};