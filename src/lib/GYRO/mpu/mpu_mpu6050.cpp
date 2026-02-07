#include "targets.h"

#if defined(HAS_GYRO)
#include "mpu_mpu6050.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "logging.h"
#include "config.h"

#define I2C_MASTER_FREQ_HZ 400000
#define MPU6050_INT_DPM             (1<<MPU6050_INTERRUPT_DMP_INT_BIT)    // 0x02
#define MPU6050_INT_FIFO_OFLOW      (1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT) // 0x10


#define gscale ((250. / 32768.0) / 100) // gyro default 250 LSB per d/s

// MPU control/status vars
//static bool dmpReady = false;    // set true if DMP init was successful
static uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
//static uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
static uint16_t fifoCount;       // count of all bytes currently in FIFO
static uint8_t fifoBuffer[64];   // FIFO storage buffer

static unsigned long last_gyro_update;

// Orientation related variables
bool orientationIsWrong;    // flag to say that orientation is wrong and so avoid any process of raw data
static uint8_t mpuOrientationH=0;
static uint8_t mpuOrientationV=0;

static uint8_t orientationX;       // contain the index 0,1, 2 of aRaw[] and gRaw[] to be moved in oax and ogx
static uint8_t orientationY;       // idem for oay and ogy
static uint8_t orientationZ;       // idem for oaz and ogz
static int8_t orientationSignX;    // contains the sign (1 or -1 ) to apply to oax and ogx
static int8_t orientationSignY;    // idem for oay and ogy
static int8_t orientationSignZ;    // idem for oaz and ogz


// Generic
//const char* mpuOrientationNames[8] = {
//    "FRONT(X+)", "BACK(X-)", "LEFT(Y+)", "RIGHT(Y-)", "UP(Z+)", "DOWN(Z-)", "WRONG", "WRONG"};

// HR8EG
const char *mpuOrientationNames[8] = 
    {"QRC Dn(X+)","QRC Up(X-)","Pins Up(Y+)","Pins Dn(Y-)","Lbl Up(Z+)","Lbl Dn(Z-)","WRONG","WRONG"};


static int8_t orientationList[36][6] = {
{3,3,3,0,0,0}, {3,3,3,0,0,0}, {1,2,0,1,1,1}, {1,2,0,-1,-1,1}, {2,1,0,1,-1,1}, {2,1,0,-1,1,1},\

{3,3,3,0,0,0}, {3,3,3,0,0,0}, {1,2,0,1,-1,-1}, {1,2,0,-1,1,-1}, {2,1,0,1,1,-1}, {2,1,0,-1,-1,-1},\

{0,2,1,1,-1,1}, {0,2,1,-1,1,1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}, {2,0,1,1,1,1}, {2,0,1,-1,-1,1},\

{0,2,1,1,1,-1}, {0,2,1,-1,-1,-1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}, {2,0,1,1,-1,-1}, {2,0,1,-1,1,-1},\

{0,1,2,1,1,1}, {0,1,2,-1,-1,1}, {1,0,2,1,-1,1}, {1,0,2,-1,1,1}, {3,3,3,0,0,0}, {3,3,3,0,0,0},\

{0,1,2,1,-1,-1}, {0,1,2,-1,1,-1}, {1,0,2,1,1,-1}, {1,0,2,-1,-1,-1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}};

static uint8_t accScaleCode, gyroScaleCode;
static float   accScale1G, gyroScaleRad, gyroScaleDeg;



//static VectorInt16 aa;      // [x, y, z]            accel sensor measurements
//static VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
//static VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
static Quaternion q;        // [w, x, y, z]         quaternion container
static VectorInt16 v_gyro;
static VectorFloat gravity; // [x, y, z]            gravity vector    
static float euler[3];      // [psi, theta, phi]    Euler angle container
static float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container

#ifdef DEBUG_GYRO_STATS
/**
 * For debugging print useful gyro state
 */
void MPUDev_MPU6050::print_gyro_stats()
{
    if (millis() - last_gyro_stats_time < 500)
        return;

    // Calculate gyro update rate in HZ
    int update_rate = 1.0 /
                      ((micros() - last_gyro_update) / 1000000.0);

    char rate_str[5]; sprintf(rate_str, "%4d", update_rate);

    char pitch_str[8]; sprintf(pitch_str, "%6.2f", ypr[1] * 180 / M_PI);
    char roll_str[8]; sprintf(roll_str, "%6.2f", ypr[2] * 180 / M_PI);
    char yaw_str[8]; sprintf(yaw_str, "%6.2f", ypr[0] * 180 / M_PI);

    char gyro_x[8]; sprintf(gyro_x, "%6.2f", (double) v_gyro.x);
    char gyro_y[8]; sprintf(gyro_y, "%6.2f", (double) v_gyro.y);
    char gyro_z[8]; sprintf(gyro_z, "%6.2f", (double) v_gyro.z);

    char gravity_x[8]; sprintf(gravity_x, "%4.2f", gravity.x);
    char gravity_y[8]; sprintf(gravity_y, "%4.2f", gravity.y);
    char gravity_z[8]; sprintf(gravity_z, "%4.2f", gravity.z);

    char debug_line[128];
    sprintf(debug_line,
        "Pitch: %.2f Roll: %.2f Yaw: %.2f"
        , ypr[1], ypr[2], ypr[0]
    );
    DBGLN(debug_line);

    // Uncomment lines needed for debugging
    DBGLN(
        // "%s HZ "
        "Gain %f "
        "Pitch:%s Roll:%s Yaw:%s "
        // "e1: %f, e2: %f, e3: %f "
        "Qw: %f Qx: %f Qy: %f Qz: %f "
        // "Gyro x: %s Gyro y: %s Gyro z: %s "
        "Gravity gX: %s gY: %s gZ: %s "
        // ,rate_str
        , gyro.gain
        ,pitch_str, roll_str, yaw_str
        // ,gyro.euler[0], gyro.euler[1], gyro.euler[2]
        ,q.w, q.x, q.y, q.z
        // ,gyro_x, gyro_y, gyro_z
        ,gravity_x, gravity_y, gravity_z
        );

    last_gyro_stats_time = millis();
}
#endif

bool MPUDev_MPU6050::initialize() {
    Wire.setClock(I2C_MASTER_FREQ_HZ);
    mpu =  new MPU6050();

    if (!mpu->testConnection()) 
    {
        mpu = nullptr;
        return false;
    }

    accScaleCode = MPU6050_ACCEL_FS_2;
    accScale1G = 16384.0;

    gyroScaleCode = MPU6050_GYRO_FS_2000;  
    gyroScaleRad = 2000.0 / 32768.0 / 180 * PI;  //   multiply adc by this to get rad°/s
    gyroScaleDeg = 2000.0 / 32768.0 / 100;       //   multiply adc by this to get deg°/s

    orientationIsWrong = true;
    return true;
}

void MPUDev_MPU6050::calibrate()
{
    // Run the calibration
    mpu->CalibrateAccel(8);
    mpu->CalibrateGyro(8);

    config.SetAccelCalibration(
        mpu->getXAccelOffset(),
        mpu->getYAccelOffset(),
        mpu->getZAccelOffset()
    );
    config.SetGyroCalibration(
        mpu->getXGyroOffset(),
        mpu->getYGyroOffset(),
        mpu->getZGyroOffset()
    );

    start();
}

void MPUDev_MPU6050::start() {
    DBGLN("MPU6050(Start)");
    
    mpu->reset();
    vTaskDelay(50 * portTICK_PERIOD_MS);

    mpu->setFullScaleAccelRange(accScaleCode);
    mpu->setFullScaleGyroRange(gyroScaleCode);
    mpu->setMasterClockSpeed(MPU6050_CLOCK_DIV_400); // 400kHz
    

    /*  dmpInitialize do all this
    mpu->setRate(0);              // Max rate?
    mpu->setSleepEnabled(false);
    // INT_PIN_CFG
    mpu->setInterruptMode(0);         // INT_LEVEL_HIGH
    mpu->setInterruptDrive(0);        // INT_OPEN_DIS (push-pull)
    mpu->setInterruptLatch(0);        // LATCH_INT_DIS (50us)
    mpu->setInterruptLatchClear(0);   // INT_RD_CLEAR_DIS (read-only)
    mpu->setFSyncInterruptLevel(0);   // FSYNC_INT_LEVEL_HIGH (active-high)
    mpu->setFSyncInterruptEnabled(0); // FSYNC_INT_DIS
    mpu->setI2CBypassEnabled(1);      // I2C_BYPASS_EN
    mpu->setClockOutputEnabled(0);    // CLOCK_DIS
    mpu->setExternalFrameSync(0);
    */

    mpu->dmpInitialize();


    const rx_config_gyro_calibration_t *offsets;
    offsets = config.GetAccelCalibration();
    mpu->setXAccelOffset(offsets->x);
    mpu->setYAccelOffset(offsets->y);
    mpu->setZAccelOffset(offsets->z);

    offsets = config.GetGyroCalibration();
    mpu->setXGyroOffset(offsets->x);
    mpu->setYGyroOffset(offsets->y);
    mpu->setZGyroOffset(offsets->z);

    mpu->setDMPEnabled(true);

    setupOrientation();
}

uint8_t MPUDev_MPU6050::event() {
    return DURATION_IGNORE;
}

bool MPUDev_MPU6050::isRunning() {
    return !orientationIsWrong;
}

/**
 * This method is used instead of mpu->dmpGetYawPitchRoll() as that method has
 * issues when gravity switches at high pitch angles.
*/
void MPUDev_MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
{
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity -> y , gravity -> z);

    // NOTE: This is buggy at high pitch angles when gravity flips
    // if (gravity -> z < 0) {
    //     if(data[1] > 0) {
    //         data[1] = PI - data[1];
    //     } else {
    //         data[1] = -PI - data[1];
    //     }
    // }
}

static void applyOrientation(VectorInt16 *v)
{
    // take care of the orientation of the sensor in the model
    float t[3];
    t[0] = v->x;
    t[1] = v->y;
    t[2] = v->z;

    v->x = t[orientationX] * orientationSignX;
    v->y = t[orientationY] * orientationSignY;
    v->z = t[orientationZ] * orientationSignZ;
}

static void applyOrientation(Quaternion *q)
{
    // take care of the orientation of the sensor in the model
    float t[3];
    t[0] = q->x;
    t[1] = q->y;
    t[2] = q->z;

    q->x = t[orientationX] * orientationSignX;
    q->y = t[orientationY] * orientationSignY;
    q->z = t[orientationZ] * orientationSignZ;
}

bool MPUDev_MPU6050::read(float accel_rpy[], float angle_rpy[]) {
    mpuIntStatus = mpu->getIntStatus();
    fifoCount = mpu->getFIFOCount();
    if ((mpuIntStatus & MPU6050_INT_FIFO_OFLOW) || fifoCount == 1024) 
    {
        DBGLN("Resetting gyro FIFO buffer");
        mpu->resetFIFO();
        return false;
    }
    else if (mpuIntStatus & MPU6050_INT_DPM) // 0x02
    {
        int result = mpu->GetCurrentFIFOPacket(fifoBuffer, 28);
        if (result != 1)
            return DURATION_IMMEDIATELY;

        mpu->dmpGetGyro(&v_gyro, fifoBuffer);
        mpu->dmpGetQuaternion(&q, fifoBuffer); // [w, x, y, z] quaternion container

        if (!orientationIsWrong) 
        {
            applyOrientation(&v_gyro);
            applyOrientation(&q);
        }

        mpu->dmpGetEuler(euler, &q);
        mpu->dmpGetGravity(&gravity, &q);
        dmpGetYawPitchRoll(ypr, &q, &gravity);

        accel_rpy[0] = v_gyro.x * gscale; // Roll
        accel_rpy[1] = v_gyro.y * gscale; // Pitch
        accel_rpy[2] = v_gyro.z * gscale; // Yaw
        
        angle_rpy[0] = ypr[2];  // Roll
        angle_rpy[1] = ypr[1];  // Pitch
        angle_rpy[2] = ypr[0];  // Yaw
    }
    else
    {
        return false;
    }

    #ifdef DEBUG_GYRO_STATS
    print_gyro_stats();
    #endif

        // unsigned long time_since_update = micros() - last_gyro_update;
        last_gyro_update = micros();
    return true;
}


void MPUDev_MPU6050::setupOrientation()
{
    uint8_t idx;
    orientationIsWrong = false;
    mpuOrientationH = config.GetGyroOrientationH();
    mpuOrientationV = config.GetGyroOrientationV();

    if (mpuOrientationH>5 || mpuOrientationV>5 ) 
    {
        orientationIsWrong = true;
        DBGLN("Orientation is WRONG");
        return;
    }
    idx = mpuOrientationH *6 + mpuOrientationV;  // into a number in range 0/35
    if (orientationList[idx][3] == 0){   // check that combination H and V is valid
        orientationIsWrong = true;
        return;
    }
    orientationX = orientationList[idx][0]; // orientation list contains e.g. 3,2,1,-1,1,1 (first are the index to map, last 3 the sign)
    orientationY = orientationList[idx][1];
    orientationZ = orientationList[idx][2];
    orientationSignX = orientationList[idx][3];
    orientationSignY = orientationList[idx][4];
    orientationSignZ = orientationList[idx][5];

    DBGLN("OrientationH: %s", mpuOrientationNames[mpuOrientationH]);
    DBGLN("OrientationV: %s", mpuOrientationNames[mpuOrientationV]);
}

static void findGravity(int32_t ax, int32_t ay, int32_t az, uint8_t &idx ){
    // find the index and sign of gravity 
    //      idx:  0=X, 1=Y, 2=Z; 
    //      sign: 1=gravity is the opposite (normally Z axis is up and give 1) 
    if ((float) ax >  (accScale1G*0.7)) { idx = 0 ;
    } else if ((float) ax < -(accScale1G*0.7)) { idx = 1 ;
    } else if ((float) ay >  (accScale1G*0.7)) { idx = 2 ;
    } else if ((float) ay < -(accScale1G*0.7)) { idx = 3 ;
    } else if ((float) az >  (accScale1G*0.7)) { idx = 4 ;
    } else if ((float) az < -(accScale1G*0.7)) { idx = 5 ;
    } else { idx= 6; };
    
    DBGLN("findGravty(): ax=%d  ay=%d  az=%d  yawIdx=%d  scale=%f", ax , ay ,  az, idx, accScale1G);
}    

uint8_t MPUDev_MPU6050::readAndGetGravity(){ // return index of orientation; return 6 in case of error
    int16_t ax,ay,az ;
    int16_t gx,gy,gz ;
    uint8_t idx = 6; 
    mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    findGravity( ax , ay , az , idx);
	return idx;
}

void MPUDev_MPU6050::OrientationHorizontalExecute()  // 
{
    orientationIsWrong = true;
    mpuOrientationH = 6;
    mpuOrientationV = 6;
    
    DBGLN("Horizontal Detection...");
    uint8_t idx = readAndGetGravity(); // // read the Acc and detect which face is on the upper side 
    if (idx > 5){
         DBGLN("Error during horizontal orientation: direction of gravity has not been found");
         return;
    }
    DBGLN("Upper face of MPU (when model is horizontal) is %s", mpuOrientationNames[idx]);
    mpuOrientationH =  idx ; // save the orientationH      

    
}

void MPUDev_MPU6050::OrientationVerticalExecute() {
    mpuOrientationV = 6;
    DBGLN("Vertical Detection...");
    uint8_t idx = readAndGetGravity(); // // read the Acc and detect which face is on the upper side 
    if (idx > 5){
        DBGLN("Error during vertical orientation: direction of gravity has not been found");
    }
    DBGLN("Upper face (with nose up) is %s",mpuOrientationNames[idx]);
    mpuOrientationV =  idx ; // save the orientationV

    config.SetGyroOrientation(mpuOrientationH,mpuOrientationV);      
    setupOrientation();  // refresh the parameters linked to the orientation  
    start();
}

#endif
