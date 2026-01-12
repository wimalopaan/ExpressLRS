#include "targets.h"
#if !defined(UNIT_TEST)
#include "RXEndpoint.h"
#include "POWERMGNT.h"
#include "config.h"
#include "deferred.h"
#include "devServoOutput.h"
#include "helpers.h"
#include "logging.h"

#if defined(HAS_GYRO)
#include "gyro.h"
extern gyro_event_t gyro_event;
#endif

#define RX_HAS_SERIAL1 (GPIO_PIN_SERIAL1_TX != UNDEF_PIN || OPT_HAS_SERVO_OUTPUT)

extern void reconfigureSerial();
#if defined(PLATFORM_ESP32)
extern void reconfigureSerial1();
#endif
extern bool BindingModeRequest;

extern RXEndpoint crsfReceiver;

#if defined(Regulatory_Domain_EU_CE_2400)
#if defined(RADIO_LR1121)
char strPowerLevels[] = "10/10;25/25;25/50;25/100;25/250;25/500;25/1000;25/2000;MatchTX ";
#else
char strPowerLevels[] = "10;25;50;100;250;500;1000;2000;MatchTX ";
#endif
#else
char strPowerLevels[] = "10;25;50;100;250;500;1000;2000;MatchTX ";
#endif
static char modelString[] = "000";
static char pwmModes[] = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off;DShot;DShot 3D;Serial RX;Serial TX;I2C SCL;I2C SDA;Serial2 RX;Serial2 TX";

#if defined(HAS_GYRO)
static char gyroEnabled[] = "Off;On";
//  needs to match gyro_status_t
static char *gyroStatus[] = {"Off","Not Detected","Need Cali","Running"};

//Generic BOARD
//static char gyroOrientation[] = "FRONT(X+);BACK(X-);LEFT(Y+);RIGHT(Y-);UP(Z+);DOWN(Z-);WRONG;WRONG";

// HR8EG
static char gyroOrientation[] = "QRC UP(X+);QRC Down(X-);Pins UP(Y+);Pins Down(Y-);Lbl Up(Z+);Lbl Down(Z-);WRONG;WRONG";


// Must match mixer.h: gyro_input_channel_function_t
static const char gyroInputChannelModes[] = "None;Roll;Pitch;Yaw;Mode;Gain";
// Must match mixer.h: gyro_output_channel_function_t
static const char gyroOutputChannelModes[] = "None;Aileron;Elevator;Rudder;Elevon;V Tail";
// Must match gyro.h gyro_mode_t
static const char gyroModes[] = "Off;Rate;SAFE;Level;Launch;Hover";
// Must match gyro_axis_t
static const char gyroAxis[] = "Roll;Pitch;Yaw";
#endif

static selectionParameter luaSerialProtocol = {
    {"Protocol", CRSF_TEXT_SELECTION},
    0, // value
    "CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro;HoTT Telemetry;MAVLink;DisplayPort;GPS",
    STR_EMPTYSPACE
};

#if defined(PLATFORM_ESP32)
static selectionParameter luaSerial1Protocol = {
    {"Protocol2", CRSF_TEXT_SELECTION},
    0, // value
    "Off;CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro;HoTT Telemetry;Tramp;SmartAudio;DisplayPort;GPS",
    STR_EMPTYSPACE
};
#endif

static selectionParameter luaSBUSFailsafeMode = {
    {"SBUS failsafe", CRSF_TEXT_SELECTION},
    0, // value
    "No Pulses;Last Pos",
    STR_EMPTYSPACE
};

static int8Parameter luaTargetSysId = {
  {"Target SysID", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value - default to 1
      (uint8_t)1,       // min
      (uint8_t)255,     // max
    }
  },
  STR_EMPTYSPACE
};
static int8Parameter luaSourceSysId = {
  {"Source SysID", CRSF_UINT8},
  {
    {
      (uint8_t)255,       // value - default to 255
      (uint8_t)1,         // min
      (uint8_t)255,       // max
    }
  },
  STR_EMPTYSPACE
};

static selectionParameter luaTlmPower = {
    {"Tlm Power", CRSF_TEXT_SELECTION},
    0, // value
    strPowerLevels,
    "mW"
};

static selectionParameter luaAntennaMode = {
    {"Ant. Mode", CRSF_TEXT_SELECTION},
    0, // value
    "Antenna A;Antenna B;Diversity",
    STR_EMPTYSPACE
};

static folderParameter luaTeamraceFolder = {
    {"Team Race", CRSF_FOLDER},
};

static selectionParameter luaTeamraceChannel = {
    {"Channel", CRSF_TEXT_SELECTION},
    0, // value
    "AUX2;AUX3;AUX4;AUX5;AUX6;AUX7;AUX8;AUX9;AUX10;AUX11;AUX12",
    STR_EMPTYSPACE
};

static selectionParameter luaTeamracePosition = {
    {"Position", CRSF_TEXT_SELECTION},
    0, // value
    "Disabled;1/Low;2;3;Mid;4;5;6/High",
    STR_EMPTYSPACE
};

//----------------------------Info-----------------------------------

static stringParameter luaModelNumber = {
    {"Model Id", CRSF_INFO},
    modelString
};

static stringParameter luaELRSversion = {
    {version, CRSF_INFO},
    commit
};

//----------------------------Info-----------------------------------

//---------------------------- WiFi -----------------------------

// --------------------------- Gyro Setup ---------------------------------

#if defined(HAS_GYRO)

static selectionParameter luaGyroEnabled = {
    {"Enabled", CRSF_TEXT_SELECTION},
    0, // value
    gyroEnabled,
    STR_EMPTYSPACE
};

static stringParameter luaGyroStatus = {
    {"Status", CRSF_INFO},
    "" // value
};

static int8Parameter luaGyroLaunchAngle = {
  {"Launch Angle", CRSF_UINT8},
  {
    {
      (uint8_t)10, // value, not zero-based
      0,           // min
      60,          // max
    }
  },
  "deg"
};

static int8Parameter luaGyroSAFEPitch = {
  {"SAFE Pitch", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static int8Parameter luaGyroSAFERoll = {
  {"SAFE Roll", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static int8Parameter luaGyroLevelPitch = {
  {"Level Pitch", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static int8Parameter luaGyroLevelRoll = {
  {"Level Roll", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static int8Parameter luaGyroHoverStrength = {
  //------------ Max length on RM Pocket
  {"Hover Auth", CRSF_UINT8},
  {
    {
      (uint8_t)8,  // value, not zero-based
      0,           // min
      15,          // max
    }
  },
  STR_EMPTYSPACE
};


void RXEndpoint::luaparamGyroCalibrate(propertiesCommon *item, uint8_t arg)
{
  static uint8_t calStep = 0; //Global

  commandStep_e newStep;
  const char *msg;
 
  DBGLN("Calibration Workflow BEGIN: command=[%d],calStep=[%d]",arg,calStep);
  if (arg == lcsClick)
  {
    // Step 1: Horizontal
    calStep = 0;
    DBGLN("Calibrating Gyro: Gyro Ready=%s",gyro.initialized?"True":"False");
    newStep = lcsAskConfirm;
    msg = "Plane Horizontal?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    if (calStep == 0) {
      msg = "Horizontal Cal";
      calStep++;
      sendCommandResponse((commandParameter *)item, newStep, msg);
      gyro_event = GYRO_EVENT_HORIZONTAL_CALIBRATE;
      devicesTriggerEvent(EVENT_CONFIG_GYRO_CHANGED);
    } else
    if (calStep == 1) {
      msg = "Vertical Cal";
      calStep++;
      sendCommandResponse((commandParameter *)item, newStep, msg);
      gyro_event = GYRO_EVENT_VERTICAL_CALIBRATE;
      devicesTriggerEvent(EVENT_CONFIG_GYRO_CHANGED);
    } else
    if (calStep == 2) {
      // Calibration Done
      newStep = lcsIdle;
      msg = STR_EMPTYSPACE;
      sendCommandResponse((commandParameter *)item, newStep, msg);
    }
    DBGLN("Calibrating Workflow RETURN: newStep=[%d],msg=[%s], calStep=[%d]",newStep,msg,calStep);
    return;
  }
  else if (arg == lcsQuery)
  {
    if (gyro_event!=GYRO_EVENT_NONE) { // Not completed yet??
      // Still calibrating
      newStep = lcsExecuting;
      msg = "Calibrating ";
    } else
    if (calStep==1) {
      newStep = lcsAskConfirm;
      msg = "Plane Nose Up?";
    } else
    if (calStep==2) {
      newStep = lcsAskConfirm;
      msg = "Calibration Done";
      // Trigger reload of values
      devicesTriggerEvent(EVENT_CONFIG_GYRO_CHANGED);
      updateParameters();
    } else {
      msg = STR_EMPTYSPACE;
      newStep = lcsIdle;
    }
  }
  else // idle
  {
      newStep = lcsIdle;
      msg = STR_EMPTYSPACE;
  }

  DBGLN("Calibrating Workflow RETURN: newStep=[%d],msg=[%s], calStep=[%d]",newStep,msg,calStep);
  sendCommandResponse((commandParameter *)item, newStep, msg);
}

void RXEndpoint::luaparamGyroSubtrims(propertiesCommon *item, uint8_t arg)
{
  commandStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Set subtrims?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Setting subtrims";
    sendCommandResponse((commandParameter *)item, newStep, msg);
    gyro_event = GYRO_EVENT_SUBTRIMS;
    devicesTriggerEvent(EVENT_CONFIG_GYRO_CHANGED);
    return;
  }
  else if (arg == lcsQuery)
  {
    if (gyro_event!=GYRO_EVENT_NONE) {
      msg = "Setting subtrims";
      newStep = lcsExecuting;
    } else {
      newStep = lcsIdle;
      msg = STR_EMPTYSPACE;
    }
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendCommandResponse((commandParameter *)item, newStep, msg);
}

static struct commandParameter luaGyroAutoOrientation = {
    {"Auto Orientation                                                                                                                                                                                                ", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

static struct commandParameter luaGyroSubtrims = {
    {"Set Subtrims", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

static folderParameter luaGyroMainFolder = {
    {"Gyro", CRSF_FOLDER},
};

static folderParameter luaGyroModelFolder = {
    {"Model Setup", CRSF_FOLDER},
};

static folderParameter luaGyroModesFolder = {
    {"F-Mode Switch", CRSF_FOLDER},
};

static folderParameter luaGyroGainFolder = {
    {"Gains", CRSF_FOLDER},
};

static folderParameter luaGyroInputFolder = {
    {"Inputs", CRSF_FOLDER},
};

static folderParameter luaGyroOutputFolder = {
    {"Outputs", CRSF_FOLDER},
};

static folderParameter luaGyroSettingsFolder = {
    {"F-Mode Settings", CRSF_FOLDER},
};

static folderParameter luaGyroCalibrationFolder = {
    {"Calibration", CRSF_FOLDER},
};

static folderParameter luaGyroRxOrientationFolder = {
    {"RX Orientation", CRSF_FOLDER},
};

static folderParameter luaGyroStickCalibrationFolder = {
    {"Stick Calibration", CRSF_FOLDER},
};

static folderParameter luaGyroSettingsSafeFolder = {
    {"Safe", CRSF_FOLDER},
};

static folderParameter luaGyroSettingsLevelFolder = {
    {"Level", CRSF_FOLDER},
};

static folderParameter luaGyroSettingsLaunchFolder = {
    {"Launch", CRSF_FOLDER},
};

static folderParameter luaGyroSettingsHoverFolder = {
    {"Hover", CRSF_FOLDER},
};


static int8Parameter luaGyroInputChannel = {
  {"Input Ch", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static selectionParameter luaGyroInputMode = {
    {"Function", CRSF_TEXT_SELECTION},
    0, // value
    gyroInputChannelModes,
    STR_EMPTYSPACE
};

static selectionParameter luaGyroModePos1 = {
    {"Position 1", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static selectionParameter luaGyroModePos2 = {
    {"Position 2", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static selectionParameter luaGyroModePos3 = {
    {"Position 3", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static selectionParameter luaGyroModePos4 = {
    {"Position 4", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static selectionParameter luaGyroModePos5 = {
    {"Position 5", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};




static int8Parameter luaGyroOutputChannel = {
  {"Output Ch", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static selectionParameter luaGyroOutputMode = {
    {"Function", CRSF_TEXT_SELECTION},
    0, // value
    gyroOutputChannelModes,
    STR_EMPTYSPACE
};

static selectionParameter luaGyroOutputInverted = {
    {"Invert", CRSF_TEXT_SELECTION},
    0, // value
    "Off;On",
    STR_EMPTYSPACE
};

void RXEndpoint::luaparamGyroInputChannel(propertiesCommon *item, uint8_t arg)
{
  setUint8Value(&luaGyroInputChannel, arg);
  // Trigger reload of values for the selected channel
  devicesTriggerEvent(EVENT_CONFIG_GYRO_CHANGED);
}

static void luaparamGyroInputMode(propertiesCommon *item, uint8_t arg)
{
    const uint8_t ch = luaGyroInputChannel.properties.u.value - 1;
    rx_config_gyro_channel_t newCh;
    newCh.raw = config.GetGyroChannel(ch)->raw;
    newCh.val.input_mode = arg;
    config.SetGyroChannelRaw(ch, newCh.raw);
    gyro.reload();
}

void RXEndpoint::luaparamGyroOutputChannel(propertiesCommon *item, uint8_t arg)
{
  setUint8Value(&luaGyroOutputChannel, arg);
  // Trigger reload of values for the selected channel
  devicesTriggerEvent(EVENT_CONFIG_GYRO_CHANGED);
}

void RXEndpoint::luaparamGyroOutputMode(propertiesCommon *item, uint8_t arg)
{
    const uint8_t ch = luaGyroOutputChannel.properties.u.value - 1;
    rx_config_gyro_channel_t newCh;
    newCh.raw = config.GetGyroChannel(ch)->raw;
    newCh.val.output_mode = arg;
    config.SetGyroChannelRaw(ch, newCh.raw);
    gyro.reload();
}

void RXEndpoint::luaparamGyroOutputInverted(propertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaGyroOutputChannel.properties.u.value - 1;
  rx_config_gyro_channel_t newCh;
  newCh.raw = config.GetGyroChannel(ch)->raw;
  newCh.val.inverted = arg;

  config.SetGyroChannelRaw(ch, newCh.raw);
  gyro.reload();
}

static selectionParameter luaGyroOrientationH = {
    {"Face Hor", CRSF_TEXT_SELECTION},
    6, // value
    gyroOrientation,
    STR_EMPTYSPACE
};

static selectionParameter luaGyroOrientationV = {
    {"Face Vert", CRSF_TEXT_SELECTION},
    6, // value
    gyroOrientation,
    STR_EMPTYSPACE
};

// contents of "Gyro Gains" folder, per axis subfolders
static selectionParameter luaGyroGainAxis = {
    {"Gyro Axis", CRSF_TEXT_SELECTION},
    0, // value
    gyroAxis,
    STR_EMPTYSPACE
};

static int8Parameter luaGyroPIDRateP = {
  {"P Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};

static int8Parameter luaGyroPIDRateI = {
  {"I Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};

static int8Parameter luaGyroPIDRateD = {
  {"D Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};

static int8Parameter luaGyroPIDGain = {
  {"Axis Gain", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      255            // max
    }
  },
  STR_EMPTYSPACE
};

static void luaparamGyroPIDRateP(propertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_P, arg);
  gyro.reload();
}

static void luaparamGyroPIDRateI(propertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_I, arg);
  gyro.reload();
}

static void luaparamGyroPIDRateD(propertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_D, arg);
  gyro.reload();
}

static void luaparamGyroPIDGain(propertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDGain(axis, arg);
  gyro.reload();
}

#endif // USE_GYRO


//---------------------------- WiFi -----------------------------

//---------------------------- Output Mapping -----------------------------

static folderParameter luaMappingFolder = {
    {"Output Mapping", CRSF_FOLDER},
};

static int8Parameter luaMappingChannelOut = {
  {"Output Ch", CRSF_UINT8},
  {
    {
      (uint8_t)5,       // value - start on AUX1, value is 1-16, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static int8Parameter luaMappingChannelIn = {
  {"Input Ch", CRSF_UINT8},
  {
    {
      0,                 // value
      1,                 // min
      CRSF_NUM_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static selectionParameter luaMappingOutputMode = {
    {"Output Mode", CRSF_TEXT_SELECTION},
    0, // value
    pwmModes,
    STR_EMPTYSPACE
};

static selectionParameter luaMappingInverted = {
    {"Invert", CRSF_TEXT_SELECTION},
    0, // value
    "Off;On",
    STR_EMPTYSPACE
};

static commandParameter luaSetFailsafe = {
    {"Set Failsafe Pos", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

#if defined(HAS_GYRO)
const char STR_US[] = " us";
static int16Parameter luaMappingChannelLimitMin = {
  {"Limit Min us", CRSF_UINT16},
  {
    {
      0U,  // value
      0U,  // min
      65535U, // max
    }
  },
  STR_US
};

static int16Parameter luaMappingChannelLimitMax = {
  {"Limit Max us", CRSF_INT16},
  {
    {
      2135, // value
      1501, // min
      2135, // max
    }
  },
  STR_US
};
#endif


//---------------------------- Output Mapping -----------------------------

static selectionParameter luaBindStorage = {
    {"Bind Storage", CRSF_TEXT_SELECTION},
    0, // value
    "Persistent;Volatile;Returnable;Administered",
    STR_EMPTYSPACE
};

static commandParameter luaBindMode = {
    {STR_EMPTYSPACE, CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

void RXEndpoint::luaparamMappingChannelOut(propertiesCommon *item, uint8_t arg)
{
    bool sclAssigned = false;
    bool sdaAssigned = false;
#if defined(PLATFORM_ESP32)
    bool serial1rxAssigned = false;
    bool serial1txAssigned = false;
#endif

    const char *no1Option    = ";";
    const char *no2Options   = ";;";
    const char *serial_RX    = ";Serial RX";
    const char *serial_TX    = ";Serial TX";
    const char *i2c_SCL      = ";I2C SCL;";
    const char *i2c_SDA      = ";;I2C SDA";
    const char *i2c_BOTH     = ";I2C SCL;I2C SDA";
#if defined(PLATFORM_ESP32)
    const char *serial1_RX   = ";Serial2 RX;";
    const char *serial1_TX   = ";;Serial2 TX";
    const char *serial1_BOTH = ";Serial2 RX;Serial2 TX";
    const char *dshot        = ";DShot;DShot 3D";
#endif

    const char *pModeString;


    // find out if use once only modes have already been assigned
    for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
    {
      if (ch == (arg -1))
        continue;

      eServoOutputMode mode = (eServoOutputMode)config.GetPwmChannel(ch)->val.mode;

      if (mode == somSCL)
        sclAssigned = true;

      if (mode == somSDA)
        sdaAssigned = true;

#if defined(PLATFORM_ESP32)
      if (mode == somSerial1RX)
        serial1rxAssigned = true;

      if (mode == somSerial1TX)
        serial1txAssigned = true;
#endif
    }

    setUint8Value(&luaMappingChannelOut, arg);

    // When the selected output channel changes, update the available PWM modes for that pin
    // Truncate the select options before the ; following On/Off
    pwmModes[50] = '\0';

#if defined(PLATFORM_ESP32)
    // DShot output (2 options)
    // ;DShot;DShot3D
    if (GPIO_PIN_PWM_OUTPUTS[arg-1] != 0)   // DShot doesn't work with GPIO0, exclude it
    {
        pModeString = dshot;
    }
    else
#endif
    {
        pModeString = no2Options;
    }
    strcat(pwmModes, pModeString);

    // SerialIO outputs (1 option)
    // ;[Serial RX] | [Serial TX]
    if (GPIO_PIN_PWM_OUTPUTS[arg-1] == U0RXD_GPIO_NUM)
    {
        pModeString = serial_RX;
    }
    else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == U0TXD_GPIO_NUM)
    {
        pModeString = serial_TX;
    }
    else
    {
        pModeString = no1Option;
    }
    strcat(pwmModes, pModeString);

    // I2C pins (2 options)
    // ;[I2C SCL] ;[I2C SDA]
    if (GPIO_PIN_SCL != UNDEF_PIN || GPIO_PIN_SDA != UNDEF_PIN)
    {
        // If the target defines SCL/SDA then those pins MUST be used
        if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SCL)
        {
            pModeString = i2c_SCL;
        }
        else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SDA)
        {
            pModeString = i2c_SDA;
        }
        else
        {
            pModeString = no2Options;
        }
    }
    else
    {
        // otherwise allow any pin to be either SCL or SDA but only once
        if (sclAssigned && !sdaAssigned)
        {
            pModeString = i2c_SDA;
        }
        else if (sdaAssigned && !sclAssigned)
        {
            pModeString = i2c_SCL;
        }
        else if (!sclAssigned && !sdaAssigned)
        {
            pModeString = i2c_BOTH;
        }
        else
        {
            pModeString = no2Options;
        }
    }
    strcat(pwmModes, pModeString);

    // nothing to do for unsupported somPwm mode
    strcat(pwmModes, no1Option);

#if defined(PLATFORM_ESP32)
    // secondary Serial pins (2 options)
    // ;[SERIAL2 RX] ;[SERIAL2_TX]
    if (GPIO_PIN_SERIAL1_RX != UNDEF_PIN || GPIO_PIN_SERIAL1_TX != UNDEF_PIN)
    {
        // If the target defines Serial2 RX/TX then those pins MUST be used
        if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SERIAL1_RX)
        {
            pModeString = serial1_RX;
        }
        else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SERIAL1_TX)
        {
            pModeString = serial1_TX;
        }
        else
        {
            pModeString = no2Options;
        }
    }
    else
    {   // otherwise allow any pin to be either RX or TX but only once
        if (serial1txAssigned && !serial1rxAssigned)
        {
            pModeString = serial1_RX;
        }
        else if (serial1rxAssigned && !serial1txAssigned)
        {
            pModeString = serial1_TX;
        }

        else if (!serial1rxAssigned && !serial1txAssigned)
        {
            pModeString = serial1_BOTH;
        }
        else
        {
            pModeString = no2Options;
        }
    }
    strcat(pwmModes, pModeString);
#endif

    // trim off trailing semicolons (assumes pwmModes has at least 1 non-semicolon)
    for (auto lastPos = strlen(pwmModes)-1; pwmModes[lastPos] == ';'; lastPos--)
    {
        pwmModes[lastPos] = '\0';
    }

    // update the related fields to represent the selected channel
    const rx_config_pwm_t *pwmCh = config.GetPwmChannel(luaMappingChannelOut.properties.u.value - 1);
    setUint8Value(&luaMappingChannelIn, pwmCh->val.inputChannel + 1);
    setTextSelectionValue(&luaMappingOutputMode, pwmCh->val.mode);
    setTextSelectionValue(&luaMappingInverted, pwmCh->val.inverted);
}

static void luaparamMappingChannelIn(propertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  newPwmCh.val.inputChannel = arg - 1; // convert 1-16 -> 0-15

  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void configureSerialPin(uint8_t sibling, uint8_t oldMode, uint8_t newMode)
{
  for (int ch=0 ; ch<GPIO_PIN_PWM_OUTPUTS_COUNT ; ch++)
  {
    if (GPIO_PIN_PWM_OUTPUTS[ch] == sibling)
    {
      // Retain as much of the sibling's current config as possible
      rx_config_pwm_t siblingPinConfig;
      siblingPinConfig.raw = config.GetPwmChannel(ch)->raw;

      // If the new mode is serial, the sibling is also forced to serial
      if (newMode == somSerial)
      {
        siblingPinConfig.val.mode = somSerial;
      }
      // If the new mode is not serial, and the sibling is serial, set the sibling to PWM (50Hz)
      else if (siblingPinConfig.val.mode == somSerial)
      {
        siblingPinConfig.val.mode = som50Hz;
      }

      config.SetPwmChannelRaw(ch, siblingPinConfig.raw);
      break;
    }
  }

  if (oldMode != newMode)
  {
    deferExecutionMillis(100, [](){
      reconfigureSerial();
    });
  }
}

static void luaparamMappingOutputMode(propertiesCommon *item, uint8_t arg)
{
  UNUSED(item);
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  uint8_t oldMode = newPwmCh.val.mode;
  newPwmCh.val.mode = arg;

  // Check if pin == 1/3 and do other pin adjustment accordingly
  if (GPIO_PIN_PWM_OUTPUTS[ch] == 1)
  {
    configureSerialPin(3, oldMode, newPwmCh.val.mode);
  }
  else if (GPIO_PIN_PWM_OUTPUTS[ch] == 3)
  {
    configureSerialPin(1, oldMode, newPwmCh.val.mode);
  }
  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void luaparamMappingInverted(propertiesCommon *item, uint8_t arg)
{
  UNUSED(item);
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  newPwmCh.val.inverted = arg;

  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

void RXEndpoint::luaparamSetFailsafe(propertiesCommon *item, uint8_t arg)
{
  commandStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Set failsafe to curr?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Setting failsafe";
    servoCurrentToFailsafeConfig();
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendCommandResponse((commandParameter *)item, newStep, msg);
}

#if defined(HAS_GYRO)
static void luaparamMappingChannelLimitMin(propertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_limits_t limits;
  limits.raw = config.GetPwmChannelLimits(ch)->raw;
  // limits.val.min = arg;
  char dbgline[128] = "";
  uint16_t num = luaMappingChannelLimitMin.properties.u.value;
  sprintf(dbgline, "ul: %ul ud: %ud", num, num);
  DBGLN(dbgline);
  DBGLN("*** lua min value: %d", luaMappingChannelLimitMin.properties.u.value);
  DBGLN("*** lua min value: %d", luaMappingChannelLimitMin.properties.s.value);
  limits.val.min = (uint16_t) luaMappingChannelLimitMin.properties.u.value;
  config.SetPwmChannelLimitsRaw(ch, limits.raw);
}

static void luaparamMappingChannelLimitMax(propertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_limits_t limits;
  limits.raw = config.GetPwmChannelLimits(ch)->raw;
  // limits.val.max = arg;
  limits.val.max = luaMappingChannelLimitMax.properties.u.value;
  config.SetPwmChannelLimitsRaw(ch, limits.raw);
}
#endif


static void luaparamSetPower(propertiesCommon* item, uint8_t arg)
{
  UNUSED(item);
  uint8_t newPower = arg + POWERMGNT::getMinPower();
  if (newPower > POWERMGNT::getMaxPower())
  {
    newPower = PWR_MATCH_TX;
  }

  config.SetPower(newPower);
  // POWERMGNT::setPower() will be called in updatePower() in the main loop
}

void RXEndpoint::registerParameters()
{
  registerParameter(&luaSerialProtocol, [](propertiesCommon* item, uint8_t arg){
    config.SetSerialProtocol((eSerialProtocol)arg);
    if (config.IsModified()) {
      deferExecutionMillis(100, [](){
        reconfigureSerial();
      });
    }
  });

#if defined(PLATFORM_ESP32)
  if (RX_HAS_SERIAL1)
  {
    registerParameter(&luaSerial1Protocol, [](propertiesCommon* item, uint8_t arg){
      config.SetSerial1Protocol((eSerial1Protocol)arg);
      if (config.IsModified()) {
        deferExecutionMillis(100, [](){
          reconfigureSerial1();
        });
      }
    });
  }
#endif

  registerParameter(&luaSBUSFailsafeMode, [](propertiesCommon* item, uint8_t arg){
    config.SetFailsafeMode((eFailsafeMode)arg);
  });

  registerParameter(&luaTargetSysId, [](propertiesCommon* item, uint8_t arg){
    config.SetTargetSysId((uint8_t)arg);
  });
  registerParameter(&luaSourceSysId, [](propertiesCommon* item, uint8_t arg){
    config.SetSourceSysId((uint8_t)arg);
  });

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    registerParameter(&luaAntennaMode, [](propertiesCommon* item, uint8_t arg){
      config.SetAntennaMode(arg);
    });
  }

  if (POWERMGNT::getMinPower() != POWERMGNT::getMaxPower())
  {
    filterOptions(&luaTlmPower, POWERMGNT::getMinPower(), POWERMGNT::getMaxPower(), strPowerLevels);
    strcat(strPowerLevels, ";MatchTX ");
    registerParameter(&luaTlmPower, &luaparamSetPower);
  }

  // Teamrace
  registerParameter(&luaTeamraceFolder);
  registerParameter(&luaTeamraceChannel, [](propertiesCommon* item, uint8_t arg) {
    config.SetTeamraceChannel(arg + AUX2);
  }, luaTeamraceFolder.common.id);
  registerParameter(&luaTeamracePosition, [](propertiesCommon* item, uint8_t arg) {
    config.SetTeamracePosition(arg);
  }, luaTeamraceFolder.common.id);

  if (OPT_HAS_SERVO_OUTPUT)
  {
    luaparamMappingChannelOut(&luaMappingOutputMode.common, luaMappingChannelOut.properties.u.value);
    registerParameter(&luaMappingFolder);
    registerParameter(&luaMappingChannelOut, [&](propertiesCommon* item, uint8_t arg) {
        luaparamMappingChannelOut(item, arg);
    }, luaMappingFolder.common.id);
    registerParameter(&luaMappingChannelIn, &luaparamMappingChannelIn, luaMappingFolder.common.id);
    registerParameter(&luaMappingOutputMode, &luaparamMappingOutputMode, luaMappingFolder.common.id);
    registerParameter(&luaMappingInverted, &luaparamMappingInverted, luaMappingFolder.common.id);
    registerParameter(&luaSetFailsafe, [&](propertiesCommon* item, uint8_t arg) {
        luaparamSetFailsafe(item, arg);
    });

#if defined(HAS_GYRO)
    // -- Servo Output Limits
    registerParameter(&luaMappingChannelLimitMin, &luaparamMappingChannelLimitMin, luaMappingFolder.common.id);
    registerParameter(&luaMappingChannelLimitMax, &luaparamMappingChannelLimitMax, luaMappingFolder.common.id);


    registerParameter(&luaGyroMainFolder);
      // ----- Gyro Main
      registerParameter(&luaGyroEnabled, [&] (propertiesCommon* item, uint8_t arg) {
        config.SetGyroEnabled((bool) arg);
        // Trigger reload of values for the selected channel
        devicesTriggerEvent(EVENT_CONFIG_GYRO_CHANGED);
        gyro.reload();
      }, luaGyroMainFolder.common.id);

      registerParameter(&luaGyroStatus, [&] (propertiesCommon* item, uint8_t arg) {}, luaGyroMainFolder.common.id);

        registerParameter(&luaGyroModelFolder,nullptr,luaGyroMainFolder.common.id);
            registerParameter(&luaGyroModesFolder,nullptr,luaGyroModelFolder.common.id);
            registerParameter(&luaGyroInputFolder,nullptr,luaGyroModelFolder.common.id);
            registerParameter(&luaGyroOutputFolder,nullptr,luaGyroModelFolder.common.id);        
        registerParameter(&luaGyroSettingsFolder,nullptr,luaGyroMainFolder.common.id);
            registerParameter(&luaGyroGainFolder,nullptr,luaGyroSettingsFolder.common.id);     
            registerParameter(&luaGyroSettingsSafeFolder,nullptr,luaGyroSettingsFolder.common.id);
            registerParameter(&luaGyroSettingsLevelFolder,nullptr,luaGyroSettingsFolder.common.id);
            registerParameter(&luaGyroSettingsLaunchFolder,nullptr,luaGyroSettingsFolder.common.id);
            registerParameter(&luaGyroSettingsHoverFolder,nullptr,luaGyroSettingsFolder.common.id);
        registerParameter(&luaGyroCalibrationFolder,nullptr,luaGyroMainFolder.common.id);
            registerParameter(&luaGyroRxOrientationFolder,nullptr,luaGyroCalibrationFolder.common.id);
            registerParameter(&luaGyroStickCalibrationFolder,nullptr,luaGyroCalibrationFolder.common.id);
          
    

            
     // ----- Gyro Model->Modes
    registerParameter(&luaGyroModePos1, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroModePos(0, (gyro_mode_t) arg);
    }, luaGyroModesFolder.common.id);
    registerParameter(&luaGyroModePos2, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroModePos(1, (gyro_mode_t) arg);
    }, luaGyroModesFolder.common.id);
    registerParameter(&luaGyroModePos3, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroModePos(2, (gyro_mode_t) arg);
    }, luaGyroModesFolder.common.id);
    registerParameter(&luaGyroModePos4, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroModePos(3, (gyro_mode_t) arg);
    }, luaGyroModesFolder.common.id);
    registerParameter(&luaGyroModePos5, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroModePos(4, (gyro_mode_t) arg);
    }, luaGyroModesFolder.common.id);


    // ----- Gyro Settings->Gain
    registerParameter(&luaGyroGainAxis, [&] (propertiesCommon* item, uint8_t arg) {
        setTextSelectionValue(&luaGyroGainAxis, arg);
        // Trigger reload of values for the selected channel
        devicesTriggerEvent(EVENT_CONFIG_GYRO_CHANGED);
    }, luaGyroGainFolder.common.id);
    registerParameter(&luaGyroPIDRateP, &luaparamGyroPIDRateP, luaGyroGainFolder.common.id);
    registerParameter(&luaGyroPIDRateI, &luaparamGyroPIDRateI, luaGyroGainFolder.common.id);
    registerParameter(&luaGyroPIDRateD, &luaparamGyroPIDRateD, luaGyroGainFolder.common.id);
    registerParameter(&luaGyroPIDGain, &luaparamGyroPIDGain, luaGyroGainFolder.common.id);

    // ----- Gyro Model->Input
    registerParameter(&luaGyroInputChannel, [&] (propertiesCommon* item, uint8_t arg) {
      luaparamGyroInputChannel(item,arg);
    }, luaGyroInputFolder.common.id);
    registerParameter(&luaGyroInputMode, &luaparamGyroInputMode, luaGyroInputFolder.common.id);

    // ----- Gyro Model->Output
    registerParameter(&luaGyroOutputChannel,  [&] (propertiesCommon* item, uint8_t arg) {
       luaparamGyroOutputChannel(item,arg);
    }, luaGyroOutputFolder.common.id);
    registerParameter(&luaGyroOutputMode, [&] (propertiesCommon* item, uint8_t arg) { 
        luaparamGyroOutputMode(item,arg);
    }, luaGyroOutputFolder.common.id);
    registerParameter(&luaGyroOutputInverted, [&] (propertiesCommon* item, uint8_t arg) { 
        luaparamGyroOutputInverted(item,arg);
    }, luaGyroOutputFolder.common.id);
   
    // ----- Gyro -> Calibration -> RxOrientation
    registerParameter(&luaGyroOrientationH,  [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroOrientation(arg, config.GetGyroOrientationV()); 
      updateParameters();
    }, luaGyroRxOrientationFolder.common.id);

    registerParameter(&luaGyroOrientationV,  [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroOrientation(config.GetGyroOrientationH(),arg);
      updateParameters();
    }, luaGyroRxOrientationFolder.common.id);

    registerParameter(&luaGyroAutoOrientation, [this](propertiesCommon* item, uint8_t arg) {
      luaparamGyroCalibrate(item, arg); 
    }, luaGyroRxOrientationFolder.common.id);

    // ----- Gyro -> Calibration -> Stick Calibration
    registerParameter(&luaGyroSubtrims, [this](propertiesCommon* item, uint8_t arg) { 
      luaparamGyroSubtrims(item, arg); 
    }, luaGyroStickCalibrationFolder.common.id);


    // Gyro Settings->Safe
    registerParameter(&luaGyroSAFEPitch, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroSAFEPitch(arg);
    }, luaGyroSettingsSafeFolder.common.id);
    registerParameter(&luaGyroSAFERoll, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroSAFERoll(arg);
    }, luaGyroSettingsSafeFolder.common.id);

    // Gyro Settings->Level
    registerParameter(&luaGyroLevelPitch, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroLevelPitch(arg);
    }, luaGyroSettingsLevelFolder.common.id);
    registerParameter(&luaGyroLevelRoll, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroLevelRoll(arg);
    }, luaGyroSettingsLevelFolder.common.id);
    
    // Gyro Setting->Launch
    registerParameter(&luaGyroLaunchAngle, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroLaunchAngle(arg);
    }, luaGyroSettingsLaunchFolder.common.id);
    
    // Gyro Settings->Hover
    registerParameter(&luaGyroHoverStrength, [&] (propertiesCommon* item, uint8_t arg) {
      config.SetGyroHoverStrength(arg);
    }, luaGyroSettingsHoverFolder.common.id);
#endif
  }

  registerParameter(&luaBindStorage, [](propertiesCommon* item, uint8_t arg) {
    config.SetBindStorage((rx_config_bindstorage_t)arg);
  });
  registerParameter(&luaBindMode, [this](propertiesCommon* item, uint8_t arg){
    // Complete when TX polls for status i.e. going back to idle, because we're going to lose connection
    if (arg == lcsQuery) {
      deferExecutionMillis(200, EnterBindingModeSafely);
    }
    sendCommandResponse(&luaBindMode, arg < 5 ? lcsExecuting : lcsIdle, arg < 5 ? "Entering..." : "");
  });

  registerParameter(&luaModelNumber);
  registerParameter(&luaELRSversion);
}

static void updateBindModeLabel()
{
  if (config.IsOnLoan())
    luaBindMode.common.name = "Return Model";
  else
    luaBindMode.common.name = "Enter Bind Mode";
}

void RXEndpoint::updateParameters()
{
  setTextSelectionValue(&luaSerialProtocol, config.GetSerialProtocol());
#if defined(PLATFORM_ESP32)
  if (RX_HAS_SERIAL1)
  {
    setTextSelectionValue(&luaSerial1Protocol, config.GetSerial1Protocol());
  }
#endif

  setTextSelectionValue(&luaSBUSFailsafeMode, config.GetFailsafeMode());

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    setTextSelectionValue(&luaAntennaMode, config.GetAntennaMode());
  }

  if (MinPower != MaxPower)
  {
    // The last item (for MatchTX) will be MaxPower - MinPower + 1
    uint8_t luaPwrVal = (config.GetPower() == PWR_MATCH_TX) ? POWERMGNT::getMaxPower() + 1 : config.GetPower();
    setTextSelectionValue(&luaTlmPower, luaPwrVal - POWERMGNT::getMinPower());
  }

  // Teamrace
  setTextSelectionValue(&luaTeamraceChannel, config.GetTeamraceChannel() - AUX2);
  setTextSelectionValue(&luaTeamracePosition, config.GetTeamracePosition());

  if (OPT_HAS_SERVO_OUTPUT)
  {
    const rx_config_pwm_t *pwmCh = config.GetPwmChannel(luaMappingChannelOut.properties.u.value - 1);
    setUint8Value(&luaMappingChannelIn, pwmCh->val.inputChannel + 1);
    setTextSelectionValue(&luaMappingOutputMode, pwmCh->val.mode);
    setTextSelectionValue(&luaMappingInverted, pwmCh->val.inverted);

#if defined(HAS_GYRO)
    setTextSelectionValue(&luaGyroEnabled, config.GetGyroEnabled());
    setStringValue(&luaGyroStatus,gyroStatus[gyro.getStatus()]);

    const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(luaMappingChannelOut.properties.u.value - 1);
    setUint16Value(&luaMappingChannelLimitMin, (uint16_t) limits->val.min);
    setUint16Value(&luaMappingChannelLimitMax, (uint16_t) limits->val.max);

    const rx_config_gyro_channel_t *gyroChIn = config.GetGyroChannel(luaGyroInputChannel.properties.u.value - 1);
    setTextSelectionValue(&luaGyroInputMode, gyroChIn->val.input_mode);
    
    const rx_config_gyro_channel_t *gyroChOut = config.GetGyroChannel(luaGyroOutputChannel.properties.u.value - 1);
    setTextSelectionValue(&luaGyroOutputMode, gyroChOut->val.output_mode);
    setTextSelectionValue(&luaGyroOutputInverted, gyroChOut->val.inverted);

    const rx_config_gyro_mode_pos_t *gyroModes = config.GetGyroModePos();
    setTextSelectionValue(&luaGyroModePos1, gyroModes->val.pos1);
    setTextSelectionValue(&luaGyroModePos2, gyroModes->val.pos2);
    setTextSelectionValue(&luaGyroModePos3, gyroModes->val.pos3);
    setTextSelectionValue(&luaGyroModePos4, gyroModes->val.pos4);
    setTextSelectionValue(&luaGyroModePos5, gyroModes->val.pos5);


    const rx_config_gyro_gains_t *gyroGains = config.GetGyroGains((gyro_axis_t) (luaGyroGainAxis.value));
    setUint8Value(&luaGyroPIDRateP, gyroGains->p);
    setUint8Value(&luaGyroPIDRateI, gyroGains->i);
    setUint8Value(&luaGyroPIDRateD, gyroGains->d);
    setUint8Value(&luaGyroPIDGain, gyroGains->gain);

    setTextSelectionValue(&luaGyroOrientationH, config.GetGyroOrientationH());
    setTextSelectionValue(&luaGyroOrientationV, config.GetGyroOrientationV());

    setUint8Value(&luaGyroSAFEPitch, config.GetGyroSAFEPitch());
    setUint8Value(&luaGyroSAFERoll, config.GetGyroSAFERoll());

    setUint8Value(&luaGyroLevelPitch, config.GetGyroLevelPitch());
    setUint8Value(&luaGyroLevelRoll, config.GetGyroLevelRoll());

    setUint8Value(&luaGyroLaunchAngle, config.GetGyroLaunchAngle());

    setUint8Value(&luaGyroHoverStrength, config.GetGyroHoverStrength());


    #endif // HAS_GYRO
  }

  if (config.GetModelId() == 255)
  {
    setStringValue(&luaModelNumber, "Off");
  }
  else
  {
    itoa(config.GetModelId(), modelString, 10);
    setStringValue(&luaModelNumber, modelString);
  }
  setTextSelectionValue(&luaBindStorage, config.GetBindStorage());
  updateBindModeLabel();

  if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
  {
    setUint8Value(&luaSourceSysId, config.GetSourceSysId() == 0 ? 255 : config.GetSourceSysId());  //display Source sysID if 0 display 255 to mimic logic in SerialMavlink.cpp
    setUint8Value(&luaTargetSysId, config.GetTargetSysId() == 0 ? 1 : config.GetTargetSysId());  //display Target sysID if 0 display 1 to mimic logic in SerialMavlink.cpp
    LUA_FIELD_SHOW(luaSourceSysId)
    LUA_FIELD_SHOW(luaTargetSysId)
  }
  else
  {
    LUA_FIELD_HIDE(luaSourceSysId)
    LUA_FIELD_HIDE(luaTargetSysId)
  }
}
#endif