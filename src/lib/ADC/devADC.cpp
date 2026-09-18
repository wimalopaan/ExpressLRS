#include "targets.h"
#if defined(TARGET_TX)
#include "common.h"
#include "devADC.h"
#include "logging.h"

#define ADC_READING_PERIOD_MS 20

#if defined(WMEXTENSION) && defined(WMRXTX_ANALOG)
# define CALIBRATION_TIMEOUT_MS 10000
# include "../lib/OPTIONS/options.h"
# include "../include/crsf_protocol.h"
# include "../AnalogVbat/median.h"
# include "../CONFIG/config.h"
static volatile int analogReadings[ADC_MAX_DEVICES + MAX_ADC_CHANNELS];
# if defined(PLATFORM_ESP32)
# include "esp_adc_cal.h"
# endif
# include "../PWM/PWM.h"
struct Gauge {
    int channel = -1;
    int lowMillis = 6400;
    int highMillis = 7400;
    int scale = 60; // less than 1000
    int duty = 0;
};
static Gauge gauge;
#else
static volatile int analogReadings[ADC_MAX_DEVICES];
#endif

int getADCReading(adc_reading reading)
{
    return analogReadings[reading];
}

#if defined(WMEXTENSION) && defined (WMRXTX_ANALOG)
struct CalibState {
    volatile uint32_t counter = 0;
};
static CalibState calibState;

void setGaugeMin(uint16_t v) {
    gauge.lowMillis = v;
}
void setGaugeMax(uint16_t v) {
    gauge.highMillis = v;
}
void setGaugeScale(uint16_t v) {
    gauge.scale = v;
}

static bool initialize() {
#if defined(PLATFORM_ESP32)
    int atten = hardware_int(HARDWARE_vbat_atten);
    if (atten != -1) {
        analogSetPinAttenuation(hardware_pin(HARDWARE_vbat), (adc_attenuation_t)atten);
    }
#endif
    return true;    
}
void startInputCalibration() {
    calibState.counter = CALIBRATION_TIMEOUT_MS / ADC_READING_PERIOD_MS;
    for (int ch = 0; ch < (CRSF_NUM_CHANNELS + CRSF_EXTRA_CHANNELS); ++ch) {
        ChannelData[ch] = CRSF_CHANNEL_VALUE_MID;
    }
    const int maxAdcChannels = std::min(MAX_ADC_CHANNELS, GPIO_PIN_ADC_INPUTS_COUNT);
    for (int ch = 0; ch < maxAdcChannels; ++ch) {
        const uint16_t chValue = analogReadings[ADC_MAX_DEVICES + ch];
        tx_config_t::analog_calibration_t::chan_calib c;
        c.min = c.mid = c.max = chValue;
        config.SetCalibration(ch, c, false);
    }    
}
bool isInputCalbrationRunning() {
    return (calibState.counter > 0);
}
uint16_t analogToCrsf(const uint16_t ch, const uint16_t an) {
    const tx_config_t::analog_calibration_t::chan_calib c = config.GetCalibration(ch);
    const int d = (an - c.mid);
    float dn = 0.0f;
    if (d < 0) {
        dn = (1.0f * d) / std::max(100, (c.mid - c.min));
    }
    else if (d > 0) {
        dn = (1.0f * d) / std::max(100, (c.max - c.mid));
    }
    const int delta = dn * (CRSF_CHANNEL_VALUE_STD_MAX - CRSF_CHANNEL_VALUE_STD_MIN) / 2;
    const int result = CRSF_CHANNEL_VALUE_MID + delta;
    return std::max(std::min(result, CRSF_CHANNEL_VALUE_STD_MAX), CRSF_CHANNEL_VALUE_STD_MIN);
}
static int32_t adcToMilliVolts(uint32_t adc) {
    const int offset = hardware_int(HARDWARE_vbat_offset);
    const int scale = hardware_int(HARDWARE_vbat_scale);
    if (offset < 0 && adc <= (uint32_t)(-offset)){
        return 0;
    }
    return (((int32_t)adc - offset) * 10000) / scale;
}
#endif

static int start()
{
#if defined(GPIO_PIN_JOYSTICK)
    if (GPIO_PIN_JOYSTICK != UNDEF_PIN)
    {
        return DURATION_IMMEDIATELY;
    }
#endif
#if defined(GPIO_PIN_PA_PDET)
    if (GPIO_PIN_PA_PDET != UNDEF_PIN)
    {
        return DURATION_IMMEDIATELY;
    }
#endif
#if defined(WMEXTENSION) && defined(WMRXTX_ANALOG)
    if (GPIO_PIN_ADC_INPUTS_COUNT > 0) {

#if defined(PLATFORM_ESP32)
        analogReadResolution(12);
        int atten = hardware_int(HARDWARE_vbat_atten);
        if (atten != -1) {
            DBGLN("atten: %d", atten);
            const bool useCal = atten > ADC_11db;
            if (useCal) {
                atten -= (ADC_11db + 1);
                DBGLN("usecal atten: %d", atten);
                static esp_adc_cal_characteristics_t cx;
                const int sourcePin = hardware_pin(HARDWARE_vbat);
                const int8_t channel = digitalPinToAnalogChannel(sourcePin);
                const adc_unit_t unit = (channel > (SOC_ADC_MAX_CHANNEL_NUM - 1)) ? ADC_UNIT_2 : ADC_UNIT_1;
                esp_adc_cal_characterize(unit, (adc_atten_t)atten, ADC_WIDTH_BIT_12, 3300, &cx);
            }
            analogSetPinAttenuation(hardware_pin(HARDWARE_vbat), (adc_attenuation_t)atten);
        }
#endif
        if (int gaugePin = hardware_pin(HARDWARE_gauge_pwm); gaugePin != UNDEF_PIN) {
            gauge.channel = PWM.allocate(gaugePin, 10000);        
            DBGLN("GaugeChannel: %u / %u", gauge.channel, gaugePin);
            PWM.setDuty(gauge.channel, 100);            
        }
        
        return DURATION_IMMEDIATELY;
    }
#endif    
    return DURATION_NEVER;
}

static int timeout()
{
    extern volatile bool busyTransmitting;
    static bool fullWait = true;

    // if called because of a full-timeout and the main loop is transmitting then we will
    // leave the fullWait flag true and return with an immediate timeout so we can wait for
    // the main loop to finish transmitting, which will pop us into the next state.
    if (fullWait && busyTransmitting && connectionState < MODE_STATES) return DURATION_IMMEDIATELY;
    fullWait = false;

    // If the main loop is NOT transmitting then return with an immediate timeout until it transitions
    // to transmitting
    if (!busyTransmitting && connectionState < MODE_STATES) return DURATION_IMMEDIATELY;

    // If we reach this point we are assured that the main loop has just transitioned from
    // not transmitting to transmitting, so it's safe to read the ADC
#if defined(GPIO_PIN_JOYSTICK)
    if (GPIO_PIN_JOYSTICK != UNDEF_PIN)
    {
#if defined(PLATFORM_ESP32)
        analogReadings[ADC_JOYSTICK] = analogReadMilliVolts(GPIO_PIN_JOYSTICK);
#else
        analogReadings[ADC_JOYSTICK] = analogRead(GPIO_PIN_JOYSTICK);
#endif
    }
#endif
#if defined(GPIO_PIN_PA_PDET)
    if (GPIO_PIN_PA_PDET != UNDEF_PIN)
    {
        analogReadings[ADC_PA_PDET] = analogReadMilliVolts(GPIO_PIN_PA_PDET);
    }
#endif
#if defined(WMEXTENSION) && defined (WMRXTX_ANALOG)
    const int maxAdcChannels = std::min(MAX_ADC_CHANNELS, GPIO_PIN_ADC_INPUTS_COUNT);
    const float f = 0.1f;
    for (int ch = 0; ch < maxAdcChannels; ++ch) {
        const int8_t pin = GPIO_PIN_ADC_INPUTS[ch];
        analogReadings[ADC_MAX_DEVICES + ch] = (1.0f - f) * analogReadings[ADC_MAX_DEVICES + ch] + f * analogRead(pin);
    }
    const uint32_t vbat = analogRead(hardware_pin(HARDWARE_vbat));
    static MedianAvgFilter<uint16_t, 5> smooth;
    static uint32_t vbatMillis;
    if (smooth.add(vbat) == 0) {
        vbatMillis = adcToMilliVolts(smooth.calc());
    }
    if (calibState.counter > 0) {
        --calibState.counter;
        for (int ch = 0; ch < maxAdcChannels; ++ch) {
            const uint16_t chValue = analogReadings[ADC_MAX_DEVICES + ch];
            tx_config_t::analog_calibration_t::chan_calib c = config.GetCalibration(ch);
            if (chValue > c.max) {
                c.max = chValue;
                config.SetCalibration(ch, c, false);
                calibState.counter = CALIBRATION_TIMEOUT_MS / ADC_READING_PERIOD_MS;
            }
            if (chValue < c.min) {
                c.min = chValue;
                config.SetCalibration(ch, c, false);
                calibState.counter = CALIBRATION_TIMEOUT_MS / ADC_READING_PERIOD_MS;
            }
        }
        if (calibState.counter == 0) {
            config.SetVBatCalib(vbatMillis);
            config.event(EVENT_CONFIG_CALIBRATION_CHANGED);
        }
    }
    else {
        for (int ch = 0; ch < (CRSF_NUM_CHANNELS + CRSF_EXTRA_CHANNELS); ++ch) {
            if (ch < maxAdcChannels) {
                const float f = (1.0f * config.GetVBatCalib()) / vbatMillis;
                ChannelData[ch] = analogToCrsf(ch, (f * analogReadings[ADC_MAX_DEVICES + ch]));
            }
            else {
                ChannelData[ch] = CRSF_CHANNEL_VALUE_MID;
            }
        }
        if (gauge.channel >= 0) {
            const int gaugeD = vbatMillis - gauge.lowMillis;
            if (gaugeD <= 0) {
                PWM.setDuty(gauge.channel, 0);                        
            }
            else {
                const int gd = std::max(1, (gauge.highMillis - gauge.lowMillis));
                gauge.duty = std::min(gauge.scale, (gaugeD * gauge.scale) / gd);
                PWM.setDuty(gauge.channel, gauge.duty);                        
            }
        }
    }    
    static int counter = 0;
    if (++counter > 20) {
        counter = 0;
        DBGLN("ADC pin: %u, vbat: %u, a0: %u, a1: %u", hardware_pin(HARDWARE_vbat), vbat, analogReadings[ADC_MAX_DEVICES + 0], analogReadings[ADC_MAX_DEVICES + 1]);
        DBGLN("VBatMillis: %u", vbatMillis);
        DBGLN("CH0: %u, min %u, max %u", ChannelData[0], config.GetCalibration(0).min, config.GetCalibration(0).max);
        DBGLN("Gauge ch: %u duty: %u", gauge.channel, gauge.duty);
    }
#endif
    fullWait = true;
    return ADC_READING_PERIOD_MS;
}

device_t ADC_device = {
    #if defined(WMEXTENSION) && defined(WMRXTX_ANALOG)
    .initialize = initialize,
    #else
    .initialize = nullptr,
    #endif
    .start = start,
    .event = nullptr,
    .timeout = timeout,
};
#endif
