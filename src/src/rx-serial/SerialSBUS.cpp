#include "SerialSBUS.h"
#include "CRSF.h"
#include "device.h"
#include "config.h"

#if defined(TARGET_RX)

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

const auto UNCONNECTED_CALLBACK_INTERVAL_MS = 10;
const auto SBUS_CALLBACK_INTERVAL_MS = 9;

uint32_t SerialSBUS::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    static auto sendPackets = false;
    bool effectivelyFailsafed = failsafe || (!connectionHasModelMatch) || (!teamraceHasModelMatch);
    if ((effectivelyFailsafed && config.GetFailsafeMode() == FAILSAFE_NO_PULSES) || (!sendPackets && connectionState != connected))
    {
        return UNCONNECTED_CALLBACK_INTERVAL_MS;
    }
    sendPackets = true;

    if ((!frameAvailable && !frameMissed && !effectivelyFailsafed) || _outputPort->availableForWrite() < 25)
    {
        return DURATION_IMMEDIATELY;
    }

    // TODO: if failsafeMode == FAILSAFE_SET_POSITION then we use the set positions rather than the last values
    crsf_channels_s PackedRCdataOut;

#if !defined(TARGET_RX_GHOST_ATTO_V1)
#if defined(PLATFORM_ESP32)
    extern Stream* serial_protocol_tx;
    extern Stream* serial1_protocol_tx;

    if (((config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO) && streamOut == serial_protocol_tx)||
        ((config.GetSerial1Protocol() == PROTOCOL_SERIAL1_DJI_RS_PRO) && streamOut == serial1_protocol_tx))
#else
    if (config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO)
#endif
    {
        PackedRCdataOut.ch0 = fmap(channelData[0], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch1 = fmap(channelData[1], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch2 = fmap(channelData[2], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch3 = fmap(channelData[3], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch4 = fmap(channelData[5], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Record start/stop and photo
        PackedRCdataOut.ch5 = fmap(channelData[6], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Mode
        PackedRCdataOut.ch6 = fmap(channelData[7], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 176,  848); // Recenter and Selfie
        PackedRCdataOut.ch7 = fmap(channelData[8], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch8 = fmap(channelData[9], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch9 = fmap(channelData[10], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch10 = fmap(channelData[11], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch11 = fmap(channelData[12], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch12 = fmap(channelData[13], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch13 = fmap(channelData[14], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch14 = fmap(channelData[15], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch15 = channelData[4] < CRSF_CHANNEL_VALUE_MID ? 352 : 1696;
    }
    else
#endif
    {
        if (mFlags & 0b01) { // output upper channels
            PackedRCdataOut.ch0 = mChannels[0];
            PackedRCdataOut.ch1 = mChannels[1];
            PackedRCdataOut.ch2 = mChannels[2];
            PackedRCdataOut.ch3 = mChannels[3];
            PackedRCdataOut.ch4 = mChannels[4];
            PackedRCdataOut.ch5 = mChannels[5];
            PackedRCdataOut.ch6 = mChannels[6];
            PackedRCdataOut.ch7 = mChannels[7];
            PackedRCdataOut.ch8 = mChannels[8];
            PackedRCdataOut.ch9 = mChannels[9];
            PackedRCdataOut.ch10 = mChannels[10];
            PackedRCdataOut.ch11 = mChannels[11];
            PackedRCdataOut.ch12 = mChannels[12];
            PackedRCdataOut.ch13 = mChannels[13];
            PackedRCdataOut.ch14 = mChannels[14];
            PackedRCdataOut.ch15 = mChannels[15];
        }
        else {
            PackedRCdataOut.ch0 = channelData[0];
            PackedRCdataOut.ch1 = channelData[1];
            PackedRCdataOut.ch2 = channelData[2];
            PackedRCdataOut.ch3 = channelData[3];
            PackedRCdataOut.ch4 = channelData[4];
            PackedRCdataOut.ch5 = channelData[5];
            PackedRCdataOut.ch6 = channelData[6];
            PackedRCdataOut.ch7 = channelData[7];
            PackedRCdataOut.ch8 = channelData[8];
            PackedRCdataOut.ch9 = channelData[9];
            PackedRCdataOut.ch10 = channelData[10];
            PackedRCdataOut.ch11 = channelData[11];
            PackedRCdataOut.ch12 = channelData[12];
            PackedRCdataOut.ch13 = channelData[13];
            PackedRCdataOut.ch14 = channelData[14];
            PackedRCdataOut.ch15 = channelData[15];
        }
    }

    uint8_t extraData = 0;
    extraData |= effectivelyFailsafed ? SBUS_FLAG_FAILSAFE_ACTIVE : 0;
    extraData |= frameMissed ? SBUS_FLAG_SIGNAL_LOSS : 0;

    _outputPort->write(0x0F);    // HEADER
    _outputPort->write((byte *)&PackedRCdataOut, sizeof(PackedRCdataOut));
    _outputPort->write((uint8_t)extraData);    // ch 17, 18, lost packet, failsafe
    _outputPort->write((uint8_t)0x00);    // FOOTER
    return SBUS_CALLBACK_INTERVAL_MS;
}

void SerialSBUS::queueMSPFrameTransmission(uint8_t* const data) {
#if defined(PLATFORM_ESP32)
    extern Stream* serial1_protocol_tx;
#endif
    const uint8_t destAddress = data[3];
    const uint8_t srcAddress = data[4];
    const uint8_t realm = data[5];
    const uint8_t cmd = data[6];
    if ((srcAddress == 0xea) && (destAddress >= 0xc0) && (destAddress <= 0xcf)) {
        if (realm == 0xa0) { // cruise controller
            DBGLN("SBUS Realm CC: cmd: %d %d", cmd, mFlags);
            if (cmd == 0x03) {
                mFlags = 0; // normal channel order
            }
            else if (cmd == 0x04) { // flags, 16 channels as 8-bit
#if defined(PLATFORM_ESP32)
                if (streamOut == serial1_protocol_tx) { // only for Serial2
                    mFlags = data[7];
                }
#else
                mFlags = data[7];
#endif
                for(uint8_t i = 0; i < 16; ++i) {
                    const int8_t ch8bit = data[8 + i];
                    mChannels[i] = (int32_t(ch8bit) * ((CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) / 2)) / 127 + CRSF_CHANNEL_VALUE_MID; 
                }
            }
        }
    }
}

#endif
