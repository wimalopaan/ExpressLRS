#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(TARGET_RX) && defined(WMESPNOW_SLAVE)

#include "rx_espnow.h"

#include "CRSFRouter.h"
#include "OTA.h"
#include "RxRadioMode.h"
#include "common.h"
#include "config.h"
#include "devServoOutput.h"
#include "rx-serial/devSerialIO.h"

extern RxConfig config;
extern bool webserverPreventAutoStart;

static_assert(EspNowProtocol::RcChannels == CRSF_NUM_CHANNELS, "RC channel count mismatch");

bool EspNowConnector::enabled()
{
    return isEspNowSlaveMode();
}

bool EspNowConnector::selectRadioMode()
{
#if defined(RADIO_SX127X)
    const int irq = GPIO_PIN_DIO0;
    const int irq2 = GPIO_PIN_DIO0_2;
#else
    const int irq = GPIO_PIN_DIO1;
    const int irq2 = GPIO_PIN_DIO1_2;
#endif
    rxRadioMode = classifyRxRadioPins(
        {{GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI, GPIO_PIN_NSS, irq}},
        {{GPIO_PIN_NSS_2, irq2}},
        {GPIO_PIN_DIO0, GPIO_PIN_DIO0_2, GPIO_PIN_DIO1, GPIO_PIN_DIO1_2,
         GPIO_PIN_BUSY, GPIO_PIN_BUSY_2, GPIO_PIN_RST, GPIO_PIN_RST_2,
         GPIO_PIN_ANT_CTRL, GPIO_PIN_ANT_GROUP, GPIO_PIN_PA_ENABLE, GPIO_PIN_RFamp_APC2,
         GPIO_PIN_RX_ENABLE, GPIO_PIN_TX_ENABLE, GPIO_PIN_RX_ENABLE_2, GPIO_PIN_TX_ENABLE_2});
    return rxRadioMode != RxRadioMode::Invalid;
}

bool EspNowConnector::initialize()
{
    if (!enabled())
        return false;
#if defined(RADIO_SX128X)
    const uint8_t index = enumRatetoIndexSafe(RATE_LORA_2G4_100HZ_8CH);
#else
    const uint8_t index = enumRatetoIndexSafe(RATE_LORA_900_100HZ_8CH);
#endif
    ExpressLRS_currAirRate_Modparams = get_elrs_airRateConfig(index);
    ExpressLRS_currAirRate_RFperfParams = get_elrs_RFperfParams(index);
    // Without an RF link only the metadata for 16-channel serial/PWM output is needed.
    OtaUpdateSerializers(smHybridOr16ch, OTA8_PACKET_SIZE);
    return true;
}

void EspNowConnector::prepareTick()
{
}

void EspNowConnector::deliver(EspNowProtocol::Frame &frame)
{
    auto *message = reinterpret_cast<crsf_header_t *>(frame.data.data());
    if (message->type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        crsfRouter.processMessage(this, message);
        return;
    }
    if (!link.connected(micros()))
        return;
    const bool armed = EspNowProtocol::unpackChannels(frame, ChannelData);
    if (isArmed != armed)
    {
        isArmed = armed;
        devicesTriggerEvent(EVENT_ARM_FLAG_CHANGED);
    }
    // OTA channels also bypass message routing and notify each configured serial/PWM output.
    crsfRCFrameAvailable();
    if (teamraceHasModelMatch)
        servoNewChannelsAvailable();
    sendImmediateRC();
}

void EspNowConnector::updateConnection(const uint32_t now)
{
    webserverPreventAutoStart = link.hasMaster(now);
    const bool active = link.connected(now);
    linkStats.uplink_Link_quality = active ? 100 : 0;
    if (connectionState >= MODE_STATES)
        return;
    connectionHasModelMatch = active;
    if (connectionState == (active ? connected : disconnected))
        return;
    setConnectionState(active ? connected : disconnected);
    if (active && config.GetPowerOnCounter() != 0)
        config.SetPowerOnCounter(0);
    if (!active && isArmed)
    {
        isArmed = false;
        devicesTriggerEvent(EVENT_ARM_FLAG_CHANGED);
    }
}

void EspNowConnector::updatePeers()
{
    const auto &peer = link.masterMac();
    if (knownPeers[0] != peer)
    {
        radio.removePeer(knownPeers[0]);
        knownPeers[0] = peer;
    }
}
#endif
