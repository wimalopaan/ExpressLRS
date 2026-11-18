#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(TARGET_RX) && !defined(WMESPNOW_SLAVE)

#include "rx_espnow.h"

#include "CRSFRouter.h"
#include "OTA.h"
#include "common.h"
#include "logging.h"

static_assert(EspNowProtocol::RcChannels == CRSF_NUM_CHANNELS, "RC channel count mismatch");

bool EspNowConnector::enabled()
{
    return true;
}

bool EspNowConnector::selectRadioMode()
{
    return true;
}

bool EspNowConnector::initialize()
{
    return true;
}

void EspNowConnector::prepareTick()
{
    link.setChannels(EspNowProtocol::packChannels(ChannelData, isArmed), connectionState == connected && connectionHasModelMatch);
}

void EspNowConnector::deliver(EspNowProtocol::Frame &frame)
{
    crsfRouter.processMessage(this, reinterpret_cast<crsf_header_t *>(frame.data.data()));
}

void EspNowConnector::updateConnection(const uint32_t)
{
}

void EspNowConnector::updatePeers()
{
    for (uint8_t i = 0; i < EspNowProtocol::MaxSlaves; ++i)
    {
        const auto &peer = link.peerMac(i);
        if (knownPeers[i] != peer)
        {
            radio.removePeer(knownPeers[i]);
            const auto address = static_cast<crsf_addr_e>(EspNowProtocol::slaveAddress(i));
            removeDevice(address);
            if (peer != EspNowProtocol::Mac{})
                addDevice(address);
            knownPeers[i] = peer;
            DBGLN("ESP-NOW slot %u address %02x: %02x:%02x:%02x:%02x:%02x:%02x",
                  i, address, peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);
        }
    }
}
#endif
