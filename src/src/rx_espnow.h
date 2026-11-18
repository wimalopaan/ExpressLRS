#pragma once

#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(TARGET_RX)

#if defined(WMESPNOW_SLAVE)
#include "../lib/EspNowLink/EspNowSlaveLink.h"
#else
#include "../lib/EspNowLink/EspNowMasterLink.h"
#endif
#include "../lib/EspNowLink/EspNowEvents.h"
#include "../lib/EspNowRadio/EspNowRadio.h"
#include "CRSFConnector.h"
#include "device.h"

class EspNowConnector final : public CRSFConnector
{
public:
    void forwardMessage(const crsf_header_t *const message) override;
    void start();
    void stop();
    void poll();
    void onDeviceEvent();
    void onReceive(const uint8_t *const mac, const uint8_t *const data, const size_t size);
    void onSent(const bool success);
    // Role specific: false if this role must not start the ESP-NOW link.
    static bool enabled();
    // Role specific: selects the RX radio mode at boot; false if the hardware configuration is invalid.
    static bool selectRadioMode();
    // Role specific: called once before the radio setup; false disables the ESP-NOW device.
    static bool initialize();

private:
    enum class State : uint8_t
    {
        Stopped,
        WaitingToStart,
        Ready
    };
#if defined(WMESPNOW_SLAVE)
    EspNowSlaveLink link;
#else
    EspNowMasterLink link;
#endif
    EspNowRadio radio;
    EspNowEvents events;
    EspNowProtocol::Uid uid{};
    EspNowProtocol::Mac mac{};
    std::array<EspNowProtocol::Mac, EspNowProtocol::MaxSlaves> knownPeers{};
    volatile State state = State::Stopped;
    uint32_t lastStartAttempt = 0;
    uint32_t lastErrorReport = 0;
    uint32_t reportedErrors = 0;

    bool running() const
    {
        return state == State::Ready;
    }
    void processQueuedEvents();
    void waitForBinding();
    bool uidChanged() const;
    void applyUidChange();
    void tick();
    void reportErrors();

    // Role specific, implemented in rx_espnow_master.cpp or rx_espnow_slave.cpp.
    void prepareTick();
    void deliver(EspNowProtocol::Frame &frame);
    void updateConnection(const uint32_t now);
    void updatePeers();
};

extern EspNowConnector espNow;
extern device_t ESPNOW_device;
#endif
