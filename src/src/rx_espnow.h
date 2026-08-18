#pragma once

#include <cstdint>
#include <array>

#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "device.h"
#include "rx_wmextension.h"

#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(TARGET_RX)

extern device_t ESPNOWSender_device;

struct EspNowMaster : public CRSFConnector {
    void forwardMessage(const crsf_header_t *message) override;
    void processBytes(const uint8_t* data, int len);
    void start();
    void stop();
    void tick();
    bool menuMode();
    void sendHeartbeat();
    void timeout();
private:
#if defined(WMESPNOW_RECV)
    void rcPacketToChannelsData(const crsf_header_t *message);
#else
    void sendRCFrame();
#endif
    
    CRSFParser crsfParser;
    bool isInitialized = false;
    std::array<uint8_t, 64 - 3 + 6> inBuffer;
    std::array<uint8_t, 64 - 3 + 6> outBuffer;
    uint32_t mConnectCounter = 0;
    uint32_t mMenuCounter = 0;
    uint32_t mChSendCount = 0;
    uint32_t mChRecvCount = 0;
};
#endif
