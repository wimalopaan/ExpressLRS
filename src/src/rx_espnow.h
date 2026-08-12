#pragma once

#include <cstdint>
#include <array>

#include "CRSFParser.h"
#include "CRSFRouter.h"

#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "rx_wmextension.h"

struct EspNowMaster : public CRSFConnector {
    EspNowMaster();
    ~EspNowMaster();
    
    void forwardMessage(const crsf_header_t *message) override;

    void processBytes(const uint8_t* data, int len);
    
    void start();
    void sendSwitches(const MultiSwitch* msw);
private:
    CRSFParser crsfParser;
    bool s_initialized = false;
    std::array<uint8_t, 64> inBuffer;
    std::array<uint8_t, 64> outBuffer;
    
    uint16_t mUid = 1234;
};
#endif
