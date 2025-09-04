#pragma once

#include <cstdint>
#include <array>

struct MultiSwitch {
    using switches_t = std::array<uint8_t, 8>;
    using ledColor_t = std::array<uint32_t, 64>;

    static void decode(const uint8_t*);
    static const switches_t& switches();
    static bool state(uint8_t);
    static bool ledState(uint8_t);
    static uint32_t ledColor(uint8_t);
    static bool hasData();
    private:    
    static bool mHasData;
    static const uint8_t minAddress;
    static const uint8_t maxAddress;
    static const uint8_t minLedAddress;
    static const uint8_t maxLedAddress;
    static switches_t mSwitches;
    static switches_t mLeds;
    static ledColor_t mLedColors;
};
