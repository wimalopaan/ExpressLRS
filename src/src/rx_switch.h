#pragma once

#include <cstdint>
#include <array>

struct MultiSwitch {
    using switches_t = std::array<uint8_t, 8>;

    static void decode(const uint8_t* data);
    static const switches_t& switches();
    static bool state(uint8_t sw);
    static bool hasData();
    private:    
    static bool mHasData;
    static const uint8_t minAddress;
    static const uint8_t maxAddress;
    static switches_t mSwitches;
};
