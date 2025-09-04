#include "rx_switch.h"

#include "targets.h"
#include "FIFO.h"
#include "device.h"

const uint8_t MultiSwitch::minAddress = 240;
const uint8_t MultiSwitch::maxAddress = minAddress + 8 - 1;
const uint8_t MultiSwitch::minLedAddress = maxAddress + 1;
const uint8_t MultiSwitch::maxLedAddress = minLedAddress + 8 - 1;
MultiSwitch::switches_t MultiSwitch::mSwitches{};
MultiSwitch::switches_t MultiSwitch::mLeds{};
MultiSwitch::ledColor_t MultiSwitch::mLedColors = []{
    MultiSwitch::ledColor_t cc{};
    for(auto& c : cc) {
        c = (0 << 16) + (32 << 8) + (0 << 0);
    }
    return cc;
}();
bool MultiSwitch::mHasData = false;

bool MultiSwitch::hasData() {
    return mHasData;
}
const MultiSwitch::switches_t& MultiSwitch::switches() {
    return mSwitches;
}
bool MultiSwitch::state(const uint8_t sw) {
    if (sw < 64) {
        const uint8_t swidx = sw / 8;
        const uint8_t swbit = sw % 8;
        return (mSwitches[swidx] & (1 << swbit));
    }
    return false;
}
bool MultiSwitch::ledState(const uint8_t led) {
    if (led < 64) {
        const uint8_t lidx = led / 8;
        const uint8_t lbit = led % 8;
        return (mLeds[lidx] & (1 << lbit));
    }
    return false;
}
uint32_t MultiSwitch::ledColor(const uint8_t led) {
    if (led < mLedColors.size()) {
        return mLedColors[led];
    }
    return 0;
}
void MultiSwitch::decode(const uint8_t* const data) {
    const uint8_t destAddress = data[3];
    const uint8_t srcAddress = data[4];
    const uint8_t realm = data[5];
    const uint8_t cmd = data[6];
    if ((srcAddress == 0xea) && (destAddress >= 0xc0) && (destAddress <= 0xcf)) {
        if (realm == 0xa1) { // switch
            DBGLN("MSW Realm SW: cmd: %d", cmd);
            if (cmd == 0x01) { // set
                const uint8_t swAddress = data[7];
                const uint16_t sw = data[8];
                if ((swAddress >= minAddress) && (swAddress <= maxAddress)) {
                    mHasData = true;
                    const uint8_t group = swAddress - minAddress;
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool on = (((sw >> i) & 0b01) > 0);
                        const uint8_t mask = (1 << i);
                        if (on) {
                            mSwitches[group] |= mask;
                        }
                        else {
                            mSwitches[group] &= ~mask;
                        }
                    }
                }
                else if ((swAddress >= minLedAddress) && (swAddress <= maxLedAddress)) {
                    mHasData = true;
                    const uint8_t group = swAddress - minLedAddress;
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool on = (((sw >> i) & 0b01) > 0);
                        const uint8_t mask = (1 << i);
                        if (on) {
                            mLeds[group] |= mask;
                        }
                        else {
                            mLeds[group] &= ~mask;
                        }
                    }
                }
            }
            else if (cmd == 0x07) { // set4
                const uint8_t swAddress = data[7];
                const uint16_t sw = (data[8] << 8) + data[9];
                DBGLN("MSW Set4: adr: %d, v: %d", swAddress, sw);
                if ((swAddress >= minAddress) && (swAddress <= maxAddress)) {
                    mHasData = true;
                    const uint8_t group = swAddress - minAddress;
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool on = (((sw >> (2 * i)) & 0b11) > 0);
                        const uint8_t mask = (1 << i);
                        if (on) {
                            DBGLN("MSW Set4 sw on:", i);
                            mSwitches[group] |= mask;
                        }
                        else {
                            DBGLN("MSW Set4 sw off:", i);
                            mSwitches[group] &= ~mask;
                        }
                    }
                }
                else if ((swAddress >= minLedAddress) && (swAddress <= maxLedAddress)) {
                    mHasData = true;
                    const uint8_t group = swAddress - minLedAddress;
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool on = (((sw >> (2 * i)) & 0b11) > 0);
                        const uint8_t mask = (1 << i);
                        if (on) {
                            DBGLN("MSW Set4 led on:", i);
                            mLeds[group] |= mask;
                        }
                        else {
                            DBGLN("MSW Set4 led off:", i);
                            mLeds[group] &= ~mask;
                        }
                    }
                }
            }
            else if (cmd == 0x09) { // Set4M
                const uint8_t count = data[7];
                for(uint8_t i = 0; i < count; ++i) {
                    const uint8_t swAddress = data[8 + 3 * i];
                    const uint16_t sw = (data[9 + 3 * i] << 8) + data[10 + 3 * i];
                    DBGLN("MSW Set4M: cnt: %d, adr: %d, v: %d", count, swAddress, sw);
                    if ((swAddress >= minAddress) && (swAddress <= maxAddress)) {
                        mHasData = true;
                        const uint8_t swGroup = swAddress - minAddress;
                        for(uint8_t k = 0; k < 8; ++k) {
                            const bool on = (((sw >> (2 * k)) & 0b11) > 0);
                            const uint8_t mask = (1 << k);
                            if (on) {
                                mSwitches[swGroup] |= mask;
                            }
                            else {
                                mSwitches[swGroup] &= ~mask;
                            }
                        }
                    }
                    else if ((swAddress >= minLedAddress) && (swAddress <= maxLedAddress)) {
                        mHasData = true;
                        const uint8_t swGroup = swAddress - minLedAddress;
                        for(uint8_t k = 0; k < 8; ++k) {
                            const bool on = (((sw >> (2 * k)) & 0b11) > 0);
                            const uint8_t mask = (1 << k);
                            if (on) {
                                mLeds[swGroup] |= mask;
                            }
                            else {
                                mLeds[swGroup] &= ~mask;
                            }
                        }
                    }
                }
            }
            else if (cmd == 0x08) { // set64
                const uint8_t swAddress = data[7];
                const uint8_t swGroup = (data[8] & 0x07);
                const uint16_t swSwitches = (data[9] << 8) + data[10];
                DBGLN("MSW Set64: adr: %d, grp: %d, v: %d", swAddress, swGroup, swSwitches);
                if (swAddress == minAddress) {
                    mHasData = true;
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool on = (((swSwitches >> (2 * i)) & 0b11) > 0);
                        const uint8_t mask = (1 << i);
                        if (on) {
                            mSwitches[swGroup] |= mask;
                        }
                        else {
                            mSwitches[swGroup] &= ~mask;
                        }
                    }
                }
            }
        }
    }
} 