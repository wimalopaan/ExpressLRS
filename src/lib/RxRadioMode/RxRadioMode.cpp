#include "RxRadioMode.h"

#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(WMESPNOW_SLAVE) && defined(TARGET_RX)
RxRadioMode rxRadioMode = RxRadioMode::Invalid;
#else
RxRadioMode rxRadioMode = RxRadioMode::Ota;
#endif

RxRadioMode classifyRxRadioPins(const std::array<int, 5> &primary, const std::array<int, 2> &secondary, std::initializer_list<int> optional)
{
    bool any = false;
    bool complete = true;
    for (const int pin : primary)
    {
        if (pin < -1)
            return RxRadioMode::Invalid;
        any |= pin >= 0;
        complete &= pin >= 0;
    }
    for (const int pin : secondary)
    {
        if (pin < -1)
            return RxRadioMode::Invalid;
        any |= pin >= 0;
    }
    for (const int pin : optional)
    {
        if (pin < -1)
            return RxRadioMode::Invalid;
        any |= pin >= 0;
    }
    if (!any)
        return RxRadioMode::EspNowSlave;
    if (!complete || ((secondary[0] >= 0) != (secondary[1] >= 0)))
        return RxRadioMode::Invalid;
    return RxRadioMode::Ota;
}
