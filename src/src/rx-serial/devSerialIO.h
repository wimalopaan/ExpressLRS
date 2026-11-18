#pragma once

#include "device.h"

extern device_t Serial0_device;
#if defined(PLATFORM_ESP32)
extern device_t Serial1_device;
#endif
#if defined(WMEXTENSION) && defined(WMSERIAL2) && defined(PLATFORM_ESP32) && defined(TARGET_RX)
extern device_t Serial2_device;
#endif
extern void sendImmediateRC();
extern void handleSerialIO();
extern void crsfRCFrameAvailable();
extern void crsfRCFrameMissed();
