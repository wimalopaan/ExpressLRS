#pragma once

#include "../EspNowLink/EspNowProtocol.h"

class EspNowRadio
{
public:
    using ReceiveCallback = void (*)(const uint8_t *const source, const uint8_t *const data, const size_t size);
    using SentCallback = void (*)(const bool success);

    bool start(EspNowProtocol::Mac &mac, const ReceiveCallback receive, const SentCallback sent);
    void stop();
    // Success means queued; SentCallback reports completion.
    bool send(EspNowProtocol::Packet &packet);
    void removePeer(const EspNowProtocol::Mac &peer);
    static uint32_t newSession();

private:
    bool ensurePeer(const EspNowProtocol::Mac &peer);
};
