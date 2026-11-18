#pragma once

#include "EspNowProtocol.h"

class EspNowSlaveLink
{
public:
    void reset(const EspNowProtocol::Uid &uid, const EspNowProtocol::Mac &mac, const uint32_t session, const uint32_t now);
    bool enqueue(const uint8_t *const data, const size_t size);
    bool tick(const uint32_t now, EspNowProtocol::Packet &packet);
    void sent(const uint32_t now, const bool success);
    bool receive(const EspNowProtocol::Mac &source, const uint8_t *const data, const size_t size, const uint32_t receivedAt);
    bool popFrame(EspNowProtocol::Frame &frame)
    {
        return incoming.pop(frame);
    }
    void expire(const uint32_t now);

    bool hasMaster(const uint32_t now) const;
    bool connected(const uint32_t now) const;
    bool sending() const
    {
        return awaitingSend;
    }
    uint8_t assignedSlot() const
    {
        return slot;
    }
    const EspNowProtocol::Mac &masterMac() const
    {
        return master;
    }
    uint32_t errors() const
    {
        return errorCount;
    }

private:
    EspNowProtocol::Uid uid{};
    EspNowProtocol::Mac mac{};
    EspNowProtocol::Mac master{};
    EspNowProtocol::Queue<EspNowProtocol::Frame> outgoing;
    EspNowProtocol::Queue<EspNowProtocol::Frame> incoming;
    bool channelsValid = false;
    bool masterSeen = false;
    bool responseSent = false;
    bool awaitingSend = false;
    uint8_t slot = EspNowProtocol::NoSlot;
    uint8_t pendingFrames = 0;
    uint16_t sequence = 0;
    uint32_t session = 0;
    uint32_t lastMaster = 0;
    uint32_t lastCycle = 0;
    uint32_t joinDelay = 0;
    uint32_t discoveryRandom = 1;
    bool joinThisCycle = false;
    uint32_t errorCount = 0;

    void seedDiscoveryRandom(const uint32_t seed);
    uint32_t nextDiscoveryRandom();
    void clearFrames();
    bool acceptMaster(const EspNowProtocol::Mac &source, const EspNowProtocol::PacketView &packet, const uint32_t receivedAt);
    void beginPacket(EspNowProtocol::Packet &packet);
    void appendQueued(EspNowProtocol::Packet &packet);
    void acceptFrames(const Span<const uint8_t> frames);
};
