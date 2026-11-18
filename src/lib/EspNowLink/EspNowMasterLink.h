#pragma once

#include "EspNowProtocol.h"

class EspNowMasterLink
{
public:
    void reset(const EspNowProtocol::Uid &uid, const EspNowProtocol::Mac &mac, const uint32_t session, const uint32_t now);
    bool enqueue(const uint8_t *const data, const size_t size);
    void setChannels(const EspNowProtocol::Frame &frame, const bool valid);
    bool tick(const uint32_t now, EspNowProtocol::Packet &packet);
    void sent(const uint32_t now, const bool success);
    bool receive(const EspNowProtocol::Mac &source, const uint8_t *const data, const size_t size, const uint32_t receivedAt);
    bool popFrame(EspNowProtocol::Frame &frame)
    {
        return incoming.pop(frame);
    }
    void expire(const uint32_t now);

    bool sending() const
    {
        return awaitingSend;
    }
    const EspNowProtocol::Mac &peerMac(const uint8_t index) const
    {
        return peers[index].mac;
    }
    uint32_t errors() const
    {
        return errorCount;
    }

private:
    enum class Phase : uint8_t
    {
        Startup,
        Idle,
        Polling,
        WaitingForReply,
        Discovery
    };
    static constexpr uint8_t UnassignedRecipients = 0xff;

    struct Peer
    {
        EspNowProtocol::Mac mac{};
        uint32_t lastSeen = 0;
    };

    struct PendingFrame
    {
        EspNowProtocol::Frame frame;
        uint8_t recipients;
    };

    EspNowProtocol::Uid uid{};
    EspNowProtocol::Mac mac{};
    std::array<Peer, EspNowProtocol::MaxSlaves> peers{};
    EspNowProtocol::Queue<PendingFrame> outgoing;
    EspNowProtocol::Queue<EspNowProtocol::Frame> incoming;
    EspNowProtocol::Frame channels;
    bool channelsValid = false;
    bool awaitingSend = false;
    bool replyReceived = false;
    uint8_t activeSlot = EspNowProtocol::NoSlot;
    uint16_t pendingQueueMask = 0;
    Phase phase = Phase::Startup;
    uint16_t sequence = 0;
    uint32_t session = 0;
    uint32_t lastCycle = 0;
    uint32_t waitStart = 0; // Reference time of the current phase wait.
    uint32_t errorCount = 0;

    bool buildPacket(EspNowProtocol::Packet &packet);
    void beginPacket(EspNowProtocol::Packet &packet);
    void appendQueued(EspNowProtocol::Packet &packet);
    uint8_t peerMask() const;
    void discardDelivered();
    bool acceptJoin(const EspNowProtocol::Mac &source, const uint32_t receivedAt);
    bool acceptReply(const EspNowProtocol::Mac &source, const EspNowProtocol::PacketView &packet, const uint32_t receivedAt);
    void acceptFrames(const Span<const uint8_t> frames, const uint8_t peerSlot);
};
