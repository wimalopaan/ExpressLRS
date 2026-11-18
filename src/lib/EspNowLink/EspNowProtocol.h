#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "Span.h"
#include "crsf_protocol.h"

namespace EspNowProtocol
{
// Delivery semantics: CRSF frames are delivered at least once. A frame is
// retransmitted when the sender sees a failed MAC-level send, even if the
// peer already received it (lost ACK). Receivers do not deduplicate, so
// non-idempotent CRSF commands may be executed twice. RC data is not
// affected: only the latest channels are sent each cycle.
constexpr uint8_t Version = 4;
constexpr uint8_t MaxSlaves = 4;
// Slaves use the regular receiver address locally and on air; the master maps
// it to SlaveAddressBase + slot so that several slaves can be addressed.
constexpr uint8_t SlaveAddressBase = 0xD0;
constexpr unsigned RcChannels = 16;
constexpr uint8_t NoSlot = 0xff;
constexpr size_t MaxPacketSize = 136;
constexpr uint32_t CycleUs = 10000;
constexpr uint32_t ReplyStartUs = 200;
constexpr uint32_t SendWindowUs = 600;
constexpr uint32_t ReplyTimeoutUs = 20000;
constexpr uint32_t GuardUs = 200;
constexpr uint32_t DiscoveryWindowUs = 1600;
constexpr uint32_t PeerTimeoutUs = 500000;
constexpr uint32_t RcTimeoutUs = 100000;
constexpr size_t HeaderSize = 17;
constexpr size_t CycleHeaderSize = HeaderSize + 1;
constexpr size_t ReplyHeaderSize = HeaderSize + 6 + 1;
constexpr size_t MaxReplyPayloadSize = CRSF_MAX_PACKET_LEN;
constexpr size_t QueueSize = 16;

using Mac = std::array<uint8_t, 6>;
using Uid = std::array<uint8_t, 6>;

extern const Mac Broadcast;
extern const Mac EmptyMac;

enum class Kind : uint8_t
{
    Cycle = 1,
    Reply = 2,
    Join = 3,
    Discovery = 4
};

// The frame payload refers to the received buffer and must not outlive it.
struct PacketView
{
    Kind kind = Kind::Discovery;
    uint32_t session = 0;
    uint16_t sequence = 0;
    bool channelsValid = false;
    uint8_t slot = NoSlot;
    Mac masterMac{};
    Span<const uint8_t> frames{};
};

struct Frame
{
    Frame() = default;

    explicit Frame(const Span<const uint8_t> bytes)
    {
        if (bytes.data() != nullptr && bytes.size() <= data.size())
        {
            std::memcpy(data.data(), bytes.data(), bytes.size());
            size = static_cast<uint8_t>(bytes.size());
        }
    }

    std::array<uint8_t, CRSF_MAX_PACKET_LEN> data{};
    uint8_t size = 0;
};

struct Packet
{
    bool append(const Frame &frame)
    {
        if (frame.size == 0 || frame.size > frame.data.size() ||
            size_t(size) + frame.size > data.size())
            return false;

        std::memcpy(data.data() + size, frame.data.data(), frame.size);
        size += frame.size;
        return true;
    }

    Mac destination{};
    std::array<uint8_t, MaxPacketSize> data{};
    uint8_t size = 0;
};

template <typename Entry>
struct Queue
{
    std::array<Entry, QueueSize> entries{};
    uint8_t head = 0;
    uint8_t size = 0;

    bool push(const Entry &entry)
    {
        if (size == QueueSize)
            return false;
        entries[(head + size++) % QueueSize] = entry;
        return true;
    }

    bool pop(Entry &entry)
    {
        if (!size)
            return false;
        entry = entries[head];
        head = (head + 1) % QueueSize;
        --size;
        return true;
    }

    Entry &at(const uint8_t index)
    {
        return entries[(head + index) % QueueSize];
    }
    const Entry &at(const uint8_t index) const
    {
        return entries[(head + index) % QueueSize];
    }
    void clear()
    {
        head = size = 0;
    }
};

uint16_t read16(const uint8_t *const data);
uint32_t read32(const uint8_t *const data);
uint8_t slaveAddress(const uint8_t slot);
bool validFrame(const uint8_t *const data, const size_t size);
void updateCrc(Frame &frame);
bool validateFrames(const uint8_t *data, size_t size);
// Packs RcChannels 11-bit channels into an RC_CHANNELS_PACKED frame with status byte.
Frame packChannels(const uint32_t *const channels, const bool armed);
// Unpacks a valid RC_CHANNELS_PACKED frame and returns the armed state.
bool unpackChannels(const Frame &frame, uint32_t *const channels);
// Maps the receiver origin of a slave frame to the alias of the given slot.
void aliasOrigin(Frame &frame, const uint8_t slot);
// Maps the alias of the given slot to the receiver address; false if addressed to another device's receiver.
bool unaliasDestination(Frame &frame, const uint8_t slot);
bool decodePacket(const Mac &source, const Span<const uint8_t> bytes, const Uid &uid, const Mac &ownMac, PacketView &packet);
void writeHeader(Packet &packet, const Kind kind, const Uid &uid, const uint32_t session, const uint16_t sequence, const bool channelsValid);
void writeMasterHeader(Packet &packet, const Uid &uid, const uint32_t session, const uint16_t sequence, const bool channelsValid, const uint8_t slot);
void writeSlaveHeader(Packet &packet, const Uid &uid, const uint32_t session, const uint16_t sequence, const bool channelsValid, const Mac &masterMac, const uint8_t slot);

// Calls handler(Frame &) for each frame of a payload already checked by validateFrames().
template <typename Handler>
void forEachFrame(const Span<const uint8_t> frames, Handler handler)
{
    const uint8_t *data = frames.data();
    size_t size = frames.size();
    while (size >= 2)
    {
        Frame frame{Span<const uint8_t>{data, size_t(data[1]) + 2}};
        if (frame.size == 0 || frame.size > size)
            return;
        data += frame.size;
        size -= frame.size;
        handler(frame);
    }
}
} // namespace EspNowProtocol
