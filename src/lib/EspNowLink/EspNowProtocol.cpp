#include "EspNowProtocol.h"

#include <cstring>

#include "crc.h"

namespace EspNowProtocol
{
const Mac Broadcast = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
const Mac EmptyMac{};

namespace
{
GENERIC_CRC8 crc(CRSF_CRC_POLY);
}

uint16_t read16(const uint8_t *const data)
{
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
}

uint32_t read32(const uint8_t *const data)
{
    return uint32_t(data[0]) | (uint32_t(data[1]) << 8) | (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
}

uint8_t slaveAddress(const uint8_t index)
{
    return index < MaxSlaves ? SlaveAddressBase + index : 0;
}

bool validFrame(const uint8_t *const data, const size_t size)
{
    if (size < 4 || size > CRSF_MAX_PACKET_LEN || size_t(data[1]) + 2 != size)
        return false;
    if (data[2] >= CRSF_FRAMETYPE_DEVICE_PING && size < 6)
        return false;
    if (data[2] == CRSF_FRAMETYPE_HEARTBEAT && size != 6)
        return false;
    if (data[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && size != 26 && size != 27)
        return false;
    return crc.calc(data + 2, size - 3) == data[size - 1];
}

void updateCrc(Frame &frame)
{
    frame.data[frame.size - 1] = crc.calc(frame.data.data() + 2, frame.size - 3);
}

Frame packChannels(const uint32_t *const channels, const bool armed)
{
    constexpr size_t ChannelBytes = RcChannels * 11 / 8;
    Frame frame;
    frame.size = ChannelBytes + 5;
    frame.data[0] = CRSF_SYNC_BYTE;
    frame.data[1] = frame.size - 2;
    frame.data[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    uint32_t accumulator = 0;
    unsigned bits = 0;
    uint8_t *output = frame.data.data() + 3;
    for (unsigned i = 0; i < RcChannels; ++i)
    {
        accumulator |= (channels[i] & 0x7ff) << bits;
        bits += 11;
        while (bits >= 8)
        {
            *output++ = accumulator;
            accumulator >>= 8;
            bits -= 8;
        }
    }
    *output = armed ? CRSF_CHANNELS_STATUS_ARMED : 0;
    updateCrc(frame);
    return frame;
}

bool unpackChannels(const Frame &frame, uint32_t *const channels)
{
    constexpr unsigned Aux1 = 4;
    uint32_t accumulator = 0;
    unsigned bits = 0;
    const uint8_t *input = frame.data.data() + 3;
    for (unsigned i = 0; i < RcChannels; ++i)
    {
        while (bits < 11)
        {
            accumulator |= uint32_t(*input++) << bits;
            bits += 8;
        }
        channels[i] = accumulator & 0x7ff;
        accumulator >>= 11;
        bits -= 11;
    }
    // Without a status byte, or in CH5 arming mode, AUX1 carries the arm switch.
    if (frame.size == 26 || (*input & CRSF_CHANNELS_STATUS_ARMING_MODE_CH5))
        return CRSF_to_BIT(channels[Aux1]);
    return *input & CRSF_CHANNELS_STATUS_ARMED;
}

void aliasOrigin(Frame &frame, const uint8_t slot)
{
    const uint8_t type = frame.data[2];
    if ((type >= CRSF_FRAMETYPE_DEVICE_PING || type == CRSF_FRAMETYPE_HEARTBEAT) &&
        frame.data[4] == CRSF_ADDRESS_CRSF_RECEIVER)
    {
        frame.data[4] = slaveAddress(slot);
        updateCrc(frame);
    }
}

bool unaliasDestination(Frame &frame, const uint8_t slot)
{
    if (frame.data[2] < CRSF_FRAMETYPE_DEVICE_PING)
        return true;
    if (frame.data[3] == CRSF_ADDRESS_CRSF_RECEIVER)
        return false;
    for (uint8_t i = 0; i < MaxSlaves; ++i)
    {
        if (frame.data[3] == slaveAddress(i))
        {
            if (i != slot)
                return false;
            frame.data[3] = CRSF_ADDRESS_CRSF_RECEIVER;
            updateCrc(frame);
            return true;
        }
    }
    return true;
}

bool validateFrames(const uint8_t *data, size_t size)
{
    while (size)
    {
        if (size < 4)
            return false;
        const size_t frameSize = size_t(data[1]) + 2;
        if (frameSize > size || !validFrame(data, frameSize))
            return false;
        size -= frameSize;
        data += frameSize;
    }
    return true;
}

bool decodePacket(const Mac &source, const Span<const uint8_t> bytes, const Uid &uid, const Mac &ownMac, PacketView &packet)
{
    if (bytes.data() == nullptr || bytes.size() < HeaderSize || bytes.size() > MaxPacketSize ||
        bytes[0] != 'E' || bytes[1] != 'N' || bytes[2] != Version ||
        std::memcmp(bytes.data() + 4, uid.data(), uid.size()) ||
        source == EmptyMac || (source[0] & 1) || source == ownMac)
        return false;

    PacketView decoded;
    decoded.kind = static_cast<Kind>(bytes[3]);
    decoded.session = read32(bytes.data() + 10);
    decoded.sequence = read16(bytes.data() + 14);
    decoded.channelsValid = (bytes[16] & 1) != 0;
    size_t frameOffset = HeaderSize;
    switch (decoded.kind)
    {
    case Kind::Discovery:
        if (bytes.size() != HeaderSize)
            return false;
        break;
    case Kind::Cycle:
        if (bytes.size() < CycleHeaderSize + 26 || bytes[HeaderSize] >= MaxSlaves)
            return false;
        decoded.slot = bytes[HeaderSize];
        frameOffset = CycleHeaderSize;
        break;
    case Kind::Reply:
    case Kind::Join:
        if (bytes.size() < ReplyHeaderSize)
            return false;
        std::memcpy(decoded.masterMac.data(), bytes.data() + HeaderSize, decoded.masterMac.size());
        decoded.slot = bytes[ReplyHeaderSize - 1];
        if ((decoded.kind == Kind::Join && (bytes.size() != ReplyHeaderSize || decoded.slot != NoSlot)) ||
            (decoded.kind == Kind::Reply && decoded.slot >= MaxSlaves))
            return false;
        frameOffset = ReplyHeaderSize;
        break;
    default:
        return false;
    }
    decoded.frames = Span<const uint8_t>{bytes.data() + frameOffset, bytes.size() - frameOffset};
    if (!validateFrames(decoded.frames.data(), decoded.frames.size()) ||
        (decoded.kind == Kind::Cycle && decoded.frames[2] != CRSF_FRAMETYPE_RC_CHANNELS_PACKED))
        return false;

    packet = decoded;
    return true;
}

void writeHeader(Packet &packet, const Kind kind, const Uid &uid, const uint32_t session, const uint16_t sequence, const bool channelsValid)
{
    // EN, version, kind, UID[6], session LE32, sequence LE16, RC-valid.
    packet.data[0] = 'E';
    packet.data[1] = 'N';
    packet.data[2] = Version;
    packet.data[3] = static_cast<uint8_t>(kind);
    std::memcpy(packet.data.data() + 4, uid.data(), uid.size());
    for (uint8_t i = 0; i < 4; ++i)
        packet.data[10 + i] = session >> (i * 8);
    packet.data[14] = sequence;
    packet.data[15] = sequence >> 8;
    packet.data[16] = channelsValid ? 1 : 0;
    packet.size = HeaderSize;
}

void writeMasterHeader(Packet &packet, const Uid &uid, const uint32_t session, const uint16_t sequence, const bool channelsValid, const uint8_t slot)
{
    writeHeader(packet, slot == NoSlot ? Kind::Discovery : Kind::Cycle, uid, session, sequence, channelsValid);
    if (slot != NoSlot)
        packet.data[packet.size++] = slot;
}

void writeSlaveHeader(Packet &packet, const Uid &uid, const uint32_t session, const uint16_t sequence, const bool channelsValid, const Mac &masterMac, const uint8_t slot)
{
    writeHeader(packet, slot == NoSlot ? Kind::Join : Kind::Reply, uid, session, sequence, channelsValid);
    std::memcpy(packet.data.data() + packet.size, masterMac.data(), masterMac.size());
    packet.size += masterMac.size();
    packet.data[packet.size++] = slot;
}
} // namespace EspNowProtocol
