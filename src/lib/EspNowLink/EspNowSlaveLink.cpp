#include "EspNowSlaveLink.h"

using namespace EspNowProtocol;

void EspNowSlaveLink::reset(const Uid &newUid, const Mac &newMac, const uint32_t newSession, const uint32_t now)
{
    uid = newUid;
    mac = newMac;
    session = newSession;
    master = EmptyMac;
    clearFrames();
    channelsValid = masterSeen = responseSent = awaitingSend = false;
    slot = NoSlot;
    sequence = 0;
    lastMaster = lastCycle = now;
    errorCount = 0;
    seedDiscoveryRandom(newSession);
}

void EspNowSlaveLink::clearFrames()
{
    outgoing.clear();
    incoming.clear();
    pendingFrames = 0;
}

bool EspNowSlaveLink::enqueue(const uint8_t *const data, const size_t size)
{
    if (!validFrame(data, size))
    {
        ++errorCount;
        return false;
    }
    Frame frame{Span<const uint8_t>{data, size}};
    if (!outgoing.push(frame))
    {
        ++errorCount;
        return false;
    }
    return true;
}

void EspNowSlaveLink::beginPacket(Packet &packet)
{
    writeSlaveHeader(packet, uid, session, sequence, channelsValid, master, slot);
    pendingFrames = 0;
    awaitingSend = true;
}

void EspNowSlaveLink::appendQueued(Packet &packet)
{
    for (uint8_t i = 0; i < outgoing.size && pendingFrames < QueueSize; ++i)
    {
        const Frame &frame = outgoing.at(i);
        if (packet.size + frame.size > ReplyHeaderSize + MaxReplyPayloadSize ||
            !packet.append(frame))
            break;
        ++pendingFrames;
    }
}

bool EspNowSlaveLink::tick(const uint32_t now, Packet &packet)
{
    expire(now);
    if (awaitingSend)
        return false;
    if (!hasMaster(now) || responseSent)
        return false;
    const uint32_t elapsed = now - lastMaster;
    const uint32_t start = ReplyStartUs + (slot == NoSlot ? joinDelay : 0);
    // Skip late starts; MAC retries already in flight are allowed to finish.
    if (elapsed < start || elapsed - start > SendWindowUs)
        return false;
    if (slot == NoSlot && !joinThisCycle)
        return false;
    responseSent = true;
    packet.destination = slot == NoSlot ? Broadcast : master;
    beginPacket(packet);
    if (slot != NoSlot)
        appendQueued(packet);
    return true;
}

void EspNowSlaveLink::sent(const uint32_t, const bool success)
{
    if (!awaitingSend)
        return;
    awaitingSend = false;
    if (success)
    {
        Frame discarded;
        while (pendingFrames > 0)
        {
            outgoing.pop(discarded);
            --pendingFrames;
        }
    }
    else
    {
        ++errorCount;
    }
    pendingFrames = 0;
}

void EspNowSlaveLink::acceptFrames(const Span<const uint8_t> frames)
{
    forEachFrame(frames, [this](Frame &frame) {
        if (frame.data[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && !channelsValid)
            return;
        if (!unaliasDestination(frame, slot))
            return;
        if (!incoming.push(frame))
            ++errorCount;
    });
}

bool EspNowSlaveLink::acceptMaster(const Mac &source, const PacketView &packet, const uint32_t receivedAt)
{
    expire(receivedAt);
    if (hasMaster(receivedAt) && source != master)
        return false;
    if (masterSeen && source == master && packet.session == session &&
        int16_t(packet.sequence - sequence) <= 0)
        return false;
    const bool sameMaster = masterSeen && source == master && session == packet.session;
    const uint8_t newSlot = sameMaster && packet.kind == Kind::Discovery ? slot : packet.slot;
    if (!sameMaster || slot != newSlot)
        clearFrames();
    master = source;
    masterSeen = true;
    session = packet.session;
    sequence = packet.sequence;
    lastMaster = receivedAt;
    slot = newSlot;
    return true;
}

bool EspNowSlaveLink::receive(const Mac &source, const uint8_t *const data, const size_t size, const uint32_t receivedAt)
{
    PacketView packet;
    if (!decodePacket(source, Span<const uint8_t>{data, size}, uid, mac, packet))
    {
        ++errorCount;
        return false;
    }
    if (packet.kind != Kind::Cycle && packet.kind != Kind::Discovery)
        return false;
    if (!acceptMaster(source, packet, receivedAt))
        return false;

    responseSent = packet.kind == Kind::Discovery && slot != NoSlot;
    const uint32_t random = nextDiscoveryRandom();
    joinDelay = random % 201;                  // Add 0..200 us to ReplyStartUs.
    joinThisCycle = ((random >> 16) & 3) == 0; // Join on approximately one in four discoveries.
    if (packet.kind == Kind::Cycle)
    {
        lastCycle = receivedAt;
        channelsValid = packet.channelsValid;
        acceptFrames(packet.frames);
    }
    return true;
}

void EspNowSlaveLink::seedDiscoveryRandom(const uint32_t seed)
{
    discoveryRandom = seed;
    for (uint8_t value : mac)
        discoveryRandom = discoveryRandom * 33 + value;
    if (!discoveryRandom)
        discoveryRandom = 1;
}

uint32_t EspNowSlaveLink::nextDiscoveryRandom()
{
    // Xorshift32; each slave instance maintains its own generator state.
    discoveryRandom ^= discoveryRandom << 13;
    discoveryRandom ^= discoveryRandom >> 17;
    discoveryRandom ^= discoveryRandom << 5;
    return discoveryRandom;
}

void EspNowSlaveLink::expire(const uint32_t now)
{
    const bool masterExpired = masterSeen && !hasMaster(now);
    const bool registrationExpired = slot != NoSlot && uint32_t(now - lastCycle) >= RcTimeoutUs;
    if (masterExpired || registrationExpired)
    {
        if (masterExpired)
            masterSeen = false;
        responseSent = true;
        slot = NoSlot;
        clearFrames();
    }
}

bool EspNowSlaveLink::hasMaster(const uint32_t now) const
{
    return masterSeen && uint32_t(now - lastMaster) < RcTimeoutUs;
}

bool EspNowSlaveLink::connected(const uint32_t now) const
{
    return hasMaster(now) && slot != NoSlot && channelsValid && uint32_t(now - lastCycle) < RcTimeoutUs;
}
