#include "EspNowMasterLink.h"

using namespace EspNowProtocol;

void EspNowMasterLink::reset(const Uid &newUid, const Mac &newMac, const uint32_t newSession, const uint32_t now)
{
    uid = newUid;
    mac = newMac;
    session = newSession;
    peers = {};
    outgoing.clear();
    incoming.clear();
    channels = {};
    channelsValid = awaitingSend = replyReceived = false;
    activeSlot = NoSlot;
    phase = Phase::Startup;
    sequence = 0;
    lastCycle = now - CycleUs;
    waitStart = now;
    pendingQueueMask = 0;
    errorCount = 0;
}

bool EspNowMasterLink::enqueue(const uint8_t *const data, const size_t size)
{
    if (!validFrame(data, size) || data[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        ++errorCount;
        return false;
    }
    Frame frame{Span<const uint8_t>{data, size}};
    const uint8_t recipients = peerMask();
    if (!outgoing.push({frame, recipients ? recipients : UnassignedRecipients}))
    {
        ++errorCount;
        return false;
    }
    return true;
}

void EspNowMasterLink::setChannels(const Frame &frame, const bool valid)
{
    channels = frame;
    channelsValid = valid;
}

void EspNowMasterLink::beginPacket(Packet &packet)
{
    writeMasterHeader(packet, uid, session, sequence, channelsValid, activeSlot);
    pendingQueueMask = 0;
    awaitingSend = true;
}

void EspNowMasterLink::appendQueued(Packet &packet)
{
    // The RC frame plus all appended frames must fit into the slave's receive queue.
    size_t frames = 1;
    for (uint8_t i = 0; i < outgoing.size && frames < QueueSize; ++i)
    {
        auto &entry = outgoing.at(i);
        if (entry.recipients == UnassignedRecipients)
            entry.recipients = peerMask();
        if (!(entry.recipients & (1 << activeSlot)))
            continue;
        if (!packet.append(entry.frame))
            break;
        ++frames;
        pendingQueueMask |= uint16_t(1) << i;
    }
}

uint8_t EspNowMasterLink::peerMask() const
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < MaxSlaves; ++i)
        if (peers[i].mac != EmptyMac)
            mask |= 1 << i;
    return mask;
}

void EspNowMasterLink::discardDelivered()
{
    PendingFrame discarded;
    while (outgoing.size && outgoing.at(0).recipients == 0)
        outgoing.pop(discarded);
}

bool EspNowMasterLink::tick(const uint32_t now, Packet &packet)
{
    expire(now);
    if (awaitingSend)
        return false;
    discardDelivered();
    if (channels.size == 0)
        return false;
    switch (phase)
    {
    case Phase::Startup:
        // Let outstanding replies from the previous session finish before discovery.
        if (uint32_t(now - waitStart) < ReplyTimeoutUs)
            return false;
        phase = Phase::Idle;
        break;
    case Phase::Idle:
        if (uint32_t(now - lastCycle) < CycleUs)
            return false;
        lastCycle = now;
        activeSlot = 0;
        phase = Phase::Polling;
        break;
    case Phase::WaitingForReply: {
        const uint32_t wait = replyReceived ? GuardUs : ReplyTimeoutUs;
        if (uint32_t(now - waitStart) < wait)
            return false;
        ++activeSlot;
        phase = Phase::Polling;
        break;
    }
    case Phase::Discovery:
        if (uint32_t(now - waitStart) < DiscoveryWindowUs)
            return false;
        phase = Phase::Idle;
        break;
    case Phase::Polling:
        while (activeSlot < MaxSlaves && peers[activeSlot].mac == EmptyMac)
            ++activeSlot;
        ++sequence;
        if (activeSlot == MaxSlaves)
        {
            activeSlot = NoSlot;
            phase = Phase::Discovery;
        }
        else
        {
            phase = Phase::WaitingForReply;
            replyReceived = false;
        }
        waitStart = now;
        return buildPacket(packet);
    }
    return false;
}

bool EspNowMasterLink::buildPacket(Packet &packet)
{
    packet.destination = activeSlot == NoSlot ? Broadcast : peers[activeSlot].mac;
    beginPacket(packet);
    if (activeSlot == NoSlot)
        return true;
    if (!packet.append(channels))
    {
        awaitingSend = false;
        ++errorCount;
        return false;
    }
    appendQueued(packet);
    return true;
}

void EspNowMasterLink::sent(const uint32_t now, const bool success)
{
    if (!awaitingSend)
        return;
    awaitingSend = false;
    waitStart = now;
    if (success)
    {
        if (activeSlot != NoSlot)
        {
            for (uint8_t i = 0; i < outgoing.size; ++i)
                if (pendingQueueMask & (uint16_t(1) << i))
                    outgoing.at(i).recipients &= ~(1 << activeSlot);
        }
    }
    else
    {
        ++errorCount;
    }
    pendingQueueMask = 0;
}

void EspNowMasterLink::acceptFrames(const Span<const uint8_t> frames, const uint8_t peerSlot)
{
    forEachFrame(frames, [this, peerSlot](Frame &frame) {
        if (frame.data[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            ++errorCount;
            return;
        }
        aliasOrigin(frame, peerSlot);
        if (!incoming.push(frame))
            ++errorCount;
    });
}

bool EspNowMasterLink::receive(const Mac &source, const uint8_t *const data, const size_t size, const uint32_t receivedAt)
{
    PacketView packet;
    if (!decodePacket(source, Span<const uint8_t>{data, size}, uid, mac, packet))
    {
        ++errorCount;
        return false;
    }
    if (packet.session != session || packet.sequence != sequence || packet.masterMac != mac)
        return false;
    if (packet.kind == Kind::Join)
        return acceptJoin(source, receivedAt);
    return acceptReply(source, packet, receivedAt);
}

bool EspNowMasterLink::acceptJoin(const Mac &source, const uint32_t receivedAt)
{
    if (phase != Phase::Discovery || uint32_t(receivedAt - waitStart) >= DiscoveryWindowUs)
        return false;
    for (const Peer &peer : peers)
        if (peer.mac == source)
            return true;
    for (Peer &peer : peers)
    {
        if (peer.mac == EmptyMac)
        {
            peer.mac = source;
            peer.lastSeen = receivedAt;
            return true;
        }
    }
    ++errorCount;
    return false;
}

bool EspNowMasterLink::acceptReply(const Mac &source, const PacketView &packet, const uint32_t receivedAt)
{
    if (packet.kind != Kind::Reply || phase != Phase::WaitingForReply || packet.slot != activeSlot ||
        peers[packet.slot].mac != source || (!awaitingSend && uint32_t(receivedAt - waitStart) >= ReplyTimeoutUs))
    {
        ++errorCount;
        return false;
    }
    if (replyReceived)
        return false;
    replyReceived = true;
    waitStart = receivedAt;
    peers[packet.slot].lastSeen = receivedAt;
    acceptFrames(packet.frames, packet.slot);
    return true;
}

void EspNowMasterLink::expire(const uint32_t now)
{
    for (uint8_t i = 0; i < MaxSlaves; ++i)
    {
        if (peers[i].mac != EmptyMac && uint32_t(now - peers[i].lastSeen) >= PeerTimeoutUs)
        {
            peers[i] = {};
            for (uint8_t j = 0; j < outgoing.size; ++j)
                if (outgoing.at(j).recipients != UnassignedRecipients)
                    outgoing.at(j).recipients &= ~(1 << i);
        }
    }
}
