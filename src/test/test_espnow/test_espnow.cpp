#include "EspNowEvents.h"
#include "EspNowMasterLink.h"
#include "EspNowSlaveLink.h"
#include "RxRadioMode.h"
#include "unity.h"

#include <algorithm>
#include <cstring>
#include <vector>

namespace
{
const EspNowProtocol::Uid Uid = {{1, 2, 3, 4, 5, 6}};
const EspNowProtocol::Mac MasterMac = {{2, 0, 0, 0, 0, 10}};
const EspNowProtocol::Mac Broadcast = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
const EspNowProtocol::Mac SlaveMacs[5] = {
    {{2, 0, 0, 0, 0, 1}}, {{2, 0, 0, 0, 0, 2}}, {{2, 0, 0, 0, 0, 3}}, {{2, 0, 0, 0, 0, 4}}, {{2, 0, 0, 0, 0, 5}}};

EspNowMasterLink master;
EspNowSlaveLink slaves[5];

struct Transmission
{
    EspNowProtocol::Packet packet;
    uint32_t started = 0;
    uint32_t duration = 0;
    bool active = false;
    bool failed = false;
};

Transmission air[6];
std::vector<EspNowProtocol::Frame> received[6];
EspNowProtocol::Packet lastPoll[5];
EspNowProtocol::Packet lastReply[5];
uint32_t pollCount[5], lastPollTime[5], maxPollGap[5];
uint32_t timeUs, masterDuration, replyDuration;
unsigned enabledSlaves, discoveryCount;
int silentSlave, failingDownlink;

EspNowProtocol::Frame rcFrame()
{
    EspNowProtocol::Frame frame;
    frame.size = 27;
    frame.data[0] = CRSF_SYNC_BYTE;
    frame.data[1] = 25;
    frame.data[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    for (unsigned i = 3; i < 25; ++i)
        frame.data[i] = i * 7;
    frame.data[25] = CRSF_CHANNELS_STATUS_ARMED;
    EspNowProtocol::updateCrc(frame);
    return frame;
}

EspNowProtocol::Frame extended(const uint8_t type, const uint8_t destination, const uint8_t origin, const uint8_t size = 8)
{
    EspNowProtocol::Frame frame;
    frame.size = size;
    frame.data[0] = CRSF_SYNC_BYTE;
    frame.data[1] = size - 2;
    frame.data[2] = type;
    frame.data[3] = destination;
    frame.data[4] = origin;
    EspNowProtocol::updateCrc(frame);
    return frame;
}

template <typename Link>
void enqueue(Link &link, const EspNowProtocol::Frame &frame)
{
    TEST_ASSERT_TRUE(link.enqueue(frame.data.data(), frame.size));
}

void step()
{
    for (unsigned i = 0; i <= enabledSlaves; ++i)
    {
        auto &tx = air[i];
        if (!tx.active || uint32_t(timeUs - tx.started) < tx.duration)
            continue;
        tx.active = false;
        if (i)
            slaves[i - 1].sent(timeUs, !tx.failed);
        else
            master.sent(timeUs, !tx.failed);
        if (tx.failed)
            continue;
        if (i == 0)
        {
            if (tx.packet.destination == Broadcast)
            {
                TEST_ASSERT_EQUAL_UINT8(4, tx.packet.data[3]);
                TEST_ASSERT_EQUAL_UINT(EspNowProtocol::HeaderSize, tx.packet.size);
                ++discoveryCount;
                for (unsigned j = 0; j < enabledSlaves; ++j)
                    if (int(j) != silentSlave)
                        slaves[j].receive(MasterMac, tx.packet.data.data(), tx.packet.size, timeUs);
            }
            else
            {
                bool found = false;
                for (unsigned j = 0; j < enabledSlaves; ++j)
                {
                    if (tx.packet.destination != SlaveMacs[j])
                        continue;
                    found = true;
                    if (int(j) != silentSlave)
                        TEST_ASSERT_TRUE(slaves[j].receive(MasterMac, tx.packet.data.data(), tx.packet.size, timeUs));
                }
                TEST_ASSERT_TRUE(found);
            }
        }
        else
        {
            if (tx.packet.data[3] == 3)
                TEST_ASSERT_EQUAL_UINT8_ARRAY(Broadcast.data(), tx.packet.destination.data(), 6);
            else
            {
                TEST_ASSERT_EQUAL_UINT8_ARRAY(MasterMac.data(), tx.packet.destination.data(), 6);
                lastReply[i - 1] = tx.packet;
            }
            master.receive(SlaveMacs[i - 1], tx.packet.data.data(), tx.packet.size, timeUs);
        }
    }

    for (unsigned i = 0; i <= enabledSlaves; ++i)
    {
        if (i && int(i - 1) == silentSlave)
            continue;
        EspNowProtocol::Packet packet;
        if (!(i ? slaves[i - 1].tick(timeUs, packet) : master.tick(timeUs, packet)))
            continue;
        TEST_ASSERT_FALSE(air[i].active);
        TEST_ASSERT_LESS_OR_EQUAL_UINT(EspNowProtocol::MaxPacketSize, packet.size);
        if (packet.destination != Broadcast)
            for (const auto &other : air)
                TEST_ASSERT_FALSE(other.active);
        auto &tx = air[i];
        tx.packet = packet;
        tx.started = timeUs;
        tx.duration = i ? replyDuration : masterDuration;
        tx.failed = false;
        tx.active = true;
        if (!i && packet.destination != Broadcast)
        {
            for (unsigned j = 0; j < enabledSlaves; ++j)
            {
                if (packet.destination != SlaveMacs[j])
                    continue;
                if (pollCount[j])
                    maxPollGap[j] = std::max(maxPollGap[j], uint32_t(timeUs - lastPollTime[j]));
                ++pollCount[j];
                lastPollTime[j] = timeUs;
                lastPoll[j] = packet;
                tx.failed = int(j) == failingDownlink;
            }
        }
    }
    for (unsigned i = 0; i < 6; ++i)
    {
        EspNowProtocol::Frame frame;
        while (i ? slaves[i - 1].popFrame(frame) : master.popFrame(frame))
            received[i].push_back(frame);
    }
    timeUs += 25;
}

void run(const uint32_t duration)
{
    const uint32_t start = timeUs;
    while (uint32_t(timeUs - start) < duration)
        step();
}

void clearReceived()
{
    for (auto &frames : received)
        frames.clear();
}

void join(const unsigned count = 4)
{
    enabledSlaves = count;
    run(400000);
    bool busy;
    do
    {
        step();
        busy = false;
        for (const auto &tx : air)
            busy |= tx.active;
    } while (busy);
    clearReceived();
}

unsigned countFrames(const unsigned index, const uint8_t type)
{
    unsigned count = 0;
    for (const auto &frame : received[index])
        count += frame.data[2] == type;
    return count;
}
} // namespace

void setUp()
{
    timeUs = 0;
    masterDuration = replyDuration = 250;
    enabledSlaves = 4;
    discoveryCount = 0;
    silentSlave = failingDownlink = -1;
    for (auto &tx : air)
        tx = {};
    for (unsigned i = 0; i < 5; ++i)
    {
        pollCount[i] = lastPollTime[i] = maxPollGap[i] = 0;
        lastPoll[i] = lastReply[i] = {};
        slaves[i].reset(Uid, SlaveMacs[i], 999, timeUs);
    }
    master.reset(Uid, MasterMac, 123, timeUs);
    master.setChannels(rcFrame(), true);
    clearReceived();
}

void tearDown()
{
}

void test_discovery_and_registered_traffic_use_distinct_destinations()
{
    join();
    bool slots[4]{};
    for (unsigned i = 0; i < 4; ++i)
    {
        const uint8_t slot = slaves[i].assignedSlot();
        TEST_ASSERT_LESS_THAN_UINT8(4, slot);
        TEST_ASSERT_FALSE(slots[slot]);
        slots[slot] = true;
        TEST_ASSERT_TRUE(pollCount[i] > 0);
        TEST_ASSERT_EQUAL_UINT32(EspNowProtocol::CycleUs, maxPollGap[i]);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(SlaveMacs[i].data(), master.peerMac(slot).data(), 6);
        TEST_ASSERT_TRUE(slaves[i].connected(timeUs));
        const auto channels = rcFrame();
        TEST_ASSERT_EQUAL_UINT(EspNowProtocol::CycleHeaderSize + channels.size, lastPoll[i].size);
        TEST_ASSERT_EQUAL_UINT8(slot, lastPoll[i].data[EspNowProtocol::HeaderSize]);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(channels.data.data(), lastPoll[i].data.data() + EspNowProtocol::CycleHeaderSize, channels.size);
    }
    TEST_ASSERT_TRUE(discoveryCount > 0);
    TEST_ASSERT_EQUAL_UINT32(0, master.errors());
}

void test_slow_downlink_retries_extend_cycle_without_overlapping_transmissions()
{
    join();
    masterDuration = 12000;
    run(240000);
    for (unsigned i = 0; i < 4; ++i)
    {
        TEST_ASSERT_TRUE(maxPollGap[i] > EspNowProtocol::CycleUs);
        TEST_ASSERT_TRUE(slaves[i].connected(timeUs));
    }
}

void test_slow_reply_retries_extend_cycle_and_are_accepted()
{
    join();
    replyDuration = 12000;
    run(240000);
    for (unsigned i = 0; i < 4; ++i)
    {
        TEST_ASSERT_TRUE(maxPollGap[i] > EspNowProtocol::CycleUs);
        TEST_ASSERT_TRUE(slaves[i].connected(timeUs));
    }
    TEST_ASSERT_EQUAL_UINT32(0, master.errors());
}

void test_early_reply_does_not_bypass_pending_send_completion()
{
    join(1);
    for (unsigned n = 0; n < 1000 && !(air[0].active && air[0].packet.destination != Broadcast); ++n)
        step();
    TEST_ASSERT_TRUE(air[0].active);
    const uint32_t started = air[0].started;
    auto reply = lastReply[0];
    std::memcpy(reply.data.data() + 10, air[0].packet.data.data() + 10, 6);
    TEST_ASSERT_TRUE(master.receive(SlaveMacs[0], reply.data.data(), reply.size, started + 30000));
    EspNowProtocol::Packet packet;
    TEST_ASSERT_FALSE(master.tick(started + 35000, packet));
    master.sent(started + 40000, true);
    TEST_ASSERT_FALSE(master.tick(started + 40000 + EspNowProtocol::GuardUs - 1, packet));
    TEST_ASSERT_FALSE(master.tick(started + 40000 + EspNowProtocol::GuardUs, packet));
    TEST_ASSERT_TRUE(master.tick(started + 40000 + EspNowProtocol::GuardUs + 25, packet));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(Broadcast.data(), packet.destination.data(), 6);
}

void test_late_join_silent_peer_removal_and_rejoin()
{
    join(1);
    TEST_ASSERT_EQUAL_UINT8(EspNowProtocol::NoSlot, slaves[1].assignedSlot());
    join(2);
    const uint8_t oldSlot = slaves[0].assignedSlot();
    silentSlave = 0;
    run(700000);
    const EspNowProtocol::Mac empty{};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(empty.data(), master.peerMac(oldSlot).data(), 6);
    TEST_ASSERT_TRUE(slaves[1].connected(timeUs));
    silentSlave = -1;
    run(400000);
    TEST_ASSERT_TRUE(slaves[0].connected(timeUs));
}

void test_uid_source_and_duplicate_filtering()
{
    auto otherUid = Uid;
    ++otherUid[0];
    slaves[0].reset(otherUid, SlaveMacs[0], 1, timeUs);
    enabledSlaves = 1;
    run(40000);
    TEST_ASSERT_FALSE(slaves[0].hasMaster(timeUs));
    slaves[0].reset(Uid, SlaveMacs[0], 1, timeUs);
    join(1);
    const auto &packet = lastPoll[0];
    TEST_ASSERT_FALSE(slaves[0].receive(SlaveMacs[1], packet.data.data(), packet.size, timeUs));
    TEST_ASSERT_FALSE(slaves[0].receive(MasterMac, packet.data.data(), packet.size, timeUs));
    const auto &reply = lastReply[0];
    TEST_ASSERT_FALSE(master.receive(SlaveMacs[1], reply.data.data(), reply.size, timeUs));
    TEST_ASSERT_FALSE(master.receive(SlaveMacs[0], reply.data.data(), reply.size, timeUs));
}

void test_addressed_parameters_and_aliased_replies_for_all_four_slaves()
{
    join();
    for (unsigned selected = 0; selected < 4; ++selected)
    {
        clearReceived();
        const uint8_t address = EspNowProtocol::slaveAddress(slaves[selected].assignedSlot());
        enqueue(master, extended(CRSF_FRAMETYPE_PARAMETER_READ, address, CRSF_ADDRESS_RADIO_TRANSMITTER));
        enqueue(slaves[selected], extended(CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_CRSF_RECEIVER));
        run(30000);
        for (unsigned i = 0; i < 4; ++i)
        {
            TEST_ASSERT_EQUAL_UINT(i == selected ? 1 : 0, countFrames(i + 1, CRSF_FRAMETYPE_PARAMETER_READ));
            for (const auto &frame : received[i + 1])
                if (frame.data[2] == CRSF_FRAMETYPE_PARAMETER_READ)
                    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_CRSF_RECEIVER, frame.data[3]);
        }
        TEST_ASSERT_EQUAL_UINT(1, countFrames(0, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY));
        TEST_ASSERT_EQUAL_UINT8(address, received[0].back().data[4]);
    }
}

void test_rc_and_maximum_crsf_frame_reach_every_peer_once()
{
    join();
    const auto frame = extended(CRSF_FRAMETYPE_DEVICE_INFO, 0, 0xea, CRSF_MAX_PACKET_LEN);
    enqueue(master, frame);
    run(30000);
    for (unsigned i = 1; i <= 4; ++i)
    {
        TEST_ASSERT_TRUE(countFrames(i, CRSF_FRAMETYPE_RC_CHANNELS_PACKED) > 0);
        TEST_ASSERT_EQUAL_UINT(1, countFrames(i, CRSF_FRAMETYPE_DEVICE_INFO));
        for (const auto &actual : received[i])
            if (actual.data[2] == CRSF_FRAMETYPE_DEVICE_INFO)
                TEST_ASSERT_EQUAL_UINT8_ARRAY(frame.data.data(), actual.data.data(), frame.size);
    }
}

void test_failed_peer_retains_its_messages_without_duplicates_for_other_peers()
{
    join();
    failingDownlink = 0;
    enqueue(master, extended(CRSF_FRAMETYPE_DEVICE_PING, 0, 0xea));
    run(100000);
    TEST_ASSERT_EQUAL_UINT(0, countFrames(1, CRSF_FRAMETYPE_DEVICE_PING));
    for (unsigned i = 2; i <= 4; ++i)
        TEST_ASSERT_EQUAL_UINT(1, countFrames(i, CRSF_FRAMETYPE_DEVICE_PING));
    failingDownlink = -1;
    run(100000);
    for (unsigned i = 1; i <= 4; ++i)
        TEST_ASSERT_EQUAL_UINT(1, countFrames(i, CRSF_FRAMETYPE_DEVICE_PING));
    TEST_ASSERT_TRUE(master.errors() > 0);
}

void test_nonextended_telemetry_and_rc_failsafe()
{
    join(1);
    EspNowProtocol::Frame telemetry;
    telemetry.size = 14;
    telemetry.data[0] = CRSF_SYNC_BYTE;
    telemetry.data[1] = 12;
    telemetry.data[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
    EspNowProtocol::updateCrc(telemetry);
    enqueue(master, telemetry);
    master.setChannels(rcFrame(), false);
    auto routed = rcFrame();
    routed.data[3] = 42;
    EspNowProtocol::updateCrc(routed);
    TEST_ASSERT_FALSE(master.enqueue(routed.data.data(), routed.size));
    run(EspNowProtocol::RcTimeoutUs + 30000);
    TEST_ASSERT_FALSE(slaves[0].connected(timeUs));
    TEST_ASSERT_EQUAL_UINT(0, countFrames(1, CRSF_FRAMETYPE_RC_CHANNELS_PACKED));
    TEST_ASSERT_EQUAL_UINT(1, countFrames(1, CRSF_FRAMETYPE_LINK_STATISTICS));
    const uint8_t slot = slaves[0].assignedSlot();
    TEST_ASSERT_LESS_THAN_UINT8(EspNowProtocol::MaxSlaves, slot);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(SlaveMacs[0].data(), master.peerMac(slot).data(), 6);
    master.setChannels(rcFrame(), true);
    run(30000);
    TEST_ASSERT_TRUE(slaves[0].connected(timeUs));
    TEST_ASSERT_FALSE(slaves[0].connected(timeUs + EspNowProtocol::RcTimeoutUs));
}

void test_master_drops_rc_frames_from_slave_replies()
{
    join(1);
    EspNowProtocol::Frame telemetry;
    telemetry.size = 14;
    telemetry.data[0] = CRSF_SYNC_BYTE;
    telemetry.data[1] = 12;
    telemetry.data[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
    EspNowProtocol::updateCrc(telemetry);
    enqueue(slaves[0], rcFrame());
    enqueue(slaves[0], telemetry);
    run(30000);
    TEST_ASSERT_EQUAL_UINT(0, countFrames(0, CRSF_FRAMETYPE_RC_CHANNELS_PACKED));
    TEST_ASSERT_EQUAL_UINT(1, countFrames(0, CRSF_FRAMETYPE_LINK_STATISTICS));
    TEST_ASSERT_EQUAL_UINT32(1, master.errors());
}

void test_removed_peer_releases_pending_queue_recipients()
{
    join(2);
    silentSlave = failingDownlink = 0;
    const auto ping = extended(CRSF_FRAMETYPE_DEVICE_PING, 0, 0xea);
    enqueue(master, ping);
    run(700000);
    TEST_ASSERT_EQUAL_UINT(1, countFrames(2, CRSF_FRAMETYPE_DEVICE_PING));
    enqueue(master, ping);
    run(30000);
    TEST_ASSERT_EQUAL_UINT(2, countFrames(2, CRSF_FRAMETYPE_DEVICE_PING));
    TEST_ASSERT_TRUE(slaves[1].connected(timeUs));
}

void test_malformed_poll_does_not_refresh_heartbeat()
{
    join(1);
    auto packet = lastPoll[0];
    packet.data[packet.size - 1] ^= 1;
    const uint32_t late = timeUs + EspNowProtocol::RcTimeoutUs;
    TEST_ASSERT_FALSE(slaves[0].receive(MasterMac, packet.data.data(), packet.size, late));
    TEST_ASSERT_FALSE(slaves[0].receive(MasterMac, packet.data.data(), 8, late));
    TEST_ASSERT_FALSE(slaves[0].receive(MasterMac, packet.data.data(), packet.size - 1, late));
    TEST_ASSERT_FALSE(slaves[0].connected(late));
}

void test_cycle_rejects_invalid_slots_and_truncated_headers()
{
    join(1);
    auto packet = lastPoll[0];
    EspNowSlaveLink receiver;
    receiver.reset(Uid, SlaveMacs[0], 999, timeUs);
    const uint8_t slot = packet.data[EspNowProtocol::HeaderSize];
    packet.data[EspNowProtocol::HeaderSize] = EspNowProtocol::MaxSlaves;
    TEST_ASSERT_FALSE(receiver.receive(MasterMac, packet.data.data(), packet.size, timeUs));
    packet.data[EspNowProtocol::HeaderSize] = EspNowProtocol::NoSlot;
    TEST_ASSERT_FALSE(receiver.receive(MasterMac, packet.data.data(), packet.size, timeUs));
    packet.data[EspNowProtocol::HeaderSize] = slot;
    TEST_ASSERT_FALSE(receiver.receive(MasterMac, packet.data.data(), EspNowProtocol::HeaderSize, timeUs));
    TEST_ASSERT_FALSE(receiver.receive(MasterMac, packet.data.data(), EspNowProtocol::CycleHeaderSize, timeUs));
    TEST_ASSERT_FALSE(receiver.hasMaster(timeUs));
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, packet.data.data(), packet.size, timeUs));
    TEST_ASSERT_EQUAL_UINT8(slot, receiver.assignedSlot());
    TEST_ASSERT_TRUE(receiver.connected(timeUs));
}

void test_slave_rejects_previous_wire_format()
{
    join(1);
    EspNowProtocol::Packet legacy;
    EspNowProtocol::writeHeader(legacy, EspNowProtocol::Kind::Cycle, Uid, 123, 1, true);
    legacy.data[2] = 2;
    for (uint8_t i = 0; i < EspNowProtocol::MaxSlaves; ++i)
    {
        const auto &mac = master.peerMac(i);
        std::memcpy(legacy.data.data() + legacy.size, mac.data(), mac.size());
        legacy.size += mac.size();
    }
    TEST_ASSERT_TRUE(legacy.append(rcFrame()));
    EspNowSlaveLink receiver;
    receiver.reset(Uid, SlaveMacs[0], 999, timeUs);
    TEST_ASSERT_FALSE(receiver.receive(MasterMac, legacy.data.data(), legacy.size, timeUs));
    TEST_ASSERT_FALSE(receiver.hasMaster(timeUs));
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, lastPoll[0].data.data(), lastPoll[0].size, timeUs));
    TEST_ASSERT_TRUE(receiver.connected(timeUs));
}

void test_discovery_preserves_registration_and_pending_telemetry()
{
    join(1);
    auto cycle = lastPoll[0];
    EspNowSlaveLink receiver;
    receiver.reset(Uid, SlaveMacs[0], 999, timeUs);
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, cycle.data.data(), cycle.size, timeUs));
    const uint8_t slot = receiver.assignedSlot();
    const auto telemetry = extended(CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_CRSF_RECEIVER);
    enqueue(receiver, telemetry);

    uint16_t sequence = EspNowProtocol::read16(cycle.data.data() + 14) + 1;
    EspNowProtocol::Packet discovery;
    EspNowProtocol::writeHeader(discovery, EspNowProtocol::Kind::Discovery, Uid,
                                EspNowProtocol::read32(cycle.data.data() + 10), sequence, true);
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, discovery.data.data(), discovery.size, timeUs + 1000));
    TEST_ASSERT_EQUAL_UINT8(slot, receiver.assignedSlot());
    TEST_ASSERT_TRUE(receiver.connected(timeUs + 1000));
    EspNowProtocol::Packet reply;
    TEST_ASSERT_FALSE(receiver.tick(timeUs + 1000 + EspNowProtocol::ReplyStartUs, reply));

    ++sequence;
    cycle.data[14] = sequence;
    cycle.data[15] = sequence >> 8;
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, cycle.data.data(), cycle.size, timeUs + 2000));
    TEST_ASSERT_TRUE(receiver.tick(timeUs + 2000 + EspNowProtocol::ReplyStartUs, reply));
    TEST_ASSERT_EQUAL_UINT(EspNowProtocol::ReplyHeaderSize + telemetry.size, reply.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(telemetry.data.data(), reply.data.data() + EspNowProtocol::ReplyHeaderSize, telemetry.size);
}

void test_discovery_does_not_extend_rc_freshness_before_expiry_is_processed()
{
    join(1);
    const auto &cycle = lastPoll[0];
    EspNowSlaveLink receiver;
    receiver.reset(Uid, SlaveMacs[0], 999, timeUs);
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, cycle.data.data(), cycle.size, timeUs));

    EspNowProtocol::Packet discovery;
    EspNowProtocol::writeHeader(discovery, EspNowProtocol::Kind::Discovery, Uid,
                                EspNowProtocol::read32(cycle.data.data() + 10),
                                EspNowProtocol::read16(cycle.data.data() + 14) + 1, true);
    const uint32_t deadline = timeUs + EspNowProtocol::RcTimeoutUs;
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, discovery.data.data(), discovery.size, deadline - 1));
    TEST_ASSERT_TRUE(receiver.connected(deadline - 1));
    TEST_ASSERT_TRUE(receiver.hasMaster(deadline));
    TEST_ASSERT_FALSE(receiver.connected(deadline));
    TEST_ASSERT_EQUAL_UINT8(cycle.data[EspNowProtocol::HeaderSize], receiver.assignedSlot());
}

void test_discovery_alone_expires_registration_and_allows_rejoining()
{
    join(1);
    auto cycle = lastPoll[0];
    EspNowSlaveLink receiver;
    receiver.reset(Uid, SlaveMacs[0], 999, timeUs);
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, cycle.data.data(), cycle.size, timeUs));
    const uint8_t slot = receiver.assignedSlot();
    uint16_t sequence = EspNowProtocol::read16(cycle.data.data() + 14);
    EspNowProtocol::Packet discovery, reply;
    uint32_t now = timeUs;
    bool joined = false;
    for (unsigned i = 1; i <= 64; ++i)
    {
        now = timeUs + i * EspNowProtocol::CycleUs;
        EspNowProtocol::writeHeader(discovery, EspNowProtocol::Kind::Discovery, Uid,
                                    EspNowProtocol::read32(cycle.data.data() + 10), ++sequence, true);
        TEST_ASSERT_TRUE(receiver.receive(MasterMac, discovery.data.data(), discovery.size, now));
        TEST_ASSERT_TRUE(receiver.hasMaster(now));
        if (uint32_t(now - timeUs) < EspNowProtocol::RcTimeoutUs)
        {
            TEST_ASSERT_EQUAL_UINT8(slot, receiver.assignedSlot());
            continue;
        }
        TEST_ASSERT_EQUAL_UINT8(EspNowProtocol::NoSlot, receiver.assignedSlot());
        TEST_ASSERT_FALSE(receiver.connected(now));
        if (receiver.tick(now + EspNowProtocol::ReplyStartUs + 200, reply))
        {
            TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(EspNowProtocol::Kind::Join), reply.data[3]);
            TEST_ASSERT_EQUAL_UINT8(EspNowProtocol::NoSlot, reply.data[EspNowProtocol::ReplyHeaderSize - 1]);
            receiver.sent(now + 1000, true);
            joined = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(joined);
    ++sequence;
    cycle.data[14] = sequence;
    cycle.data[15] = sequence >> 8;
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, cycle.data.data(), cycle.size, now + 2000));
    TEST_ASSERT_EQUAL_UINT8(slot, receiver.assignedSlot());
    TEST_ASSERT_TRUE(receiver.connected(now + 2000));
}

void test_discovery_rejects_mac_tables_and_previous_protocol_version()
{
    EspNowSlaveLink receiver;
    receiver.reset(Uid, SlaveMacs[0], 999, timeUs);
    EspNowProtocol::Packet discovery;
    EspNowProtocol::writeHeader(discovery, EspNowProtocol::Kind::Discovery, Uid, 123, 1, true);
    discovery.size += EspNowProtocol::MaxSlaves * SlaveMacs[0].size();
    TEST_ASSERT_FALSE(receiver.receive(MasterMac, discovery.data.data(), discovery.size, timeUs));
    discovery.data[2] = 3;
    TEST_ASSERT_FALSE(receiver.receive(MasterMac, discovery.data.data(), discovery.size, timeUs));
    discovery.size = EspNowProtocol::HeaderSize;
    discovery.data[2] = EspNowProtocol::Version;
    TEST_ASSERT_TRUE(receiver.receive(MasterMac, discovery.data.data(), discovery.size, timeUs));
    TEST_ASSERT_EQUAL_UINT8(EspNowProtocol::NoSlot, receiver.assignedSlot());
}

void test_master_restart_requires_fresh_registration()
{
    join(2);
    air[0].active = false;
    master.reset(Uid, MasterMac, 456, timeUs);
    master.setChannels(rcFrame(), true);
    run(24000);
    TEST_ASSERT_FALSE(slaves[0].connected(timeUs));
    TEST_ASSERT_FALSE(slaves[1].connected(timeUs));
    run(400000);
    TEST_ASSERT_TRUE(slaves[0].connected(timeUs));
    TEST_ASSERT_TRUE(slaves[1].connected(timeUs));
}

void test_latest_local_channels_are_delivered_to_every_peer_without_backlog()
{
    join();
    auto channels = rcFrame();
    for (unsigned i = 0; i < 50; ++i)
    {
        channels.data[3] = i;
        EspNowProtocol::updateCrc(channels);
        master.setChannels(channels, true);
    }
    run(30000);
    for (unsigned i = 1; i <= 4; ++i)
    {
        TEST_ASSERT_TRUE(received[i].size() >= 2);
        for (const auto &frame : received[i])
            TEST_ASSERT_EQUAL_UINT8_ARRAY(channels.data.data(), frame.data.data(), channels.size);
    }
}

void test_routed_rc_cannot_replace_local_channels_or_fill_message_queue()
{
    join();
    auto routed = rcFrame();
    routed.data[3] = 42;
    EspNowProtocol::updateCrc(routed);
    for (unsigned i = 0; i <= EspNowProtocol::QueueSize; ++i)
        TEST_ASSERT_FALSE(master.enqueue(routed.data.data(), routed.size));
    enqueue(master, extended(CRSF_FRAMETYPE_DEVICE_PING, 0, 0xea));
    run(30000);
    const auto channels = rcFrame();
    for (unsigned i = 1; i <= 4; ++i)
    {
        TEST_ASSERT_TRUE(countFrames(i, CRSF_FRAMETYPE_RC_CHANNELS_PACKED) > 0);
        TEST_ASSERT_EQUAL_UINT(1, countFrames(i, CRSF_FRAMETYPE_DEVICE_PING));
        for (const auto &frame : received[i])
            if (frame.data[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
                TEST_ASSERT_EQUAL_UINT8_ARRAY(channels.data.data(), frame.data.data(), channels.size);
    }
}

void test_fifth_slave_cannot_displace_registered_peers()
{
    join();
    enabledSlaves = 5;
    run(400000);
    TEST_ASSERT_EQUAL_UINT8(EspNowProtocol::NoSlot, slaves[4].assignedSlot());
    TEST_ASSERT_FALSE(slaves[4].connected(timeUs));
    for (unsigned i = 0; i < 4; ++i)
        TEST_ASSERT_EQUAL_UINT8_ARRAY(SlaveMacs[i].data(), master.peerMac(slaves[i].assignedSlot()).data(), 6);
}

void test_late_join_receives_current_local_channels()
{
    join(1);
    auto channels = rcFrame();
    channels.data[3] = 42;
    EspNowProtocol::updateCrc(channels);
    master.setChannels(channels, true);
    run(30000);
    clearReceived();
    const auto normal = rcFrame();
    master.setChannels(normal, true);
    enabledSlaves = 2;
    run(150000);
    TEST_ASSERT_TRUE(slaves[1].connected(timeUs));
    TEST_ASSERT_TRUE(countFrames(2, CRSF_FRAMETYPE_RC_CHANNELS_PACKED) > 0);
    for (const auto &frame : received[2])
        TEST_ASSERT_EQUAL_UINT8_ARRAY(normal.data.data(), frame.data.data(), normal.size);
}

void test_queue_overflow_is_reported_and_waiting_messages_survive_discovery()
{
    const auto ping = extended(CRSF_FRAMETYPE_DEVICE_PING, 0, 0xea);
    for (unsigned i = 0; i < 16; ++i)
        enqueue(master, ping);
    TEST_ASSERT_FALSE(master.enqueue(ping.data.data(), ping.size));
    TEST_ASSERT_EQUAL_UINT32(1, master.errors());
    enabledSlaves = 1;
    run(200000);
    TEST_ASSERT_EQUAL_UINT(16, countFrames(1, CRSF_FRAMETYPE_DEVICE_PING));
}

void test_micros_wrap_does_not_break_discovery_or_unicast()
{
    timeUs = 0xfffff000;
    master.reset(Uid, MasterMac, 123, timeUs);
    master.setChannels(rcFrame(), true);
    for (unsigned i = 0; i < 5; ++i)
        slaves[i].reset(Uid, SlaveMacs[i], 999, timeUs);
    join();
    for (unsigned i = 0; i < 4; ++i)
        TEST_ASSERT_TRUE(slaves[i].connected(timeUs));
    TEST_ASSERT_EQUAL_UINT32(0, master.errors());
}

void test_master_yields_between_phases_and_preserves_discovery_and_cycle_waits()
{
    EspNowMasterLink link;
    link.reset(Uid, MasterMac, 123, 0);
    link.setChannels(rcFrame(), true);
    EspNowProtocol::Packet packet;
    const uint32_t start = EspNowProtocol::ReplyTimeoutUs;

    TEST_ASSERT_FALSE(link.tick(start - 1, packet));
    TEST_ASSERT_FALSE(link.tick(start, packet));
    TEST_ASSERT_FALSE(link.tick(start, packet));
    TEST_ASSERT_TRUE(link.tick(start, packet));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(Broadcast.data(), packet.destination.data(), 6);

    const uint32_t sentAt = start + 250;
    TEST_ASSERT_FALSE(link.tick(sentAt, packet));
    link.sent(sentAt, true);
    const uint32_t discoveryEnd = sentAt + EspNowProtocol::DiscoveryWindowUs;
    TEST_ASSERT_FALSE(link.tick(discoveryEnd - 1, packet));
    TEST_ASSERT_FALSE(link.tick(discoveryEnd, packet));
    TEST_ASSERT_FALSE(link.tick(discoveryEnd, packet));
    TEST_ASSERT_FALSE(link.tick(start + EspNowProtocol::CycleUs - 1, packet));
    TEST_ASSERT_FALSE(link.tick(start + EspNowProtocol::CycleUs, packet));
    TEST_ASSERT_TRUE(link.tick(start + EspNowProtocol::CycleUs, packet));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(Broadcast.data(), packet.destination.data(), 6);
}

void test_wrapped_queues_preserve_recipients_and_slave_telemetry()
{
    join(2);
    for (uint8_t round = 0; round < 32; ++round)
    {
        clearReceived();
        auto ping = extended(CRSF_FRAMETYPE_DEVICE_PING, CRSF_ADDRESS_BROADCAST, CRSF_ADDRESS_RADIO_TRANSMITTER);
        ping.data[5] = round;
        EspNowProtocol::updateCrc(ping);
        enqueue(master, ping);

        auto telemetry = extended(CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_CRSF_RECEIVER);
        telemetry.data[5] = round;
        EspNowProtocol::updateCrc(telemetry);
        enqueue(slaves[0], telemetry);
        enqueue(slaves[1], telemetry);
        run(30000);

        for (unsigned index = 1; index <= 2; ++index)
        {
            TEST_ASSERT_EQUAL_UINT(1, countFrames(index, CRSF_FRAMETYPE_DEVICE_PING));
            for (const auto &frame : received[index])
                if (frame.data[2] == CRSF_FRAMETYPE_DEVICE_PING)
                    TEST_ASSERT_EQUAL_UINT8_ARRAY(ping.data.data(), frame.data.data(), ping.size);
        }
        TEST_ASSERT_EQUAL_UINT(2, countFrames(0, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY));
        for (const auto &frame : received[0])
        {
            TEST_ASSERT_EQUAL_UINT8(round, frame.data[5]);
            TEST_ASSERT_TRUE(EspNowProtocol::validFrame(frame.data.data(), frame.size));
            TEST_ASSERT_TRUE(frame.data[4] == EspNowProtocol::slaveAddress(slaves[0].assignedSlot()) ||
                             frame.data[4] == EspNowProtocol::slaveAddress(slaves[1].assignedSlot()));
        }
    }
}

void test_packet_codec_preserves_cycle_wire_format_and_decodes_payload_view()
{
    const uint8_t expected[] = {
        'E', 'N', 4, 1, 1, 2, 3, 4, 5, 6, 0x12, 0x34, 0x56, 0x78, 0xcd, 0xab, 1, 3};
    EspNowProtocol::Packet packet;
    EspNowProtocol::writeMasterHeader(packet, Uid, 0x78563412, 0xabcd, true, 3);
    TEST_ASSERT_EQUAL_UINT(sizeof(expected), packet.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.data.data(), sizeof(expected));
    const auto channels = rcFrame();
    TEST_ASSERT_TRUE(packet.append(channels));

    EspNowProtocol::PacketView decoded;
    TEST_ASSERT_TRUE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(EspNowProtocol::Kind::Cycle), static_cast<uint8_t>(decoded.kind));
    TEST_ASSERT_EQUAL_HEX32(0x78563412, decoded.session);
    TEST_ASSERT_EQUAL_HEX16(0xabcd, decoded.sequence);
    TEST_ASSERT_TRUE(decoded.channelsValid);
    TEST_ASSERT_EQUAL_UINT8(3, decoded.slot);
    TEST_ASSERT_EQUAL_UINT(channels.size, decoded.frames.size());
    TEST_ASSERT_EQUAL_PTR(packet.data.data() + sizeof(expected), decoded.frames.data());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(channels.data.data(), decoded.frames.data(), channels.size);

    EspNowProtocol::writeMasterHeader(packet, Uid, 123, 7, false, EspNowProtocol::NoSlot);
    TEST_ASSERT_TRUE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(EspNowProtocol::Kind::Discovery), static_cast<uint8_t>(decoded.kind));
    TEST_ASSERT_FALSE(decoded.channelsValid);
    TEST_ASSERT_EQUAL_UINT8(EspNowProtocol::NoSlot, decoded.slot);
    TEST_ASSERT_EQUAL_UINT(0, decoded.frames.size());
}

void test_packet_codec_preserves_reply_wire_format_and_validates_join()
{
    const uint8_t expected[] = {
        'E', 'N', 4, 2, 1, 2, 3, 4, 5, 6, 0x12, 0x34, 0x56, 0x78, 0xcd, 0xab, 0,
        2, 0, 0, 0, 0, 10, 2};
    EspNowProtocol::Packet packet;
    EspNowProtocol::writeSlaveHeader(packet, Uid, 0x78563412, 0xabcd, false, MasterMac, 2);
    TEST_ASSERT_EQUAL_UINT(sizeof(expected), packet.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.data.data(), sizeof(expected));
    const auto telemetry = extended(CRSF_FRAMETYPE_DEVICE_INFO, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_CRSF_RECEIVER, CRSF_MAX_PACKET_LEN);
    TEST_ASSERT_TRUE(packet.append(telemetry));
    EspNowProtocol::PacketView decoded;
    TEST_ASSERT_TRUE(EspNowProtocol::decodePacket(SlaveMacs[0], Span<const uint8_t>{packet.data.data(), packet.size}, Uid, MasterMac, decoded));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(EspNowProtocol::Kind::Reply), static_cast<uint8_t>(decoded.kind));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(MasterMac.data(), decoded.masterMac.data(), MasterMac.size());
    TEST_ASSERT_EQUAL_UINT8(2, decoded.slot);
    TEST_ASSERT_EQUAL_UINT(telemetry.size, decoded.frames.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(telemetry.data.data(), decoded.frames.data(), telemetry.size);
    packet.data[EspNowProtocol::ReplyHeaderSize - 1] = EspNowProtocol::MaxSlaves;
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(SlaveMacs[0], Span<const uint8_t>{packet.data.data(), packet.size}, Uid, MasterMac, decoded));

    EspNowProtocol::writeSlaveHeader(packet, Uid, 123, 7, false, MasterMac, EspNowProtocol::NoSlot);
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(SlaveMacs[0], Span<const uint8_t>{packet.data.data(), size_t(packet.size) - 1}, Uid, MasterMac, decoded));
    TEST_ASSERT_TRUE(EspNowProtocol::decodePacket(SlaveMacs[0], Span<const uint8_t>{packet.data.data(), packet.size}, Uid, MasterMac, decoded));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(EspNowProtocol::Kind::Join), static_cast<uint8_t>(decoded.kind));
    TEST_ASSERT_EQUAL_UINT8(EspNowProtocol::NoSlot, decoded.slot);
    TEST_ASSERT_EQUAL_UINT(0, decoded.frames.size());
    packet.data[EspNowProtocol::ReplyHeaderSize - 1] = 0;
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(SlaveMacs[0], Span<const uint8_t>{packet.data.data(), packet.size}, Uid, MasterMac, decoded));
    packet.data[EspNowProtocol::ReplyHeaderSize - 1] = EspNowProtocol::NoSlot;
    TEST_ASSERT_TRUE(packet.append(telemetry));
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(SlaveMacs[0], Span<const uint8_t>{packet.data.data(), packet.size}, Uid, MasterMac, decoded));
}

void test_packet_decoder_rejects_invalid_envelopes_without_replacing_output()
{
    EspNowProtocol::Packet packet;
    EspNowProtocol::writeMasterHeader(packet, Uid, 123, 19, true, EspNowProtocol::NoSlot);
    EspNowProtocol::PacketView decoded;
    TEST_ASSERT_TRUE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{}, Uid, SlaveMacs[0], decoded));
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), size_t(packet.size) - 1}, Uid, SlaveMacs[0], decoded));
    const std::array<uint8_t, EspNowProtocol::MaxPacketSize + 1> oversized{};
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{oversized.data(), oversized.size()}, Uid, SlaveMacs[0], decoded));
    for (const auto &source : {EspNowProtocol::EmptyMac, Broadcast, SlaveMacs[0]})
        TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(source, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
    auto otherUid = Uid;
    ++otherUid[0];
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, otherUid, SlaveMacs[0], decoded));
    packet.data[0] ^= 1;
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
    packet.data[0] ^= 1;
    packet.data[3] = 0xff;
    packet.data[14] = 20;
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(EspNowProtocol::Kind::Discovery), static_cast<uint8_t>(decoded.kind));
    TEST_ASSERT_EQUAL_UINT16(19, decoded.sequence);
    TEST_ASSERT_EQUAL_UINT(0, decoded.frames.size());
}

void test_packet_decoder_requires_rc_first_and_checks_all_crsf_frames()
{
    EspNowProtocol::Packet packet;
    EspNowProtocol::writeMasterHeader(packet, Uid, 123, 7, true, 0);
    EspNowProtocol::PacketView decoded;
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
    const auto notChannels = extended(CRSF_FRAMETYPE_DEVICE_INFO, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_CRSF_RECEIVER, 27);
    TEST_ASSERT_TRUE(packet.append(notChannels));
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));

    EspNowProtocol::writeMasterHeader(packet, Uid, 123, 7, true, 0);
    TEST_ASSERT_TRUE(packet.append(rcFrame()));
    TEST_ASSERT_TRUE(packet.append(extended(CRSF_FRAMETYPE_DEVICE_INFO, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_CRSF_RECEIVER)));
    TEST_ASSERT_TRUE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
    packet.data[packet.size - 1] ^= 1;
    TEST_ASSERT_FALSE(EspNowProtocol::decodePacket(MasterMac, Span<const uint8_t>{packet.data.data(), packet.size}, Uid, SlaveMacs[0], decoded));
}

void test_slave_address_aliasing_rewrites_heartbeat_and_filters_other_slaves()
{
    auto heartbeat = extended(CRSF_FRAMETYPE_HEARTBEAT, 0, CRSF_ADDRESS_CRSF_RECEIVER, 6);
    EspNowProtocol::aliasOrigin(heartbeat, 2);
    TEST_ASSERT_EQUAL_UINT8(EspNowProtocol::slaveAddress(2), heartbeat.data[4]);
    TEST_ASSERT_TRUE(EspNowProtocol::validFrame(heartbeat.data.data(), heartbeat.size));

    auto own = extended(CRSF_FRAMETYPE_PARAMETER_READ, EspNowProtocol::slaveAddress(1), CRSF_ADDRESS_CRSF_RECEIVER);
    TEST_ASSERT_TRUE(EspNowProtocol::unaliasDestination(own, 1));
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_CRSF_RECEIVER, own.data[3]);
    TEST_ASSERT_TRUE(EspNowProtocol::validFrame(own.data.data(), own.size));

    auto other = extended(CRSF_FRAMETYPE_PARAMETER_READ, EspNowProtocol::slaveAddress(3), CRSF_ADDRESS_CRSF_RECEIVER);
    TEST_ASSERT_FALSE(EspNowProtocol::unaliasDestination(other, 1));

    auto master = extended(CRSF_FRAMETYPE_PARAMETER_READ, CRSF_ADDRESS_CRSF_RECEIVER, CRSF_ADDRESS_RADIO_TRANSMITTER);
    TEST_ASSERT_FALSE(EspNowProtocol::unaliasDestination(master, 1));
}

void test_channel_codec_round_trips_and_uses_ch5_without_status_byte()
{
    uint32_t channels[EspNowProtocol::RcChannels];
    for (unsigned i = 0; i < EspNowProtocol::RcChannels; ++i)
        channels[i] = (i * 131 + 172) & 0x7ff;
    const auto frame = EspNowProtocol::packChannels(channels, true);
    TEST_ASSERT_EQUAL_UINT8(27, frame.size);
    TEST_ASSERT_TRUE(EspNowProtocol::validFrame(frame.data.data(), frame.size));
    uint32_t decoded[EspNowProtocol::RcChannels]{};
    TEST_ASSERT_TRUE(EspNowProtocol::unpackChannels(frame, decoded));
    TEST_ASSERT_EQUAL_UINT32_ARRAY(channels, decoded, EspNowProtocol::RcChannels);
    TEST_ASSERT_FALSE(EspNowProtocol::unpackChannels(EspNowProtocol::packChannels(channels, false), decoded));

    auto noStatus = frame;
    noStatus.size = 26;
    noStatus.data[1] = 24;
    channels[4] = CRSF_CHANNEL_VALUE_STD_MAX;
    std::memcpy(noStatus.data.data() + 3, EspNowProtocol::packChannels(channels, false).data.data() + 3, 22);
    TEST_ASSERT_TRUE(EspNowProtocol::unpackChannels(noStatus, decoded));
}

void test_event_queue_round_trips_and_keeps_room_for_send_completion()
{
    static EspNowEvents events;
    events.clear();
    const auto frame = extended(CRSF_FRAMETYPE_DEVICE_PING, 0, 0xea);
    std::array<uint8_t, EspNowProtocol::MaxPacketSize> packet;
    packet.fill(0x5a);
    events.pushFrame(frame.data.data(), frame.size);
    events.pushReceived(SlaveMacs[1].data(), 0x12345678, packet.data(), packet.size());
    events.pushReceived(SlaveMacs[1].data(), 0, packet.data(), packet.size() + 1);
    TEST_ASSERT_EQUAL_UINT32(1, events.dropped());

    EspNowEvents::Event event;
    TEST_ASSERT_TRUE(events.pop(event));
    TEST_ASSERT_TRUE(event.type == EspNowEvents::Type::Frame);
    TEST_ASSERT_EQUAL_UINT8(frame.size, event.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame.data.data(), event.data.data(), frame.size);
    TEST_ASSERT_TRUE(events.pop(event));
    TEST_ASSERT_TRUE(event.type == EspNowEvents::Type::Received);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(SlaveMacs[1].data(), event.source.data(), 6);
    TEST_ASSERT_EQUAL_UINT32(0x12345678, event.time);
    TEST_ASSERT_EQUAL_UINT8(packet.size(), event.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(packet.data(), event.data.data(), packet.size());
    TEST_ASSERT_FALSE(events.pop(event));

    // Fill the queue completely with large and small records; a send completion must still fit.
    unsigned received = 0, frames = 0;
    for (uint32_t dropped = events.dropped(); events.dropped() == dropped; ++received)
        events.pushReceived(SlaveMacs[0].data(), 0, packet.data(), packet.size());
    --received;
    for (uint32_t dropped = events.dropped(); events.dropped() == dropped; ++frames)
        events.pushFrame(frame.data.data(), frame.size);
    --frames;
    events.pushSent(42, true);
    const uint32_t dropped = events.dropped();

    unsigned poppedReceived = 0, poppedFrames = 0, poppedSent = 0;
    while (events.pop(event))
    {
        poppedReceived += event.type == EspNowEvents::Type::Received;
        poppedFrames += event.type == EspNowEvents::Type::Frame;
        if (event.type == EspNowEvents::Type::Sent)
        {
            ++poppedSent;
            TEST_ASSERT_EQUAL_UINT32(42, event.time);
            TEST_ASSERT_TRUE(event.success);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(3, dropped);
    TEST_ASSERT_TRUE(received > 0);
    TEST_ASSERT_EQUAL_UINT(received, poppedReceived);
    TEST_ASSERT_EQUAL_UINT(frames, poppedFrames);
    TEST_ASSERT_EQUAL_UINT(1, poppedSent);
}

void test_frame_constructor_copies_span_into_owned_storage()
{
    auto source = rcFrame();
    const auto expected = source;
    EspNowProtocol::Frame frame{Span<const uint8_t>{source.data.data(), source.size}};
    source.data[3] ^= 0xff;
    TEST_ASSERT_EQUAL_UINT8(expected.size, frame.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data.data(), frame.data.data(), frame.size);
    TEST_ASSERT_TRUE(EspNowProtocol::validFrame(frame.data.data(), frame.size));
}

void test_frame_constructor_accepts_empty_and_capacity_sized_spans()
{
    const EspNowProtocol::Frame empty{Span<const uint8_t>{}};
    TEST_ASSERT_EQUAL_UINT8(0, empty.size);

    std::array<uint8_t, CRSF_MAX_PACKET_LEN> bytes;
    bytes.fill(0xa5);
    const EspNowProtocol::Frame emptyWithStorage{Span<const uint8_t>{bytes.data(), 0}};
    TEST_ASSERT_EQUAL_UINT8(0, emptyWithStorage.size);

    const EspNowProtocol::Frame full{Span<const uint8_t>{bytes.data(), bytes.size()}};
    TEST_ASSERT_EQUAL_UINT8(bytes.size(), full.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(bytes.data(), full.data.data(), full.size);
}

void test_frame_constructor_rejects_oversized_span_without_copying()
{
    std::array<uint8_t, CRSF_MAX_PACKET_LEN + 1> bytes;
    bytes.fill(0xa5);
    const EspNowProtocol::Frame frame{Span<const uint8_t>{bytes.data(), bytes.size()}};
    TEST_ASSERT_EQUAL_UINT8(0, frame.size);
    const std::array<uint8_t, CRSF_MAX_PACKET_LEN> empty{};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(empty.data(), frame.data.data(), empty.size());
}

void test_packet_append_preserves_prefix_and_copies_frame()
{
    EspNowProtocol::Packet packet;
    packet.data.fill(0x5a);
    packet.size = EspNowProtocol::HeaderSize;
    const auto before = packet;
    const auto frame = rcFrame();

    TEST_ASSERT_TRUE(packet.append(frame));
    TEST_ASSERT_EQUAL_UINT(before.size + frame.size, packet.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(before.data.data(), packet.data.data(), before.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame.data.data(), packet.data.data() + before.size, frame.size);
}

void test_packet_append_accepts_exact_fit_and_preserves_packet_on_overflow()
{
    const auto frame = rcFrame();
    EspNowProtocol::Packet packet;
    packet.size = EspNowProtocol::MaxPacketSize - frame.size;
    TEST_ASSERT_TRUE(packet.append(frame));
    TEST_ASSERT_EQUAL_UINT(EspNowProtocol::MaxPacketSize, packet.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame.data.data(), packet.data.data() + packet.size - frame.size, frame.size);

    packet.size = EspNowProtocol::MaxPacketSize - frame.size + 1;
    const auto before = packet;
    TEST_ASSERT_FALSE(packet.append(frame));
    TEST_ASSERT_EQUAL_UINT8(before.size, packet.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(before.data.data(), packet.data.data(), packet.data.size());
}

void test_packet_append_rejects_invalid_frame_sizes_without_modifying_packet()
{
    EspNowProtocol::Packet packet;
    packet.data.fill(0x5a);
    packet.size = EspNowProtocol::HeaderSize;
    const auto before = packet;
    EspNowProtocol::Frame frame;
    TEST_ASSERT_FALSE(packet.append(frame));
    frame.size = frame.data.size() + 1;
    TEST_ASSERT_FALSE(packet.append(frame));
    TEST_ASSERT_EQUAL_UINT8(before.size, packet.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(before.data.data(), packet.data.data(), packet.data.size());
}

void test_master_does_not_send_when_rc_frame_cannot_be_appended()
{
    join(1);
    const auto polls = pollCount[0];
    auto invalid = rcFrame();
    invalid.size = invalid.data.size() + 1;
    master.setChannels(invalid, true);
    run(50000);
    TEST_ASSERT_EQUAL_UINT32(polls, pollCount[0]);
    TEST_ASSERT_TRUE(master.errors() > 0);

    master.setChannels(rcFrame(), true);
    run(50000);
    TEST_ASSERT_TRUE(pollCount[0] > polls);
    TEST_ASSERT_TRUE(slaves[0].connected(timeUs));
}

void test_radio_pins_select_ota_or_hf_free_slave_without_optional_pins()
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RxRadioMode::EspNowSlave),
                          static_cast<int>(classifyRxRadioPins({{-1, -1, -1, -1, -1}}, {{-1, -1}}, {-1, -1})));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RxRadioMode::Ota),
                          static_cast<int>(classifyRxRadioPins({{0, 1, 2, 3, 4}}, {{-1, -1}}, {-1, -1})));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RxRadioMode::Ota),
                          static_cast<int>(classifyRxRadioPins({{0, 1, 2, 3, 4}}, {{5, 6}}, {7, 8})));
}

void test_partial_radio_pins_never_fall_back_to_espnow()
{
    const std::array<int, 5> configured = {{0, 1, 2, 3, 4}};
    for (unsigned missing = 0; missing < configured.size(); ++missing)
    {
        auto pins = configured;
        pins[missing] = -1;
        TEST_ASSERT_EQUAL_INT(static_cast<int>(RxRadioMode::Invalid),
                              static_cast<int>(classifyRxRadioPins(pins, {{-1, -1}}, {})));
    }
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RxRadioMode::Invalid),
                          static_cast<int>(classifyRxRadioPins({{-1, -1, -1, -1, -1}}, {{-1, -1}}, {0})));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RxRadioMode::Invalid),
                          static_cast<int>(classifyRxRadioPins(configured, {{5, -1}}, {})));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RxRadioMode::Invalid),
                          static_cast<int>(classifyRxRadioPins(configured, {{-1, 6}}, {})));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RxRadioMode::Invalid),
                          static_cast<int>(classifyRxRadioPins(configured, {{-1, -1}}, {-2})));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_packet_codec_preserves_cycle_wire_format_and_decodes_payload_view);
    RUN_TEST(test_packet_codec_preserves_reply_wire_format_and_validates_join);
    RUN_TEST(test_packet_decoder_rejects_invalid_envelopes_without_replacing_output);
    RUN_TEST(test_packet_decoder_requires_rc_first_and_checks_all_crsf_frames);
    RUN_TEST(test_slave_address_aliasing_rewrites_heartbeat_and_filters_other_slaves);
    RUN_TEST(test_channel_codec_round_trips_and_uses_ch5_without_status_byte);
    RUN_TEST(test_event_queue_round_trips_and_keeps_room_for_send_completion);
    RUN_TEST(test_frame_constructor_copies_span_into_owned_storage);
    RUN_TEST(test_frame_constructor_accepts_empty_and_capacity_sized_spans);
    RUN_TEST(test_frame_constructor_rejects_oversized_span_without_copying);
    RUN_TEST(test_packet_append_preserves_prefix_and_copies_frame);
    RUN_TEST(test_packet_append_accepts_exact_fit_and_preserves_packet_on_overflow);
    RUN_TEST(test_packet_append_rejects_invalid_frame_sizes_without_modifying_packet);
    RUN_TEST(test_master_does_not_send_when_rc_frame_cannot_be_appended);
    RUN_TEST(test_radio_pins_select_ota_or_hf_free_slave_without_optional_pins);
    RUN_TEST(test_partial_radio_pins_never_fall_back_to_espnow);
    RUN_TEST(test_discovery_and_registered_traffic_use_distinct_destinations);
    RUN_TEST(test_slow_downlink_retries_extend_cycle_without_overlapping_transmissions);
    RUN_TEST(test_slow_reply_retries_extend_cycle_and_are_accepted);
    RUN_TEST(test_early_reply_does_not_bypass_pending_send_completion);
    RUN_TEST(test_late_join_silent_peer_removal_and_rejoin);
    RUN_TEST(test_uid_source_and_duplicate_filtering);
    RUN_TEST(test_addressed_parameters_and_aliased_replies_for_all_four_slaves);
    RUN_TEST(test_rc_and_maximum_crsf_frame_reach_every_peer_once);
    RUN_TEST(test_failed_peer_retains_its_messages_without_duplicates_for_other_peers);
    RUN_TEST(test_nonextended_telemetry_and_rc_failsafe);
    RUN_TEST(test_master_drops_rc_frames_from_slave_replies);
    RUN_TEST(test_removed_peer_releases_pending_queue_recipients);
    RUN_TEST(test_malformed_poll_does_not_refresh_heartbeat);
    RUN_TEST(test_cycle_rejects_invalid_slots_and_truncated_headers);
    RUN_TEST(test_slave_rejects_previous_wire_format);
    RUN_TEST(test_discovery_preserves_registration_and_pending_telemetry);
    RUN_TEST(test_discovery_does_not_extend_rc_freshness_before_expiry_is_processed);
    RUN_TEST(test_discovery_alone_expires_registration_and_allows_rejoining);
    RUN_TEST(test_discovery_rejects_mac_tables_and_previous_protocol_version);
    RUN_TEST(test_master_restart_requires_fresh_registration);
    RUN_TEST(test_latest_local_channels_are_delivered_to_every_peer_without_backlog);
    RUN_TEST(test_routed_rc_cannot_replace_local_channels_or_fill_message_queue);
    RUN_TEST(test_fifth_slave_cannot_displace_registered_peers);
    RUN_TEST(test_late_join_receives_current_local_channels);
    RUN_TEST(test_queue_overflow_is_reported_and_waiting_messages_survive_discovery);
    RUN_TEST(test_micros_wrap_does_not_break_discovery_or_unicast);
    RUN_TEST(test_master_yields_between_phases_and_preserves_discovery_and_cycle_waits);
    RUN_TEST(test_wrapped_queues_preserve_recipients_and_slave_telemetry);
    return UNITY_END();
}
