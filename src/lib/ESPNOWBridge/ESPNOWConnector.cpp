#include "ESPNOWConnector.h"

#if defined(ESPNOW_BRIDGE_ENABLED)

#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/queue.h>
#include <cstring>

#include "CRSFRouter.h"
#include "logging.h"

#ifndef ESPNOW_BRIDGE_CHANNEL
#define ESPNOW_BRIDGE_CHANNEL 1
#endif

namespace {
constexpr uint8_t ESPNOW_FRAME_MAGIC = 0xE5;
constexpr uint8_t ESPNOW_HEADER_LEN = 3; // magic + 16-bit pair ID
constexpr uint8_t broadcastAddr[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr UBaseType_t RX_QUEUE_LENGTH = 8;
constexpr UBaseType_t TX_QUEUE_LENGTH = 16;
// ESP-NOW payload after the pair ID header; CRSF frames are packed into it
constexpr uint8_t ESPNOW_MAX_PAYLOAD = ESP_NOW_MAX_DATA_LEN - ESPNOW_HEADER_LEN;
// Packets sent per slot. Keeps the master slot and the slave window short.
constexpr uint8_t MAX_BURST = 4;

#ifndef ESPNOW_SLOT_GUARD_US
// Slave waits this long after the last master frame before transmitting. Must exceed the master burst airtime.
#define ESPNOW_SLOT_GUARD_US 2000
#endif

struct RxFrame
{
    uint8_t len;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
};

struct TxFrame
{
    uint8_t len;
    uint8_t data[CRSF_FRAME_SIZE_MAX];
};

// FNV-1a over the binding UID; both receivers with the same binding phrase derive the same ID
uint16_t pairIdFromUid(const uint8_t *uid, const uint8_t uidLen)
{
    uint32_t hash = 2166136261u;
    for (uint8_t i = 0; i < uidLen; ++i)
    {
        hash ^= uid[i];
        hash *= 16777619u;
    }
    return (uint16_t)(hash ^ (hash >> 16));
}
}

ESPNOWConnector espnowBridge;

bool ESPNOWConnector::begin(const uint8_t *uid, const uint8_t uidLen)
{
    pairId = pairIdFromUid(uid, uidLen);

    rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(RxFrame));
    txQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(TxFrame));
    if (rxQueue == nullptr || txQueue == nullptr)
    {
        DBGLN("ESP-NOW queue alloc failed");
        return false;
    }

#if defined(ESPNOW_BRIDGE_SLAVE)
    esp_timer_create_args_t slotArgs {};
    slotArgs.callback = ESPNOWConnector::onSlaveSlot;
    slotArgs.name = "espnow_slot";
    if (esp_timer_create(&slotArgs, &slotTimer) != ESP_OK)
    {
        DBGLN("ESP-NOW slot timer failed");
        return false;
    }
#endif

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);
    esp_wifi_set_channel(ESPNOW_BRIDGE_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK)
    {
        DBGLN("ESP-NOW init failed");
        return false;
    }
    esp_now_register_recv_cb(ESPNOWConnector::onReceive);

    esp_now_peer_info_t peer {};
    memcpy(peer.peer_addr, broadcastAddr, ESP_NOW_ETH_ALEN);
    peer.channel = ESPNOW_BRIDGE_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK)
    {
        DBGLN("ESP-NOW add broadcast peer failed");
        return false;
    }

    crsfRouter.addConnector(this);
    active = true;
    DBGLN("ESP-NOW bridge active, pair id %04x", pairId);
    return true;
}

void ESPNOWConnector::forwardMessage(const crsf_header_t *message)
{
#if defined(ESPNOW_BRIDGE_SLAVE)
    // Link statistics describe the slave's own RF link and must not replace the master's
    if (message->type == CRSF_FRAMETYPE_LINK_STATISTICS)
    {
        return;
    }
#endif
    const auto *data = (const uint8_t *)message;
    enqueueFrame(data, CRSF_FRAME_SIZE(data[CRSF_TELEMETRY_LENGTH_INDEX]));
}

void ESPNOWConnector::enqueueFrame(const uint8_t *frame, const uint8_t frameLen)
{
    if (!active || frameLen > CRSF_FRAME_SIZE_MAX)
    {
        return;
    }
    TxFrame item;
    item.len = frameLen;
    memcpy(item.data, frame, frameLen);
    xQueueSend(txQueue, &item, 0);
}

void ESPNOWConnector::transmitSlot(const uint8_t *rcFrame, const uint8_t rcFrameLen)
{
    if (!active || rcFrameLen > ESPNOW_MAX_PAYLOAD)
    {
        return;
    }
    // The RC frame goes first, queued frames are packed behind it
    uint8_t payload[ESPNOW_MAX_PAYLOAD];
    memcpy(payload, rcFrame, rcFrameLen);
    sendPacket(payload, fillFromQueue(payload, rcFrameLen));

    sendQueued(MAX_BURST - 1);
}

uint8_t ESPNOWConnector::fillFromQueue(uint8_t *payload, uint8_t used)
{
    TxFrame item;
    while (xQueuePeek(txQueue, &item, 0) == pdTRUE && used + item.len <= ESPNOW_MAX_PAYLOAD)
    {
        xQueueReceive(txQueue, &item, 0);
        memcpy(payload + used, item.data, item.len);
        used += item.len;
    }
    return used;
}

void ESPNOWConnector::sendPacket(const uint8_t *payload, const uint8_t len)
{
    if (len == 0)
    {
        return;
    }
    uint8_t buffer[ESPNOW_HEADER_LEN + ESPNOW_MAX_PAYLOAD];
    buffer[0] = ESPNOW_FRAME_MAGIC;
    buffer[1] = pairId & 0xFF;
    buffer[2] = pairId >> 8;
    memcpy(buffer + ESPNOW_HEADER_LEN, payload, len);
    esp_now_send(broadcastAddr, buffer, ESPNOW_HEADER_LEN + len);
}

void ESPNOWConnector::sendQueued(const uint8_t maxPackets)
{
    uint8_t payload[ESPNOW_MAX_PAYLOAD];
    for (uint8_t i = 0; i < maxPackets && uxQueueMessagesWaiting(txQueue) > 0; ++i)
    {
        sendPacket(payload, fillFromQueue(payload, 0));
    }
}

void ESPNOWConnector::onSlaveSlot(void *arg)
{
    (void)arg;
    espnowBridge.sendQueued(MAX_BURST);
}

void ESPNOWConnector::onReceive(const esp_now_recv_info_t *info, const uint8_t *data, const int len)
{
    (void)info;
    if (len <= ESPNOW_HEADER_LEN || len > ESP_NOW_MAX_DATA_LEN)
    {
        return;
    }
    const uint16_t receivedPairId = data[1] | (data[2] << 8);
    if (data[0] != ESPNOW_FRAME_MAGIC || receivedPairId != espnowBridge.pairId)
    {
        return;
    }

#if defined(ESPNOW_BRIDGE_SLAVE)
    // The master burst ends with its last frame: transmit our queue after the guard time from there
    esp_timer_stop(espnowBridge.slotTimer);
    esp_timer_start_once(espnowBridge.slotTimer, ESPNOW_SLOT_GUARD_US);
#endif

    // Runs in the WiFi task: only copy the frame, the router and radio are not thread-safe
    RxFrame frame;
    frame.len = (uint8_t)len;
    memcpy(frame.data, data, len);
    xQueueSend(espnowBridge.rxQueue, &frame, 0);
}

void ESPNOWConnector::process()
{
    if (!active)
    {
        return;
    }
    RxFrame frame;
    while (xQueueReceive(rxQueue, &frame, 0) == pdTRUE)
    {
        handleReceived(frame.data + ESPNOW_HEADER_LEN, frame.len - ESPNOW_HEADER_LEN);
    }
}

void ESPNOWConnector::handleReceived(const uint8_t *payload, const int len)
{
    // A packet holds one or more complete CRSF frames back to back; each is delimited by its length field
    int pos = 0;
    while (pos + CRSF_FRAME_NOT_COUNTED_BYTES <= len)
    {
        const uint8_t *frame = payload + pos;
        const uint8_t declared = frame[CRSF_TELEMETRY_LENGTH_INDEX];
        const int frameLen = CRSF_FRAME_SIZE(declared);
        if (declared < CRSF_TELEMETRY_CRC_LENGTH + 1 || pos + frameLen > len)
        {
            return;
        }

        // CRC covers type through payload, i.e. everything after sync and length, excluding the CRC byte itself
        const uint8_t crc = crsfRouter.crsf_crc.calc(frame + CRSF_FRAME_NOT_COUNTED_BYTES, declared - CRSF_TELEMETRY_CRC_LENGTH);
        if (crc != frame[frameLen - 1])
        {
            return;
        }

        crsfRouter.processMessage(this, (const crsf_header_t *)frame);
        pos += frameLen;
    }
}

#endif // ESPNOW_BRIDGE_ENABLED
