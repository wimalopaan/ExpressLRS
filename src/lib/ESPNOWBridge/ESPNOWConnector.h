#ifndef ESPNOW_CONNECTOR_H
#define ESPNOW_CONNECTOR_H

#if defined(PLATFORM_ESP32) && (defined(ESPNOW_BRIDGE_MASTER) || defined(ESPNOW_BRIDGE_SLAVE))
#define ESPNOW_BRIDGE_ENABLED 1
#endif

#if defined(ESPNOW_BRIDGE_ENABLED)

#include "CRSFConnector.h"
#include <esp_now.h>
#include <esp_timer.h>
#include <freertos/queue.h>

/**
 * @brief Tunnels CRSF frames between a master and a slave receiver via ESP-NOW.
 *
 * Both receivers are built with the same binding UID; a 16-bit pair ID derived
 * from it is carried in every ESP-NOW frame so unrelated receivers are ignored.
 * Frames received over ESP-NOW are injected into the CRSF router with this
 * connector as origin, so they are never echoed back over ESP-NOW.
 */
class ESPNOWConnector final : public CRSFConnector {
public:
    /**
     * @brief Starts WiFi in STA mode, initialises ESP-NOW and registers with the CRSF router.
     * @param uid      Binding UID (identical on master and slave)
     * @param uidLen   Length of the UID
     * @return true on success
     */
    bool begin(const uint8_t *uid, uint8_t uidLen);

    void forwardMessage(const crsf_header_t *message) override;

    /**
     * @brief Queues a CRSF frame; it is sent in the next slot of this receiver.
     */
    void enqueueFrame(const uint8_t *frame, uint8_t frameLen);

    /**
     * @brief Master only: sends the RC frame, then queued frames, in the master slot.
     * Called once per RC frame, so the RC frame also serves as the slot timing reference.
     */
    void transmitSlot(const uint8_t *rcFrame, uint8_t rcFrameLen);

    /**
     * @brief Processes ESP-NOW frames received since the last call. Call from loop(), never from the WiFi task.
     */
    void process();

private:
    static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len);
    static void onSlaveSlot(void *arg);
    void handleReceived(const uint8_t *data, int len);
    void sendQueued(uint8_t maxPackets);
    uint8_t fillFromQueue(uint8_t *payload, uint8_t used);
    void sendPacket(const uint8_t *payload, uint8_t len);

    QueueHandle_t rxQueue = nullptr;
    QueueHandle_t txQueue = nullptr;
    esp_timer_handle_t slotTimer = nullptr;
    uint16_t pairId = 0;
    bool active = false;
};

extern ESPNOWConnector espnowBridge;

#endif // ESPNOW_BRIDGE_ENABLED
#endif // ESPNOW_CONNECTOR_H
