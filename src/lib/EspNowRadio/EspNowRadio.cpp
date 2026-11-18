#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(TARGET_RX)

#include "EspNowRadio.h"

#include <cstring>

#if defined(PLATFORM_ESP32)
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
extern "C"
{
#include <user_interface.h>
}
#endif

#include "logging.h"

namespace
{
constexpr uint8_t WifiChannel = 1;
EspNowRadio::ReceiveCallback receiveCallback = nullptr;
EspNowRadio::SentCallback sentCallback = nullptr;

#if defined(PLATFORM_ESP32)
void onDataReceived(const uint8_t *const mac, const uint8_t *const data, const int size)
{
    if (size > 0)
        receiveCallback(mac, data, size);
}

void onDataSent(const uint8_t *const, const esp_now_send_status_t status)
{
    sentCallback(status == ESP_NOW_SEND_SUCCESS);
}
#else
void onDataReceived(uint8_t *const mac, uint8_t *const data, const uint8_t size)
{
    receiveCallback(mac, data, size);
}

void onDataSent(uint8_t *const, const uint8_t status)
{
    sentCallback(status == 0);
}
#endif
} // namespace

uint32_t EspNowRadio::newSession()
{
#if defined(PLATFORM_ESP32)
    return esp_random();
#else
    return os_random();
#endif
}

bool EspNowRadio::start(EspNowProtocol::Mac &mac, const ReceiveCallback receive, const SentCallback sent)
{
    if (receive == nullptr || sent == nullptr)
    {
        ERRLN("ESP-NOW callbacks missing");
        return false;
    }
#if defined(PLATFORM_ESP8266)
    WiFi.forceSleepWake();
    delay(10);
#endif
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    if (!WiFi.mode(WIFI_STA))
    {
        ERRLN("ESP-NOW Wi-Fi station setup failed");
        return false;
    }
    WiFi.disconnect();
    WiFi.macAddress(mac.data());
    bool ready = false;
#if defined(PLATFORM_ESP32)
    if (esp_wifi_set_channel(WifiChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
        esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK)
    {
        ERRLN("ESP-NOW Wi-Fi channel/power-save setup failed");
        return false;
    }
    const auto initResult = esp_now_init();
#else
    if (!wifi_set_channel(WifiChannel) || !wifi_set_sleep_type(NONE_SLEEP_T))
    {
        ERRLN("ESP-NOW Wi-Fi channel/power-save setup failed");
        return false;
    }
    const auto initResult = esp_now_init();
#endif
    if (initResult != 0)
    {
        ERRLN("ESP-NOW init failed: %d", initResult);
        return false;
    }
#if defined(PLATFORM_ESP32)
    ready = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M) == ESP_OK;
    esp_now_peer_info_t peer{};
    std::memcpy(peer.peer_addr, EspNowProtocol::Broadcast.data(), EspNowProtocol::Broadcast.size());
    peer.channel = WifiChannel;
    peer.ifidx = WIFI_IF_STA;
    ready = ready && esp_now_add_peer(&peer) == ESP_OK;
#else
    ready = esp_now_set_self_role(ESP_NOW_ROLE_COMBO) == 0 &&
            wifi_set_user_fixed_rate(FIXED_RATE_MASK_STA, PHY_RATE_6) == 0;
    auto peerAddress = EspNowProtocol::Broadcast;
    ready = ready && esp_now_add_peer(peerAddress.data(), ESP_NOW_ROLE_COMBO, WifiChannel, nullptr, 0) == 0;
#endif
    receiveCallback = receive;
    sentCallback = sent;
    ready = ready && esp_now_register_recv_cb(onDataReceived) == 0 && esp_now_register_send_cb(onDataSent) == 0;
    if (!ready)
    {
        ERRLN("ESP-NOW rate/peer/callback setup failed");
        if (esp_now_deinit() != 0)
            ERRLN("ESP-NOW cleanup failed");
        return false;
    }
    DBGLN("ESP-NOW started: %s", WiFi.macAddress().c_str());
    return true;
}

void EspNowRadio::stop()
{
    if (esp_now_deinit() != 0)
        ERRLN("ESP-NOW deinit failed");
}

bool EspNowRadio::ensurePeer(const EspNowProtocol::Mac &peer)
{
    auto address = peer;
    if (esp_now_is_peer_exist(address.data()) > 0)
        return true;
#if defined(PLATFORM_ESP32)
    esp_now_peer_info_t info{};
    std::memcpy(info.peer_addr, address.data(), address.size());
    info.channel = WifiChannel;
    info.ifidx = WIFI_IF_STA;
    const auto result = esp_now_add_peer(&info);
#else
    const auto result = esp_now_add_peer(address.data(), ESP_NOW_ROLE_COMBO, WifiChannel, nullptr, 0);
#endif
    if (result != 0)
        ERRLN("ESP-NOW add peer failed: %d", result);
    return result == 0;
}

void EspNowRadio::removePeer(const EspNowProtocol::Mac &peer)
{
    if (peer == EspNowProtocol::EmptyMac)
        return;
    auto address = peer;
    if (esp_now_is_peer_exist(address.data()) > 0)
    {
        const auto result = esp_now_del_peer(address.data());
        if (result != 0)
            ERRLN("ESP-NOW remove peer failed: %d", result);
    }
}

bool EspNowRadio::send(EspNowProtocol::Packet &packet)
{
    auto destination = packet.destination;
    if (!ensurePeer(destination))
        return false;
    const auto result = esp_now_send(destination.data(), packet.data.data(), packet.size);
    if (result != 0)
        ERRLN("ESP-NOW send failed: %d", result);
    return result == 0;
}

#endif
