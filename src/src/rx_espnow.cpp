#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include <cstring>

#include "rx_espnow.h"
#include "rx_wmextension.h"

#include "targets.h"
#include "common.h"
#include "logging.h"

#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <WiFi.h>

extern EspNowMaster espNow;

static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static void onDataSent(const uint8_t* const mac_addr, const esp_now_send_status_t status) {
}
static void onDataRecv(const uint8_t* const mac_addr, const uint8_t* const data, const int len) {
    espNow.processBytes(data, len);
}
EspNowMaster::EspNowMaster() {
    crsfRouter.addConnector(this);    
}
EspNowMaster::~EspNowMaster() {
    crsfRouter.removeConnector(this);    
}
void EspNowMaster::forwardMessage(const crsf_header_t* const message) {
    start();
    const auto* const data = (uint8_t *)message;
    const uint8_t length = data[CRSF_TELEMETRY_LENGTH_INDEX] + CRSF_FRAME_NOT_COUNTED_BYTES - 3;
    if (length < 64) {
        std::memcpy(&inBuffer[2], &data[2], length);
        inBuffer[0] = mUid >> 8;
        inBuffer[0] = mUid;
        if (inBuffer[2] == CRSF_FRAMETYPE_DEVICE_PING) {
            DBGLN("E PG");        
        }
        else if (inBuffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            DBGLN("E RC");        
        }
        else if (inBuffer[2] == CRSF_FRAMETYPE_LINK_STATISTICS) {
            DBGLN("E LK");        
        }
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&inBuffer[0], length);
        if (result != ESP_OK) {
            DBGLN("EspNow send NOK: %u", result);        
        }
    }
}
// void EspNowMaster::forwardMessage(const crsf_header_t* const message) {
//     start();
//     const auto* const data = (uint8_t *)message;
//     const uint8_t totalBufferLen = data[CRSF_TELEMETRY_LENGTH_INDEX] + CRSF_FRAME_NOT_COUNTED_BYTES;
//     if (data[2] == CRSF_FRAMETYPE_DEVICE_PING) {
//         DBGLN("E PG");        
//     }
//     else if (data[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
//         DBGLN("E RC");        
//     }
//     else if (data[2] == CRSF_FRAMETYPE_LINK_STATISTICS) {
//         DBGLN("E LK");        
//     }
//     if (totalBufferLen <= CRSF_FRAME_SIZE_MAX) {
//         DBGLN("EspNow forward: %u, %u %u", data[0], data[1], data[2]);        
//         esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&data[0], totalBufferLen);
//         if (result != ESP_OK) {
//             DBGLN("EspNow send NOK: %u", result);        
//         }
//     }
// }
void EspNowMaster::processBytes(const uint8_t* const data, const int len) {
    DBGLN("EspNow process bytes");        
    // crsfParser.processBytes(this, data, len);

    const uint16_t uid = (data[0] << 8) + data[1];
    
    if (uid == mUid) {
        const uint8_t crsfLen = len + 3 - 2;
        std::memcpy(&inBuffer[2], &data[2], len - 2);
        inBuffer[0] = 0xc8;
        inBuffer[1] = crsfLen;
        const uint8_t crc = crsfRouter.crsf_crc.calc(&inBuffer[2], len);
        inBuffer[len + 2] = crc;
        const crsf_header_t *header = (crsf_header_t *)&inBuffer[0];
        crsfRouter.processMessage(this, header);
    }
}
void EspNowMaster::start() {
    if (s_initialized) return;

    DBGLN("EspNow start");
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // ensure STA only, no AP
    if (esp_now_init() != ESP_OK) {
        DBGLN("esp_now_init failed");
        return;
    }
    
    esp_now_peer_info_t s_peerInfo;
    memset(&s_peerInfo, 0, sizeof(s_peerInfo));
    memcpy(s_peerInfo.peer_addr, broadcastAddress, 6);
    s_peerInfo.channel = 0;
    s_peerInfo.encrypt = false;

    esp_err_t err = esp_now_add_peer(&s_peerInfo);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        DBGLN("esp_now_add_peer failed %d", err);
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    s_initialized = true;    
}

void EspNowMaster::sendSwitches(const MultiSwitch* msw){
    DBGLN("EspNow send");
    if (!s_initialized) {
        start();
    }
    
    uint8_t data = 0;
    if (msw) {
        data = msw->switches()[0];
    }
    
    esp_now_peer_info_t s_peerInfo;
    memset(&s_peerInfo, 0, sizeof(s_peerInfo));
    memcpy(s_peerInfo.peer_addr, broadcastAddress, 6);
    s_peerInfo.channel = 0;
    s_peerInfo.encrypt = false;

    esp_err_t err = esp_now_add_peer(&s_peerInfo);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        DBGLN("esp_now_add_peer failed %d", err);
    }

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&data, sizeof(data));
    if (result == ESP_OK) {
        DBGLN("EspNow send OK");        
    }
    else {
        DBGLN("EspNow send NOK: %u", result);        
    }
}

#endif
