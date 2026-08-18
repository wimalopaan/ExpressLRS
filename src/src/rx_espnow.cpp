#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(TARGET_RX)

#include <cstring>

#include "rx_espnow.h"
#include "rx_wmextension.h"

#include "targets.h"
#include "common.h"
#include "logging.h"
#include "OTA.h"
#include "config.h"

#if defined(PLATFORM_ESP32)
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#endif
#if defined(PLATFORM_ESP8266)
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

#ifndef ESPNOW_TIMEOUT_MS
# define ESPNOW_TIMEOUT_MS 5
#endif

#ifndef ESPNOW_CHANNELS_MS
# define ESPNOW_CHANNELS_MS 20
#endif

extern RxConfig config;
extern uint32_t ChannelData[CRSF_NUM_CHANNELS + CRSF_EXTRA_CHANNELS];
extern CRSFRouter crsfRouter;
extern EspNowMaster espNow;
extern bool webserverPreventAutoStart;

extern void servoNewChannelsAvailable();

static std::array<uint8_t, 6> broadcastAddress = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if defined(WMESPNOW_RECV)
#if defined(PLATFORM_ESP32)
static std::array<uint8_t, 6> peerAddress = {0xA4, 0xF0, 0x0F, 0x23, 0xF4, 0x50};
static std::array<uint8_t, 6> ownAddress  = {0x4C, 0x75, 0x25, 0xA9, 0x0B, 0x94};
#endif
#if defined(PLATFORM_ESP8266)
static std::array<uint8_t, 6> peerAddress = {0xA4, 0xF0, 0x0F, 0x23, 0xF4, 0x50};
static std::array<uint8_t, 6> ownAddress  = {0x24, 0xA1, 0x60, 0x1F, 0x19, 0x2E};
#endif
#else
// static std::array<uint8_t, 6> peerAddress  = {0x4C, 0x75, 0x25, 0xA9, 0x0B, 0x94};
static std::array<uint8_t, 6> peerAddress = {0x24, 0xA1, 0x60, 0x1F, 0x19, 0x2E}; // esp8266
static std::array<uint8_t, 6> ownAddress   = {0xA4, 0xF0, 0x0F, 0x23, 0xF4, 0x50};
#endif

namespace {
    std::array<volatile uint8_t, 64 + 6 - 3 + 1> recv_buffer;
    volatile bool recv_buffer_ready = false;
    volatile uint32_t recv_millis = 0;
    void onDataRecvGeneric(const uint8_t* mac_addr, const uint8_t* data, const uint8_t len){
        if (recv_buffer_ready) {
            return;
        }
        if ((len >= 6) && (len <= (64 - 3 + 6))) {
            recv_buffer[0] = len;
            std::memcpy((uint8_t*)&recv_buffer[1], data, len);
            recv_millis = millis();
            recv_buffer_ready = true;
        }
    }
#if defined(PLATFORM_ESP32)
    void onDataSent(const uint8_t* const mac_addr, const esp_now_send_status_t status) {
    }
    void onDataRecv(const uint8_t* const mac_addr, const uint8_t* const data, const int len) {
        onDataRecvGeneric(mac_addr, data, len);
    }
#endif
#if defined(PLATFORM_ESP8266) 
    void onDataRecv(uint8_t* mac_addr, uint8_t* data, uint8_t len) {
        onDataRecvGeneric(mac_addr, data, len);
    }
#endif
}
#if defined(WMESPNOW_RECV)
// downlink
// slave -> master
void EspNowMaster::forwardMessage(const crsf_header_t* const message) {
    if (!isInitialized) {
        return;
    }
    const uint8_t* const data = (uint8_t *)message;
    const uint8_t length = data[CRSF_TELEMETRY_LENGTH_INDEX] + CRSF_FRAME_NOT_COUNTED_BYTES - 3; // no start, length, crc
    if (length <= (64 - 3)) {        
        if (message->type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            return;
        }
        else if (message->type == CRSF_FRAMETYPE_ELRS_STATUS) {
            return;
        }
        if (mMenuCounter > 0) {
            if (message->type < CRSF_FRAMETYPE_DEVICE_PING) {
                return;
            }
        }
        DBGLN("E fwd dst: %u src: %u, type: %u", message->payload[0], message->payload[1], message->type);                    
        std::memcpy(&outBuffer[0], config.GetUID(), 6);
        std::memcpy(&outBuffer[6], &data[2], length);
        const auto result = esp_now_send(&peerAddress[0], (uint8_t*)&outBuffer[0], length + 6);
#if defined(PLATFORM_ESP32)
        if (result != ESP_OK) {
            DBGLN("EspNow send NOK: %u", result);        
        }
#endif
#if defined(PLATFORM_ESP8266)
        if (result) {
            DBGLN("EspNow send NOK: %u", result);        
        }
#endif
    }
}
// master -> slave
void EspNowMaster::processBytes(const uint8_t* const data, const int len) {
    if (!isInitialized) {
        return;
    }
    if (len < 6) {
        return;
    }
    if (len > (64 - 3 + 6)) {
        return;
    }
    // DBGLN("EspNow process bytes");        
    
    if (std::memcmp(data, config.GetUID(), 6) == 0) {        
        mConnectCounter = 2000 / ESPNOW_TIMEOUT_MS; // 2000ms timeout

        // len = 6 (uid) + total - 3 = total + 3
        // total = len + 3 - 6 = len - 3
        // crsfLen = total - 2 = len - 5 = length including crc
        
        const uint8_t crsfLen = len - 5;
        std::memcpy(&inBuffer[2], &data[6], len - 6);
        inBuffer[0] = 0xc8;
        inBuffer[1] = crsfLen;
        const uint8_t crc = crsfRouter.crsf_crc.calc(&inBuffer[2], crsfLen - 1);
        inBuffer[crsfLen + 2 - 1] = crc;
        
        const crsf_header_t* const header = (crsf_header_t *)&inBuffer[0];

        if (header->type >= CRSF_FRAMETYPE_DEVICE_PING) {
            if (header->type < CRSF_FRAMETYPE_ELRS_STATUS) {
                mMenuCounter = 10000 / ESPNOW_TIMEOUT_MS;
                DBGLN("E menu start");
            }
        }
        if (header->type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            // DBGLN("E proc LK disc");
            return;
        }
        else if (header->type == CRSF_FRAMETYPE_ELRS_STATUS) {
            // DBGLN("E proc ES disc");
            return;
        }
        if (header->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            // DBGLN("rc channels");
            if (mChRecvCount > 10) {
                rcPacketToChannelsData(header);
                servoNewChannelsAvailable();
            }
            else {
                ++mChRecvCount;
            }
        }
        else {
            DBGLN("E proc dst: %u src: %u, type: %u, rc: %u", header->payload[0], header->payload[1], header->type, mChRecvCount);
            crsfRouter.processMessage(this, header);
        }
    }
}
#else
// uplink
// master -> slave
void EspNowMaster::forwardMessage(const crsf_header_t* const message) {
    if (!isInitialized) {
        return;
    }
    const uint8_t* const data = (uint8_t *)message;
    const uint8_t length = data[CRSF_TELEMETRY_LENGTH_INDEX] + CRSF_FRAME_NOT_COUNTED_BYTES - 3; // no start, length, crc
    if (length <= (64 - 3)) { 
        if (message->type >= CRSF_FRAMETYPE_DEVICE_PING) {
            if (message->type < CRSF_FRAMETYPE_ELRS_STATUS) {
                mMenuCounter = 10000 / ESPNOW_TIMEOUT_MS;
                DBGLN("E menu start");
            }
            DBGLN("E fwd dst: %u src: %u, type: %u, count: %u", message->payload[0], message->payload[1], message->type, mChSendCount);            
        }
        else { // type < CRSF_FRAMETYPE_DEVICE_PING
            // send no telemetry except channels
            if (!((message->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) || 
                  (message->type == CRSF_FRAMETYPE_RC_CHANNELS_EXTENDED))) {
                DBGLN("E fwd telem disc");
                return; 
            }
        }
        std::memcpy(&outBuffer[0], config.GetUID(), 6);
        std::memcpy(&outBuffer[6], &data[2], length);
        const auto result = esp_now_send(&peerAddress[0], (uint8_t*)&outBuffer[0], length + 6);
#if defined(PLATFORM_ESP32)
        if (result != ESP_OK) {
            DBGLN("EspNow send NOK: %u", result);        
        }
#endif
#if defined(PLATFORM_ESP8266)
        if (result) {
            DBGLN("EspNow send NOK: %u", result);        
        }
#endif
    }
}
// slave -> master
void EspNowMaster::processBytes(const uint8_t* const data, const int len) {
    if (!isInitialized) {
        return;
    }
    if (len < 6) {
        return;
    }
    // DBGLN("EspNow process bytes");        
    
    if (std::memcmp(data, config.GetUID(), 6) == 0) {        
        // len = 6 (uid) + total - 3 = total + 3
        // total = len + 3 - 6 = len - 3
        // crsfLen = total - 2 = len - 5 = length including crc
        
        const uint8_t crsfLen = len - 5;
        std::memcpy(&inBuffer[2], &data[6], len - 6);
        inBuffer[0] = 0xc8;
        inBuffer[1] = crsfLen;
        const uint8_t crc = crsfRouter.crsf_crc.calc(&inBuffer[2], crsfLen - 1);
        inBuffer[crsfLen + 2 - 1] = crc;
        
        const crsf_header_t* const header = (crsf_header_t *)&inBuffer[0];

        if (header->type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            DBGLN("E proc LK disc");
            return;
        }
        else if (header->type == CRSF_FRAMETYPE_ELRS_STATUS) {
            DBGLN("E proc ES disc");
            return;
        }
        DBGLN("E proc dst: %u src: %u, type: %u, size: %u, s: %u, count: %u, m: %u, rm: %u", header->payload[0], header->payload[1], header->type, header->frame_size, crsfLen-1, mChRecvCount, millis(), recv_millis);
        crsfRouter.processMessage(this, header);
    }
}
#endif
#if defined(WMESPNOW_RECV)
void EspNowMaster::rcPacketToChannelsData(const crsf_header_t* const message) {
    const auto payload = (uint8_t *)message + sizeof(crsf_header_t);
    constexpr unsigned srcBits = 11;
    constexpr unsigned dstBits = 11;
    constexpr unsigned inputChannelMask = (1 << srcBits) - 1;
    constexpr unsigned precisionShift = dstBits - srcBits;

    uint32_t localChannelData[CRSF_NUM_CHANNELS];

    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    unsigned readByteIndex = 0;
    for (uint32_t & n : localChannelData){
        while (bitsMerged < srcBits){
            const uint8_t readByte = payload[readByteIndex++];
            readValue |= ((uint32_t) readByte) << bitsMerged;
            bitsMerged += 8;
        }
        //printf("rv=%x(%x) bm=%u\n", readValue, (readValue & inputChannelMask), bitsMerged);
        n = (readValue & inputChannelMask) << precisionShift;
        readValue >>= srcBits;
        bitsMerged -= srcBits;
    }
    for(uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        ChannelData[i] = localChannelData[i];
    }
    bool armCmd;
    if (message->frame_size == CRSF_FRAME_SIZE(sizeof(crsf_channels_t))){
        armCmd = CRSF_to_BIT(localChannelData[AUX1]);       // no status byte present, us CH5 to arm
    }
    else{
        const uint8_t status = payload[readByteIndex];
        if (status & CRSF_CHANNELS_STATUS_ARMING_MODE_CH5){
            armCmd = CRSF_to_BIT(localChannelData[AUX1]);   // status byte present and Arm using CH5 selected
        }
        else{
            armCmd = status & CRSF_CHANNELS_STATUS_ARMED;   // status byte present and Arm using Switch selected
        }
    }

    // monitoring arming state
    if (isArmed != armCmd){
        isArmed = armCmd;
#if defined(PLATFORM_ESP32)
        devicesTriggerEvent(EVENT_ARM_FLAG_CHANGED);
#endif
    }
}
#endif
void EspNowMaster::stop() {
    DBGLN("EspNow stop");
    isInitialized = false;
#if defined(PLATFORM_ESP32)
    if (esp_now_deinit() != ESP_OK) {
        DBGLN("esp_now_deinit failed");
        return;
    }
#endif
#if defined(PLATFORM_ESP8266)
    if (!esp_now_deinit()) {
        DBGLN("esp_now_deinit failed");
        return;
    }
#endif
}
void EspNowMaster::start() {
    if (isInitialized) {
        return;
    }
    DBGLN("EspNow start");
    
#if defined(PLATFORM_ESP8266)
    WiFi.forceSleepWake();
    delay(10);
#endif
    
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false); //*
    WiFi.disconnect(); // ensure STA only, no AP
    WiFi.mode(WIFI_OFF);
    
    WiFi.mode(WIFI_STA);
    
    const String a = WiFi.macAddress();
    DBG("Mac: %s", a.c_str());
    
#if defined(PLATFORM_ESP32)
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); //*
    
    if (esp_now_init() != ESP_OK) {
        DBGLN("esp_now_init failed");
        return;
    }
    
    if (esp_wifi_set_max_tx_power(80) != ESP_OK) {
        DBGLN("esp_now set power failed");
        // return;        
    }
            
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    // memcpy(peerInfo.peer_addr, &broadcastAddress[0], 6);
    memcpy(peerInfo.peer_addr, &peerAddress[0], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peerInfo);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        DBGLN("esp_now_add_peer failed %d", err);
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
#endif
#if defined(PLATFORM_ESP8266)
    if (!esp_now_init()) {
        DBGLN("esp_now_init failed");
        // return;        
    }
    if (!esp_now_set_self_role(ESP_NOW_ROLE_COMBO)) {
        DBGLN("esp_set role failed");
        // return;                
    }
    WiFi.setOutputPower(20.5);

    const auto err = esp_now_add_peer(&peerAddress[0], ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    if (err != 0) {
        DBGLN("esp_now_add_peer failed %d", err);
        return;
    }
    esp_now_register_recv_cb(onDataRecv);    
#endif
    setConnectionState(tentative);
    isInitialized = true;    
}
void EspNowMaster::sendHeartbeat(){
    CRSF_MK_FRAME_T(uint16_t) rcPacket = {0};
    rcPacket.p = CRSF_ADDRESS_CRSF_RECEIVER;
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&rcPacket, CRSF_FRAMETYPE_HEARTBEAT, sizeof(rcPacket) - 2);
    espNow.forwardMessage((crsf_header_t*)&rcPacket);
}
#if !defined(WMESPNOW_RECV)
void EspNowMaster::sendRCFrame(){
    CRSF_MK_FRAME_T(crsf_channels_t) rcPacket = {0};
    constexpr unsigned dstBits = 11;
    uint8_t* dst = reinterpret_cast<uint8_t *>(&rcPacket.p);
    uint32_t accumulator = 0;
    unsigned bitCnt = 0;
    
    for (unsigned ch = 0; ch < CRSF_NUM_CHANNELS; ++ch){
        uint32_t val = ChannelData[ch] & ((1u << dstBits) - 1);
        accumulator |= (val << bitCnt);
        bitCnt += dstBits;
        while (bitCnt >= 8){
            *dst++ = (uint8_t)accumulator;
            accumulator >>= 8;
            bitCnt -= 8;
        }
    }
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&rcPacket, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, sizeof(rcPacket) - 2);
    forwardMessage((crsf_header_t*)&rcPacket);    
}
#endif
void EspNowMaster::tick() {
    static uint16_t counter = 0;
    if (++counter > 100) {
        counter = 0;
        DBGLN("E timeout s: %u", connectionState);
    } 
    if (mConnectCounter > 0) {
        --mConnectCounter;
        webserverPreventAutoStart = true;
        if (mChRecvCount > 10) {
            if (connectionState != connected) {
                connectionHasModelMatch = true;
                setConnectionState(connected);
            }
        }
    }    
    else {
        mChRecvCount = 0;
        webserverPreventAutoStart = false;
        if (connectionState != disconnected) {
            setConnectionState(disconnected);
        }
    }
}
bool EspNowMaster::menuMode(){
    if (mMenuCounter > 0) {
        --mMenuCounter;
        return true;
    }
    return false;
}
#if defined(WMESPNOW_RECV)
// slave
void EspNowMaster::timeout() {
    if (!isInitialized) {
        return;
    }
    if (recv_buffer_ready && (millis() >= (recv_millis + ESPNOW_TIMEOUT_MS))) {
        processBytes((uint8_t*)&recv_buffer[1], recv_buffer[0]);
        recv_buffer_ready = false;
    }    
    tick();
    menuMode();
}    
#else
// master
void EspNowMaster::timeout() {
    if (!isInitialized) {
        return;
    }
    static uint8_t counter = 0;
    if (recv_buffer_ready && (millis() >= (recv_millis + ESPNOW_TIMEOUT_MS))) {
        processBytes((uint8_t*)&recv_buffer[1], recv_buffer[0]);
        recv_buffer_ready = false;
    }    
    const bool inMenuMode = menuMode();
    
    if (++counter >= (ESPNOW_CHANNELS_MS / ESPNOW_TIMEOUT_MS)) {
        counter = 0;
        return;
    }
    if (inMenuMode) {
        sendHeartbeat();
    }
    else {
        if (connectionState == connected) {
            sendRCFrame();
        }
    }
}
#endif

namespace {
    bool initialize(){
        return true;
    }
    int start(){
        return DURATION_IMMEDIATELY;
    }
    int event(){
        return DURATION_IGNORE;
    }
    int timeout(){
        espNow.timeout();
        return ESPNOW_TIMEOUT_MS;
    }
}

device_t ESPNOWSender_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_NONE
};
#endif
