#pragma once

#include <array>
#include <initializer_list>

// RX radio mode selection (WMESPNOW_SLAVE), evaluated once at boot from hardware.json.
// ESP-NOW slave mode requires ALL of the following keys to be absent or -1:
//
//   SPI / radio      : radio_sck, radio_miso, radio_mosi, radio_nss,
//                      radio_dio0, radio_dio1, radio_busy, radio_rst
//   second radio     : radio_nss_2, radio_dio0_2, radio_dio1_2, radio_busy_2, radio_rst_2
//   antenna          : ant_ctrl, ant_group
//   PA / LNA         : power_enable, power_apc2, power_rxen, power_txen,
//                      power_rxen_2, power_txen_2
//
// - none set                                    -> EspNowSlave
// - SCK, MISO, MOSI, NSS, DIO1 (SX127x: DIO0)
//   complete (and NSS_2/DIO_2 both or neither)  -> Ota
// - partial configuration or a pin value < -1   -> Invalid (hardware init fails)
//
// Other pins (serial, LED, button, PWM outputs) are not considered.
// The pin list lives in EspNowConnector::selectRadioMode() (src/rx_espnow_slave.cpp).
// Mode changes take effect after a reboot.
enum class RxRadioMode
{
    Ota,
    EspNowSlave,
    Invalid
};

RxRadioMode classifyRxRadioPins(const std::array<int, 5> &primary, const std::array<int, 2> &secondary, std::initializer_list<int> optional);

// Selected once at boot, after loading hardware.json.
extern RxRadioMode rxRadioMode;

inline bool isRadioEnabled()
{
    return rxRadioMode == RxRadioMode::Ota;
}

inline bool isEspNowSlaveMode()
{
    return rxRadioMode == RxRadioMode::EspNowSlave;
}
