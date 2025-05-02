#include "SerialSUMD.h"
#include "CRSF.h"
#include "device.h"

#define SUMD_HEADER_SIZE		3														// 3 Bytes header
#define SUMD_DATA_SIZE_16CH		(16*2)													// 2 Bytes per channel
#define SUMD_CRC_SIZE			2														// 16 bit CRC
#define SUMD_FRAME_16CH_LEN		(SUMD_HEADER_SIZE+SUMD_DATA_SIZE_16CH+SUMD_CRC_SIZE)

const auto SUMD_CALLBACK_INTERVAL_MS = 10;

uint32_t SerialSUMD::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    if (!frameAvailable) {
        return DURATION_IMMEDIATELY;
    }

	  uint8_t outBuffer[SUMD_FRAME_16CH_LEN];

	  outBuffer[0] = 0xA8;		//Graupner
	  outBuffer[1] = 0x01;	  //SUMD
	  outBuffer[2] = 0x10;		//16CH

    uint16_t us = (CRSF_to_US(channelData[0]) << 3);
    outBuffer[3] = us >> 8;
    outBuffer[4] = us & 0x00ff;
    us = (CRSF_to_US(channelData[1]) << 3);
    outBuffer[5] = us >> 8;
    outBuffer[6] = us & 0x00ff;
    us = (CRSF_to_US(channelData[2]) << 3);
    outBuffer[7] = us >> 8;
    outBuffer[8] = us & 0x00ff;
    us = (CRSF_to_US(channelData[3]) << 3);
    outBuffer[9] = us >> 8;
    outBuffer[10] = us & 0x00ff;
    us = (CRSF_to_US(channelData[7]) << 3); //channel 8 mapped to 5 to move arm channel away from the aileron function
    outBuffer[11] = us >> 8;
    outBuffer[12] = us & 0x00ff;
    us = (CRSF_to_US(channelData[5]) << 3);
    outBuffer[13] = us >> 8;
    outBuffer[14] = us & 0x00ff;
    us = (CRSF_to_US(channelData[6]) << 3);
    outBuffer[15] = us >> 8;
    outBuffer[16] = us & 0x00ff;
    us = (CRSF_to_US(channelData[4]) << 3); //channel 5 mapped to 8
    outBuffer[17] = us >> 8;
    outBuffer[18] = us & 0x00ff;
    us = (CRSF_to_US(channelData[8]) << 3);
    outBuffer[19] = us >> 8;
    outBuffer[20] = us & 0x00ff;
    us = (CRSF_to_US(channelData[9]) << 3);
    outBuffer[21] = us >> 8;
    outBuffer[22] = us & 0x00ff;
    us = (CRSF_to_US(channelData[10]) << 3);
    outBuffer[23] = us >> 8;
    outBuffer[24] = us & 0x00ff;
    us = (CRSF_to_US(channelData[11]) << 3);
    outBuffer[25] = us >> 8;
    outBuffer[26] = us & 0x00ff;
    us = (CRSF_to_US(channelData[12]) << 3);
    outBuffer[27] = us >> 8;
    outBuffer[28] = us & 0x00ff;
    us = (CRSF_to_US(channelData[13]) << 3);
    outBuffer[29] = us >> 8;
    outBuffer[30] = us & 0x00ff;
    us = (CRSF_to_US(channelData[14]) << 3);
    outBuffer[31] = us >> 8;
    outBuffer[32] = us & 0x00ff;
    us = (CRSF_to_US(channelData[15]) << 3);
    outBuffer[33] = us >> 8;
    outBuffer[34] = us & 0x00ff;

	  uint16_t crc = crc2Byte.calc(outBuffer, (SUMD_HEADER_SIZE + SUMD_DATA_SIZE_16CH), 0);
	  outBuffer[35] = (uint8_t)(crc >> 8);
	  outBuffer[36] = (uint8_t)(crc & 0x00ff);

	  _outputPort->write(outBuffer, sizeof(outBuffer));

    return SUMD_CALLBACK_INTERVAL_MS;
}

#define SUMD3_HEADER_SIZE		3														// 3 Bytes header
#define SUMD3_DATA_SIZE	        (16*2)													// 2 Bytes per channel
#define SUMD3_CMD_SIZE	        4													//
#define SUMD3_CRC_SIZE			2														// 16 bit CRC
#define SUMD3_FRAME_LEN	(SUMD3_HEADER_SIZE + SUMD3_DATA_SIZE + SUMD3_CMD_SIZE + SUMD3_CRC_SIZE)

uint32_t SerialSUMD3::sendRCFrame(const bool frameAvailable, const bool frameMissed,
                                  uint32_t* const channelData)
{
    static_assert(SUMD3_FRAME_LEN == 41);

    if (!frameAvailable) {
        return DURATION_IMMEDIATELY;
    }

    uint8_t outBuffer[SUMD3_FRAME_LEN];

    outBuffer[0] = 0xA8;		//Graupner
    outBuffer[1] = 0x03;	    //SUMD
    outBuffer[2] = 18; 		//16CH

    switch(state) {
    case State::CH1_16:
        composeFrame(0x02, channelData, outBuffer);
        state = State::CH1_8_17_24;
        break;
    case State::CH1_8_17_24:
        composeFrame(0x03, channelData, outBuffer);
        state = State::CH1_8_25_32;
        break;
    case State::CH1_8_25_32:
        composeFrame(0x04, channelData, outBuffer);
        state = State::CH1_12_SW;
        break;
    case State::CH1_12_SW:
        composeFrame(0x05, channelData, outBuffer);
        state = State::CH1_16;
        break;
    }

    uint16_t crc = crc2Byte.calc(outBuffer, (SUMD3_HEADER_SIZE + SUMD3_DATA_SIZE + SUMD3_CMD_SIZE), 0);
    outBuffer[39] = (uint8_t)(crc >> 8);
    outBuffer[40] = (uint8_t)(crc & 0x00ff);

    _outputPort->write(outBuffer, sizeof(outBuffer));

    return SUMD_CALLBACK_INTERVAL_MS;
}

void SerialSUMD3::queueMSPFrameTransmission(uint8_t* data) {
    const uint8_t destAddress = data[3];
    const uint8_t srcAddress = data[4];
    const uint8_t realm = data[5];
    const uint8_t cmd = data[6];
    if ((srcAddress == 0xea) && (destAddress >= 0xc0) && (destAddress <= 0xcf)) {
        if (realm == 0xa1) { // switch
            if (cmd == 0x01) { // set
                const uint8_t swAddress = data[7];
                const uint16_t sw = data[8];
                DBGLN("SUMDV3 Set: adr: %d, v: %d", swAddress, sw);
                if ((swAddress >= minAddress) && (swAddress <= maxAddress)) {
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool on = (((sw >> i) & 0b01) > 0);
                        const uint8_t mask = (1 << i);
                        if (on) {
                            mSwitches[0] |= mask;
                        }
                        else {
                            mSwitches[0] &= ~mask;
                        }
                    }
                }
            }
            else if (cmd == 0x07) { // set4
                const uint8_t swAddress = data[7];
                const uint16_t sw = (data[8] << 8) + data[9];
                DBGLN("SUMDV3 Set4: adr: %d, v: %d", swAddress, sw);
                if ((swAddress >= minAddress) && (swAddress <= maxAddress)) {
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool on = (((sw >> (2 * i)) & 0b11) > 0);
                        const uint8_t mask = (1 << i);
                        if (on) {
                            mSwitches[0] |= mask;
                        }
                        else {
                            mSwitches[0] &= ~mask;
                        }
                    }
                }
            }
            else if (cmd == 0x08) { // set64
                const uint8_t swAddress = data[7];
                const uint8_t swGroup = (data[8] & 0x07);
                const uint16_t swSwitches = (data[9] << 8) + data[10];
                DBGLN("SUMDV3 Set64: adr: %d, grp: %d, v: %d", swAddress, swGroup, swSwitches);
                if ((swAddress >= minAddress) && (swAddress <= maxAddress)) {
                    for(uint8_t i = 0; i < 8; ++i) {
                        const uint8_t n = swGroup * 8 + i;
                        const bool on = (((swSwitches >> (2 * i)) & 0b11) > 0);
                        const uint8_t mask = (1 << i);
                        if (on) {
                            mSwitches[swGroup] |= mask;
                        }
                        else {
                            mSwitches[swGroup] &= ~mask;
                        }
                    }
                }
            }
        }
    }
}

void SerialSUMD3::composeFrame(const uint8_t fCode, const uint32_t* const channelData, uint8_t* const data) {
    uint8_t i = 3;
    if (fCode == 0x02) {
        for(uint8_t c = 0; c < 16; c++) {
            const uint16_t us = (CRSF_to_US(channelData[c]) << 3);
            data[i++] = us >> 8;
            data[i++] = us & 0x00ff;
        }
    }
    else if (fCode == 0x03) {
        for(uint8_t c = 0; c < 8; c++) {
            const uint16_t us = (CRSF_to_US(channelData[c]) << 3);
            data[i++] = us >> 8;
            data[i++] = us & 0x00ff;
        }
        for(uint8_t c = 17; c < 24; c++) {
            const uint16_t us = (CRSF_to_US(channelData[c]) << 3);
            data[i++] = us >> 8;
            data[i++] = us & 0x00ff;
        }
    }
    else if (fCode == 0x04) {
        for(uint8_t c = 0; c < 8; ++c) {
            const uint16_t us = (CRSF_to_US(channelData[c]) << 3);
            data[i++] = us >> 8;
            data[i++] = us & 0x00ff;
        }
        for(uint8_t c = 25; c < 32; ++c) {
            const uint16_t us = (CRSF_to_US(channelData[c]) << 3);
            data[i++] = us >> 8;
            data[i++] = us & 0x00ff;
        }
    }
    else if (fCode == 0x05) {
        for(uint8_t c = 0; c < 12; ++c) {
            const uint16_t us = (CRSF_to_US(channelData[c]) << 3);
            data[i++] = us >> 8;
            data[i++] = us & 0x00ff;
        }
        data[i++] = mSwitches[1];
        data[i++] = mSwitches[0];
        data[i++] = mSwitches[3];
        data[i++] = mSwitches[2];
        data[i++] = mSwitches[5];
        data[i++] = mSwitches[4];
        data[i++] = mSwitches[7];
        data[i++] = mSwitches[6];
    }
    data[i++] = fCode;
    data[i++] = 0; // res
    data[i++] = 0; // mode
    data[i++] = 0; // sub
}
