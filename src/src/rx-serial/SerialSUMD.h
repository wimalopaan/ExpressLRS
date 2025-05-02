#include "SerialIO.h"
#include "crc.h"

#include <array>

class SerialSUMD : public SerialIO {
public:
    explicit SerialSUMD(Stream &out, Stream &in) : SerialIO(&out, &in) { crc2Byte.init(16, 0x1021); }
    virtual ~SerialSUMD() {}

    void queueLinkStatisticsPacket() override {}
    void queueMSPFrameTransmission(uint8_t* data) override {}
    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    Crc2Byte crc2Byte;
    void processBytes(uint8_t *bytes, uint16_t size) override {};
};

class SerialSUMD3 : public SerialIO {
public:
    explicit SerialSUMD3(Stream &out, Stream &in) : SerialIO(&out, &in) { crc2Byte.init(16, 0x1021); }
    virtual ~SerialSUMD3() {}

    void queueLinkStatisticsPacket() override {}
    void queueMSPFrameTransmission(uint8_t* data) override;
    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t* channelData) override;

private:
    Crc2Byte crc2Byte;
    void processBytes(uint8_t *bytes, uint16_t size) override {};

    enum class State : uint8_t {CH1_16, CH1_8_17_24, CH1_8_25_32, CH1_12_SW};

    void composeFrame(const uint8_t, const uint32_t* const channels, uint8_t* data);

    const uint8_t minAddress = 240;
    const uint8_t maxAddress = minAddress + 8 - 1;
    State state = State::CH1_16;
    std::array<uint8_t, 8> mSwitches{};
};
