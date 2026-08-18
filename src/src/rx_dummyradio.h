#pragma once

#include "SX12xxDriverCommon.h"
#include "deferred.h" // falls deferExecutionMicros gebraucht werden soll

class DummyRadio : public SX12xxDriverCommon
{
public:
    DummyRadio();

    bool Begin(uint32_t minimumFrequency, uint32_t maximumFrequency);
    void End();

    // SX127x-style overloads
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint8_t preambleLen, uint8_t syncWord, bool InvertIQ, uint8_t _PayloadLength);
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint8_t preambleLen, bool InvertIQ, uint8_t _PayloadLength);

    // SX1280 / LR1121-style overload (modulation als uint8_t)
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength,
                uint32_t flrcSyncWord, uint16_t flrcCrcSeed, uint8_t modulation);
    
    
    void SetMode(uint8_t /*mode*/, SX12XX_Radio_Number_t /*radioNumber*/) {}
    void SetFrequencyReg(uint32_t /*freq*/, SX12XX_Radio_Number_t /*radioNumber*/, bool /*doRx*/ = false) {}
    void SetOutputPower(int8_t /*power*/) {}
    void SetTxIdleMode() {}

    void TXnb(uint8_t * /*data*/, bool /*sendGeminiBuffer*/, uint8_t * /*dataGemini*/, SX12XX_Radio_Number_t /*radioNumber*/);
    void RXnb();
    void TXnbISR(); // falls irgendwo direkt aufgerufen

    uint16_t GetIrqStatus(SX12XX_Radio_Number_t /*radioNumber*/) { return 0; }
    void ClearIrqStatus(uint16_t /*irqMask*/, SX12XX_Radio_Number_t /*radioNumber*/) {}

    bool GetRxBufferAddr(SX12XX_Radio_Number_t /*radioNumber*/, uint8_t * /*rxBufferAddr*/) { return false; }
    int8_t GetRssiInst(SX12XX_Radio_Number_t /*radioNumber*/) { return 0; }
    void GetLastPacketStats() {}
    void CheckForSecondPacket() {}

    bool FrequencyErrorAvailable() const {return false;}
    bool GetFrequencyErrorbool(SX12XX_Radio_Number_t /*radioNumber*/){return false;}
    
    void startCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber){}
    
    // Update helpers used by e.g. LR1121/lr1121.cpp (stubbed)
    void BeginUpdate(SX12XX_Radio_Number_t /*radio*/, size_t /*expectedFilesize*/) {}
    void WriteUpdateBytes(const uint8_t * /*buf*/, size_t /*len*/) {}
    int EndUpdate() { return 0; }

private:
    // internal helpers if needed
};
