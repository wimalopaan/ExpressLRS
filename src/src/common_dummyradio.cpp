#include "common_dummyradio.h"
#include "logging.h"

DummyRadio::DummyRadio()
{
    // Set sensible defaults
    currFreq = 0;
    PayloadLength = 0;
    IQinverted = false;
    processingPacketRadio = SX12XX_Radio_NONE;
    transmittingRadio = SX12XX_Radio_NONE;
    strongestReceivingRadio = SX12XX_Radio_NONE;
    LastPacketRSSI = 0;
    LastPacketRSSI2 = 0;
    LastPacketSNRRaw = 0;
}

bool DummyRadio::Begin(uint32_t /*minimumFrequency*/, uint32_t /*maximumFrequency*/){
    DBGLN("DummyRadio: Begin called (RF disabled)");
    return true;
}
bool DummyRadio::Begin(){
    DBGLN("DummyRadio: Begin called (RF disabled)");
    return true;
}

void DummyRadio::End(){
}

void DummyRadio::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                        uint8_t preambleLen, uint8_t /*syncWord*/, bool InvertIQ, uint8_t _PayloadLength){
    currFreq = freq;
    PayloadLength = _PayloadLength;
    IQinverted = InvertIQ;
    // DBGLN("DummyRadio::Config(SX127x-style) freq=%u bw=%u sf=%u cr=%u preamble=%u pl=%u",
    //       (unsigned)freq, (unsigned)bw, (unsigned)sf, (unsigned)cr, (unsigned)preambleLen, (unsigned)_PayloadLength);
}
void DummyRadio::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                        uint8_t preambleLen, bool InvertIQ, uint8_t _PayloadLength){
    Config(bw, sf, cr, freq, preambleLen, (uint8_t)0, InvertIQ, _PayloadLength);
}

void DummyRadio::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                        uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength,
                        uint32_t flrcSyncWord, uint16_t flrcCrcSeed, uint8_t modulation){
    currFreq = freq;
    PayloadLength = PayloadLength;
    IQinverted = InvertIQ;
    (void)flrcSyncWord; (void)flrcCrcSeed; (void)modulation; // unused in dummy
    // DBGLN("DummyRadio::Config(SX1280-style) freq=%u bw=%u sf=%u cr=%u preamble=%u pl=%u mod=%u",
    //       (unsigned)freq, (unsigned)bw, (unsigned)sf, (unsigned)cr, (unsigned)PreambleLength, (unsigned)PayloadLength, (unsigned)modulation);
}

void DummyRadio::TXnb(uint8_t * /*data*/, bool /*sendGeminiBuffer*/, uint8_t * /*dataGemini*/, SX12XX_Radio_Number_t /*radioNumber*/){
    // Emulate immediate TX-done. In real code TXdoneCallback is expected to be invoked later (ISR-like).
    // To be closer to real behaviour, schedule TXdone via deferred execution or call directly:
    if (TXdoneCallback){
        // If you have deferExecutionMicros available and want to emulate TOA:
        // deferExecutionMicros(1000, [&](){ TXdoneCallback(); });
        // Simpler: call directly:
        TXdoneCallback();
    }
}
void DummyRadio::RXnb(){
}
void DummyRadio::TXnbISR(){
    // called by ISR implementations in drivers — just notify callback
    if (TXdoneCallback)
        TXdoneCallback();
}
