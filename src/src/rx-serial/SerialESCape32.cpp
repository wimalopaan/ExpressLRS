#include "SerialESCape32.h"
#include "common.h"
#include "config.h"
#include "device.h"

#if defined(WMEXTENSION) && defined(WMESCAPE32) && defined(PLATFORM_ESP32) && defined(TARGET_RX)

volatile bool reconfigureSerials = false;

extern std::array<SerialEvent, 2> serial_events;

void serialEvent(const SerialEvent e) {
    DBGLN("SerialEvent: %u", (uint8_t)e);
    for(SerialEvent& serial_evt: serial_events) {
        serial_evt = SerialEvent((uint8_t)serial_evt | (uint8_t)e);
    }
}
template<typename F>
static void onSerialEvent(SerialEvent& serial_evt, const SerialEvent e, F f) {
    if ((uint8_t)serial_evt & (uint8_t)e) {
        f();
        serial_evt = SerialEvent((uint8_t)serial_evt & ~((uint8_t)e));        
    }
}
SerialESCape32::SerialESCape32(Stream &out, Stream &in, ESCape32Status& status, SerialEvent& event, const uint8_t telem_id) : 
    SerialIO{&out, &in}, mParser{status}, mBuffer{new ESCape32Buffer<>}, mEvent{event}, mTelemID{telem_id} {
    DBGLN("SerialESCape32");
    mEvent = SerialEvent::None;
}
SerialESCape32::~SerialESCape32() {
    delete mBuffer;
    mEvent = SerialEvent::None;
}
uint32_t SerialESCape32::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) {
    // DBGLN("SerialESCape32::sendRCFrame");
    const auto oldState = mState;
    ++mStateCounter;
    switch(mState) {
    case State::Start:
        mState = State::AsciiPreIdle;
        mChangeInputTelemMode = false;
        break;
    case State::Idle:
        mState = State::Probe;
        onSerialEvent(mEvent, SerialEvent::WriteBootloader, [&]{
            mMultiBlockState.reset();
            mState = State::ResetWriteProtection;
        });
        onSerialEvent(mEvent, SerialEvent::WriteFirmware, [&]{
            mMultiBlockState.reset();
            mState = State::EraseSignature;
        });
        break;
    case State::Probe:
        if (mParser) {
            mState = State::Info;
        }
        else {
            if (mGoodCounter > 0) {
                --mGoodCounter;
            }
            else {
                mParser.escape32_status.clear();           
            }
            mState = State::Idle;
        }
        break;
    case State::Info:
        if (mParser) {
            mState = State::Read;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::Read:
        if (mParser) {
            mGoodCounter = 3;
            mState = State::Idle;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::ResetWriteProtection:
        mState = State::ResetWriteProtectionCheck;
        break;
    case State::ResetWriteProtectionCheck:
        if (mParser) {
            mParser.escape32_status.update = "Unlock Bootloader";                
            mState = State::WriteBootloader;
        }
        else {
            mParser.escape32_status.update = "Unlock Bootloader failed";                
            mState = State::Idle;
        }
        break;
    case State::WriteBootloader:
        mState = State::WriteBootloaderCheck;
        break;
    case State::WriteBootloaderCheck:
        if (mParser && firmwareBuffer) {
            mMultiBlockState.mPosition += 1024;
            if (mMultiBlockState.mPosition >= firmwareBuffer->length) {
                mMultiBlockState.reset();                
                mParser.escape32_status.update = "Wrote Bootloader";                
                mState = State::SetWriteProtection;
            }
            else {
                mState = State::WriteBootloader;
            }
        }
        else {
            mParser.escape32_status.update = "Bootloader failed";                
            mState = State::Idle;
        }
        break;
    case State::SetWriteProtection:
        mState = State::SetWriteProtectionCheck;
        break;
    case State::SetWriteProtectionCheck:
        if (mParser) {
            mParser.escape32_status.update = "Writeprotect Bootloader";                
            mState = State::Idle;
        }
        else {
            mParser.escape32_status.update = "Writeprotect Bootloader failed";                
            mState = State::Idle;
        }
        break;
    case State::EraseSignature:
        mState = State::EraseSignatureCheck;
        break;
    case State::EraseSignatureCheck:
        if (mParser) {
            if (++mMultiBlockState.mNumber >= 2) {
                mMultiBlockState.reset();
                mMultiBlockState.mPosition = 2048;
                mState = State::WriteFirmware;
                mParser.escape32_status.update = "Signature erased";                
            }
            else {
                mState = State::EraseSignature;
            }
        }
        else {
            mParser.escape32_status.update = "Signature erase failed";                
            mState = State::Idle;
        }
        break;
    case State::WriteFirmware:
        mState = State::WriteFirmwareCheck;
        break;
    case State::WriteFirmwareCheck:
        if (mParser && firmwareBuffer) {
            mMultiBlockState.mPosition += 1024;
            if (mMultiBlockState.mPosition >= firmwareBuffer->length) {
                mState = State::WriteSignature;
                mMultiBlockState.reset();                
                mParser.escape32_status.update = "Wrote Firmware";                
            }
            else {
                mState = State::WriteFirmware;
            }
        }
        else {
            mParser.escape32_status.update = "Firmware failed";                
            mState = State::Idle;
        }
        break;        
    case State::WriteSignature:
        mState = State::WriteSignatureCheck;
        break;
    case State::WriteSignatureCheck:
        if (mParser) {
            mMultiBlockState.mPosition += 1024;
            if (mMultiBlockState.mPosition >= 2048) {
                mMultiBlockState.reset();                
                mParser.escape32_status.update = "Wrote Signature";                
                mChangeInputTelemMode = true;
                mState = State::AsciiPreIdle;
            }
            else {
                mState = State::WriteSignature;
            }
        }
        else {
            mParser.escape32_status.update = "Write Signature failed";                
            mState = State::Idle;
        }
        break;
    case State::AsciiPreIdle:
        mState = State::AsciiPreIdleWait;
        break;
    case State::AsciiPreIdleWait:
        if (mStateCounter >= PreIdleWaitCount) {
            mState = State::AsciiInfo;
        }
        break;
    case State::AsciiInfo:
        mState = State::AsciiInfoCkeck;
        break;
    case State::AsciiInfoCkeck:
        if (mParser) {
            mState = State::AsciiGetInputMode;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::AsciiGetInputMode:
        mState = State::AsciiGetInputModeCheck;
        break;
    case State::AsciiGetInputModeCheck:
        if (mParser) {
            mState = State::AsciiGetTelemMode;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::AsciiGetTelemMode:
        mState = State::AsciiGetTelemModeCheck;
        break;
    case State::AsciiGetTelemModeCheck:
        if (mParser) {
            mState = State::AsciiGetTelemID;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::AsciiGetTelemID:
        mState = State::AsciiGetTelemIDCheck;
        break;
    case State::AsciiGetTelemIDCheck:
        if (mParser) {
            if (mChangeInputTelemMode) {
                mState = State::SetCrsfInputMode;
            }
            else {
                mState = State::Idle;
            }
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::SetCrsfInputMode:
        mState = State::SetCrsfInputModeCheck;
        break;
    case State::SetCrsfInputModeCheck:
        if (mParser) {
            mState = State::SetCrsfTelemMode;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::SetCrsfTelemMode:
        mState = State::SetCrsfTelemModeCheck;
        break;
    case State::SetCrsfTelemModeCheck:
        if (mParser) {
            mState = State::SetCrsfTelemID;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::SetCrsfTelemID:
        mState = State::SetCrsfTelemIDCheck;
        break;
    case State::SetCrsfTelemIDCheck:
        if (mParser) {
            mState = State::AsciiSave;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::AsciiSave:
        mState = State::AsciiSaveCheck;
        break;
    case State::AsciiSaveCheck:
        if (mParser) {
            mState = State::Idle;
            mGoodCounter = 10;
        }
        else {
            mState = State::AsciiPreIdle;
        }
        break;
    }
    if (oldState != mState) {
        mStateCounter = 0;
        switch(mState) {
        case State::Start:
            break;
        case State::Idle:
            break;
        case State::Probe:
            probe();
            break;
        case State::Info:
            info();
            break;
        case State::Read:
            read();
            break;
        case State::ResetWriteProtection:
            setWriteProtection(0x33); // all off
            break;
        case State::ResetWriteProtectionCheck:
            break;
        case State::WriteBootloader:
            sendBootloader();
            break;        
        case State::WriteBootloaderCheck:
            break;        
        case State::SetWriteProtection:
            setWriteProtection(0x44); // protect bootloader
            break;
        case State::SetWriteProtectionCheck:
            break;
        case State::EraseSignature:
            eraseSignature();
            break;
        case State::EraseSignatureCheck:
            break;
        case State::WriteFirmware:
            sendFirmware();
            break;        
        case State::WriteFirmwareCheck:
            break;
        case State::WriteSignature:
            sendFirmware();
            break;
        case State::WriteSignatureCheck:
            break;
        case State::AsciiPreIdle:
            break;
        case State::AsciiPreIdleWait:
            break;
        case State::AsciiInfo:
            asciiInfo();
            break;
        case State::AsciiInfoCkeck:
            break;
        case State::AsciiGetInputMode:
            getInputSetting();
            break;
        case State::AsciiGetInputModeCheck:
            break;
        case State::AsciiGetTelemMode:
            getTelemSetting();
            break;
        case State::AsciiGetTelemModeCheck:
            break;
        case State::AsciiGetTelemID:
            getTelemID();
            break;
        case State::AsciiGetTelemIDCheck:
            break;
        case State::SetCrsfInputMode:
            setCrsfInputMode();
            break;
        case State::SetCrsfInputModeCheck:
            break;
        case State::SetCrsfTelemMode:
            setCrsfTelemMode();
            break;
        case State::SetCrsfTelemModeCheck:
            break;
        case State::SetCrsfTelemID:
            setCrsfTelemID();
            break;
        case State::SetCrsfTelemIDCheck:
            break;
        case State::AsciiSave:
            save();
            break;
        case State::AsciiSaveCheck:
            break;
        }        
    }
    return ESCAPE32_CALLBACK_INTERVAL_MS;
}

void SerialESCape32::send() {
    if (mBuffer) {
        mSkipFirstReceivedByte = true; // we allways receie a meaningless first byte
        _outputPort->write(mBuffer->data(), mBuffer->length());        
#if !defined(WMESCAPE32_USE_SIMULTANEOUSLY)
        _outputPort->flush();
#endif
    }
}
void SerialESCape32::send(const String& s) {
    mSkipFirstReceivedByte = true; // we allways receie a meaningless first byte
    _outputPort->write(s.c_str(), s.length());        
#if !defined(WMESCAPE32_USE_SIMULTANEOUSLY)
    _outputPort->flush();    
#endif
}
void SerialESCape32::probe() {
    DBGLN("SerialESCape32::probe");   
    mBuffer->clear();
    (*mBuffer) += CMD_PROBE;
    mParser.set(Parser::State::Probe);
    send();
}
void SerialESCape32::info() {
    // DBGLN("SerialESCape32::info");   
    mBuffer->clear();
    (*mBuffer) += CMD_INFO;
    mParser.set(Parser::State::Info);
    send();
}
void SerialESCape32::read() {
    // DBGLN("SerialESCape32::read");   
    uint8_t start   = 0x00;
    uint8_t len     = 0x04;
    mBuffer->clear();
    (*mBuffer) += CMD_READ;
    (*mBuffer) += start;
    (*mBuffer) += len;
    mParser.set(Parser::State::Read);    
    send();
}
void SerialESCape32::setWriteProtection(const uint8_t wrp) {
    DBGLN("SerialESCape32::set write protection: %u", wrp);   
    mBuffer->clear();
    (*mBuffer) += CMD_SETWRP;
    (*mBuffer) += wrp;
    mParser.set(Parser::State::Probe);
    send();    
}
void SerialESCape32::sendBootloader(){
    if (firmwareBuffer && firmwareBuffer->isBootloader) {
        DBGLN("SerialESCape32::sendBoot: %s, p: %u", &firmwareBuffer->name[0], mMultiBlockState.mPosition);       
        mBuffer->clear();
        if (mMultiBlockState.mPosition == 0) {
            (*mBuffer) += CMD_UPDATE;
        }
        const uint32_t length = std::min((firmwareBuffer->length - mMultiBlockState.mPosition), 1024U);
        (*mBuffer) += std::span{&firmwareBuffer->data[mMultiBlockState.mPosition], length};
        mParser.set(Parser::State::WriteBootloader);
        send();
    }
    else {
        DBGLN("SerialESCape32::sendBoot NO BOOTLOADER");       
    }
}

void SerialESCape32::eraseSignature() {
    if (firmwareBuffer && !firmwareBuffer->isBootloader) {
        DBGLN("SerialESCape32::eraseSig: %s, p: %u", &firmwareBuffer->name[0], mMultiBlockState.mPosition);       
        mBuffer->clear();
        (*mBuffer) += CMD_WRITE;
        (*mBuffer) += mMultiBlockState.mNumber;
        std::array<uint8_t, 8> data;
        std::fill(std::begin(data), std::end(data), 0xff);
        (*mBuffer) += data;
        mParser.set(Parser::State::EraseWriteFirmware);
        send();
    }
    else {
        DBGLN("SerialESCape32::eraseSig NO FIRMWARE");       
    }    
}

void SerialESCape32::sendFirmware(){
    if (firmwareBuffer && !firmwareBuffer->isBootloader) {
        mBuffer->clear();
        (*mBuffer) += CMD_WRITE;
        (*mBuffer) += mMultiBlockState.mPosition / 1024;
        const uint32_t length = std::min((firmwareBuffer->length - mMultiBlockState.mPosition), 1024U);
        DBGLN("SerialESCape32::sendFW: %s, p: %u, fwl: %u, l: %u", &firmwareBuffer->name[0], mMultiBlockState.mPosition, firmwareBuffer->length, length);       
        (*mBuffer) += std::span{&firmwareBuffer->data[mMultiBlockState.mPosition], length};
        mParser.set(Parser::State::EraseWriteFirmware);
        send();
    }
    else {
        DBGLN("SerialESCape32::sendFW NO FIRMWARE");       
    }
}

void SerialESCape32::asciiInfo() {
    DBGLN("SerialESCape32::asciiInfo");   
    mParser.set(Parser::State::Ascii);
    send(String{"info\n"});
}
void SerialESCape32::getInputSetting() {
    DBGLN("SerialESCape32::getInputSetting");   
    mParser.set(Parser::State::Ascii);
    send(String{"get input_mode\n"});
}
void SerialESCape32::getTelemSetting() {
    DBGLN("SerialESCape32::getTelemSetting");   
    mParser.set(Parser::State::Ascii);
    send(String{"get telem_mode\n"});
}
void SerialESCape32::getTelemID() {
    DBGLN("SerialESCape32::getTelemID");   
    mParser.set(Parser::State::Ascii);
    send(String{"get telem_phid\n"});
}
void SerialESCape32::setCrsfInputMode(){
    DBGLN("SerialESCape32::setCrsfInputMode");   
    mParser.set(Parser::State::Ascii);    
    send(String{"set input_mode 5\n"});
}
void SerialESCape32::setCrsfTelemMode(){
    DBGLN("SerialESCape32::setCrsfTelemMode");   
    mParser.set(Parser::State::Ascii);    
    send(String{"set telem_mode 4\n"});
}
void SerialESCape32::setCrsfTelemID(){
    DBGLN("SerialESCape32::setCrsfTelemID");   
    mParser.set(Parser::State::Ascii);    
    send(String{"set telem_phid "} + mTelemID + "\n");
}
void SerialESCape32::save(){
    DBGLN("SerialESCape32::save");   
    mParser.set(Parser::State::Ascii);    
    send(String{"save\n"});
}
void SerialESCape32::Parser::set(const State s){
    mState = s;
    mIndex = 0;
}
SerialESCape32::Parser::operator bool() const {
    return !(mState == State::Error);
}
void SerialESCape32::Parser::process(const uint8_t b){
    DBGLN("proc %u %u", b, mIndex);
    if (mIndex < mData.size()) {
        mData[mIndex++] = b;
    }
    switch(mState) {
    case State::Idle:
        break;
    case State::Probe:
        if (mIndex == 2) {
            probe();
        }
        break;
    case State::Info:
        if (mIndex == (2 + 32 + 4)) {
            info();
        }
        break;
    case State::Read:
        if (mIndex == (2 + 20 + 4)) {
            read();
        }
        break;
    case State::EraseWriteFirmware:
        if (mIndex == 2) {
            writeOK();
        }
        break;
    case State::WriteBootloader:
        if (mIndex == 2) {
            writeOK();
        }
        break;
    case State::Ascii:
        if (b == '\n') {
            if (mIndex == 3) {
                parseAsciiOk();
            }
            else if ((mIndex == strlen("input_mode: 0\n")) || (mIndex == strlen("telem_mode: 0\n"))) {
                parseAsciiModes();
            }
            mIndex = 0;
        }
        break;
    case State::Ok:
        break;
    case State::Error:
        break;
    }
}

void SerialESCape32::Parser::parseAsciiModes() {
    char modeString[16];
    uint32_t mode = -1;
    sscanf((const char*)&mData[0], "%15s %u", modeString, &mode);
    if (strncmp(modeString, "input_m", strlen("input_m")) == 0) {
        DBGLN("AsciiModes: input_mode: %u", mode);
        mCfgInfo.mInputMode = mode;
        switch(mode) {
        case 0:
            escape32_status.input = "Servo";
            break;
        case 1:
            escape32_status.input = "Analog";
            break;
        case 2:
            escape32_status.input = "Serial";
            break;
        case 3:
            escape32_status.input = "IBus";
            break;
        case 4:
            escape32_status.input = "Sbus(2)";
            break;
        case 5:
            escape32_status.input = "CRSF";
            break;
        case 6:
            escape32_status.input = "EXBus";
            break;
        case 7:
            escape32_status.input = "Hott(SumDV3)";
            break;
        default:
            escape32_status.input = "Unknown";
            break;
        }
    }
    else if (strncmp(modeString, "telem_m", strlen("telem_m")) == 0) {
        DBGLN("AsciiModes: telem_mode: %u", mode);
        mCfgInfo.mTelemMode = mode;
        switch(mode) {
        case 0:
            escape32_status.telem = "Kiss";
            break;
        case 1:
            escape32_status.telem = "Kiss(auto)";
            break;
        case 2:
            escape32_status.telem = "IBus";
            break;
        case 3:
            escape32_status.telem = "S.Port";
            break;
        case 4:
            escape32_status.telem = "CRSF";
            break;
        case 5:
            escape32_status.telem = "MSB";
            break;
        case 6:
            escape32_status.telem = "Hott(Telem)";
            break;
        default:
            escape32_status.telem = "unknown";
            break;
        }
    }
    else if (strncmp(modeString, "telem_phid", strlen("telem_phid")) == 0) {
        DBGLN("AsciiModes: telem_phid: %u", mode);
        mCfgInfo.mTelemID = mode;
        escape32_status.id = String{mode};
    }
}

void SerialESCape32::Parser::parseAsciiOk() {
    if (strncmp((char*)&mData[0], "OK\n", 3) == 0) {
        mState = State::Ok;
        return;
    }
    mState = State::Error;
}

uint8_t SerialESCape32::Parser::valueAt(const uint8_t index) {
    return ((mData[index] ^ mData[index + 1]) == 0xff) ? mData[index] : 0xff;
}
void SerialESCape32::Parser::probe() {
    if (valueAt(0) == 0x00) {
        mState = State::Ok;
        escape32_status.actual = "Connected";
        DBGLN("probe ok");
    }
    else {
        DBGLN("probe error");
        mState = State::Error;
    }
}
void SerialESCape32::Parser::writeOK() {
    if (valueAt(0) == 0x00) {
        mState = State::Ok;
        escape32_status.actual = "Write successful";
    }
    else {
        mState = State::Error;
        escape32_status.actual = "Write failed";
    }
}
bool SerialESCape32::Parser::checkData(const uint16_t payloadLength) {
    uint8_t count = valueAt(0);
    if (count == 0xff) {
        return false;
    }
    uint16_t length = (count + 1) * 4;
    if (length != payloadLength) {
        return false;
    }
    uint32_t crc = 0;
    memcpy(&crc, &mData[2 + payloadLength + 0], sizeof(crc));
    if (esp_crc32_le(0, &mData[2], length) != crc) {
        return false;
    }
    return true;
}

void SerialESCape32::Parser::info() {
    DBGLN("Info");
    if (checkData(32)) {
        mBLInfo.mRevision = mData[2];
        mBLInfo.mPin = mData[3];
        mBLInfo.mMcu = mData[4] | (mData[5] << 8) | (mData[6] << 16) | (mData[7] << 24);
        DBGLN("BL Rev: %u, IO: %u, MCU: %u", mBLInfo.mRevision, mBLInfo.mPin, mBLInfo.mMcu);
        escape32_status.bootloader = String{"R"} + mBLInfo.mRevision;
        mState = State::Ok;
        escape32_status.actual = "Read OK";
        return;
    }
    escape32_status.actual = "Read failed";
    mState = State::Error;
}
void SerialESCape32::Parser::read() {
    DBGLN("Read");
    if (checkData(20)) {
        if ((mData[2] == 0xea) && (mData[3] == 0x32)) {
            mFWInfo.mRevision = mData[4];
            mFWInfo.mPatch = mData[5];
            memcpy(&mFWInfo.mTarget[0], &mData[6], mFWInfo.mTarget.size() - 1);
            DBGLN("Read: %u %s", mFWInfo.mRevision, &mFWInfo.mTarget[0]);
            escape32_status.firmware = String{"R"} + mFWInfo.mRevision + '.' + mFWInfo.mPatch;
            escape32_status.target = String{&mFWInfo.mTarget[0]};
            mState = State::Ok;
            escape32_status.actual = "Read OK";
            return;
        }
        else {
            escape32_status.actual = "No Firmware";            
        }
    }    
    escape32_status.actual = "Read failed";
    mState = State::Error;
}

void SerialESCape32::processBytes(uint8_t* const bytes, const uint16_t size) {
    for(uint16_t i = 0; i < size; ++i) {
        if (mSkipFirstReceivedByte) {
            mSkipFirstReceivedByte = false;
        }
        else {
            mParser.process(bytes[i]);
        }
    }
}
#endif
