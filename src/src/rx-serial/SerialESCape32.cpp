#include "SerialESCape32.h"
#include "common.h"
#include "config.h"
#include "device.h"


ESCape32Status escape32_status;

volatile bool setupSerial1Special = false;

static SerialEvent serial_evt = SerialEvent::None;

void serialEvent(const SerialEvent e) {
    DBGLN("SerialEvent: %u", (uint8_t)e);
    serial_evt = SerialEvent((uint8_t)serial_evt | (uint8_t)e);
}

template<typename F>
void onSerialEvent(const SerialEvent e, F f) {
    if ((uint8_t)serial_evt & (uint8_t)e) {
        f();
        serial_evt = SerialEvent((uint8_t)serial_evt & ~((uint8_t)e));        
    }
}

SerialESCape32::SerialESCape32(Stream &out, Stream &in) : 
    SerialIO{&out, &in}, mBuffer{new ESCape32Buffer<>} {
    DBGLN("SerialESCape32");
}

SerialESCape32::~SerialESCape32() {
    delete mBuffer;
}
uint32_t SerialESCape32::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) {
    // DBGLN("SerialESCape32::sendRCFrame");
    const auto oldState = mState;
    switch(mState) {
    case State::Idle:
        mState = State::Probe;
        onSerialEvent(SerialEvent::WriteBootloader, [&]{
            mMultiBlockState.reset();
            mState = State::WriteBootloader;
        });
        onSerialEvent(SerialEvent::WriteFirmware, [&]{
            mMultiBlockState.reset();
            mState = State::EraseSignature;
        });
        break;
    case State::Probe:
        if (mParser) {
            mState = State::Info;
        }
        else {
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
            mState = State::Idle;
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::Show:
        break;
    case State::WriteBootloader:
        mState = State::WriteBootloaderCheck;
        break;
    case State::WriteBootloaderCheck:
        if (mParser) {
            mState = State::WriteBootloader;
        }
        else {
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
            }
            else {
                mState = State::EraseSignature;
            }
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::WriteFirmware:
        mState = State::WriteFirmwareCheck;
        break;
    case State::WriteFirmwareCheck:
        if (mParser) {
            mMultiBlockState.mPosition += 1024;
            if (mMultiBlockState.mPosition >= firmwareBuffer->length) {
                mState = State::WriteSignature;
                mMultiBlockState.reset();                
            }
            else {
                mState = State::WriteFirmware;
            }
        }
        else {
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
                mState = State::Idle;
                mMultiBlockState.reset();                
            }
            else {
                mState = State::WriteSignature;
            }
        }
        else {
            mState = State::Idle;
        }
        break;
    case State::SetCrsfInputMode:
        mState = State::Idle;
        break;
    case State::SetCrsfTelemMode:
        mState = State::Idle;
        break;
    }
    if (oldState != mState) {
        switch(mState) {
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
        case State::Show:
            show();
            break;
        case State::WriteBootloader:
            sendBootloader();
            break;        
        case State::WriteBootloaderCheck:
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
        case State::SetCrsfInputMode:
            mState = State::Idle;
            break;
        case State::SetCrsfTelemMode:
            mState = State::Idle;
            break;
        }        
    }
    
    return ESCAPE32_CALLBACK_INTERVAL_MS;
}

void SerialESCape32::send() {
    if (mBuffer) {
        _outputPort->write(mBuffer->data(), mBuffer->length());        
        _outputPort->flush();
        mSkipFirstReceivedByte = true; // we allways receie a meaningless first byte
    }
}
void SerialESCape32::probe() {
    // DBGLN("SerialESCape32::probe");   
    mBuffer->clear();
    (*mBuffer) += CMD_PROBE;
    send();
    mParser.set(Parser::State::Probe);
}
void SerialESCape32::info() {
    // DBGLN("SerialESCape32::info");   
    mBuffer->clear();
    (*mBuffer) += CMD_INFO;
    send();
    mParser.set(Parser::State::Info);
}
void SerialESCape32::read() {
    // DBGLN("SerialESCape32::read");   
    uint8_t start   = 0x00;
    uint8_t len     = 0x04;
    mBuffer->clear();
    (*mBuffer) += CMD_READ;
    (*mBuffer) += start;
    (*mBuffer) += len;
    send();
    mParser.set(Parser::State::Read);    
}

void SerialESCape32::sendBootloader(){
    if (firmwareBuffer && firmwareBuffer->isBootloader) {
        DBGLN("SerialESCape32::sendBoot: %s, p: %u", &firmwareBuffer->name[0], mMultiBlockState.mPosition);       
            
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
        send();
        mParser.set(Parser::State::EraseWriteFirmware);
    }
    else {
        DBGLN("SerialESCape32::eraseSig NO FIRMWARE");       
    }    
}

void SerialESCape32::sendFirmware(){
    if (firmwareBuffer && !firmwareBuffer->isBootloader) {
        DBGLN("SerialESCape32::sendFW: %s, p: %u, l: %u", &firmwareBuffer->name[0], mMultiBlockState.mPosition, firmwareBuffer->length);       
        mBuffer->clear();
        (*mBuffer) += CMD_WRITE;
        (*mBuffer) += mMultiBlockState.mPosition / 1024;
        (*mBuffer) += std::span{&firmwareBuffer->data[mMultiBlockState.mPosition], 1024};
        send();
        mParser.set(Parser::State::EraseWriteFirmware);
            
    }
    else {
        DBGLN("SerialESCape32::sendFW NO FIRMWARE");       
    }
}

void SerialESCape32::show() {
    DBGLN("SerialESCape32::show");   
}

void SerialESCape32::Parser::set(const State s){
    mState = s;
    mIndex = 0;
}
SerialESCape32::Parser::operator bool() const {
    return mState == State::Ok;
}
void SerialESCape32::Parser::process(const uint8_t b){
    if (mIndex < mData.size()) {
        mData[mIndex++] = b;
    }
    switch(mState) {
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
    default:
        break;
    }
}

uint8_t SerialESCape32::Parser::valueAt(const uint8_t index) {
    return ((mData[index] ^ mData[index + 1]) == 0xff) ? mData[index] : 0xff;
}
void SerialESCape32::Parser::probe() {
    if (valueAt(0) == 0x00) {
        mState = State::Ok;
        escape32_status.actual = "Connected";
    }
    else {
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
    if (checkData(32)) {
        mBLInfo.mRevision = mData[2];
        mBLInfo.mPin = mData[3];
        mBLInfo.mMcu = mData[4] | (mData[5] << 8) | (mData[6] << 16) | (mData[7] << 24);
        // DBGLN("BL Rev: %u, IO: %u, MCU: %u", mBLInfo.mRevision, mBLInfo.mPin, mBLInfo.mMcu);
        escape32_status.bootloader = String{"R"} + mBLInfo.mRevision;
        mState = State::Ok;
        escape32_status.actual = "Read OK";
        return;
    }
    escape32_status.actual = "Read failed";
    mState = State::Error;
}
void SerialESCape32::Parser::read() {
    if (checkData(20)) {
        if ((mData[2] == 0xea) && (mData[3] == 0x32)) {
            mFWInfo.mRevision = mData[4];
            mFWInfo.mPatch = mData[5];
            memcpy(&mFWInfo.mTarget[0], &mData[6], 16);
            // DBGLN("Read: %u %s", mFWInfo.mRevision, &mFWInfo.mTarget[0]);
            escape32_status.firmware = String{"R"} + mFWInfo.mRevision + '.' + mFWInfo.mPatch;
            escape32_status.target = String{&mFWInfo.mTarget[0]};
            mState = State::Ok;
            escape32_status.actual = "Read OK";
            return;
        }
    }    
    escape32_status.actual = "Read failed";
    mState = State::Error;
}

void SerialESCape32::processBytes(uint8_t* const bytes, const uint16_t size) {
    if (mSkipFirstReceivedByte) {
        mSkipFirstReceivedByte = false;
        return;
    }
    for(uint16_t i = 0; i < size; ++i) {
        mParser.process(bytes[i]);
    }
}
