#pragma once

#if defined(WMEXTENSION) && defined(WMESCAPE32) && defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "SerialIO.h"
#include "esp_crc.h"

#ifdef __cpp_lib_span
#include <span>
#else 
namespace std {
    struct span {
        uint8_t* data;
        size_t length;
    };
}
#endif

extern volatile bool setupSerial1Special;

enum class SerialEvent : uint8_t {
    None            = 0,
    WriteFirmware   = 1 << 0,
    WriteBootloader = 1 << 1
};

void serialEvent(SerialEvent e);

class SerialESCape32 : public SerialIO {
    static constexpr auto ESCAPE32_CALLBACK_INTERVAL_MS = 50;
    static constexpr auto PreIdleWait_MS = 2500;
    static constexpr auto PreIdleWaitCount = PreIdleWait_MS / ESCAPE32_CALLBACK_INTERVAL_MS;
    static constexpr uint8_t CMD_PROBE  = 0;
    static constexpr uint8_t CMD_INFO   = 1;
    static constexpr uint8_t CMD_READ   = 2;
    static constexpr uint8_t CMD_WRITE  = 3;
    static constexpr uint8_t CMD_UPDATE = 4;
    static constexpr uint8_t CMD_SETWRP = 5;
    
    template<size_t Size = (1024 + 32)>
    struct ESCape32Buffer {
        void clear() {
            mIndex = 0;
        }
        size_t length() const {
            return mIndex;
        }
        const uint8_t* data() const {
            return &mData[0];
        }
        ESCape32Buffer& operator+=(const uint8_t v) {
            push(v);
            push(~v);
            return *this;
        }
        template<size_t L>
        ESCape32Buffer& operator+=(const std::array<uint8_t, L>& data) {
            (*this) += (L / 4 - 1);
            for(const auto& d: data) {
                push(d);
            }
            const uint32_t crc = esp_crc32_le(0, &data[0], L);
            for(uint8_t i = 0; i < sizeof(crc); ++i) {
                push(((uint8_t*)&crc)[i]);
            }
            return *this;
        }
        ESCape32Buffer& operator+=(const std::span& sp) {
            (*this) += (sp.length / 4 - 1);
            for(size_t i = 0; i < sp.length; ++i) {
                push(sp.data[i]);
            }
            const uint32_t crc = esp_crc32_le(0, &sp.data[0], sp.length);
            for(uint8_t i = 0; i < sizeof(crc); ++i) {
                push(((uint8_t*)&crc)[i]);
            }
            return *this;
        }
    private:
        void push(const uint8_t v) {
            if (mIndex < Size) {
                mData[mIndex++] = v;
            }
        }
        size_t mIndex{};
        std::array<uint8_t, Size> mData{};
    };    
    enum class State : uint8_t {
        Start,
        Idle,
        Probe,
        Info,
        Read,
        SetCrsfInputMode,
        SetCrsfInputModeCheck,
        SetCrsfTelemMode,
        SetCrsfTelemModeCheck,
        WriteBootloader,
        WriteBootloaderCheck,
        EraseSignature,
        EraseSignatureCheck,
        WriteFirmware,
        WriteFirmwareCheck,
        WriteSignature,
        WriteSignatureCheck,
        AsciiPreIdle,
        AsciiPreIdleWait,
        AsciiInfo,
        AsciiInfoCkeck,
        AsciiGetInputMode,
        AsciiGetInputModeCheck,
        AsciiGetTelemMode,
        AsciiGetTelemModeCheck,
        AsciiSave,
        AsciiSaveCheck
    };
    struct Parser {
        struct BootLoaderInfo {
            uint8_t mRevision{};
            uint8_t mPin{};
            uint32_t mMcu{};
        };
        struct FirmwareInfo {
            uint8_t mRevision{};
            uint8_t mPatch{};
            std::array<char, 16> mTarget{};            
        };
        struct ConfigInfo {
            uint8_t mInputMode{};
            uint8_t mTelemMode{};
        };

        enum class State : uint8_t {Idle, Probe, Info, Read, 
                                    EraseWriteFirmware, WriteBootloader, 
                                    Ascii,
                                    Ok, Error};
        void process(const uint8_t b);
        void set(const State s);
        explicit operator bool() const;
    private:
        void probe();
        void writeOK();
        void info();
        void read();
        void parseAsciiOk();
        void parseAsciiModes();
        uint8_t valueAt(uint8_t index);
        bool checkData(uint16_t payloadLength);
        State mState = State::Idle;
        uint8_t mIndex{};
        std::array<uint8_t, 64> mData{};
        BootLoaderInfo mBLInfo;
        FirmwareInfo mFWInfo;
        ConfigInfo mCfgInfo;
    };
public:
    explicit SerialESCape32(Stream &out, Stream &in);
    ~SerialESCape32() override;
    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;
protected:
    void processBytes(uint8_t *bytes, uint16_t size) override;
private:
    void send();
    void send(const String&);
    
    void getInputSetting();
    void getTelemSetting();
    void asciiInfo();
    
    void setCrsfInputMode();
    void setCrsfTelemMode();
    
    void save();
    
    void probe();
    void info();
    void read();
    
    void sendBootloader();
    
    void eraseSignature();
    void sendFirmware();
    
    State mState = State::Start;
    uint16_t mStateCounter = 0;
    
    bool mSkipFirstReceivedByte = false;
    Parser mParser;
    ESCape32Buffer<>* mBuffer = nullptr;
    
    uint8_t mGoodCounter = 0;
    struct MultiBlockState {
        void reset() {
            mNumber = 0;
            mPosition = 0;
        }
        uint16_t mNumber{};
        uint32_t mPosition{};
    };
    MultiBlockState mMultiBlockState;
};
#endif
