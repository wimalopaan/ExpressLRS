#pragma once

#include "EspNowProtocol.h"
#include "FIFO.h"

// Thread-safe queue that moves radio callbacks and routed CRSF frames into the main loop.
class EspNowEvents
{
public:
    enum class Type : uint8_t
    {
        Frame,
        Received,
        Sent
    };

    struct Event
    {
        Type type = Type::Frame;
        EspNowProtocol::Mac source{};
        uint32_t time = 0;
        bool success = false;
        uint8_t size = 0;
        std::array<uint8_t, EspNowProtocol::MaxPacketSize> data{};
    };

    void pushFrame(const uint8_t *const data, const size_t size)
    {
        push(Type::Frame, nullptr, 0, data, size);
    }

    void pushReceived(const uint8_t *const source, const uint32_t time, const uint8_t *const data, const size_t size)
    {
        uint8_t head[SourceSize + TimeSize];
        std::memcpy(head, source, SourceSize);
        std::memcpy(head + SourceSize, &time, TimeSize);
        push(Type::Received, head, sizeof(head), data, size);
    }

    void pushSent(const uint32_t time, const bool success)
    {
        uint8_t head[TimeSize + 1];
        std::memcpy(head, &time, TimeSize);
        head[TimeSize] = success;
        push(Type::Sent, head, sizeof(head), nullptr, 0);
    }

    bool pop(Event &event)
    {
        uint8_t record[MaxRecordSize];
        fifo.lock();
        const uint8_t size = fifo.size() ? fifo.pop() : 0;
        if (size)
            fifo.popBytes(record, size);
        fifo.unlock();
        if (!size)
            return false;

        event.type = static_cast<Type>(record[0]);
        const uint8_t *data = record + 1;
        size_t remaining = size - 1;
        if (event.type == Type::Received)
        {
            std::memcpy(event.source.data(), data, SourceSize);
            data += SourceSize;
            remaining -= SourceSize;
        }
        if (event.type != Type::Frame)
        {
            std::memcpy(&event.time, data, TimeSize);
            data += TimeSize;
            remaining -= TimeSize;
        }
        if (event.type == Type::Sent)
        {
            event.success = data[0];
            remaining = 0;
        }
        event.size = static_cast<uint8_t>(remaining);
        std::memcpy(event.data.data(), data, remaining);
        return true;
    }

    void clear()
    {
        fifo.lock();
        fifo.flush();
        droppedCount = 0;
        fifo.unlock();
    }

    void countDropped()
    {
        fifo.lock();
        ++droppedCount;
        fifo.unlock();
    }

    uint32_t dropped()
    {
        fifo.lock();
        const uint32_t count = droppedCount;
        fifo.unlock();
        return count;
    }

private:
    static constexpr size_t SourceSize = 6;
    static constexpr size_t TimeSize = 4;
    static constexpr size_t MaxRecordSize = 1 + SourceSize + TimeSize + EspNowProtocol::MaxPacketSize;
    // Length byte, type byte and send result; always kept free so send completions are never lost.
    static constexpr size_t SentRecordSize = 2 + TimeSize + 1;

    FIFO<2048> fifo;
    uint32_t droppedCount = 0;

    void push(const Type type, const uint8_t *const head, const size_t headSize, const uint8_t *const data, const size_t size)
    {
        const size_t recordSize = 1 + headSize + size;
        const size_t reserve = type == Type::Sent ? 0 : SentRecordSize;
        fifo.lock();
        if (size <= EspNowProtocol::MaxPacketSize && fifo.available(1 + recordSize + reserve))
        {
            fifo.push(static_cast<uint8_t>(recordSize));
            fifo.push(static_cast<uint8_t>(type));
            fifo.pushBytes(head, headSize);
            fifo.pushBytes(data, size);
        }
        else
        {
            ++droppedCount;
        }
        fifo.unlock();
    }
};
