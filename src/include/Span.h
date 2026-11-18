#pragma once

#include <cstddef>

#if defined(__has_include)
#if __has_include(<span>)
#include <span>
#endif
#endif

#if defined(__cpp_lib_span) && __cpp_lib_span >= 202002L
template <typename T>
using Span = std::span<T>;
#else
template <typename T>
class Span
{
public:
    constexpr Span() noexcept = default;
    constexpr Span(T *const data, const std::size_t size) noexcept : mData(data), mSize(size)
    {
    }

    constexpr T *data() const noexcept
    {
        return mData;
    }
    constexpr std::size_t size() const noexcept
    {
        return mSize;
    }
    constexpr T &operator[](const std::size_t index) const noexcept
    {
        return mData[index];
    }

private:
    T *mData = nullptr;
    std::size_t mSize = 0;
};
#endif
