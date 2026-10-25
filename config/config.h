#pragma once

#include <cstddef>
#include <cstdint>


namespace cameraConfig
{
    inline constexpr size_t WIDTH  = 320;
    inline constexpr size_t HEIGHT = 320;
    inline constexpr size_t PIXEL_NUM = 320*320;

    // Longest Evt21Streamer::getBuffer() waits for a camera buffer before
    // returning nullptr. Long enough not to trip in normal operation, short
    // enough to notice a camera that has stopped delivering.
    inline constexpr int BUFFER_TIMEOUT_MS = 500;
}

enum class evt21_type : uint8_t
{
    TIME_HIGH = 0x8,
    NEGATIVE = 0x0,
    POSITIVE = 0x1
};