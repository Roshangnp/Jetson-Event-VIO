#pragma once

#include <cstddef>
#include <cstdint>


namespace cameraConfig
{
    inline constexpr size_t WIDTH  = 320;
    inline constexpr size_t HEIGHT = 320;
    inline constexpr size_t PIXEL_NUM = 320*320;
}

enum class evt21_type : uint8_t
{
    TIME_HIGH = 0x8,
    NEGATIVE = 0x0,
    POSITIVE = 0x1
};