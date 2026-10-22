#pragma once


#include <cstdint>
#include "cameraConfig.h"

typedef uint32_t timestamp_us;

typedef uint64_t evt21_word;
typedef uint16_t histogram[cameraConfig::PIXEL_NUM];
typedef timestamp_us time_surface[cameraConfig::PIXEL_NUM];

struct event {
    uint16_t x;
    uint16_t y;
    timestamp_us t;
    uint8_t polarity;
};