#pragma once


#include <cstdint>
#include "config.h"

using timestamp_us = uint32_t;


using evt21_word = uint64_t;
using evt21_buffer = uint64_t*;

using histogram = uint16_t[cameraConfig::PIXEL_NUM];
using time_surface = timestamp_us[cameraConfig::PIXEL_NUM];


struct event {
    uint16_t x;
    uint16_t y;
    timestamp_us t;
    uint8_t polarity;
};