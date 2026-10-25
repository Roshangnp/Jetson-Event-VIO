#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include "types.h"
#include "histogram_buffer.h"

class histograms{
    public:
    static constexpr size_t HISTOGRAM_BUFFER_COUNT = 5;
    static constexpr size_t HISTOGRAM_TIME_HIGH_WIDTH = 40;
    using histogram_buffer_array = std::array<histogram_buffer,HISTOGRAM_BUFFER_COUNT>;
    using histogram_buffer_pair = std::array<histogram_buffer*,2>;

    // recieves a evt21_buffer from evt21Streamer (of evt21Streamer_fromFile) and loops through
    // all valid words of the buffer and calls updateHistogram belonging to each histogram_buffer
    // it also handles the resting of each buffer once the difference between timeHigh_start and timeHigh_current
    // exeeds HISTOGRAM_TIME_HIGH_WIDTH. Once each each buffer is reset_() == true, the buffer is assigned a timeHigh_start
    void updateHistograms(evt21_buffer buffer, size_t word_count);

    // returns the pointers to two adjacent histogram_buffers.
    histogram_buffer_pair getReadyHistogramPair();

    private:
    evt21_word current_timeHigh;
    histogram_buffer_array histogram_buffers_{};
    size_t index1;
    size_t index2;
};