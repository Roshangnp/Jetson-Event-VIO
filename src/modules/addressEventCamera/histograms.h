#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include "types.h"
#include "histogram_buffer.h"

class histograms{
    public:
    static constexpr size_t HISTOGRAM_BUFFER_COUNT = 5;
    using histogram_buffer_array = std::array<histogram_buffer,HISTOGRAM_BUFFER_COUNT>;
    using histogram_buffer_pair = std::array<histogram_buffer*,2>;

    void updateHistograms(evt21_buffer buffer);
    histogram_buffer_pair getReadyHistogramPair();

    private:
    evt21_word current_timeHigh;
    histogram_buffer_array histogram_buffers_{};
    size_t index1;
    size_t index2;
};