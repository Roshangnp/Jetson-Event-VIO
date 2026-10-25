#include "histogram_buffer.h"

#include <stdexcept>

#include "evt21Decoder.h"

void histogram_buffer::updateHistogram(evt21_word word, evt21_type type) {
    // Safety net: histograms skips bad words before calling this, but a
    // corrupted word must never write outside hist_.
    if (!is_valid_event(word)) {
        throw std::runtime_error("EVT2.1 event is outside the sensor");
    }

    const std::size_t x = get_x(word);
    const std::size_t y = get_y(word);
    std::uint32_t valid = get_valid_mask(word);

    const std::int16_t delta = (type == evt21_type::POSITIVE) ? 1 : -1;
    std::int16_t* row = hist_.data() + y * cameraConfig::WIDTH + x;

    // Visit only the set bits; bit i is the pixel at x + i.
    while (valid != 0) {
        row[__builtin_ctz(valid)] += delta;
        valid &= valid - 1;
    }
}

histogram_array* histogram_buffer::getHistogram() {
    const bool complete = state_ == buffer_state::READY || state_ == buffer_state::HELD;
    return complete ? &hist_ : nullptr;
}

void histogram_buffer::resetHistogram() {
    hist_.fill(0);
}

size_t histogram_buffer::getCurrentTimeHighDuration() const {
    return get_time_high_duration(timeHigh_start_, timeHigh_current_);
}

void histogram_buffer::setTimeHighStart(size_t time_high) {
    timeHigh_start_ = time_high;
    timeHigh_current_ = time_high;
}

void histogram_buffer::setTimeHighCurrent(size_t time_high) {
    timeHigh_current_ = time_high;
}
