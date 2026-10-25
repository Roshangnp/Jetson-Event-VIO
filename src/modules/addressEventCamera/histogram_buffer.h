#pragma once

#include <cstddef>
#include <cstdint>
#include "types.h"

// Holds one per-pixel count histogram inline (~200 KB). No memory is allocated;
// the storage lives wherever the owning object lives.
class histogram_buffer{
    public:
    histogram_buffer() = default;

    // non-copyable, a copy would duplicate the whole histogram
    histogram_buffer(const histogram_buffer&) = delete;
    histogram_buffer& operator=(const histogram_buffer&) = delete;

    // updateHistogram takes a evt21_word which is inferred to be either a positive event bit mask (POSITIVE),
    // or a negative event bit mask (NEGATIVE). The exact type is passed in as type. The evt21 type is not actively checked in this function,
    // so it must be either NEGATIVE or POSITIVE, or else the histogram will end up corrupted.
    void updateHistogram(evt21_word word, evt21_type type);

    // returns the pointer to the current histogram, if the field ready_ is false, returns nullptr.
    histogram_array* getHistogram();

    // returns ready_
    bool isReady();

    // sets ready_ to false and reset_ to true and all values of hist_ to 0
    void resetHistogram();

    // returns reset_
    bool isReset();

    // returns the difference vetween timeHigh_current_ and timeHigh_start_
    size_t getCurrentTimeHighDuration();


    private:
    evt21_word timeHigh_start_ = 0;
    evt21_word timeHigh_current_ = 0;
    histogram_array hist_{};
    bool ready_ = false;
    bool reset_ = true;
};
