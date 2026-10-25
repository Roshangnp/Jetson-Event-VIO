#pragma once

#include <cstddef>
#include <cstdint>
#include "types.h"

// What a histogram_buffer is currently doing. histograms performs every transition:
//   FREE or READY -> FILLING  (picked as the next window, contents reset)
//   FILLING -> READY          (window full)
//   READY -> HELD             (dequeued by the consumer)
//   HELD -> FREE              (queued back, or skipped by a newer dequeue)
enum class buffer_state : uint8_t
{
    FREE,     // never filled, or released; contents not needed
    FILLING,  // the active buffer, written by updateHistograms
    READY,    // window complete, available to dequeue
    HELD      // handed to the consumer, must not be written
};

// Holds one per-pixel signed event histogram inline (~200 KB). No memory is allocated;
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

    // returns the pointer to the histogram if the window is complete (READY or HELD), otherwise nullptr.
    histogram_array* getHistogram();

    // sets all values of hist_ to 0; the state is left to histograms
    void resetHistogram();

    // returns the difference between timeHigh_current_ and timeHigh_start_
    size_t getCurrentTimeHighDuration() const;

    // The time high setters take decoded values from get_time_high(), not raw words.

    // sets timeHigh_start_ (and timeHigh_current_) when the buffer begins a new window
    void setTimeHighStart(size_t time_high);

    // sets timeHigh_current_, called on each TIME_HIGH word
    void setTimeHighCurrent(size_t time_high);

    // sets sequence_, called by histograms when the buffer begins a new window
    void setSequence(uint64_t sequence) { sequence_ = sequence; }

    // returns sequence_; consecutive windows have consecutive sequence numbers
    uint64_t getSequence() const { return sequence_; }

    void setState(buffer_state state) { state_ = state; }
    buffer_state getState() const { return state_; }


    private:
    // 0 means the buffer has never been filled; histograms numbers windows from 1
    uint64_t sequence_ = 0;
    size_t timeHigh_start_ = 0;
    size_t timeHigh_current_ = 0;
    histogram_array hist_{};
    buffer_state state_ = buffer_state::FREE;
};
