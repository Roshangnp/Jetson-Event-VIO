#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include "types.h"
#include "histogram_buffer.h"

class histograms{
    public:
    // must be at least (windows per camera buffer) + 2 (1 FILLING, 1 HELD) for dequeueNextHistogramPair()
    // to see every window. A quiet scene on the Jetson gave ~6.7 windows per buffer, so 12 leaves margin
    // (~2.4 MB for the whole ring).
    static constexpr size_t HISTOGRAM_BUFFER_COUNT = 12;
    static constexpr size_t HISTOGRAM_TIME_HIGH_WIDTH = 40;
    using histogram_buffer_array = std::array<histogram_buffer,HISTOGRAM_BUFFER_COUNT>;
    using histogram_buffer_pair = std::array<histogram_buffer*,2>;

    // recieves a evt21_buffer from evt21Streamer (of evt21Streamer_fromFile) and loops through
    // all valid words of the buffer and calls updateHistogram belonging to each histogram_buffer
    // it also handles the resetting of each buffer once the difference between timeHigh_start and timeHigh_current
    // reaches HISTOGRAM_TIME_HIGH_WIDTH: the full buffer becomes READY, and the next buffer (from getNextQueuedIndex)
    // is reset, assigned a timeHigh_start and sequence number, and becomes FILLING.
    void updateHistograms(evt21_buffer buffer, size_t word_count);

    // returns the newest pair of time-adjacent ready histograms as {newer, older}, and holds
    // both so they are not overwritten. Returns {nullptr, nullptr} if there is no pair newer
    // than the last one returned, or if the newest buffer's predecessor was overwritten.
    // Consecutive pairs may overlap: {C, B} can follow {B, A}. Call queueHistogramPair()
    // before dequeuing again.
    histogram_buffer_pair dequeueHistogramPair();

    // like dequeueHistogramPair(), but returns pairs in order instead of skipping to the newest:
    // the oldest ready pair whose newer window comes after the last one returned. If the next
    // window was already overwritten, it skips to the oldest pair still available. Call it in a
    // loop (with queueHistogramPair() after each pair) until it returns {nullptr, nullptr} to
    // consume every window. Both dequeue functions share state and may be mixed.
    histogram_buffer_pair dequeueNextHistogramPair();

    // re-queues the older of the dequeued pair so it can be written to. The newer one stays
    // held because it is the older half of the next pair.
    void queueHistogramPair();

    // returns the number of corrupted event words skipped so far
    size_t getBadWordCount() const { return bad_word_count_; }

    private:
    // gives the index of the buffer to fill next: a FREE buffer if there is one, otherwise
    // the oldest READY buffer. Never a FILLING or HELD buffer.
    size_t getNextQueuedIndex();

    // returns how many buffers are HELD by the consumer
    size_t heldCount() const;

    // returns the index of the READY or HELD buffer with this sequence, or NO_INDEX
    size_t findCompleteBySequence(uint64_t sequence) const;

    // holds newer and older, releases any other HELD buffer, and records newer as the
    // last pair returned
    histogram_buffer_pair holdPair(size_t newer, size_t older);

    size_t bad_word_count_ = 0;
    // decoded value (get_time_high) of the latest TIME_HIGH word
    size_t current_timeHigh = 0;
    // false until the first TIME_HIGH word; events before it have no time reference
    bool has_timeHigh_ = false;
    // the buffer currently being filled
    size_t active_index_ = 0;
    histogram_buffer_array histogram_buffers_{};

    // marks a search that found no buffer
    static constexpr size_t NO_INDEX = HISTOGRAM_BUFFER_COUNT;

    // sequence number given to the next window that starts
    uint64_t next_sequence_ = 1;
    // sequence of the newer buffer in the last pair returned, 0 if none yet
    uint64_t last_dequeued_seq_ = 0;
};