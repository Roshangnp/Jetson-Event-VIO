#include "histograms.h"

#include <stdexcept>

#include "evt21Decoder.h"

void histograms::updateHistograms(evt21_buffer buffer, size_t word_count) {
    for (size_t i = 0; i < word_count; ++i) {
        const evt21_word word = buffer[i];
        const evt21_type type = get_evt21_word_type(word);

        switch (type) {
        case evt21_type::TIME_HIGH: {
            current_timeHigh = get_time_high(word);
            histogram_buffer& active = histogram_buffers_[active_index_];

            if (!has_timeHigh_) {
                // First time reference: start the first window here.
                active.setTimeHighStart(current_timeHigh);
                active.setSequence(next_sequence_++);
                active.setState(buffer_state::FILLING);
                has_timeHigh_ = true;
                break;
            }

            active.setTimeHighCurrent(current_timeHigh);
            if (active.getCurrentTimeHighDuration() >= HISTOGRAM_TIME_HIGH_WIDTH) {
                // Window is full: hand it off and start the next buffer.
                active.setState(buffer_state::READY);
                active_index_ = getNextQueuedIndex();

                histogram_buffer& next = histogram_buffers_[active_index_];
                next.resetHistogram();
                next.setTimeHighStart(current_timeHigh);
                next.setSequence(next_sequence_++);
                next.setState(buffer_state::FILLING);
            }
            break;
        }
        case evt21_type::NEGATIVE:
        case evt21_type::POSITIVE:
            if (!is_valid_event(word)) {
                // Skip corrupted words instead of stopping the stream.
                ++bad_word_count_;
                break;
            }
            if (has_timeHigh_) {
                histogram_buffers_[active_index_].updateHistogram(word, type);
            }
            break;
        default:
            // EXT_TRIGGER, OTHERS and CONTINUED are not used for histograms.
            break;
        }
    }
}

size_t histograms::getNextQueuedIndex() {
    // Prefer a FREE buffer so READY history is kept as long as possible;
    // otherwise overwrite the oldest READY one. At most 2 buffers are HELD
    // and the one just marked READY is the newest, so a candidate always exists.
    size_t oldest_ready = NO_INDEX;
    for (size_t i = 0; i < HISTOGRAM_BUFFER_COUNT; ++i) {
        const histogram_buffer& candidate = histogram_buffers_[i];
        if (candidate.getState() == buffer_state::FREE) {
            return i;
        }
        if (candidate.getState() == buffer_state::READY &&
            (oldest_ready == NO_INDEX ||
             candidate.getSequence() < histogram_buffers_[oldest_ready].getSequence())) {
            oldest_ready = i;
        }
    }
    if (oldest_ready == NO_INDEX) {
        throw std::logic_error("no histogram buffer is available to fill");
    }
    return oldest_ready;
}

size_t histograms::heldCount() const {
    size_t count = 0;
    for (const histogram_buffer& candidate : histogram_buffers_) {
        if (candidate.getState() == buffer_state::HELD) {
            ++count;
        }
    }
    return count;
}

histograms::histogram_buffer_pair histograms::dequeueHistogramPair() {
    if (heldCount() == 2) {
        throw std::logic_error("queue the current histogram pair first");
    }

    // The newest READY buffer.
    size_t newer = NO_INDEX;
    for (size_t i = 0; i < HISTOGRAM_BUFFER_COUNT; ++i) {
        const histogram_buffer& candidate = histogram_buffers_[i];
        if (candidate.getState() == buffer_state::READY &&
            (newer == NO_INDEX ||
             candidate.getSequence() > histogram_buffers_[newer].getSequence())) {
            newer = i;
        }
    }
    if (newer == NO_INDEX ||
        histogram_buffers_[newer].getSequence() <= last_dequeued_seq_) {
        return {nullptr, nullptr};
    }

    // The window just before it, READY or still HELD from the last pair, if it
    // has not been overwritten.
    const uint64_t newer_seq = histogram_buffers_[newer].getSequence();
    size_t older = NO_INDEX;
    for (size_t i = 0; i < HISTOGRAM_BUFFER_COUNT; ++i) {
        const histogram_buffer& candidate = histogram_buffers_[i];
        const buffer_state state = candidate.getState();
        if ((state == buffer_state::READY || state == buffer_state::HELD) &&
            candidate.getSequence() == newer_seq - 1) {
            older = i;
            break;
        }
    }
    if (older == NO_INDEX) {
        return {nullptr, nullptr};
    }

    // A buffer still held from the last pair that is not part of this one was
    // skipped because the consumer fell behind; release it.
    for (size_t i = 0; i < HISTOGRAM_BUFFER_COUNT; ++i) {
        if (i != older && histogram_buffers_[i].getState() == buffer_state::HELD) {
            histogram_buffers_[i].setState(buffer_state::FREE);
        }
    }

    histogram_buffers_[newer].setState(buffer_state::HELD);
    histogram_buffers_[older].setState(buffer_state::HELD);
    last_dequeued_seq_ = newer_seq;
    return {&histogram_buffers_[newer], &histogram_buffers_[older]};
}

void histograms::queueHistogramPair() {
    if (heldCount() != 2) {
        throw std::logic_error("there is no histogram pair to queue");
    }

    // Release the older of the two; the newer stays HELD as the older half of
    // the next pair.
    size_t older = NO_INDEX;
    for (size_t i = 0; i < HISTOGRAM_BUFFER_COUNT; ++i) {
        const histogram_buffer& candidate = histogram_buffers_[i];
        if (candidate.getState() == buffer_state::HELD &&
            (older == NO_INDEX ||
             candidate.getSequence() < histogram_buffers_[older].getSequence())) {
            older = i;
        }
    }
    histogram_buffers_[older].setState(buffer_state::FREE);
}
