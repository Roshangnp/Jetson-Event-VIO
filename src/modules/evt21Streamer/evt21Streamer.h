#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "types.h"
#include "../../drivers/genx320/genx320Driver.h"

// Provides one camera buffer at a time. Each V4L2 buffer is copied out of the driver's mmap'd memory into
// a cached copy and requeued straight away: the mapped memory is uncached, so reading it word by word is
// ~10x slower than one memcpy (32 KB in ~33 us on the Jetson) plus reading the copy.
class Evt21Streamer {
public:
    Evt21Streamer() = default;
    ~Evt21Streamer();

    Evt21Streamer(const Evt21Streamer&) = delete;
    Evt21Streamer& operator=(const Evt21Streamer&) = delete;

    // source is the video device; sensor_device is the sensor subdevice.
    void initialize(const std::string& source,
                    const std::string& sensor_device = "");
    void start();

    // Stops the stream and invalidates any outstanding buffer. Initialize again
    // before restarting.
    void stop() noexcept;

    // Wait for the next completed buffer returned by V4L2 and return a copy of it (the V4L2 buffer is
    // already back with the driver). Only wordCount() words are valid. Release before getting another;
    // the pointer is valid until release or stop.
    // Returns nullptr if no buffer arrives within cameraConfig::BUFFER_TIMEOUT_MS.
    // The stream keeps running, so the caller can simply call getBuffer() again.
    evt21_buffer getBuffer();
    void releaseBuffer();
    std::size_t wordCount() const noexcept { return word_count_; }

    // buffers the driver dropped since start(): gaps in the V4L2 sequence numbers of every buffer
    // dequeued, empty ones included. Nonzero means the camera filled buffers faster than they were
    // released.
    std::uint64_t droppedBufferCount() const noexcept { return dropped_buffer_count_; }

private:
    // CLOSED -> INITIALIZED (initialize) -> STREAMING (start)
    // STREAMING <-> HOLDING (getBuffer / releaseBuffer); stop() -> CLOSED. HOLDING means the caller
    // holds copy_; no V4L2 buffer is held.
    enum class State : std::uint8_t {
        CLOSED,       // nothing open
        INITIALIZED,  // driver open and mapped, not streaming
        STREAMING,    // streaming, no buffer held by the caller
        HOLDING       // streaming, caller holds copy_
    };

    Genx320Driver driver_;
    State state_ = State::CLOSED;
    std::size_t word_count_ = 0;
    // the cached copy getBuffer() returns, sized to the largest V4L2 buffer once in initialize()
    std::vector<evt21_word> copy_;

    // V4L2 sequence of the last dequeued buffer, for droppedBufferCount()
    bool has_sequence_ = false;
    std::uint32_t last_sequence_ = 0;
    std::uint64_t dropped_buffer_count_ = 0;
};
