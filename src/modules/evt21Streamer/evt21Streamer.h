#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "types.h"
#include "../../drivers/genx320/genx320Driver.h"

// Provides direct access to one camera buffer at a time.
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

    // Wait for the next completed buffer returned by V4L2. Only wordCount()
    // words are valid. Release before getting another; the pointer is valid
    // until release or stop.
    evt21_buffer getBuffer();
    void releaseBuffer();
    std::size_t wordCount() const noexcept { return word_count_; }

private:
    Genx320Driver driver_;
    bool initialized_ = false;
    bool started_ = false;
    bool buffer_in_use_ = false;
    std::uint32_t buffer_index_ = 0;
    std::size_t word_count_ = 0;
};
