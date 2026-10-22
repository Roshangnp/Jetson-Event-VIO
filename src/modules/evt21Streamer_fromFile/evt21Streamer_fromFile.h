#pragma once

#include <array>
#include <cstddef>
#include <fstream>
#include <string>

#include "types.h"

// Presents a raw EVT2.1 file through the same interface as Evt21Streamer.
class Evt21StreamerFromFile {
public:
    Evt21StreamerFromFile() = default;
    ~Evt21StreamerFromFile();

    Evt21StreamerFromFile(const Evt21StreamerFromFile&) = delete;
    Evt21StreamerFromFile& operator=(const Evt21StreamerFromFile&) = delete;

    // source is a headerless binary file of 64-bit EVT2.1 words in native byte
    // order (little-endian on Jetson). sensor_device is ignored; it keeps the
    // camera and file interfaces alike.
    void initialize(const std::string& source,
                    const std::string& sensor_device = "");
    void start();

    // Closes the file and invalidates any outstanding buffer. Initialize again
    // before restarting.
    void stop() noexcept;

    // Return up to 32768 bytes, or nullptr at EOF. Only wordCount() words are
    // valid. Release it before getting another buffer. The pointer remains
    // valid until releaseBuffer() or stop().
    evt21_buffer getBuffer();
    void releaseBuffer();
    std::size_t wordCount() const noexcept { return word_count_; }

private:
    static constexpr std::size_t kBufferBytes = 1024 * 32;
    std::array<evt21_word, kBufferBytes / sizeof(evt21_word)> buffer_;
    std::ifstream file_;
    bool started_ = false;
    bool buffer_in_use_ = false;
    std::size_t word_count_ = 0;
};
