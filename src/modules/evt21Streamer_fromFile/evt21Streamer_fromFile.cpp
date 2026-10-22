#include "evt21Streamer_fromFile.h"

#include <stdexcept>

Evt21StreamerFromFile::~Evt21StreamerFromFile() {
    stop();
}

void Evt21StreamerFromFile::initialize(const std::string& source,
                                       const std::string& /*sensor_device*/) {
    stop();
    file_.open(source, std::ios::binary);
    if (!file_.is_open()) {
        throw std::runtime_error("cannot open EVT2.1 file: " + source);
    }
}

void Evt21StreamerFromFile::start() {
    if (!file_.is_open() || started_) {
        throw std::logic_error("initialize the file before starting it");
    }
    started_ = true;
}

void Evt21StreamerFromFile::stop() noexcept {
    file_.close();
    file_.clear();
    started_ = false;
    buffer_in_use_ = false;
    word_count_ = 0;
}

evt21_buffer Evt21StreamerFromFile::getBuffer() {
    if (!started_) {
        throw std::logic_error("file stream is not started");
    }
    if (buffer_in_use_) {
        throw std::logic_error("release the current buffer first");
    }

    // Read the same maximum buffer size used by the camera, preserving a
    // smaller final buffer instead of discarding it.
    file_.read(reinterpret_cast<char*>(buffer_.data()), kBufferBytes);
    const std::streamsize bytes = file_.gcount();
    if (file_.bad() || (file_.fail() && !file_.eof())) {
        throw std::runtime_error("error reading EVT2.1 file");
    }
    if (bytes % sizeof(evt21_word) != 0) {
        throw std::runtime_error("EVT2.1 file ends with an incomplete word");
    }
    if (bytes == 0) {
        return nullptr;
    }

    word_count_ = static_cast<std::size_t>(bytes) / sizeof(evt21_word);
    buffer_in_use_ = true;
    return buffer_.data();
}

void Evt21StreamerFromFile::releaseBuffer() {
    if (!buffer_in_use_) {
        throw std::logic_error("there is no file buffer to release");
    }
    buffer_in_use_ = false;
    word_count_ = 0;
}
