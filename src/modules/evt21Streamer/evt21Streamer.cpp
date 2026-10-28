#include "evt21Streamer.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdexcept>

#include "../../drivers/genx320/v4l2Util.h"

using v4l2_util::fail;
using v4l2_util::xioctl;

namespace {

// Waits up to timeout_ms for the video device to report a buffer may be
// ready. Returns false on timeout. Spurious wake-ups are fine: the caller
// simply retries VIDIOC_DQBUF.
bool waitForBuffer(int fd, int timeout_ms) {
    pollfd ready{fd, POLLIN, 0};
    int result;
    do {
        result = ::poll(&ready, 1, timeout_ms);
    } while (result == -1 && errno == EINTR);
    if (result == -1) {
        fail("poll camera");
    }
    if (result == 0) {
        return false;
    }
    if (ready.revents & (POLLHUP | POLLNVAL)) {
        throw std::runtime_error("camera device is no longer available");
    }
    if (!(ready.revents & POLLIN) && (ready.revents & POLLERR)) {
        throw std::runtime_error("camera reported a polling error");
    }
    return true;
}

} // namespace

Evt21Streamer::~Evt21Streamer() {
    stop();
}

void Evt21Streamer::initialize(const std::string& source,
                               const std::string& sensor_device) {
    if (source.empty() || sensor_device.empty()) {
        throw std::invalid_argument("video and sensor devices are required");
    }

    stop();
    driver_.initialize(source, sensor_device);

    // big enough for any buffer the driver granted; kept across stop() / initialize()
    std::size_t largest = 0;
    for (std::uint32_t i = 0; i < driver_.bufferCount(); ++i) {
        largest = std::max(largest, driver_.buffer(i).size);
    }
    const std::size_t words = (largest + sizeof(evt21_word) - 1) / sizeof(evt21_word);
    if (copy_.size() < words) {
        copy_.resize(words);
    }
    state_ = State::INITIALIZED;
}

void Evt21Streamer::start() {
    if (state_ == State::CLOSED) {
        throw std::logic_error("initialize the camera before starting it");
    }
    if (state_ != State::INITIALIZED) {
        throw std::logic_error("camera stream is already started");
    }

    try {
        driver_.start();
        has_sequence_ = false;
        dropped_buffer_count_ = 0;
        state_ = State::STREAMING;
    } catch (...) {
        // A failed start can leave buffers queued; close the driver to reset it.
        stop();
        throw;
    }
}

void Evt21Streamer::stop() noexcept {
    driver_.stop();
    state_ = State::CLOSED;
    word_count_ = 0;
}

evt21_buffer Evt21Streamer::getBuffer() {
    if (state_ == State::HOLDING) {
        throw std::logic_error("release the current buffer first");
    }
    if (state_ != State::STREAMING) {
        throw std::logic_error("camera stream is not started");
    }

    // One deadline for the whole call, so spurious wake-ups and skipped empty
    // buffers cannot stretch the wait past the timeout.
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(cameraConfig::BUFFER_TIMEOUT_MS);

    for (;;) {
        // Try to take a completed buffer first and only poll() when none is
        // ready. When the camera is ahead of us this skips a syscall per buffer.
        v4l2_buffer buffer = v4l2_util::makeBuffer();
        if (xioctl(driver_.fd(), VIDIOC_DQBUF, &buffer) == -1) {
            if (errno != EAGAIN) {
                fail("VIDIOC_DQBUF");
            }
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0 ||
                !waitForBuffer(driver_.fd(), static_cast<int>(remaining))) {
                // Timed out: no buffer is held and the stream keeps running.
                return nullptr;
            }
            continue;
        }

        if (buffer.index >= driver_.bufferCount()) {
            stop();
            throw std::runtime_error("camera returned an invalid buffer index");
        }
        const Genx320Driver::Buffer& mapped = driver_.buffer(buffer.index);

        // V4L2 numbers every buffer the driver fills, so a jump means it dropped some
        if (has_sequence_) {
            dropped_buffer_count_ += static_cast<std::uint32_t>(buffer.sequence - last_sequence_ - 1);
        }
        has_sequence_ = true;
        last_sequence_ = buffer.sequence;

        if ((buffer.flags & V4L2_BUF_FLAG_ERROR) ||
            buffer.bytesused > mapped.size ||
            buffer.bytesused % sizeof(evt21_word) != 0) {
            // Give the unusable buffer back before reporting the error. A
            // requeue failure is ignored so this error is the one reported.
            v4l2_buffer requeue = v4l2_util::makeBuffer(buffer.index);
            (void)xioctl(driver_.fd(), VIDIOC_QBUF, &requeue);
            throw std::runtime_error("camera returned an invalid EVT2.1 buffer");
        }
        if (buffer.bytesused == 0) {
            // An empty buffer has no events to return; wait for the next one.
            v4l2_util::queueBuffer(driver_.fd(), buffer.index);
            continue;
        }

        // Copy out of the uncached mapping and hand the buffer straight back to the driver.
        std::memcpy(copy_.data(), mapped.data, buffer.bytesused);
        v4l2_util::queueBuffer(driver_.fd(), buffer.index);

        word_count_ = buffer.bytesused / sizeof(evt21_word);
        state_ = State::HOLDING;
        return copy_.data();
    }
}

void Evt21Streamer::releaseBuffer() {
    if (state_ != State::HOLDING) {
        throw std::logic_error("there is no camera buffer to release");
    }

    // the V4L2 buffer was already requeued in getBuffer(); only the copy is released
    state_ = State::STREAMING;
    word_count_ = 0;
}
