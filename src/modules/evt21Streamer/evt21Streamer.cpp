#include "evt21Streamer.h"

#include <cerrno>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <sys/ioctl.h>

namespace {

int xioctl(int fd, unsigned long request, void* argument) {
    int result;
    do {
        result = ::ioctl(fd, request, argument);
    } while (result == -1 && errno == EINTR);
    return result;
}

[[noreturn]] void fail(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
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
    initialized_ = true;
}

void Evt21Streamer::start() {
    if (!initialized_ || started_) {
        throw std::logic_error("initialize the camera before starting it");
    }

    try {
        driver_.start();
        started_ = true;
    } catch (...) {
        // A failed start can leave buffers queued; close the driver to reset it.
        stop();
        throw;
    }
}

void Evt21Streamer::stop() noexcept {
    driver_.stop();
    initialized_ = false;
    started_ = false;
    buffer_in_use_ = false;
    word_count_ = 0;
}

evt21_buffer Evt21Streamer::getBuffer() {
    if (!started_) {
        throw std::logic_error("camera stream is not started");
    }
    if (buffer_in_use_) {
        throw std::logic_error("release the current buffer first");
    }

    for (;;) {
        // The driver opens the video device in nonblocking mode.
        pollfd ready{driver_.fd(), POLLIN, 0};
        int result;
        do {
            result = ::poll(&ready, 1, -1);
        } while (result == -1 && errno == EINTR);
        if (result == -1) {
            fail("poll camera");
        }
        if (ready.revents & (POLLHUP | POLLNVAL)) {
            throw std::runtime_error("camera device is no longer available");
        }
        if (!(ready.revents & POLLIN)) {
            if (ready.revents & POLLERR) {
                throw std::runtime_error("camera reported a polling error");
            }
            continue;
        }

        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (xioctl(driver_.fd(), VIDIOC_DQBUF, &buffer) == -1) {
            if (errno == EAGAIN) {
                continue;
            }
            fail("VIDIOC_DQBUF");
        }

        const Genx320Driver::Buffer* mapped;
        try {
            mapped = &driver_.buffer(buffer.index);
        } catch (const std::out_of_range&) {
            stop();
            throw std::runtime_error("camera returned an invalid buffer index");
        }

        buffer_index_ = buffer.index;
        buffer_in_use_ = true;
        if ((buffer.flags & V4L2_BUF_FLAG_ERROR) ||
            buffer.bytesused > mapped->size ||
            buffer.bytesused % sizeof(evt21_word) != 0) {
            // Give an unusable buffer back before reporting the error.
            releaseBuffer();
            throw std::runtime_error("camera returned an invalid EVT2.1 buffer");
        }
        if (buffer.bytesused == 0) {
            releaseBuffer();
            continue;
        }

        word_count_ = buffer.bytesused / sizeof(evt21_word);
        return static_cast<evt21_buffer>(mapped->data);
    }
}

void Evt21Streamer::releaseBuffer() {
    if (!buffer_in_use_) {
        throw std::logic_error("there is no camera buffer to release");
    }

    // Requeue only after the caller has finished reading the mapped memory.
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = buffer_index_;
    if (xioctl(driver_.fd(), VIDIOC_QBUF, &buffer) == -1) {
        fail("VIDIOC_QBUF");
    }
    buffer_in_use_ = false;
    word_count_ = 0;
}
