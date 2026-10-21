#include "driver.h"

#include <cerrno>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdexcept>
#include <system_error>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace {

constexpr std::uint32_t kEvtFormatControl = 0x00982903U;
constexpr std::uint32_t kBufferCount = 8;

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

Genx320Driver::~Genx320Driver() {
    stop();
}

void Genx320Driver::initialize(const std::string& video_device,
                               const std::string& sensor_device) {
    try {
        // The EVT format control belongs to the sensor subdevice.
        sensor_fd_ = ::open(sensor_device.c_str(), O_RDWR | O_CLOEXEC);
        if (sensor_fd_ == -1) {
            fail("open sensor subdevice");
        }

        v4l2_control control{};
        control.id = kEvtFormatControl;
        control.value = 1; // 1 = EVT2.1
        if (xioctl(sensor_fd_, VIDIOC_S_CTRL, &control) == -1) {
            fail("VIDIOC_S_CTRL");
        }

        // The video node owns the V4L2 streaming queue.
        video_fd_ = ::open(video_device.c_str(),
                           O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (video_fd_ == -1) {
            fail("open video device");
        }

        // Ask V4L2 for eight driver-allocated MMAP buffers.
        v4l2_requestbuffers request{};
        request.count = kBufferCount;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        if (xioctl(video_fd_, VIDIOC_REQBUFS, &request) == -1) {
            fail("VIDIOC_REQBUFS");
        }

        // Map each driver buffer into this process.
        for (std::uint32_t index = 0; index < request.count; ++index) {
            v4l2_buffer query{};
            query.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            query.memory = V4L2_MEMORY_MMAP;
            query.index = index;

            if (xioctl(video_fd_, VIDIOC_QUERYBUF, &query) == -1) {
                fail("VIDIOC_QUERYBUF");
            }

            void* data = ::mmap(nullptr, query.length,
                                PROT_READ | PROT_WRITE, MAP_SHARED,
                                video_fd_, query.m.offset);
            if (data == MAP_FAILED) {
                fail("mmap");
            }

            buffers_.push_back({data, query.length});
        }
    } catch (...) {
        stop();
        throw;
    }
}

void Genx320Driver::start() {
    // Give every mapped buffer to the driver before starting DMA.
    for (std::uint32_t index = 0; index < buffers_.size(); ++index) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;

        if (xioctl(video_fd_, VIDIOC_QBUF, &buffer) == -1) {
            fail("VIDIOC_QBUF");
        }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(video_fd_, VIDIOC_STREAMON, &type) == -1) {
        fail("VIDIOC_STREAMON");
    }

    streaming_ = true;
}

void Genx320Driver::stop() noexcept {
    if (streaming_) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)xioctl(video_fd_, VIDIOC_STREAMOFF, &type);
        streaming_ = false;
    }

    for (Buffer& buffer : buffers_) {
        (void)::munmap(buffer.data, buffer.size);
    }
    buffers_.clear();

    if (video_fd_ != -1) {
        (void)::close(video_fd_);
        video_fd_ = -1;
    }

    if (sensor_fd_ != -1) {
        (void)::close(sensor_fd_);
        sensor_fd_ = -1;
    }
}

