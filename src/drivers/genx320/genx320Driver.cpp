#include "genx320Driver.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdexcept>
#include <sys/mman.h>
#include <unistd.h>

#include "v4l2Util.h"

using v4l2_util::fail;
using v4l2_util::xioctl;

namespace {

// Sensor subdevice control selecting the event format.
constexpr std::uint32_t kEvtFormatControl = 0x00982903U;
// Buffers requested from the driver; it may grant a different count.
constexpr std::uint32_t kBufferCount = 8;

} // namespace

Genx320Driver::~Genx320Driver() {
    stop();
}

void Genx320Driver::initialize(const std::string& video_device,
                               const std::string& sensor_device) {
    // Calling initialize() twice must not leak the previous fds and mappings.
    stop();

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

        // The video node owns the V4L2 streaming queue. Nonblocking so the
        // streamer can try VIDIOC_DQBUF and only poll() when nothing is ready.
        video_fd_ = ::open(video_device.c_str(),
                           O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (video_fd_ == -1) {
            fail("open video device");
        }

        // Ask V4L2 for driver-allocated MMAP buffers.
        v4l2_requestbuffers request{};
        request.count = kBufferCount;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        if (xioctl(video_fd_, VIDIOC_REQBUFS, &request) == -1) {
            fail("VIDIOC_REQBUFS");
        }
        if (request.count == 0) {
            // Fail here rather than later in start() with a less clear error.
            throw std::runtime_error("VIDIOC_REQBUFS granted no buffers");
        }

        // Map each driver buffer into this process.
        for (std::uint32_t index = 0; index < request.count; ++index) {
            v4l2_buffer query = v4l2_util::makeBuffer(index);
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
    for (std::uint32_t index = 0; index < bufferCount(); ++index) {
        v4l2_util::queueBuffer(video_fd_, index);
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
