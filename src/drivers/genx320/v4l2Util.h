#pragma once

#include <cerrno>
#include <cstdint>
#include <linux/videodev2.h>
#include <system_error>
#include <sys/ioctl.h>

// V4L2 helpers shared by Genx320Driver, Evt21Streamer and the capture test.
// Everything here is header-only so the capture test's CMake target needs no
// extra sources.
namespace v4l2_util {

// ioctl() that retries when a signal interrupts it.
inline int xioctl(int fd, unsigned long request, void* argument) {
    int result;
    do {
        result = ::ioctl(fd, request, argument);
    } while (result == -1 && errno == EINTR);
    return result;
}

// Throws std::system_error carrying the current errno.
[[noreturn]] inline void fail(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}

// Describes MMAP capture buffer `index`. VIDIOC_DQBUF ignores the index and
// fills in the one it returns.
inline v4l2_buffer makeBuffer(std::uint32_t index = 0) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    return buffer;
}

// Hands buffer `index` back to the driver so it can be filled again.
inline void queueBuffer(int fd, std::uint32_t index) {
    v4l2_buffer buffer = makeBuffer(index);
    if (xioctl(fd, VIDIOC_QBUF, &buffer) == -1) {
        fail("VIDIOC_QBUF");
    }
}

} // namespace v4l2_util
