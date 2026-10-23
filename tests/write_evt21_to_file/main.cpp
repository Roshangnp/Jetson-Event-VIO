#include "../../src/drivers/genx320/genx320Driver.h"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
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

void queueBuffer(int fd, std::uint32_t index) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    if (xioctl(fd, VIDIOC_QBUF, &buffer) == -1) {
        throw std::system_error(errno, std::generic_category(), "VIDIOC_QBUF");
    }
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "usage: " << argv[0]
                  << " <video-device> <sensor-device>\n";
        return 1;
    }

    try {
        std::ofstream output("log.raw", std::ios::binary);
        if (!output) {
            throw std::runtime_error("could not open log.raw");
        }

        Genx320Driver driver;
        driver.initialize(argv[1], argv[2]);
        driver.start();

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(1);
        while (std::chrono::steady_clock::now() < deadline) {
            const auto remaining = deadline - std::chrono::steady_clock::now();
            const auto timeout = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count());
            if (timeout <= 0) {
                break;
            }

            pollfd ready{driver.fd(), POLLIN, 0};
            const int poll_result = ::poll(&ready, 1, timeout);
            if (poll_result == 0) {
                continue;
            }
            if (poll_result == -1) {
                throw std::system_error(errno, std::generic_category(), "poll");
            }
            if (!(ready.revents & POLLIN)) {
                throw std::runtime_error("camera poll failed");
            }

            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            if (xioctl(driver.fd(), VIDIOC_DQBUF, &buffer) == -1) {
                if (errno == EAGAIN) {
                    continue;
                }
                throw std::system_error(errno, std::generic_category(), "VIDIOC_DQBUF");
            }

            const auto& mapped = driver.buffer(buffer.index);
            if (buffer.bytesused > mapped.size ||
                buffer.bytesused % sizeof(std::uint64_t) != 0) {
                throw std::runtime_error("camera returned an invalid EVT2.1 buffer");
            }

            output.write(static_cast<const char*>(mapped.data), buffer.bytesused);
            if (!output) {
                throw std::runtime_error("could not write log.raw");
            }
            queueBuffer(driver.fd(), buffer.index);
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
