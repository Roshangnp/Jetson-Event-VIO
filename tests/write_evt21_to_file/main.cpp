#include "../../src/drivers/genx320/genx320Driver.h"
#include "../../src/drivers/genx320/v4l2Util.h"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdexcept>

using v4l2_util::fail;
using v4l2_util::xioctl;

// Captures one second of raw EVT2.1 words from the camera into ./log.raw.
// Drives Genx320Driver directly (not Evt21Streamer) so the capture can stop
// on a deadline even when no buffers arrive.

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
                fail("poll");
            }
            if (!(ready.revents & POLLIN)) {
                throw std::runtime_error("camera poll failed");
            }

            v4l2_buffer buffer = v4l2_util::makeBuffer();
            if (xioctl(driver.fd(), VIDIOC_DQBUF, &buffer) == -1) {
                if (errno == EAGAIN) {
                    continue;
                }
                fail("VIDIOC_DQBUF");
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
            // Give the buffer back so the driver can fill it again.
            v4l2_util::queueBuffer(driver.fd(), buffer.index);
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
