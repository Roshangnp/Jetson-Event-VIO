#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class Genx320Driver {
public:
    struct Buffer {
        void* data;
        std::size_t size;
    };

    Genx320Driver() = default;
    ~Genx320Driver();

    Genx320Driver(const Genx320Driver&) = delete;
    Genx320Driver& operator=(const Genx320Driver&) = delete;

    // Open the sensor and capture nodes, select EVT2.1, and mmap the buffers.
    void initialize(const std::string& video_device,
                    const std::string& sensor_device);

    // Queue all buffers and start the camera stream.
    void start();

    // Stop streaming and release everything owned by this object.
    void stop() noexcept;

    // Used by the service for poll(), VIDIOC_DQBUF, and VIDIOC_QBUF.
    int fd() const noexcept { return video_fd_; }
    const Buffer& buffer(std::uint32_t index) const { return buffers_.at(index); }

private:
    int video_fd_ = -1;
    int sensor_fd_ = -1;
    bool streaming_ = false;
    std::vector<Buffer> buffers_;
};

