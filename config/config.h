#pragma once

#include <array>
#include <cstddef>
#include <cstdint>



namespace cameraConfig
{
    inline constexpr size_t WIDTH  = 320;
    inline constexpr size_t HEIGHT = 320;
    inline constexpr size_t PIXEL_NUM = 320*320;

    // Longest Evt21Streamer::getBuffer() waits for a camera buffer before
    // returning nullptr. Long enough not to trip in normal operation, short
    // enough to notice a camera that has stopped delivering.
    inline constexpr int BUFFER_TIMEOUT_MS = 500;
}

namespace flowConfig
{
    // Lucas-Kanade patches are PATCH_WIDTH x PATCH_WIDTH pixels, centred on grid points
    // GRID_STRIDE pixels apart. Patches overlap whenever GRID_STRIDE < PATCH_WIDTH.
    // GRID_STRIDE = 1 is dense flow. 5x5 over a ~2.5 ms window is verified in event-dense scenes.
    inline constexpr size_t PATCH_WIDTH = 3;
    inline constexpr size_t GRID_STRIDE = 1;
    inline constexpr uint8_t HISTOGRAM_MIN_THRESH = 5;
    inline constexpr float SHI_TOMPSI_MIN_THRESH = 1e-6f; // effectively 0; exactly 0 would let empty patches (det = 0) through as NaN flow

    static_assert(PATCH_WIDTH % 2 == 1, "PATCH_WIDTH must be odd so the patch has a centre pixel");
    static_assert(GRID_STRIDE >= 1, "GRID_STRIDE must be at least 1");

    inline constexpr size_t PATCH_RADIUS = PATCH_WIDTH / 2;

    // Grid points are kept where the whole patch fits on the sensor, so there is no padding:
    // the first centre is at PATCH_RADIUS, the last at most WIDTH - 1 - PATCH_RADIUS.
    // Dense 5x5 gives a 316 x 316 grid.
    inline constexpr size_t GRID_WIDTH  = (cameraConfig::WIDTH  - 2 * PATCH_RADIUS - 1) / GRID_STRIDE + 1;
    inline constexpr size_t GRID_HEIGHT = (cameraConfig::HEIGHT - 2 * PATCH_RADIUS - 1) / GRID_STRIDE + 1;
    inline constexpr size_t GRID_POINT_NUM = GRID_WIDTH * GRID_HEIGHT;
}

namespace filters
{
    inline constexpr float GUASSIAN_SIGMA = 1; // made up value for now
    inline constexpr size_t GUASSIAN_PATCH_WIDTH = 3;
    inline constexpr size_t SGF_PATCH_WIDTH = 3; //SG stands for Savitzky-Golay filter

    // 3x3 kernels, row-major (index = row * 3 + col, row = y). std::array rather than
    // Eigen::Matrix3f because Eigen types aren't constexpr, and so config.h stays Eigen-free.
    using kernel3x3 = std::array<float, 9>;
    inline constexpr kernel3x3 SGF_HX_3D_FIRST_ORDER_KERNAL{-1.0f/6, 0.0f, 1.0f/6,
                                                            -1.0f/6, 0.0f, 1.0f/6,
                                                            -1.0f/6, 0.0f, 1.0f/6};
    inline constexpr kernel3x3 SGF_HY_3D_FIRST_ORDER_KERNAL{-1.0f/6, -1.0f/6, -1.0f/6,
                                                             0.0f,    0.0f,    0.0f,
                                                             1.0f/6,  1.0f/6,  1.0f/6};
    inline constexpr kernel3x3 SGF_HT_3D_FIRST_ORDER_KERNAL{1.0f/9, 1.0f/9, 1.0f/9,
                                                            1.0f/9, 1.0f/9, 1.0f/9,
                                                            1.0f/9, 1.0f/9, 1.0f/9};

}

enum class evt21_type : uint8_t
{
    TIME_HIGH = 0x8,
    NEGATIVE = 0x0,
    POSITIVE = 0x1
};