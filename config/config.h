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
    // GRID_STRIDE = 1 is dense flow. 11x11 on Gaussian-smoothed (sigma 1) histograms was tuned offline
    // on a recorded hand wave: 3x3 and 5x5 patches give incoherent directions on 2.56 ms windows.
    inline constexpr size_t PATCH_WIDTH = 11;
    inline constexpr size_t GRID_STRIDE = 1;
    inline constexpr uint8_t HISTOGRAM_MIN_THRESH = 5;
    // smaller eigenvalue of the structure matrix. 1 suits PATCH_WIDTH 11 with sigma 1 smoothing (eigenvalues
    // grow with the patch and shrink with smoothing); it must stay > 0, since 0 lets empty patches
    // (det = 0) through as NaN flow
    inline constexpr float SHI_TOMPSI_MIN_THRESH = 1.0f;

    static_assert(PATCH_WIDTH % 2 == 1, "PATCH_WIDTH must be odd so the patch has a centre pixel");
    static_assert(GRID_STRIDE >= 1, "GRID_STRIDE must be at least 1");

    inline constexpr size_t PATCH_RADIUS = PATCH_WIDTH / 2;

    // Grid points are kept where the whole patch fits on the sensor, so there is no padding:
    // the first centre is at PATCH_RADIUS, the last at most WIDTH - 1 - PATCH_RADIUS.
    // Dense 11x11 gives a 310 x 310 grid.
    inline constexpr size_t GRID_WIDTH  = (cameraConfig::WIDTH  - 2 * PATCH_RADIUS - 1) / GRID_STRIDE + 1;
    inline constexpr size_t GRID_HEIGHT = (cameraConfig::HEIGHT - 2 * PATCH_RADIUS - 1) / GRID_STRIDE + 1;
    inline constexpr size_t GRID_POINT_NUM = GRID_WIDTH * GRID_HEIGHT;
}

namespace filters
{
    inline constexpr float GUASSIAN_SIGMA = 1; // made up value for now
    inline constexpr size_t GUASSIAN_PATCH_WIDTH = 3;
    inline constexpr size_t SGF_PATCH_WIDTH = 3; //SG stands for Savitzky-Golay filter

    // Gaussian smoothing of both histograms before the SGF derivatives (histogram_filters_cuda::
    // applyGuassianSmoothing). The kernel is (2 * RADIUS + 1) wide, cut off at 3 sigma.
    inline constexpr float GUASSIAN_SMOOTHING_SIGMA = 1.0f;
    inline constexpr int GUASSIAN_SMOOTHING_RADIUS = 3;

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