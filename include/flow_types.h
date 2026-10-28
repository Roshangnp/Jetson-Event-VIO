#pragma once

// Optic flow types. Kept out of types.h so modules that don't use flow don't pull in Eigen.
// Use only fixed-size Eigen types in the pipeline (no MatrixXf / VectorXf).

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <Eigen/Core>
#include "config.h"

// image frame: x right, y down, in pixels per second
using vector2d = Eigen::Vector2f;
using vector3d = Eigen::Vector3f;
using eigenValues2d = Eigen::Vector2f;
using eigenVectors2d = Eigen::Matrix2f;
using Hessian_Matrix = Eigen::Matrix2f;
using Structure_Matrix = Eigen::Matrix2f; // represents A'A in LK equation
using Time_Grad_Matrix = Eigen::Vector2f; // represents A'b in LK equation

// which convolution gives the spatial derivatives (dH/dx, dH/dy) for LK, on both the CPU and the GPU.
// A'A and A'b are built from the same derivatives, so one choice covers both.
enum class spatial_derivative { SGF, GAUSSIAN };

// which mask filter runs before LK: none (every grid point), method 1 (minimum histogram magnitude) or
// method 2 (Hessian ridge)
enum class hist_filter { NONE, MIN_MAGNITUDE, HESSIAN_RIDGE };

// one bit per grid point, indexed by grid_index (row-major). A set bit marks a grid point
// still worth running LK on. ~12.6 KB at a dense 3x3 grid, so pass it by reference.
using grid_mask = std::bitset<flowConfig::GRID_POINT_NUM>;

struct flow_patch {
    vector2d flow;
    // false if the patch had too few events or only one edge direction (aperture problem)
    bool valid;
};

// One flow estimate per grid point, row-major. 12 B per grid point, so ~1.15 MB at a dense 11x11 grid; like
// histograms it must live in static storage or a single std::make_unique, never on the stack.
struct flow_field {
    // TIME_HIGH span the flow was measured over (older window start -> newer window end)
    uint64_t time_high_start;
    uint64_t time_high_end;
    std::array<flow_patch, flowConfig::GRID_POINT_NUM> patches;
};

using Structure_Matrix_array = std::array<Structure_Matrix, flowConfig::GRID_POINT_NUM>;
using Time_Grad_Matrix_array = std::array<Time_Grad_Matrix, flowConfig::GRID_POINT_NUM>;

// Per-pixel derivative images for the CPU LK path, row-major over the sensor (pixel_index = y * WIDTH + x).
// Hxy_array is the spatial derivatives (dH/dx, dH/dy) of the newer histogram (~800 KB); Ht_array is the
// second-order temporal derivative (~400 KB). Same storage rule as flow_field: static or one make_unique.
using Hxy_array = std::array<vector2d, cameraConfig::PIXEL_NUM>;
using Ht_array = std::array<float, cameraConfig::PIXEL_NUM>;

// a Gaussian-smoothed histogram, row-major over the sensor (~400 KB)
using smoothed_histogram_array = std::array<float, cameraConfig::PIXEL_NUM>;

// Scratch for the separable LK patch sums: for every sensor row y and grid column gx, the 5 products
// (Hx^2, HxHy, Hy^2, Hx Htt, Hy Htt) summed across the patch width. Index (k * HEIGHT + y) * GRID_WIDTH + gx,
// k = product. ~2 MB at a dense 11x11 grid.
inline constexpr size_t LK_PRODUCT_NUM = 5;
using LK_row_sums_array = std::array<float, LK_PRODUCT_NUM * cameraConfig::HEIGHT * flowConfig::GRID_WIDTH>;

// pixel coordinate of a grid point's patch centre
inline constexpr size_t grid_to_pixel(size_t grid_index)
{
    return flowConfig::PATCH_RADIUS + grid_index * flowConfig::GRID_STRIDE;
}
