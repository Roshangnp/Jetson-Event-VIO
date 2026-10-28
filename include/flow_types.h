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
using eigenValues2d = Eigen::Vector2f;
using eigenVectors2d = Eigen::Matrix2f;

// one bit per grid point, indexed by grid_index (row-major). A set bit marks a grid point
// still worth running LK on. ~12.6 KB at a dense 3x3 grid, so pass it by reference.
using grid_mask = std::bitset<flowConfig::GRID_POINT_NUM>;

struct flow_patch {
    vector2d flow;
    eigenValues2d eigenValues;
    eigenVectors2d eigenVectors;
    // false if the patch had too few events or only one edge direction (aperture problem)
    bool valid;
};

// One flow estimate per grid point, row-major. Dense 5x5 is ~1.6 MB, so like histograms it
// must live in static storage or a single std::make_unique, never on the stack.
struct flow_field {
    // TIME_HIGH span the flow was measured over (older window start -> newer window start)
    uint64_t time_high_start;
    uint64_t time_high_end;
    std::array<flow_patch, flowConfig::GRID_POINT_NUM> patches;
};

// pixel coordinate of a grid point's patch centre
inline constexpr size_t grid_to_pixel(size_t grid_index)
{
    return flowConfig::PATCH_RADIUS + grid_index * flowConfig::GRID_STRIDE;
}
