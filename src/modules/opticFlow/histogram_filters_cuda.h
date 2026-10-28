#pragma once

// CUDA versions of histogram_filters. Same stages and mask semantics as the CPU namespace, but every
// pointer is device memory and the work is queued on stream (asynchronous: errors from the kernel
// itself surface at the next synchronize). Launch errors throw std::runtime_error.
//
// On the device the mask is one uint8_t per grid point (1 = valid), not grid_mask: std::bitset
// can't be used in device code, and bytes let every thread write its own grid point without atomics.

#include <cstdint>
#include <cuda_runtime_api.h>
#include "types.h"
#include "flow_types.h"

namespace histogram_filters_cuda
{
    // method 1: clears grid points whose histogram value at the patch centre is in between
    // -flowConfig::HISTOGRAM_MIN_THRESH and flowConfig::HISTOGRAM_MIN_THRESH.
    void getValidHistMask_method1(const histogram_array* d_hist, uint8_t* d_mask, cudaStream_t stream);

    // method 2: Hessian ridge condition. Not implemented yet (the Gaussian kernels and the ridge
    // criterion aren't defined); throws std::logic_error.
    void getValidHistMask_method2(const histogram_array* d_hist, uint8_t* d_mask, cudaStream_t stream);

    // smooths d_hist with a normalised (2 * GUASSIAN_SMOOTHING_RADIUS + 1)^2 Gaussian of sigma
    // GUASSIAN_SMOOTHING_SIGMA (filters, config.h) into d_smoothed, at every pixel. Reads past the sensor
    // edge are clamped (border pixels repeated), like the derivative kernels. No CPU version yet.
    void applyGuassianSmoothing(const histogram_array* d_hist, smoothed_histogram_array* d_smoothed, cudaStream_t stream);
}
