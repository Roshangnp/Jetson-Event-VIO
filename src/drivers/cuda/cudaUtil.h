#pragma once

// Header-only CUDA runtime helpers shared by the .cu modules, like v4l2Util.h for V4L2.

#include <cuda_runtime_api.h>
#include <stdexcept>
#include <string>

namespace cuda_util
{
    // throws std::runtime_error naming the failed call and CUDA's reason
    inline void check(cudaError_t result, const char* what)
    {
        if (result != cudaSuccess) {
            throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(result));
        }
    }
}
