#include "histogram_filters_cuda.h"

#include <cstddef>
#include <stdexcept>
#include "cudaUtil.h"
#include "flow_types.h"

namespace
{
    constexpr int THREADS_PER_BLOCK = 256;
    constexpr int BLOCK_COUNT = (flowConfig::GRID_POINT_NUM + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;

    __global__ void method1Kernel(const histogram_array* __restrict__ hist, uint8_t* __restrict__ mask)
    {
        size_t grid_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (grid_index >= flowConfig::GRID_POINT_NUM || !mask[grid_index]) return;

        size_t x = grid_to_pixel(grid_index % flowConfig::GRID_WIDTH);
        size_t y = grid_to_pixel(grid_index / flowConfig::GRID_WIDTH);
        int value = (*hist)[y * cameraConfig::WIDTH + x];
        if (value > -flowConfig::HISTOGRAM_MIN_THRESH && value < flowConfig::HISTOGRAM_MIN_THRESH) {
            mask[grid_index] = 0;
        }
    }
}

namespace histogram_filters_cuda
{
    void getValidHistMask_method1(const histogram_array* d_hist, uint8_t* d_mask, cudaStream_t stream)
    {
        method1Kernel<<<BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(d_hist, d_mask);
        cuda_util::check(cudaGetLastError(), "histogram_filters_cuda::getValidHistMask_method1 launch");
    }

    void getValidHistMask_method2(const histogram_array*, uint8_t*, cudaStream_t)
    {
        throw std::logic_error("histogram_filters_cuda::getValidHistMask_method2 is not implemented");
    }
}
