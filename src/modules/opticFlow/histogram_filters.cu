#include "histogram_filters_cuda.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include "cudaUtil.h"
#include "flow_types.h"

namespace
{
    constexpr int THREADS_PER_BLOCK = 256;
    constexpr int BLOCK_COUNT = (flowConfig::GRID_POINT_NUM + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    constexpr int PIXEL_BLOCK_COUNT = (cameraConfig::PIXEL_NUM + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;

    constexpr int SMOOTHING_RADIUS = filters::GUASSIAN_SMOOTHING_RADIUS;
    constexpr int SMOOTHING_WIDTH = 2 * SMOOTHING_RADIUS + 1;

    // 1D Gaussian weights, normalised; the 2D kernel is their outer product. Passed by value as a kernel
    // argument (std::exp isn't constexpr, so they're computed on the host).
    struct smoothing_weights {
        float w[SMOOTHING_WIDTH];
    };

    smoothing_weights makeSmoothingWeights()
    {
        smoothing_weights weights{};
        float sum = 0.f;
        for (int i = -SMOOTHING_RADIUS; i <= SMOOTHING_RADIUS; ++i) {
            const float sigma = filters::GUASSIAN_SMOOTHING_SIGMA;
            weights.w[i + SMOOTHING_RADIUS] = std::exp(-0.5f * static_cast<float>(i * i) / (sigma * sigma));
            sum += weights.w[i + SMOOTHING_RADIUS];
        }
        for (float& w : weights.w) {
            w /= sum;
        }
        return weights;
    }

    __global__ void gaussianSmoothingKernel(const histogram_array* __restrict__ hist,
                                            smoothed_histogram_array* __restrict__ smoothed, smoothing_weights weights)
    {
        size_t pixel_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (pixel_index >= cameraConfig::PIXEL_NUM) return;
        const int W = cameraConfig::WIDTH, H = cameraConfig::HEIGHT;
        const int x = static_cast<int>(pixel_index % W), y = static_cast<int>(pixel_index / W);
        float sum = 0.f;
        #pragma unroll
        for (int ky = -SMOOTHING_RADIUS; ky <= SMOOTHING_RADIUS; ++ky) {
            const int yy = min(max(y + ky, 0), H - 1);
            float row = 0.f;
            #pragma unroll
            for (int kx = -SMOOTHING_RADIUS; kx <= SMOOTHING_RADIUS; ++kx) {
                const int xx = min(max(x + kx, 0), W - 1);
                row += weights.w[kx + SMOOTHING_RADIUS] * (*hist)[yy * W + xx];
            }
            sum += weights.w[ky + SMOOTHING_RADIUS] * row;
        }
        (*smoothed)[pixel_index] = sum;
    }

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

    void applyGuassianSmoothing(const histogram_array* d_hist, smoothed_histogram_array* d_smoothed, cudaStream_t stream)
    {
        static const smoothing_weights weights = makeSmoothingWeights();
        gaussianSmoothingKernel<<<PIXEL_BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(d_hist, d_smoothed, weights);
        cuda_util::check(cudaGetLastError(), "histogram_filters_cuda::applyGuassianSmoothing launch");
    }
}
