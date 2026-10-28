#include "lucas_kanade_cuda.h"

#include <cstddef>
#include <stdexcept>
#include "cudaUtil.h"
#include "histograms.h"

namespace
{
    constexpr int THREADS_PER_BLOCK = 256;
    // one thread per grid point
    constexpr int BLOCK_COUNT = (flowConfig::GRID_POINT_NUM + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    // one thread per pixel
    constexpr int PIXEL_BLOCK_COUNT = (cameraConfig::PIXEL_NUM + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    // one thread per (sensor row, grid column), for the row pass of the patch sums
    constexpr size_t ROW_SUM_NUM = cameraConfig::HEIGHT * flowConfig::GRID_WIDTH;
    constexpr int ROW_SUM_BLOCK_COUNT = (ROW_SUM_NUM + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;

    constexpr int W = cameraConfig::WIDTH;
    constexpr int H = cameraConfig::HEIGHT;
    constexpr int R = flowConfig::PATCH_RADIUS;

    // one TIME_HIGH tick is 64 us, so a histogram window lasts this many seconds
    constexpr float WINDOW_SECONDS = histograms::HISTOGRAM_TIME_HIGH_WIDTH * 64e-6f;

    // the config kernels, copied into constant memory (std::array isn't usable from device code directly)
    __constant__ filters::kernel3x3 c_sgf_hx = filters::SGF_HX_3D_FIRST_ORDER_KERNAL;
    __constant__ filters::kernel3x3 c_sgf_hy = filters::SGF_HY_3D_FIRST_ORDER_KERNAL;
    __constant__ filters::kernel3x3 c_sgf_ht = filters::SGF_HT_3D_FIRST_ORDER_KERNAL;

    // The 3x3 kernels reach one pixel past the sensor edge for border pixels, so reads are clamped
    // (border pixels repeated). Matches the CPU version.
    template <typename Image>
    __device__ inline float pixelAt(const Image& hist, int x, int y)
    {
        x = min(max(x, 0), W - 1);
        y = min(max(y, 0), H - 1);
        return hist[y * W + x];
    }

    // SGF dH/dx, dH/dy at one pixel
    template <typename Image>
    __device__ inline vector2d sgfXY(const Image& hist, int x, int y)
    {
        float gx = 0.f, gy = 0.f;
        #pragma unroll
        for (int ky = -1; ky <= 1; ++ky) {
            #pragma unroll
            for (int kx = -1; kx <= 1; ++kx) {
                int k = (ky + 1) * 3 + (kx + 1);
                float v = pixelAt(hist, x + kx, y + ky);
                gx += c_sgf_hx[k] * v;
                gy += c_sgf_hy[k] * v;
            }
        }
        return vector2d(gx, gy);
    }

    // SGF temporal kernel at one pixel
    template <typename Image>
    __device__ inline float sgfT(const Image& hist, int x, int y)
    {
        float t = 0.f;
        #pragma unroll
        for (int ky = -1; ky <= 1; ++ky) {
            #pragma unroll
            for (int kx = -1; kx <= 1; ++kx) {
                t += c_sgf_ht[(ky + 1) * 3 + (kx + 1)] * pixelAt(hist, x + kx, y + ky);
            }
        }
        return t;
    }

    // evaluate a structure matrix with the Shi-Tomasi criterion, given its smaller eigenvalue
    __device__ inline bool applyShiTompsiCondition(float min_eigenvalue)
    {
        return min_eigenvalue >= flowConfig::SHI_TOMPSI_MIN_THRESH;
    }

    template <typename Image>
    __global__ void hxySGFKernel(const Image* __restrict__ hist, Hxy_array* __restrict__ hxy)
    {
        size_t pixel_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (pixel_index >= cameraConfig::PIXEL_NUM) return;
        (*hxy)[pixel_index] = sgfXY(*hist, static_cast<int>(pixel_index % W), static_cast<int>(pixel_index / W));
    }

    template <typename Image>
    __global__ void htKernel(const Image* __restrict__ newer, const Image* __restrict__ older, Ht_array* __restrict__ ht)
    {
        size_t pixel_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (pixel_index >= cameraConfig::PIXEL_NUM) return;
        int x = static_cast<int>(pixel_index % W), y = static_cast<int>(pixel_index / W);
        // second-order temporal backward difference (Rueckauer & Delbruck 2016, eq. 5)
        (*ht)[pixel_index] = (sgfT(*newer, x, y) - sgfT(*older, x, y)) * (1.f / WINDOW_SECONDS);
    }

    // Row pass of the separable patch sums: for sensor row y and grid column gx, the 5 products summed over
    // the patch width around the grid column's centre pixel. Every row is summed, whatever the mask.
    __global__ void lkRowSumsKernel(const Hxy_array* __restrict__ hxy, const Ht_array* __restrict__ ht,
                                    LK_row_sums_array* __restrict__ row_sums)
    {
        size_t index = blockIdx.x * blockDim.x + threadIdx.x;
        if (index >= ROW_SUM_NUM) return;
        const int y = static_cast<int>(index / flowConfig::GRID_WIDTH);
        const int gx = static_cast<int>(index % flowConfig::GRID_WIDTH);
        // the grid keeps centres PATCH_RADIUS from the edge, so every patch pixel is on the sensor
        const int cx = static_cast<int>(grid_to_pixel(gx));
        float sxx = 0.f, sxy = 0.f, syy = 0.f, sxt = 0.f, syt = 0.f;
        #pragma unroll
        for (int dx = -R; dx <= R; ++dx) {
            const int pixel_index = y * W + cx + dx;
            const vector2d g = (*hxy)[pixel_index];
            const float htt = (*ht)[pixel_index];
            sxx += g.x() * g.x();
            sxy += g.x() * g.y();
            syy += g.y() * g.y();
            sxt += g.x() * htt;
            syt += g.y() * htt;
        }
        constexpr size_t PLANE = ROW_SUM_NUM;   // one plane per product, so neighbouring threads write neighbouring floats
        (*row_sums)[0 * PLANE + index] = sxx;
        (*row_sums)[1 * PLANE + index] = sxy;
        (*row_sums)[2 * PLANE + index] = syy;
        (*row_sums)[3 * PLANE + index] = sxt;
        (*row_sums)[4 * PLANE + index] = syt;
    }

    // Column pass: for each valid grid point, the row sums of the patch's rows, giving A'A and A'b.
    __global__ void lkColumnSumsKernel(const LK_row_sums_array* __restrict__ row_sums,
                                       Structure_Matrix_array* __restrict__ structure,
                                       Time_Grad_Matrix_array* __restrict__ time_grad, const uint8_t* __restrict__ mask)
    {
        size_t grid_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (grid_index >= flowConfig::GRID_POINT_NUM) return;
        if (!mask[grid_index]) {
            (*structure)[grid_index].setZero();
            (*time_grad)[grid_index].setZero();
            return;
        }
        const int gx = static_cast<int>(grid_index % flowConfig::GRID_WIDTH);
        const int cy = static_cast<int>(grid_to_pixel(grid_index / flowConfig::GRID_WIDTH));
        constexpr size_t PLANE = ROW_SUM_NUM;
        float sum[LK_PRODUCT_NUM] = {0.f, 0.f, 0.f, 0.f, 0.f};
        #pragma unroll
        for (int dy = -R; dy <= R; ++dy) {
            const size_t index = static_cast<size_t>(cy + dy) * flowConfig::GRID_WIDTH + gx;
            #pragma unroll
            for (size_t k = 0; k < LK_PRODUCT_NUM; ++k) {
                sum[k] += (*row_sums)[k * PLANE + index];
            }
        }
        Structure_Matrix m;
        m << sum[0], sum[1],
             sum[1], sum[2];
        (*structure)[grid_index] = m;
        (*time_grad)[grid_index] = Time_Grad_Matrix(sum[3], sum[4]);
    }

    __global__ void lucasKanadeKernel(const Structure_Matrix_array* __restrict__ structure,
                                      const Time_Grad_Matrix_array* __restrict__ time_grad,
                                      flow_field* __restrict__ flow, uint8_t* __restrict__ mask)
    {
        size_t grid_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (grid_index >= flowConfig::GRID_POINT_NUM) return;
        flow_patch& patch = flow->patches[grid_index];
        if (!mask[grid_index]) {
            patch.flow.setZero();
            patch.valid = false;
            return;
        }

        const Structure_Matrix& m = (*structure)[grid_index];
        float a = m(0, 0), b = m(0, 1), c = m(1, 1);

        // smaller eigenvalue of the symmetric 2x2 [[a, b], [b, c]], in closed form
        float half_diff = 0.5f * (a - c);
        float l2 = 0.5f * (a + c) - sqrtf(half_diff * half_diff + b * b);

        if (!applyShiTompsiCondition(l2)) {
            mask[grid_index] = 0;
            patch.flow.setZero();
            patch.valid = false;
            return;
        }

        // A'A v = -A'b. Shi-Tomasi passing means l2 > 0, so det = l1 * l2 > 0 (l1 >= l2).
        const Time_Grad_Matrix& t = (*time_grad)[grid_index];
        float sxt = t(0), syt = t(1);
        float inv_det = 1.f / (a * c - b * b);
        patch.flow = vector2d((-c * sxt + b * syt) * inv_det,
                              ( b * sxt - a * syt) * inv_det);
        patch.valid = true;
    }
}

namespace lucas_kanade_cuda
{
    void getHxy_array(const histogram_array* d_hist, Hxy_array* d_hxy_array, spatial_derivative method, cudaStream_t stream)
    {
        if (method == spatial_derivative::GAUSSIAN) {
            throw std::logic_error("lucas_kanade_cuda::getHxy_array: GAUSSIAN is not implemented");
        }
        hxySGFKernel<<<PIXEL_BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(d_hist, d_hxy_array);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getHxy_array launch");
    }

    void getHxy_array(const smoothed_histogram_array* d_hist, Hxy_array* d_hxy_array, spatial_derivative method,
                      cudaStream_t stream)
    {
        if (method == spatial_derivative::GAUSSIAN) {
            throw std::logic_error("lucas_kanade_cuda::getHxy_array: GAUSSIAN is not implemented");
        }
        hxySGFKernel<<<PIXEL_BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(d_hist, d_hxy_array);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getHxy_array launch");
    }

    void getHt_array(const histogram_array* d_hist_newer, const histogram_array* d_hist_older, Ht_array* d_ht_array,
                     cudaStream_t stream)
    {
        htKernel<<<PIXEL_BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(d_hist_newer, d_hist_older, d_ht_array);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getHt_array launch");
    }

    void getHt_array(const smoothed_histogram_array* d_hist_newer, const smoothed_histogram_array* d_hist_older,
                     Ht_array* d_ht_array, cudaStream_t stream)
    {
        htKernel<<<PIXEL_BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(d_hist_newer, d_hist_older, d_ht_array);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getHt_array launch");
    }

    void getLKMatrix_arrays(const Hxy_array* d_hxy_array, const Ht_array* d_ht_array, LK_row_sums_array* d_row_sums,
                            Structure_Matrix_array* d_structure_matrix_array,
                            Time_Grad_Matrix_array* d_time_grad_matrix_array, const uint8_t* d_mask, cudaStream_t stream)
    {
        lkRowSumsKernel<<<ROW_SUM_BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(d_hxy_array, d_ht_array, d_row_sums);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getLKMatrix_arrays row pass launch");
        lkColumnSumsKernel<<<BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(
            d_row_sums, d_structure_matrix_array, d_time_grad_matrix_array, d_mask);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getLKMatrix_arrays column pass launch");
    }

    void LK_method(const Structure_Matrix_array* d_structure_matrix_array,
                   const Time_Grad_Matrix_array* d_time_grad_matrix_array,
                   flow_field* d_flow, uint8_t* d_mask, cudaStream_t stream)
    {
        lucasKanadeKernel<<<BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(
            d_structure_matrix_array, d_time_grad_matrix_array, d_flow, d_mask);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::LK_method launch");
    }
}
