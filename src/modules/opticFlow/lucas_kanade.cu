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

    constexpr int W = cameraConfig::WIDTH;
    constexpr int H = cameraConfig::HEIGHT;
    constexpr int R = flowConfig::PATCH_RADIUS;

    // one TIME_HIGH tick is 64 us, so a histogram window lasts this many seconds
    constexpr float WINDOW_SECONDS = histograms::HISTOGRAM_TIME_HIGH_WIDTH * 64e-6f;

    // the config kernels, copied into constant memory (std::array isn't usable from device code directly)
    __constant__ filters::kernel3x3 c_sgf_hx = filters::SGF_HX_3D_FIRST_ORDER_KERNAL;
    __constant__ filters::kernel3x3 c_sgf_hy = filters::SGF_HY_3D_FIRST_ORDER_KERNAL;
    __constant__ filters::kernel3x3 c_sgf_ht = filters::SGF_HT_3D_FIRST_ORDER_KERNAL;

    // centre pixel of a grid point
    __device__ inline void gridToPixel(size_t grid_index, int& x, int& y)
    {
        x = static_cast<int>(grid_to_pixel(grid_index % flowConfig::GRID_WIDTH));
        y = static_cast<int>(grid_to_pixel(grid_index / flowConfig::GRID_WIDTH));
    }

    // The 3x3 kernels reach one pixel past the sensor edge for border pixels, so reads are clamped
    // (border pixels repeated). Matches the CPU version.
    __device__ inline float pixelAt(const histogram_array& hist, int x, int y)
    {
        x = min(max(x, 0), W - 1);
        y = min(max(y, 0), H - 1);
        return hist[y * W + x];
    }

    // SGF dH/dx, dH/dy at one pixel
    __device__ inline vector2d sgfXY(const histogram_array& hist, int x, int y)
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
    __device__ inline float sgfT(const histogram_array& hist, int x, int y)
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

    __global__ void hxySGFKernel(const histogram_array* __restrict__ hist, Hxy_array* __restrict__ hxy)
    {
        size_t pixel_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (pixel_index >= cameraConfig::PIXEL_NUM) return;
        (*hxy)[pixel_index] = sgfXY(*hist, static_cast<int>(pixel_index % W), static_cast<int>(pixel_index / W));
    }

    __global__ void htKernel(const histogram_array* __restrict__ newer, const histogram_array* __restrict__ older,
                             Ht_array* __restrict__ ht)
    {
        size_t pixel_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (pixel_index >= cameraConfig::PIXEL_NUM) return;
        int x = static_cast<int>(pixel_index % W), y = static_cast<int>(pixel_index / W);
        // second-order temporal backward difference (Rueckauer & Delbruck 2016, eq. 5)
        (*ht)[pixel_index] = (sgfT(*newer, x, y) - sgfT(*older, x, y)) * (1.f / WINDOW_SECONDS);
    }

    __global__ void lkMatrixKernel(const Hxy_array* __restrict__ hxy, const Ht_array* __restrict__ ht,
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

        // the grid keeps centres PATCH_RADIUS from the edge, so every patch pixel is on the sensor
        int cx, cy;
        gridToPixel(grid_index, cx, cy);
        float sxx = 0.f, sxy = 0.f, syy = 0.f, sxt = 0.f, syt = 0.f;
        #pragma unroll
        for (int dy = -R; dy <= R; ++dy) {
            #pragma unroll
            for (int dx = -R; dx <= R; ++dx) {
                int pixel_index = (cy + dy) * W + (cx + dx);
                vector2d g = (*hxy)[pixel_index];
                float htt = (*ht)[pixel_index];
                sxx += g.x() * g.x();
                sxy += g.x() * g.y();
                syy += g.y() * g.y();
                sxt += g.x() * htt;
                syt += g.y() * htt;
            }
        }
        Structure_Matrix m;
        m << sxx, sxy,
             sxy, syy;
        (*structure)[grid_index] = m;
        (*time_grad)[grid_index] = Time_Grad_Matrix(sxt, syt);
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

    void getHt_array(const histogram_array* d_hist_newer, const histogram_array* d_hist_older, Ht_array* d_ht_array,
                     cudaStream_t stream)
    {
        htKernel<<<PIXEL_BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(d_hist_newer, d_hist_older, d_ht_array);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getHt_array launch");
    }

    void getLKMatrix_arrays(const Hxy_array* d_hxy_array, const Ht_array* d_ht_array,
                            Structure_Matrix_array* d_structure_matrix_array,
                            Time_Grad_Matrix_array* d_time_grad_matrix_array, const uint8_t* d_mask, cudaStream_t stream)
    {
        lkMatrixKernel<<<BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(
            d_hxy_array, d_ht_array, d_structure_matrix_array, d_time_grad_matrix_array, d_mask);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getLKMatrix_arrays launch");
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
