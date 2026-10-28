#include "lucas_kanade_cuda.h"

#include <cstddef>
#include <stdexcept>
#include "cudaUtil.h"
#include "histograms.h"

namespace
{
    constexpr int THREADS_PER_BLOCK = 256;
    constexpr int BLOCK_COUNT = (flowConfig::GRID_POINT_NUM + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;

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

    // The grid only keeps PATCH_RADIUS pixels from the edge, but the 3x3 kernels reach one further,
    // so edge reads are clamped (border pixels repeated).
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

    __global__ void structureMatrixSGFKernel(const histogram_array* __restrict__ hist,
                                             Structure_Matrix_array* __restrict__ out,
                                             const uint8_t* __restrict__ mask)
    {
        size_t grid_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (grid_index >= flowConfig::GRID_POINT_NUM) return;
        if (!mask[grid_index]) { (*out)[grid_index].setZero(); return; }

        int cx, cy;
        gridToPixel(grid_index, cx, cy);
        float sxx = 0.f, sxy = 0.f, syy = 0.f;
        #pragma unroll
        for (int dy = -R; dy <= R; ++dy) {
            #pragma unroll
            for (int dx = -R; dx <= R; ++dx) {
                vector2d g = sgfXY(*hist, cx + dx, cy + dy);
                sxx += g.x() * g.x();
                sxy += g.x() * g.y();
                syy += g.y() * g.y();
            }
        }
        Structure_Matrix m;
        m << sxx, sxy,
             sxy, syy;
        (*out)[grid_index] = m;
    }

    __global__ void timeGradMatrixKernel(const histogram_array* __restrict__ newer, const histogram_array* __restrict__ older,
                                         Time_Grad_Matrix_array* __restrict__ out, const uint8_t* __restrict__ mask)
    {
        size_t grid_index = blockIdx.x * blockDim.x + threadIdx.x;
        if (grid_index >= flowConfig::GRID_POINT_NUM) return;
        Time_Grad_Matrix t = Time_Grad_Matrix::Zero();
        if (mask[grid_index]) {
            int cx, cy;
            gridToPixel(grid_index, cx, cy);
            float sxt = 0.f, syt = 0.f;
            #pragma unroll
            for (int dy = -R; dy <= R; ++dy) {
                #pragma unroll
                for (int dx = -R; dx <= R; ++dx) {
                    int x = cx + dx, y = cy + dy;
                    vector2d g = sgfXY(*newer, x, y);
                    // second-order temporal backward difference (Rueckauer & Delbruck 2016, eq. 5)
                    float htt = (sgfT(*newer, x, y) - sgfT(*older, x, y)) * (1.f / WINDOW_SECONDS);
                    sxt += g.x() * htt;
                    syt += g.y() * htt;
                }
            }
            t(0) = sxt;
            t(1) = syt;
        }
        (*out)[grid_index] = t;
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
            patch.eigenValues.setZero();
            patch.eigenVectors.setIdentity();
            patch.valid = false;
            return;
        }

        const Structure_Matrix& m = (*structure)[grid_index];
        float a = m(0, 0), b = m(0, 1), c = m(1, 1);

        // closed-form eigen decomposition of the symmetric 2x2 [[a, b], [b, c]], largest first
        float mean = 0.5f * (a + c), half_diff = 0.5f * (a - c);
        float root = sqrtf(half_diff * half_diff + b * b);
        float l1 = mean + root, l2 = mean - root;
        vector2d v1;
        if (fabsf(b) > 1e-12f) {
            v1 = vector2d(l1 - c, b).normalized();
        } else {
            v1 = (a >= c) ? vector2d(1.f, 0.f) : vector2d(0.f, 1.f);
        }
        patch.eigenValues = eigenValues2d(l1, l2);
        patch.eigenVectors.col(0) = v1;
        patch.eigenVectors.col(1) = vector2d(-v1.y(), v1.x());

        if (!applyShiTompsiCondition(l2)) {
            mask[grid_index] = 0;
            patch.flow.setZero();
            patch.valid = false;
            return;
        }

        // A'A v = -A'b. Shi-Tomasi passing means l2 > 0, so det = l1 * l2 > 0.
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
    void getStructureMatrix_array(const histogram_array* d_hist, Structure_Matrix_array* d_structure_matrix_array,
                                  const uint8_t* d_mask, spatial_derivative method, cudaStream_t stream)
    {
        if (method == spatial_derivative::GAUSSIAN) {
            throw std::logic_error("lucas_kanade_cuda::getStructureMatrix_array: GAUSSIAN is not implemented");
        }
        structureMatrixSGFKernel<<<BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(
            d_hist, d_structure_matrix_array, d_mask);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getStructureMatrix_array launch");
    }

    void getTimeGradMatrix_array(const histogram_array* d_hist_newer, const histogram_array* d_hist_older,
                                 Time_Grad_Matrix_array* d_time_grad_matrix_array, const uint8_t* d_mask,
                                 spatial_derivative method, cudaStream_t stream)
    {
        if (method == spatial_derivative::GAUSSIAN) {
            throw std::logic_error("lucas_kanade_cuda::getTimeGradMatrix_array: GAUSSIAN is not implemented");
        }
        timeGradMatrixKernel<<<BLOCK_COUNT, THREADS_PER_BLOCK, 0, stream>>>(
            d_hist_newer, d_hist_older, d_time_grad_matrix_array, d_mask);
        cuda_util::check(cudaGetLastError(), "lucas_kanade_cuda::getTimeGradMatrix_array launch");
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
