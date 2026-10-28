#include "lucas_kanade.h"

#include <cmath>
#include <stdexcept>

namespace
{
    constexpr int W = cameraConfig::WIDTH;
    constexpr int H = cameraConfig::HEIGHT;

    // pixel_index is row-major over the sensor (y * WIDTH + x)
    void pixel_index_to_pixel_coord(size_t pixel_index, int& x, int& y)
    {
        x = static_cast<int>(pixel_index % W);
        y = static_cast<int>(pixel_index / W);
    }

    // The 3x3 kernels reach one pixel past the sensor edge for border pixels, so edge reads are
    // clamped (border pixels repeated). Matches the CUDA version.
    float pixelAt(const histogram_array& hist, int x, int y)
    {
        x = x < 0 ? 0 : (x >= W ? W - 1 : x);
        y = y < 0 ? 0 : (y >= H ? H - 1 : y);
        return hist[y * W + x];
    }

    // 3x3 kernel (row-major, row = y) centred on pixel (x, y)
    float convolve3x3(const histogram_array& hist, const filters::kernel3x3& kernel, int x, int y)
    {
        float sum = 0.f;
        for (int ky = -1; ky <= 1; ++ky) {
            for (int kx = -1; kx <= 1; ++kx) {
                sum += kernel[(ky + 1) * 3 + (kx + 1)] * pixelAt(hist, x + kx, y + ky);
            }
        }
        return sum;
    }

    // 3x3 kernel centred on an interior pixel (not on the 1-pixel border), so no clamping is needed
    inline float convolveInterior(const histogram_array& hist, const filters::kernel3x3& kernel, size_t pixel_index)
    {
        const int16_t* centre = hist.data() + pixel_index;
        float sum = 0.f;
        for (int ky = -1; ky <= 1; ++ky) {
            for (int kx = -1; kx <= 1; ++kx) {
                sum += kernel[(ky + 1) * 3 + (kx + 1)] * centre[ky * W + kx];
            }
        }
        return sum;
    }

    constexpr int R = flowConfig::PATCH_RADIUS;

    // one TIME_HIGH tick is 64 us, so a histogram window lasts this many seconds
    constexpr float WINDOW_SECONDS = histograms::HISTOGRAM_TIME_HIGH_WIDTH * 64e-6f;

    // pixel index of the centre of a grid point's patch
    size_t gridToCentrePixelIndex(size_t grid_index)
    {
        const size_t x = grid_to_pixel(grid_index % flowConfig::GRID_WIDTH);
        const size_t y = grid_to_pixel(grid_index / flowConfig::GRID_WIDTH);
        return y * W + x;
    }

    // the Shi-Tomasi test on a structure matrix's smaller eigenvalue
    bool passesShiTomasi(float min_eigenvalue)
    {
        return min_eigenvalue >= flowConfig::SHI_TOMPSI_MIN_THRESH;
    }

    // smaller eigenvalue of a symmetric 2x2 [[a, b], [b, c]], in closed form. Matches the CUDA version.
    float minEigenvalue(const Structure_Matrix& m)
    {
        const float a = m(0, 0), b = m(0, 1), c = m(1, 1);
        const float half_diff = 0.5f * (a - c);
        return 0.5f * (a + c) - std::sqrt(half_diff * half_diff + b * b);
    }
}

namespace lucas_kanade
{
    void getHxy_array(const histogram_array& hist, Hxy_array& hxy_array, spatial_derivative method)
    {
        if (method == spatial_derivative::GAUSSIAN) {
            throw std::logic_error("lucas_kanade::getHxy_array: GAUSSIAN is not implemented");
        }
        for (int y = 0; y < H; ++y) {
            const bool border_row = y == 0 || y == H - 1;
            for (int x = 0; x < W; ++x) {
                const size_t pixel_index = static_cast<size_t>(y * W + x);
                if (border_row || x == 0 || x == W - 1) {
                    hxy_array[pixel_index] = lucas_kanade_helpers::applySGF_XY(hist, pixel_index);
                } else {
                    hxy_array[pixel_index] = vector2d(convolveInterior(hist, filters::SGF_HX_3D_FIRST_ORDER_KERNAL, pixel_index),
                                                      convolveInterior(hist, filters::SGF_HY_3D_FIRST_ORDER_KERNAL, pixel_index));
                }
            }
        }
    }

    void getHt_array(const histogram_array& hist_newer, const histogram_array& hist_older, Ht_array& ht_array)
    {
        constexpr float inv_window = 1.f / WINDOW_SECONDS;
        for (int y = 0; y < H; ++y) {
            const bool border_row = y == 0 || y == H - 1;
            for (int x = 0; x < W; ++x) {
                const size_t pixel_index = static_cast<size_t>(y * W + x);
                float newer, older;
                if (border_row || x == 0 || x == W - 1) {
                    newer = lucas_kanade_helpers::applySGF_T(hist_newer, pixel_index);
                    older = lucas_kanade_helpers::applySGF_T(hist_older, pixel_index);
                } else {
                    newer = convolveInterior(hist_newer, filters::SGF_HT_3D_FIRST_ORDER_KERNAL, pixel_index);
                    older = convolveInterior(hist_older, filters::SGF_HT_3D_FIRST_ORDER_KERNAL, pixel_index);
                }
                ht_array[pixel_index] = (newer - older) * inv_window;
            }
        }
    }

    void getLKMatrix_arrays(const Hxy_array& hxy_array, const Ht_array& ht_array, Structure_Matrix_array& structure_matrix_array,
                            Time_Grad_Matrix_array& time_grad_matrix_array, const grid_mask& mask)
    {
        for (size_t grid_index = 0; grid_index < flowConfig::GRID_POINT_NUM; ++grid_index) {
            if (!mask[grid_index]) {
                structure_matrix_array[grid_index].setZero();
                time_grad_matrix_array[grid_index].setZero();
                continue;
            }
            // the grid keeps centres PATCH_RADIUS from the edge, so every patch pixel is on the sensor
            const int centre = static_cast<int>(gridToCentrePixelIndex(grid_index));
            float sxx = 0.f, sxy = 0.f, syy = 0.f, sxt = 0.f, syt = 0.f;
            for (int dy = -R; dy <= R; ++dy) {
                for (int dx = -R; dx <= R; ++dx) {
                    const size_t pixel_index = static_cast<size_t>(centre + dy * W + dx);
                    const vector2d& g = hxy_array[pixel_index];
                    const float htt = ht_array[pixel_index];
                    sxx += g.x() * g.x();
                    sxy += g.x() * g.y();
                    syy += g.y() * g.y();
                    sxt += g.x() * htt;
                    syt += g.y() * htt;
                }
            }
            structure_matrix_array[grid_index] << sxx, sxy,
                                                  sxy, syy;
            time_grad_matrix_array[grid_index] = Time_Grad_Matrix(sxt, syt);
        }
    }

    void LK_method(const Structure_Matrix_array& structure_matrix_array, const Time_Grad_Matrix_array& time_grad_matrix_array, flow_field& flow, grid_mask& mask)
    {
        for (size_t grid_index = 0; grid_index < flowConfig::GRID_POINT_NUM; ++grid_index) {
            flow_patch& patch = flow.patches[grid_index];
            if (!mask[grid_index]) {
                patch.flow.setZero();
                patch.valid = false;
                continue;
            }

            const Structure_Matrix& m = structure_matrix_array[grid_index];
            if (!passesShiTomasi(minEigenvalue(m))) {
                mask.reset(grid_index);
                patch.flow.setZero();
                patch.valid = false;
                continue;
            }

            // A'A v = -A'b. Shi-Tomasi passing means the smaller eigenvalue is > 0, so det > 0.
            const float a = m(0, 0), b = m(0, 1), c = m(1, 1);
            const float sxt = time_grad_matrix_array[grid_index](0);
            const float syt = time_grad_matrix_array[grid_index](1);
            const float inv_det = 1.f / (a * c - b * b);
            patch.flow = vector2d((-c * sxt + b * syt) * inv_det,
                                  ( b * sxt - a * syt) * inv_det);
            patch.valid = true;
        }
    }
}

namespace lucas_kanade_helpers
{
    vector2d applySGF_XY(const histogram_array& hist, size_t pixel_index)
    {
        int x, y;
        pixel_index_to_pixel_coord(pixel_index, x, y);
        return vector2d(convolve3x3(hist, filters::SGF_HX_3D_FIRST_ORDER_KERNAL, x, y),
                        convolve3x3(hist, filters::SGF_HY_3D_FIRST_ORDER_KERNAL, x, y));
    }

    float applySGF_T(const histogram_array& hist, size_t pixel_index)
    {
        int x, y;
        pixel_index_to_pixel_coord(pixel_index, x, y);
        return convolve3x3(hist, filters::SGF_HT_3D_FIRST_ORDER_KERNAL, x, y);
    }

    vector2d applyGuassianConvolution_firstOrder(const histogram_array&, size_t)
    {
        throw std::logic_error("lucas_kanade_helpers::applyGuassianConvolution_firstOrder is not implemented");
    }

    bool applyShiTompsiCondition(const Structure_Matrix& structure_matrix)
    {
        return passesShiTomasi(minEigenvalue(structure_matrix));
    }
}
