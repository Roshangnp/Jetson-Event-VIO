#include "histogram_filters.h"

#include <stdexcept>

namespace histogram_filters
{
    void getValidHistMask_method1(const histogram_array& hist, grid_mask& mask)
    {
        const int thresh = flowConfig::HISTOGRAM_MIN_THRESH;
        for (size_t grid_index = 0; grid_index < flowConfig::GRID_POINT_NUM; ++grid_index) {
            if (!mask[grid_index]) continue;
            const size_t x = grid_to_pixel(grid_index % flowConfig::GRID_WIDTH);
            const size_t y = grid_to_pixel(grid_index / flowConfig::GRID_WIDTH);
            const int value = hist[y * cameraConfig::WIDTH + x];
            if (value > -thresh && value < thresh) {
                mask.reset(grid_index);
            }
        }
    }

    void getValidHistMask_method2(const histogram_array&, grid_mask&)
    {
        throw std::logic_error("histogram_filters::getValidHistMask_method2 is not implemented");
    }
}

namespace histogram_filters_helpers
{
    Hessian_Matrix applyGuassianConvolution_secondOrder(const histogram_array&, size_t)
    {
        throw std::logic_error("histogram_filters_helpers::applyGuassianConvolution_secondOrder is not implemented");
    }

    bool applyHessianRidgeCondition(const Hessian_Matrix&)
    {
        throw std::logic_error("histogram_filters_helpers::applyHessianRidgeCondition is not implemented");
    }
}
