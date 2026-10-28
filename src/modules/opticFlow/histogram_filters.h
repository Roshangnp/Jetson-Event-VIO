#pragma once

#include "types.h"
#include "flow_types.h"
#include "histograms.h"

namespace histogram_filters
{
    // The mask filters form a pipeline: each one only considers grid points whose bit is already
    // set in mask, and clears the bits of the ones it rejects. Start with mask.set() to consider
    // every grid point, then apply filters in any order.

    // The t-dt histogram is ignored here (only the newest histogram of the histogram pair is passed in).
    // method 1: clears grid points whose histogram value is in between -flowConfig::HISTOGRAM_MIN_THRESH
    // and flowConfig::HISTOGRAM_MIN_THRESH.
    void getValidHistMask_method1(const histogram_array& hist, grid_mask& mask);

    // method 2: applies the ridge condition using the eigenvalues of the Hessian matrix (consturcted using
    // applyGuassianConvolution_secondOrder) and clears grid points that fail it.
    void getValidHistMask_method2(const histogram_array& hist, grid_mask& mask);

    // gets the partial derivitives of the histogram at grid_index in the spacial directions (dH/dx,dH/dy) using SGF
    // required directly by Lucas-Kanade
    vector2d applySGF_XY(const histogram_array& hist, size_t grid_index);

    // gets the partial derivitive of the histogram at grid_index in the temporal directions (dH/dt)
    // required directly by Lucas-Kanade
    float applySGF_T(const histogram_array& hist, size_t grid_index);

    // gets the partial derivitives of the histogram at grid_index in the spacial directions (dH/dx,dH/dy) using guassian convolution
    // required directly by Lucas-Kanade (used in place of applySGF_XY)
    vector2d applyGuassianConvolution_firstOrder(const histogram_array& hist, size_t grid_index);

    // gets the second partial derivitives of the histogram at grid_index in the spacial directions (d2H/dx2,d2H/dy2) using guassian convolution
    vector2d applyGuassianConvolution_secondOrder(const histogram_array& hist, size_t grid_index);
};
