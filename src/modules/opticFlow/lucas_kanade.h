#pragma once

#include "types.h"
#include "flow_types.h"
#include "histograms.h"

namespace lucas_kanade
{
    // The CPU path runs in passes, like the CUDA kernels: derivative images are computed once per pixel,
    // then summed over each valid grid point's patch, so no pixel's derivatives are computed twice.
    //   getHxy_array + getHt_array -> getLKMatrix_arrays -> LK_method

    // spatial derivatives (dH/dx, dH/dy) of the newer histogram at every pixel, by SGF or Gaussian
    // convolution. GAUSSIAN isn't implemented yet (throws std::logic_error).
    void getHxy_array(const histogram_array& hist, Hxy_array& hxy_array, spatial_derivative method);

    // second-order temporal derivative at every pixel: (SGF_HT * newer - SGF_HT * older) / window length
    // (Rueckauer & Delbruck 2016, eq. 5)
    void getHt_array(const histogram_array& hist_newer, const histogram_array& hist_older, Ht_array& ht_array);

    // sums the structure matrix (A'A) and A'b over each valid grid point's PATCH_WIDTH x PATCH_WIDTH patch,
    // separably: a row pass into row_sums (scratch), then a column pass per grid point, so 2 * PATCH_WIDTH
    // terms per grid point instead of PATCH_WIDTH^2. Masked grid points are zeroed.
    void getLKMatrix_arrays(const Hxy_array& hxy_array, const Ht_array& ht_array, LK_row_sums_array& row_sums,
                            Structure_Matrix_array& structure_matrix_array,
                            Time_Grad_Matrix_array& time_grad_matrix_array, const grid_mask& mask);

    // Using the Structure_Matrix_array and the Time_Grad_Matrix_array computes flow feild while accounting for the valid mask created by the filters further refined the mask with the shi tompsi helper function
    // the mask may be used after the optic flow computations so it must be updated with the shi tompsi evaluations.
    void LK_method(const Structure_Matrix_array& structure_matrix_array, const Time_Grad_Matrix_array& time_grad_matrix_array, flow_field& flow, grid_mask& mask);
};

namespace lucas_kanade_helpers
{
    // pixel_index is row-major over the sensor (y * WIDTH + x): LK needs derivatives at every pixel in a
    // patch, not just at grid points. Reads past the sensor edge are clamped (border pixels repeated).

    // gets the partial derivitives of the histogram at pixel_index in the spacial directions (dH/dx,dH/dy) using SGF
    // required directly by Lucas-Kanade
    vector2d applySGF_XY(const histogram_array& hist, size_t pixel_index);

    // gets the partial derivitive of the histogram at pixel_index in the temporal directions (dH/dt)
    // required directly by Lucas-Kanade
    float applySGF_T(const histogram_array& hist, size_t pixel_index);

    // gets the partial derivitives of the histogram at pixel_index in the spacial directions (dH/dx,dH/dy) using guassian convolution
    // required directly by Lucas-Kanade (used in place of applySGF_XY)
    vector2d applyGuassianConvolution_firstOrder(const histogram_array& hist, size_t pixel_index);

    // evaluate a structure matrix with the shi tompsi criteria
    bool applyShiTompsiCondition(const Structure_Matrix& structure_matrix);

    // returns the LK matrices per grid. Uses ether SGF or guassian convolutions for the structure matrix, the timeGradMatrix always uses SGF 
    Structure_Matrix getStructureMatrix_GuassianConvolution(const histogram_array& hist, size_t grid_index);
}