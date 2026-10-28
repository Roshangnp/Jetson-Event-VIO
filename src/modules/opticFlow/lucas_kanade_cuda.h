#pragma once

// CUDA versions of lucas_kanade. Same stages as the CPU namespace, but every pointer is device memory
// and the work is queued on stream (asynchronous: errors from the kernel itself surface at the next
// synchronize). Launch errors throw std::runtime_error. The mask is one uint8_t per grid point
// (1 = valid); see histogram_filters_cuda.h.
//
// Runs in passes like the CPU path, so each pixel's derivatives are computed once:
//   getHxy_array + getHt_array (one thread per pixel) -> getLKMatrix_arrays -> LK_method (one thread per
//   grid point). The derivative passes cover every pixel regardless of the mask.

#include <cstdint>
#include <cuda_runtime_api.h>
#include "types.h"
#include "flow_types.h"

namespace lucas_kanade_cuda
{
    // spatial derivatives (dH/dx, dH/dy) of d_hist (the newer histogram) at every pixel. GAUSSIAN isn't
    // implemented yet (throws std::logic_error).
    void getHxy_array(const histogram_array* d_hist, Hxy_array* d_hxy_array, spatial_derivative method, cudaStream_t stream);

    // second-order temporal derivative at every pixel: (SGF_HT * newer - SGF_HT * older) / window length
    // (Rueckauer & Delbruck 2016, eq. 5)
    void getHt_array(const histogram_array* d_hist_newer, const histogram_array* d_hist_older, Ht_array* d_ht_array,
                     cudaStream_t stream);

    // sums the structure matrix (A'A) and A'b over each valid grid point's PATCH_WIDTH x PATCH_WIDTH patch.
    // Masked grid points are zeroed.
    void getLKMatrix_arrays(const Hxy_array* d_hxy_array, const Ht_array* d_ht_array,
                            Structure_Matrix_array* d_structure_matrix_array,
                            Time_Grad_Matrix_array* d_time_grad_matrix_array, const uint8_t* d_mask, cudaStream_t stream);

    // solves A'A v = -A'b per valid grid point into d_flow->patches (flow in px/s). Grid points failing
    // the Shi-Tomasi condition (smaller
    // eigenvalue below flowConfig::SHI_TOMPSI_MIN_THRESH) are cleared in d_mask and marked invalid.
    // time_high_start / time_high_end aren't touched; the caller sets them.
    void LK_method(const Structure_Matrix_array* d_structure_matrix_array,
                   const Time_Grad_Matrix_array* d_time_grad_matrix_array,
                   flow_field* d_flow, uint8_t* d_mask, cudaStream_t stream);
}
