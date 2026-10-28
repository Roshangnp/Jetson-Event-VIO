#pragma once

// CUDA versions of lucas_kanade. Same stages as the CPU namespace, but every pointer is device memory
// and the work is queued on stream (asynchronous: errors from the kernel itself surface at the next
// synchronize). Launch errors throw std::runtime_error. The mask is one uint8_t per grid point
// (1 = valid); see histogram_filters_cuda.h.
//
// Each grid point is one thread. It computes the derivatives of every pixel in its patch itself, so
// a valid grid point never depends on derivatives at its (possibly masked) neighbours.

#include <cstdint>
#include <cuda_runtime_api.h>
#include "types.h"
#include "flow_types.h"

namespace lucas_kanade_cuda
{
    // structure matrix (A'A) per valid grid point from the spatial derivatives of d_hist (the newer
    // histogram). Masked grid points are zeroed. GAUSSIAN isn't implemented yet (throws std::logic_error).
    void getStructureMatrix_array(const histogram_array* d_hist, Structure_Matrix_array* d_structure_matrix_array,
                                  const uint8_t* d_mask, spatial_derivative method, cudaStream_t stream);

    // A'b per valid grid point: the spatial derivatives of d_hist_newer (method, which must match the one
    // given to getStructureMatrix_array) times the second-order temporal derivative
    // (SGF_HT * newer - SGF_HT * older) / window length. Masked grid points are zeroed. GAUSSIAN isn't
    // implemented yet (throws std::logic_error).
    void getTimeGradMatrix_array(const histogram_array* d_hist_newer, const histogram_array* d_hist_older,
                                 Time_Grad_Matrix_array* d_time_grad_matrix_array, const uint8_t* d_mask,
                                 spatial_derivative method, cudaStream_t stream);

    // solves A'A v = -A'b per valid grid point into d_flow->patches (flow in px/s, eigenvalues largest
    // first, eigenvectors as matching columns). Grid points failing the Shi-Tomasi condition (smaller
    // eigenvalue below flowConfig::SHI_TOMPSI_MIN_THRESH) are cleared in d_mask and marked invalid.
    // time_high_start / time_high_end aren't touched; the caller sets them.
    void LK_method(const Structure_Matrix_array* d_structure_matrix_array,
                   const Time_Grad_Matrix_array* d_time_grad_matrix_array,
                   flow_field* d_flow, uint8_t* d_mask, cudaStream_t stream);
}
