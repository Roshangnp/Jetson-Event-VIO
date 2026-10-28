#pragma once

#include <memory>

#include "types.h"
#include "flow_types.h"
#include "histograms.h"
#include "histogram_filters.h"
#include "lucas_kanade.h"

// Directs a histogram pair through the optic flow pipeline (filter -> derivatives -> LK) on the CPU or
// the GPU, and owns every intermediate array. One object is ~4.9 MB (flow_field alone is ~1.2 MB at a
// dense 3x3 grid), so like histograms it must live in static storage or a single std::make_unique,
// never on the stack. Non-copyable.
//
// Each run reads both histograms of the pair; they must stay HELD until it returns. Both runs are
// synchronous: the GPU run waits for its results, so the pair can be queued as soon as it returns.
class optic_flow{
    public:
    spatial_derivative spacial_derivitive_type = spatial_derivative::SGF;
    hist_filter filter_type = hist_filter::NONE;

    optic_flow();
    ~optic_flow();

    optic_flow(const optic_flow&) = delete;
    optic_flow& operator=(const optic_flow&) = delete;

    // runs the LK optical flow pipeline on the CPU, updates time_high with the latest timeHigh from hist pair
    // (time_high_start of newest histogram + time_high_width)
    const flow_field& run_LK_method_CPU(histograms::histogram_buffer_pair& hist_pair);

    // runs the LK optical flow pipeline on the GPU, updates time_high with the latest timeHigh from hist pair
    // (time_high_start of newest histogram + time_high_width). The GPU buffers (~4 MB, plus a 1.2 MB
    // flow_field copy on a discrete GPU) are allocated on the first call, which throws std::runtime_error
    // if there is no usable CUDA device. On an integrated GPU (Jetson) flow_ is pinned and the GPU writes
    // it directly; on a discrete GPU it is copied back after the run.
    const flow_field& run_LK_method_GPU(histograms::histogram_buffer_pair& hist_pair);

    // releases the GPU buffers; the next run_LK_method_GPU allocates them again. Also called by the destructor.
    void stop() noexcept;

    // the TIME_HIGH the latest run ended at (newer window start + HISTOGRAM_TIME_HIGH_WIDTH)
    const size_t& getTimeHigh() const { return time_high_; }

    private:
    // device copies of the pipeline arrays plus the stream; defined in optic_flow.cpp so this header
    // doesn't need the CUDA headers
    struct gpu_buffers;

    // checks the pair is complete and updates time_high_ and the flow_field's TIME_HIGH span
    void beginRun(const histograms::histogram_buffer_pair& hist_pair);

    flow_field flow_;
    grid_mask mask_;
    size_t time_high_ = 0;
    Structure_Matrix_array structure_matrix_array_;
    Time_Grad_Matrix_array time_grad_matrix_array_;
    Hxy_array hxy_array_;
    Ht_array ht_array_;
    std::unique_ptr<gpu_buffers> gpu_;
};
