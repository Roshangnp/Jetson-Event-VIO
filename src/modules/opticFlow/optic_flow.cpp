#include "optic_flow.h"

#include <cuda_runtime_api.h>
#include <stdexcept>

#include "cudaUtil.h"
#include "histogram_filters_cuda.h"
#include "lucas_kanade_cuda.h"

namespace
{
    template <typename T>
    void deviceAlloc(T*& pointer, size_t bytes, const char* what)
    {
        cuda_util::check(cudaMalloc(reinterpret_cast<void**>(&pointer), bytes), what);
    }
}

// Device copies of the pipeline arrays. Allocated once and kept until stop(), so a run never allocates.
struct optic_flow::gpu_buffers {
    histogram_array* pair = nullptr;   // [0] newer, [1] older
    smoothed_histogram_array* smoothed_pair = nullptr;   // [0] newer, [1] older, when gaussian_smoothing
    Hxy_array* hxy_array = nullptr;
    Ht_array* ht_array = nullptr;
    LK_row_sums_array* row_sums = nullptr;
    Structure_Matrix_array* structure_matrix_array = nullptr;
    Time_Grad_Matrix_array* time_grad_matrix_array = nullptr;
    uint8_t* mask = nullptr;           // one byte per grid point, 1 = valid
    cudaStream_t stream = nullptr;

    // Where LK writes the flow_field. On an integrated GPU (Jetson) the GPU shares the CPU's DRAM, so
    // optic_flow's own flow_ is pinned and mapped and the GPU writes it in place: no 1.2 MB copy back.
    // On a discrete GPU that would mean writing over PCIe, so it's a device copy copied back after the run.
    flow_field* flow = nullptr;
    // flow_ while it is registered (integrated GPU), else nullptr
    flow_field* mapped_host_flow = nullptr;

    explicit gpu_buffers(flow_field& host_flow)
    {
        try {
            int device = 0;
            int integrated = 0;
            cuda_util::check(cudaGetDevice(&device), "optic_flow: cudaGetDevice");
            cuda_util::check(cudaDeviceGetAttribute(&integrated, cudaDevAttrIntegrated, device),
                             "optic_flow: cudaDeviceGetAttribute(integrated)");

            cuda_util::check(cudaStreamCreate(&stream), "optic_flow: cudaStreamCreate");
            deviceAlloc(pair, 2 * sizeof(histogram_array), "optic_flow: cudaMalloc histogram pair");
            deviceAlloc(smoothed_pair, 2 * sizeof(smoothed_histogram_array), "optic_flow: cudaMalloc smoothed pair");
            deviceAlloc(hxy_array, sizeof(Hxy_array), "optic_flow: cudaMalloc spatial derivatives");
            deviceAlloc(ht_array, sizeof(Ht_array), "optic_flow: cudaMalloc temporal derivatives");
            deviceAlloc(row_sums, sizeof(LK_row_sums_array), "optic_flow: cudaMalloc patch row sums");
            deviceAlloc(structure_matrix_array, sizeof(Structure_Matrix_array), "optic_flow: cudaMalloc structure matrices");
            deviceAlloc(time_grad_matrix_array, sizeof(Time_Grad_Matrix_array), "optic_flow: cudaMalloc time gradients");
            deviceAlloc(mask, flowConfig::GRID_POINT_NUM, "optic_flow: cudaMalloc mask");

            if (integrated) {
                cuda_util::check(cudaHostRegister(&host_flow, sizeof(flow_field), cudaHostRegisterMapped),
                                 "optic_flow: cudaHostRegister flow field");
                mapped_host_flow = &host_flow;
                // the Orin reports canUseHostPointerForRegisteredMem = 0, so the kernel needs the device alias
                cuda_util::check(cudaHostGetDevicePointer(reinterpret_cast<void**>(&flow), &host_flow, 0),
                                 "optic_flow: cudaHostGetDevicePointer flow field");
            } else {
                deviceAlloc(flow, sizeof(flow_field), "optic_flow: cudaMalloc flow field");
            }
        } catch (...) {
            release();
            throw;
        }
    }

    ~gpu_buffers() { release(); }

    gpu_buffers(const gpu_buffers&) = delete;
    gpu_buffers& operator=(const gpu_buffers&) = delete;

    void release() noexcept
    {
        // errors are ignored: there is nothing useful to do with them while freeing
        if (stream != nullptr) {
            (void)cudaStreamSynchronize(stream);
        }
        (void)cudaFree(pair);
        (void)cudaFree(smoothed_pair);
        (void)cudaFree(hxy_array);
        (void)cudaFree(ht_array);
        (void)cudaFree(row_sums);
        (void)cudaFree(structure_matrix_array);
        (void)cudaFree(time_grad_matrix_array);
        (void)cudaFree(mask);
        if (mapped_host_flow != nullptr) {
            (void)cudaHostUnregister(mapped_host_flow);
        } else {
            (void)cudaFree(flow);
        }
        if (stream != nullptr) {
            (void)cudaStreamDestroy(stream);
        }
        pair = nullptr;
        smoothed_pair = nullptr;
        hxy_array = nullptr;
        ht_array = nullptr;
        row_sums = nullptr;
        structure_matrix_array = nullptr;
        time_grad_matrix_array = nullptr;
        mask = nullptr;
        flow = nullptr;
        mapped_host_flow = nullptr;
        stream = nullptr;
    }
};

optic_flow::optic_flow() = default;

optic_flow::~optic_flow()
{
    stop();
}

void optic_flow::stop() noexcept
{
    gpu_.reset();
}

void optic_flow::beginRun(const histograms::histogram_buffer_pair& hist_pair)
{
    if (hist_pair[0] == nullptr || hist_pair[1] == nullptr ||
        hist_pair[0]->getHistogram() == nullptr || hist_pair[1]->getHistogram() == nullptr) {
        throw std::logic_error("optic_flow needs a dequeued (READY or HELD) histogram pair");
    }
    time_high_ = hist_pair[0]->getTimeHighStart() + histograms::HISTOGRAM_TIME_HIGH_WIDTH;
    flow_.time_high_start = hist_pair[1]->getTimeHighStart();
    flow_.time_high_end = time_high_;
}

const flow_field& optic_flow::run_LK_method_CPU(histograms::histogram_buffer_pair& hist_pair)
{
    if (gaussian_smoothing) {
        throw std::logic_error("optic_flow: Gaussian smoothing has no CPU version yet; set gaussian_smoothing = false");
    }
    beginRun(hist_pair);
    const histogram_array& newer = *hist_pair[0]->getHistogram();
    const histogram_array& older = *hist_pair[1]->getHistogram();

    mask_.set();
    switch (filter_type) {
        case hist_filter::NONE:
            break;
        case hist_filter::MIN_MAGNITUDE:
            histogram_filters::getValidHistMask_method1(newer, mask_);
            break;
        case hist_filter::HESSIAN_RIDGE:
            histogram_filters::getValidHistMask_method2(newer, mask_);
            break;
    }

    lucas_kanade::getHxy_array(newer, hxy_array_, spacial_derivitive_type);
    lucas_kanade::getHt_array(newer, older, ht_array_);
    lucas_kanade::getLKMatrix_arrays(hxy_array_, ht_array_, row_sums_, structure_matrix_array_, time_grad_matrix_array_, mask_);
    lucas_kanade::LK_method(structure_matrix_array_, time_grad_matrix_array_, flow_, mask_);
    return flow_;
}

const flow_field& optic_flow::run_LK_method_GPU(histograms::histogram_buffer_pair& hist_pair)
{
    beginRun(hist_pair);
    if (!gpu_) {
        gpu_ = std::make_unique<gpu_buffers>(flow_);
    }
    gpu_buffers& gpu = *gpu_;

    try {
        cuda_util::check(cudaMemcpyAsync(&gpu.pair[0], hist_pair[0]->getHistogram(), sizeof(histogram_array),
                                         cudaMemcpyHostToDevice, gpu.stream), "optic_flow: copy newer histogram");
        cuda_util::check(cudaMemcpyAsync(&gpu.pair[1], hist_pair[1]->getHistogram(), sizeof(histogram_array),
                                         cudaMemcpyHostToDevice, gpu.stream), "optic_flow: copy older histogram");

        // mask.set()
        cuda_util::check(cudaMemsetAsync(gpu.mask, 1, flowConfig::GRID_POINT_NUM, gpu.stream), "optic_flow: reset mask");
        switch (filter_type) {
            case hist_filter::NONE:
                break;
            case hist_filter::MIN_MAGNITUDE:
                histogram_filters_cuda::getValidHistMask_method1(&gpu.pair[0], gpu.mask, gpu.stream);
                break;
            case hist_filter::HESSIAN_RIDGE:
                histogram_filters_cuda::getValidHistMask_method2(&gpu.pair[0], gpu.mask, gpu.stream);
                break;
        }

        // the filters above look at the raw newer histogram; the derivatives use the smoothed pair if enabled
        if (gaussian_smoothing) {
            histogram_filters_cuda::applyGuassianSmoothing(&gpu.pair[0], &gpu.smoothed_pair[0], gpu.stream);
            histogram_filters_cuda::applyGuassianSmoothing(&gpu.pair[1], &gpu.smoothed_pair[1], gpu.stream);
            lucas_kanade_cuda::getHxy_array(&gpu.smoothed_pair[0], gpu.hxy_array, spacial_derivitive_type, gpu.stream);
            lucas_kanade_cuda::getHt_array(&gpu.smoothed_pair[0], &gpu.smoothed_pair[1], gpu.ht_array, gpu.stream);
        } else {
            lucas_kanade_cuda::getHxy_array(&gpu.pair[0], gpu.hxy_array, spacial_derivitive_type, gpu.stream);
            lucas_kanade_cuda::getHt_array(&gpu.pair[0], &gpu.pair[1], gpu.ht_array, gpu.stream);
        }
        lucas_kanade_cuda::getLKMatrix_arrays(gpu.hxy_array, gpu.ht_array, gpu.row_sums, gpu.structure_matrix_array,
                                              gpu.time_grad_matrix_array, gpu.mask, gpu.stream);
        lucas_kanade_cuda::LK_method(gpu.structure_matrix_array, gpu.time_grad_matrix_array, gpu.flow, gpu.mask, gpu.stream);

        if (gpu.mapped_host_flow != nullptr) {
            // the GPU wrote flow_->patches in place; the TIME_HIGH span beginRun set is untouched
            cuda_util::check(cudaStreamSynchronize(gpu.stream), "optic_flow: GPU pipeline");
        } else {
            // the kernels don't write the TIME_HIGH span, so keep the one beginRun set
            const uint64_t time_high_start = flow_.time_high_start;
            const uint64_t time_high_end = flow_.time_high_end;
            cuda_util::check(cudaMemcpyAsync(&flow_, gpu.flow, sizeof(flow_field), cudaMemcpyDeviceToHost, gpu.stream),
                             "optic_flow: copy flow field");
            cuda_util::check(cudaStreamSynchronize(gpu.stream), "optic_flow: GPU pipeline");
            flow_.time_high_start = time_high_start;
            flow_.time_high_end = time_high_end;
        }
    } catch (...) {
        // nothing may still be reading the pair or writing flow_ once this returns
        (void)cudaStreamSynchronize(gpu.stream);
        throw;
    }
    return flow_;
}
