# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Visual-inertial odometry for a Jetson Orin NX 16 GB (ARK carrier "Super", JetPack 6 / L4T R36.5, Ubuntu 22.04) using a Prophesee GenX320 event camera (320×320). The goal is to estimate velocity from ego-motion in the event stream and send it to a PX4 flight controller.

Working so far, validated on hardware: camera → `Evt21Streamer` → event histograms → dense Lucas-Kanade optic flow (CPU and CUDA) → threaded flow/histogram file loggers. The GPU pipeline keeps up with the camera in real time (see Hardware findings). Not started: time surfaces, IMU fusion, the PX4 link. Not implemented inside the flow pipeline: the Hessian-ridge filter (method 2), Gaussian-derivative spatial derivatives, and a CPU version of the Gaussian smoothing.

## Build

There is no top-level build. Each test is its own CMake project, built in a `build/` folder inside the test directory (`mkdir build && cd build && cmake .. && cmake --build .`). The user wants to keep the CMake projects separate. All three default to `CMAKE_BUILD_TYPE=Release`: with no build type CMake passes **no `-O` flag at all**, and every build before Oct 28 2026 ran unoptimized (histogramming 3–5 ms instead of ~0.1 ms, flow copies 28 ms instead of 0.3 ms). C++17 with `-Wall -Wextra -Wpedantic`; builds should have no warnings.

| Test | Pipeline | Output |
|---|---|---|
| `tests/write_evt21_to_file` | `Genx320Driver` directly, 1 s capture | `./log.raw` (raw EVT2.1) |
| `tests/histogram_buffer_writer` | camera → `Evt21Streamer` → `histograms` → `writeHistograms`, every window in order for 100 buffers (also writes the first pair's older half) | `./histogram_recording.bin` |
| `tests/optic_flow_writer` | camera → `Evt21Streamer` → `histograms` → `optic_flow` (GPU by default) → `writeFlows` + `writeHistograms` | `./flow_recording.bin`, `./histogram_recording.bin`, timing report on stdout |

`.vscode/settings.json` points CMake Tools at `write_evt21_to_file`, so the editor shows include errors in the opticFlow headers; they build fine. Leave it unless asked.

### `optic_flow_writer`

```
./optic_flow_writer <video-device> <sensor-device> [gpu|cpu]     # on the Jetson: /dev/video0 /dev/v4l-subdev3
sudo ./optic_flow_writer /dev/video0 /dev/v4l-subdev3            # switches to MAXN_SUPER and pins clocks for the run
```

- Runs the whole pipeline for `WARMUP_TIME` (3 s: past the startup burst/blackout and the ~70 ms first GPU call) without logging, prints `logging started`, then records `RECORD_BUFFER_COUNT` (500) camera buffers. A timed-out `getBuffer()` counts toward the 500.
- Logs every `LOG_EVERY_N`-th (4) flow and its histogram pair (older then newer): ~1.56 MB per logged window, ~610 / N MB/s at ~390 windows/s. The Jetson NVMe sustains ~712 MB/s, so N = 1 is too close; 2–3 is the practical minimum. 500 buffers is ~0.5 s of camera time in a dense scene, ~5.5 s in a static one (~850 MB of files).
- Prints a summary line (flows, mean flow time, warm-up flows, written/dropped counts) and an instrumentation report: wall vs camera time, driver drops, TIME_HIGH jumps / missing ticks, and per camera buffer mean/p50/p95/max of waiting in `getBuffer`, histograms + release, flow, logging, busy time, plus the count of buffers that took longer than the camera time they cover, all repeated for high-event-rate buffers (< 3 ms of camera time). The report is only on stdout, not in the files: ask the user to paste it.
- `max_performance` (in `main.cpp`): with root, stores the clock settings (`jetson_clocks --store` to a per-run `/tmp/optic_flow_writer_clocks_<pid>.conf`, because `--store` refuses to overwrite a file), switches to the power mode named `MAXN_SUPER` (ID looked up from `nvpmodel -p --verbose`, `< /dev/null` so a reboot prompt answers no), runs `jetson_clocks`, and on exit switches the mode back, restores, and deletes the file. Without root it warns and runs on the governor's clocks. Ctrl-C skips the restore, and the power mode persists across reboots. Files written under sudo are owned by root.
- `gpu` needs `optic_flow::gaussian_smoothing` (default true); `cpu` throws while smoothing is on, since smoothing has no CPU version yet.

CMake for this test: `project(... LANGUAGES CXX CUDA)`, `CMAKE_CUDA_ARCHITECTURES` defaults to 87 (Orin), Eigen as `SYSTEM` include, `EIGEN_MPL2_ONLY`, CUDA flags `--expt-relaxed-constexpr -diag-suppress=20012` (20012 is Eigen's harmless `__host__ __device__` warning), links `Threads::Threads` and `CUDA::cudart`. If no CUDA compiler is given and none is on `PATH`, it uses `/usr/local/cuda/bin/nvcc` (JetPack doesn't put nvcc on PATH). A `build/` folder from a configure that failed for lack of a compiler must be deleted (it caches empty compiler features).

**Desktop (RTX 5070 Ti, sm_120, glibc 2.43, GCC 15):** the `nvcc` on PATH is CUDA 12.5, which can't target sm_120, and CUDA 13.1's headers clash with glibc 2.43 (`rsqrt`/`rsqrtf` declared twice). Configure with
`cmake .. -DCMAKE_CUDA_COMPILER=/usr/local/cuda-13.1/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120 -DGLIBC_243_WORKAROUND=ON`. The workaround (`-Xcompiler=-U_GNU_SOURCE -D_DEFAULT_SOURCE -D_XOPEN_SOURCE=700`) must be set before `project()`, because CMake's own compiler check already fails without it. Hand nvcc builds need the same flags. The Jetson (glibc 2.35) needs none of this.

**Jetson CUDA:** only `cuda-toolkit-12-6` is installed (from NVIDIA's Jetson apt repo, `--no-install-recommends`; no cuDNN/TensorRT/VPI/OpenCV/`nvidia-jetpack`), in `/usr/local/cuda-12.6` → `/usr/local/cuda`. nvcc is not on PATH.

### Single files

Modules include `types.h` / `config.h` by bare name: `-Iinclude -Iconfig`. Flow code also needs `-isystem external/eigen -DEIGEN_MPL2_ONLY`, plus `-Isrc/modules/addressEventCamera -Isrc/modules/opticFlow`; CUDA headers (`*_cuda.h`, `cudaUtil.h`) need `-Isrc/drivers/cuda -isystem /usr/local/cuda/include` under g++.

```
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude -Iconfig -c src/modules/addressEventCamera/evt21Decoder.cpp
```

The driver, `Evt21Streamer` and the capture test include `<linux/videodev2.h>`, so they only compile on Linux.

There is no test framework or linter. Verification so far has been ad hoc harnesses (synthetic moving Gaussian blob with known flow, CPU-vs-GPU comparison, ASan/UBSan runs, recorded-histogram replays), built outside the repo.

## Architecture

Data flows driver → streamer → addressEventCamera (decoder, histograms) → opticFlow (filters, derivatives, LK; CPU or CUDA) → fileWriter / future VIO.

- **`Genx320Driver`** (`src/drivers/genx320/`) is raw V4L2:
  - The EVT-format control (`0x00982903`, value 1 = EVT2.1) is set on the **sensor subdevice**. That fd stays open for the session because some drivers reset controls on close.
  - The MMAP streaming queue lives on the **video device**, opened `O_NONBLOCK`. 8 buffers are requested, but the driver may grant a different count, so code must use `bufferCount()`. A grant of 0 fails in `initialize()`. Each buffer is 32,768 bytes (1024 × 32 "Bayer" format) = 4,096 EVT2.1 words.
  - `initialize()` calls `stop()` first, so it is safe to call twice.
  - It exposes `fd()`, `buffer(index)` (throws `std::out_of_range`) and `bufferCount()`, and leaves DQBUF/QBUF to its owner.
- **`v4l2Util.h`** (`src/drivers/genx320/`, namespace `v4l2_util`) holds the header-only V4L2 helpers shared by the driver, the streamer and the capture test: `xioctl` (retries on EINTR), `fail` (throws `std::system_error` with errno), `makeBuffer(index)` and `queueBuffer(fd, index)`. Use these rather than writing new copies.
- **`cudaUtil.h`** (`src/drivers/cuda/`, namespace `cuda_util`): `check(cudaError_t, what)` throws `std::runtime_error` with `cudaGetErrorString`. The CUDA counterpart of `v4l2Util.h`.
- **`Evt21Streamer`** owns the driver and does DQBUF/QBUF:
  - Lifecycle: private `State` enum `CLOSED` → `INITIALIZED` → `STREAMING` ⇄ `HOLDING`; `stop()` returns it to `CLOSED`. `HOLDING` means the caller holds the copy, not a V4L2 buffer.
  - **Cached copy:** the driver's mmap'd buffers are uncached (reading 4,096 words one by one takes ~520 µs; `memcpy` takes ~33 µs). `getBuffer()` copies `bytesused` into `copy_` (a `std::vector` sized once in `initialize()` to the largest buffer), requeues the V4L2 buffer immediately, and returns `copy_.data()`. `releaseBuffer()` does no ioctl. Histogramming a buffer went from ~1.15 ms to ~0.12 ms.
  - `getBuffer()` tries `VIDIOC_DQBUF` first and only calls `poll()` on `EAGAIN`. The whole call has one deadline, `cameraConfig::BUFFER_TIMEOUT_MS` (500 ms); on timeout it returns `nullptr` with the stream still running. Empty buffers are requeued and skipped. A buffer with the error flag or a bad size is requeued and reported as `std::runtime_error`. An out-of-range index stops the stream.
  - `droppedBufferCount()`: gaps in the V4L2 sequence numbers of every dequeued buffer (empty ones included) since `start()`. **The GenX320 driver loses data without sequence gaps**, so missing TIME_HIGH ticks (the `optic_flow_writer` report) are the reliable loss signal.
- The capture test drives `Genx320Driver` directly with its own 1 s deadline. It could move onto `Evt21Streamer`.
- `Evt21StreamerFromFile` replays a recorded raw EVT2.1 file (32 KB chunks into its own array).
- **Shared streamer interface:** `Evt21Streamer` and `Evt21StreamerFromFile` must keep **identical interfaces**: `initialize(source, sensor_device)`, `start()`, `stop()`, `getBuffer()` → `evt21_buffer` (`uint64_t*`), `wordCount()`, `releaseBuffer()`, `droppedBufferCount()` (always 0 for the file). One buffer at a time; release before the next `getBuffer()`. Both hand out a copy now. The file streamer returns `nullptr` at EOF.
- **`addressEventCamera`** (`src/modules/addressEventCamera/`):
  - `evt21Decoder` holds every EVT2.1 bit layout constant. Keep new field extraction there. It provides `get_evt21_word_type`, `get_time_stamp`, `get_time_high` / `get_time_high_duration` (wrap-safe, 28-bit), `get_x` / `get_y` (`size_t`), `get_valid_mask` and `is_valid_event`.
  - `histogram_buffer` / `histograms`: a ring of `HISTOGRAM_BUFFER_COUNT` (12) per-pixel signed event histograms that hands out time-adjacent pairs. `HISTOGRAM_BUFFER_COUNT` and `HISTOGRAM_TIME_HIGH_WIDTH` (40) are `static constexpr` members in `histograms.h`, not in `config.h`. Time surfaces don't exist: only a `time_surface` type in `types.h` (a raw C array; make it a `std::array` before using it). `histogram_buffer::getTimeHighStart()` gives the decoded TIME_HIGH a window began at.
- **`fileWriter`** (`src/modules/fileWriter/`, README has both file formats and numpy dtypes). Both writers log from a background thread: `tryWrite()` copies into a preallocated ring and returns `false` (counting a drop) when full, never blocking the pipeline. Only the ring is shared between threads. `stop()` drains, joins, `fdatasync`s and closes. Rings are allocated once and kept across `stop()`/`initialize()`. They need `-pthread`. The file-only `fail()`/`writeAll()` helpers live in each .cpp's anonymous namespace, not `v4l2Util.h` (that header pulls in `<linux/videodev2.h>`).
  - `writeHistograms`: 32 slots of `uint64_t` TIME_HIGH start + raw 320×320 `int16_t` (204,808 B). `tryWrite(histogram_buffer&)` throws `std::logic_error` unless the buffer is READY/HELD and the writer started. Read with `np.fromfile(path, dtype=[('t','<u8'),('hist','<i2',(320,320))])`. To log every window once, write only `newer` of each pair (plus `older` of the first).
  - `writeFlows`: 8 slots (~9.2 MB) of `flow_record { uint64_t time_high; flow_field flow; }`, written straight from memory, padding included (`static_assert`s in `writeFlows.h` fail the build if the layout changes). `tryWrite(const flow_field&, const size_t& time_high)`, with `optic_flow::getTimeHigh()`. Record = 24 + GRID² × 12 bytes (1,153,224 at 11×11). Record offsets: `t` 0, `t_start` 8, `t_end` 16, patches 24; patch = `float flow[2]` at 0, `bool valid` at 8, 12 B.
- **Eigen** (`external/eigen`): Eigen 3.4.0 headers only, vendored; `external/eigen/VERSION` has the URL and sha256. Use only fixed-size types and `float`. Keep Eigen out of `types.h` and `config.h`. Eigen types aren't `constexpr`, so compile-time constants (kernels) are `std::array`. Fixed-size Eigen types work in CUDA device code (with `--expt-relaxed-constexpr`), so device arrays have the host layout and copy back with a plain `memcpy`.
- **Flow types** (`include/flow_types.h`):
  - `vector2d` (`Eigen::Vector2f`): flow in px/s, image frame (x right, y down). `Structure_Matrix` (`Matrix2f`, A'A), `Time_Grad_Matrix` (`Vector2f`, A'b), `Hessian_Matrix` (`Matrix2f`). `eigenValues2d` / `eigenVectors2d` are defined but unused.
  - `enum class spatial_derivative { SGF, GAUSSIAN }` and `enum class hist_filter { NONE, MIN_MAGNITUDE, HESSIAN_RIDGE }`.
  - `flow_patch { vector2d flow; bool valid; }` (12 B). The eigenvalues/eigenvectors were removed as unused.
  - `flow_field`: `time_high_start` (older window start), `time_high_end` (newer window end), one `flow_patch` per grid point, row-major (~1.15 MB at 11×11). Static storage or one `make_unique`, never the stack.
  - `grid_mask` = `std::bitset<GRID_POINT_NUM>` (CPU only; `std::bitset` can't be used in device code). Indexed by grid point.
  - Per-pixel arrays (row-major, `pixel_index = y * WIDTH + x`): `Hxy_array` (`vector2d` per pixel, ~800 KB), `Ht_array` (second-order temporal derivative, ~400 KB), `smoothed_histogram_array` (float). `Structure_Matrix_array` / `Time_Grad_Matrix_array` per grid point. `LK_row_sums_array`: separable patch-sum scratch, `LK_PRODUCT_NUM` (5) planes × HEIGHT × GRID_WIDTH (~2 MB).
  - `grid_to_pixel(grid_index)`: a patch centre's pixel coordinate along one axis.
- **Flow config** (`flowConfig` in `config.h`): `PATCH_WIDTH` 11 (odd), `GRID_STRIDE` 1 (dense), so the grid is 310×310; grid centres keep `PATCH_RADIUS` from the edge, no padding. `SHI_TOMPSI_MIN_THRESH` 1.0 (must stay > 0: 0 lets empty patches with det = 0 through as NaN flow). `HISTOGRAM_MIN_THRESH` 5 is method 1's cutoff; per-pixel window values are mostly ±1, so 5 is far too high if the filter is ever enabled (1–2). The filter stride equals the LK stride, so masks are per grid point.
- **Filter constants** (`filters` in `config.h`): `GUASSIAN_SMOOTHING_SIGMA` 1.0 / `GUASSIAN_SMOOTHING_RADIUS` 3 (7×7, cut at 3σ), the 3×3 SGF kernels as `filters::kernel3x3` (`std::array<float, 9>`, row-major, row = y), and placeholder `GUASSIAN_SIGMA` / `GUASSIAN_PATCH_WIDTH`. Use float literals (`1.0f/6`); `1/6` is integer division.
- **`opticFlow`** (`src/modules/opticFlow/`): CPU functions in namespaces, CUDA versions in `*_cuda` namespaces (`*_cuda.h` + `.cu`) taking device pointers and a `cudaStream_t` (asynchronous; launch errors throw `std::runtime_error`, kernel errors surface at the next sync). On the device the mask is one `uint8_t` per grid point (1 = valid); `cudaMemsetAsync(mask, 1, …)` is `mask.set()`. Unimplemented functions throw `std::logic_error("… is not implemented")`.
  - `histogram_filters` (CPU) / `histogram_filters_cuda`: `getValidHistMask_method1` (clears grid points whose centre pixel is within ±`HISTOGRAM_MIN_THRESH`), `getValidHistMask_method2` (stub). Filters take a mask, only look at set bits and clear rejects, so they chain. `histogram_filters_helpers::applyGuassianConvolution_secondOrder(hist, grid_index)` and `applyHessianRidgeCondition` are stubs. `histogram_filters_cuda::applyGuassianSmoothing(d_hist, d_smoothed)`: normalised 7×7 Gaussian, replicate edges (GPU only).
  - `lucas_kanade` (CPU) runs in passes: `getHxy_array(hist, hxy, spatial_derivative)` (SGF; GAUSSIAN throws), `getHt_array(newer, older, ht)` (`(SGF_HT * newer − SGF_HT * older) / window length`), `getLKMatrix_arrays(hxy, ht, row_sums, structure, time_grad, mask)` (separable: a row pass into `row_sums`, then a column pass per grid point, so 2 × PATCH_WIDTH terms instead of PATCH_WIDTH²; masked points zeroed), `LK_method(structure, time_grad, flow, mask)` (smaller eigenvalue in closed form, Shi-Tomasi test clears failing mask bits and marks them invalid, then the closed-form solve of A'A v = −A'b). Pixel helpers in `lucas_kanade_helpers`: `applySGF_XY` / `applySGF_T(hist, pixel_index)`, `applyShiTompsiCondition`; `applyGuassianConvolution_firstOrder` and `getStructureMatrix_GuassianConvolution` (declared only) are unimplemented. Derivative edge reads use replicate padding (clamped).
  - `lucas_kanade_cuda` mirrors the CPU passes (`getHxy_array` / `getHt_array` have overloads for raw `histogram_array` and `smoothed_histogram_array` input; `getLKMatrix_arrays` takes a device `LK_row_sums_array` scratch). One thread per pixel for derivatives, per (row, grid column) for the row pass, per grid point for the column pass and LK.
  - **`optic_flow`** (`optic_flow.h/.cpp`): directs a histogram pair through the pipeline and owns every array (~6.7 MB; static storage or one `make_unique`). Settings: `spacial_derivitive_type` (SGF), `filter_type` (`hist_filter::NONE`), `gaussian_smoothing` (true; GPU only). `run_LK_method_CPU(pair)` / `run_LK_method_GPU(pair)` return `const flow_field&`; both throw `std::logic_error` unless both histograms are READY/HELD, set `time_high_` (newer start + 40) and the flow span. Both are synchronous: the pair can be queued as soon as they return. `getTimeHigh()`. `stop()` frees the GPU buffers (destructor calls it). GPU buffers (`gpu_buffers`, defined only in the .cpp so the header needs no CUDA) are allocated on the first GPU call (~70–100 ms with CUDA context creation) and kept. **Integrated GPU (Jetson):** `flow_` is `cudaHostRegister`ed and mapped, and LK writes it in place (the Orin reports `canUseHostPointerForRegisteredMem = 0`, so it uses `cudaHostGetDevicePointer`). **Discrete GPU (desktop):** device copy, copied back after the run. GPU sequence: copy pair → mask set → filter → smooth ×2 → Hxy → Ht → row/column sums → LK → (discrete: copy back) → sync.
  - Naming vs. physics: a histogram value is already the intensity change over the window, so `applySGF_T` needs one histogram, and "spatial derivatives of the histogram" are mixed space-time derivatives of intensity. Known to be loose; leave it unless asked.

### Optic flow pipeline (the user's target, also in README.md)

```
histogram_pair -> filter -> spatial + temporal derivative convolution -> LK on valid grid points
```

- filter: Hessian ridge (method 2, not implemented), minimum magnitude (method 1), or nothing.
- spatial derivatives: SGF (implemented) or first-order Gaussian (not implemented). Gaussian *smoothing* of both histograms before the SGF derivatives is implemented on the GPU.
- temporal: SGF_HT on both histograms; their difference over Δt is the second-order temporal derivative (Rueckauer & Delbruck 2016, eq. 5).
- LK: structure matrix and A'b summed over the patch (eq. 2), 2×2 closed-form solve, Shi-Tomasi on the smaller eigenvalue.
- Each step can be a CUDA kernel or a CPU process; consecutive GPU stages keep data on the device.

### Histogram memory design

- Each `histogram_buffer` holds its `histogram_array` inline (`std::array<int16_t, 320*320>`, ~200 KB) and allocates nothing. Each pixel holds a signed sum over the window (+1 positive, −1 negative event); `int16_t` can't overflow within a window. `histograms` holds its 12 buffers by value in a `std::array` (~2.4 MB contiguous). Don't turn these into pointers or per-buffer heap allocations.
- `histogram_buffer` is non-copyable, so `histograms` is too.
- Exactly one `histograms` instance per pipeline, in static storage or a single `std::make_unique`; never a stack local. Don't use `static` data members inside the class. Buffer addresses stay fixed for the whole run.
- Each window covers `HISTOGRAM_TIME_HIGH_WIDTH = 40` TIME_HIGH ticks × 64 µs ≈ 2.56 ms. `resetHistogram()` zeroes in place.

### Histogram filling and hand-off

- **Windows:** `updateHistograms(buffer, word_count)` keeps state between calls, so a window can span streamer buffers. Events before the first TIME_HIGH are dropped. A full window becomes READY and `getNextQueuedIndex()` picks the next buffer (a FREE one, else the oldest READY, never FILLING or HELD); it is reset and starts at the same tick, so windows have no gaps. A TIME_HIGH jump inside a window (missing data) makes that window longer than 40 ticks.
- **Buffer state:** exactly one `buffer_state` each (`FREE`, `FILLING`, `READY`, `HELD`, enum in `histogram_buffer.h`); only `histograms` changes it. One FILLING once the first TIME_HIGH arrives, at most 2 HELD. `getHistogram()` is non-null only for READY / HELD. Plain enum for now; meant to become `std::atomic<buffer_state>` when the consumer moves to its own thread.
- **Sequence numbers:** every window gets a consecutive `sequence_` starting at 1; pairing uses sequences, not ring indexes.
- **Pair protocol:**
  - `dequeueHistogramPair()` returns `{newer, older}` for the newest ready buffer and its predecessor, or `{nullptr, nullptr}` if there's nothing newer than the last pair or the predecessor was overwritten. Consecutive pairs may overlap. A consumer that falls behind skips to the newest pair.
  - `dequeueNextHistogramPair()` returns pairs in order. Called in a loop after every `updateHistograms` it consumes every window as long as one camera buffer completes at most `HISTOGRAM_BUFFER_COUNT − 2` (10) windows; beyond that the oldest READY windows are overwritten. Both dequeues share `last_dequeued_seq_`.
  - `queueHistogramPair()` releases only the older buffer; the newer stays held as the next pair's older half. Taking a new pair frees any other HELD buffer. Dequeuing twice without queuing, or queuing with nothing held, throws `std::logic_error`.
- **Bad words:** POS/NEG words failing `is_valid_event` are skipped and counted (`getBadWordCount()`); `histogram_buffer::updateHistogram` still throws on them as a safety net.
- Not thread-safe: `updateHistograms` and dequeue/queue must run on one thread.

### Hardware findings (GenX320 + Orin NX, Oct 2026)

- **Startup:** about 200–220 ms of TIME_HIGH goes missing ≈30 ms after STREAMON (a driver quirk), and the first window has a burst of ~55,900 active pixels. `optic_flow_writer` skips the first 3 s; other pipelines should discard the first ~250 ms.
- **Windows per camera buffer** (4,096 words; sparse events cost ~1 word each, plus 40 TIME_HIGH words per window): static scene ~4–7 windows (~11–18 ms of camera time), hand at the edge ~2.3, fast hand wave across the middle (9,500–11,200 events per window) ~0.4 (~1 ms). So a fixed buffer count records very different durations.
- **Clocks decide whether the pipeline keeps up.** Unpinned, `schedutil` keeps the CPU at 729 MHz and the GPU at its 305 MHz minimum even while busy (the load is bursty), and 18–40% of camera time was lost. The board defaults to the 40W power mode (ID 4: CPU ≤ 1.5 GHz, GPU ≤ 1173 MHz); MAXN_SUPER (ID 0) lifts the CPU to 1.98 GHz, and the GPU hardware maximum is 1173 MHz in both. Modes 0 and 4 have the same online cores and power gating, so switching needs no reboot. The `jetson` user has no passwordless sudo.
- **Data loss shows up as TIME_HIGH jumps between buffers with no V4L2 sequence gaps**, exactly in buffers whose processing outlasted the camera time they cover: the driver drops data when the pipeline is behind.
- **Measured (sudo, Release, static scene):** GPU flow ≈ 400–480 µs per window (copy pair 62, mask 8, smooth ×2 92, Hxy 15, Ht 20, patch sums 120, LK solve + mapped write 85 µs); histograms + release ~0.1 ms per buffer; logging ~0.3 ms per logged window; busy ~2.3 ms per ~11 ms buffer; 0% camera time lost. With the hand at the edge: ~1.0 ms busy per 5.9 ms buffer (~18%). The dense-wave case (< 1 window per buffer) is estimated at ~20–25% load but not yet measured. CPU flow (no smoothing) ≈ 7.9 ms per window on one core: not real time.
- **LK tuning (offline, numpy copy of the pipeline that reproduces the Jetson's output exactly):** on a hand wave, 3×3 patches gave incoherent directions (33% within 45° of the hand's motion); 11×11 on σ 1 smoothed histograms with Shi-Tomasi 1 gave (−5, 219) px/s against a measured (38, 237), 81% within 45°, and no off-hand vectors. Without smoothing, large patches get the direction but underestimate speed (~60%). Single isolated events produce quantized axis-aligned flow artifacts (±260 / ±521 px/s at 3×3); hot pixels recur at (65, 314), (47, 31) and (105, 130).
- **Fast motion:** at ~2.9 px per 2.56 ms window (≈1,130 px/s), LK underestimates speed (~37% at σ 1, 11×11) and only ~50% of vectors are within 45°. Larger scales recover the speed (σ 4 / 19×19 ≈ 82%, half resolution ≈ 77%) but not the direction. Options if it matters: coarse-to-fine (pyramid) LK, shorter windows (`HISTOGRAM_TIME_HIGH_WIDTH` 20), larger fixed scale. The user is focused on the computation pipeline rather than LK accuracy for now.
- Jetson NVMe sustains ~712 MB/s writes. `scp` of a 310 MB recording took ~4 s on the current network.

### Open items

- CPU Gaussian smoothing; Hessian-ridge filter (method 2) and Gaussian-derivative spatial derivatives.
- Measure the dense-scene (< 1 window per buffer) timing report.
- Possible GPU speedups: CUDA Graph for the 9 launches, separable smoothing (49 → 14 taps), overlapping the GPU with camera reading (async launch, sync before queuing the pair).
- Real-time policy for control: flow on the newest pair only (`dequeueHistogramPair()`); doesn't help when a buffer holds ≤ 1 window.
- Record for a fixed time instead of a fixed buffer count; optionally write the timing report to a file next to the recordings.
- Grid margin: patches at the outermost grid points use edge-clamped derivatives; widening the margin by the SGF radius would avoid it.
- Hot-pixel mask; startup burst/blackout discard in the pipeline.
- Spelling: `GUASSIAN` / `KERNAL` / `Guassian` / `SHI_TOMPSI` / `Tompsi` throughout; the user hasn't asked to rename yet.
- Time surfaces, IMU fusion, PX4 link.
- Move the capture test onto `Evt21Streamer` (it also doesn't check `V4L2_BUF_FLAG_ERROR`).
- Nits: `Evt21StreamerFromFile::start()` reports "initialize the file before starting it" when already started; `evt21Streamer/README.md` misspells `genx320Drvier`.
- Add a `.gitignore` for build folders and `.bin`/`.raw` recordings.

Each module directory has a README stating its intended contract. Read it before changing a module.

## EVT2.1 format

The camera produces 64-bit little-endian words. Bits [63:60] hold the type:
- `0x0` NEG / `0x1` POS: [59:54] timestamp LSBs, [53:43] x (aligned to 32), [42:32] y, [31:0] valid mask for pixels x..x+31.
- `0x8` TIME_HIGH: [59:32] timestamp bits 33..6, so one TIME_HIGH tick is 64 µs. They arrive every tick, so a jump of more than one tick means missing data.
- Also `0xA` EXT_TRIGGER, `0xE` OTHERS, `0xF` CONTINUED. These are not in the `evt21_type` enum (`config/config.h`), so switches over it need a default case.

Timestamps are in µs and 34 bits wide. `timestamp_us` is `uint32_t` and wraps after about 71 minutes.

## Conventions

Error handling uses exceptions. Syscall failures throw `std::system_error` with errno, misuse throws `std::logic_error`, bad data and CUDA errors throw `std::runtime_error`. Shared helpers live in `v4l2Util.h` / `cudaUtil.h`; helpers used by only one file stay in its anonymous namespace. Classes owning hardware (including GPU memory) are non-copyable and release resources in a `noexcept` `stop()` that the destructor also calls. Large arrays (histograms, flow fields, derivative images) live in static storage or a single `make_unique`, never on the stack.

## Workflow

Roshan Patel is currently the only contributor. "Push this" means stage everything, untracked files included (`git add -A`), commit, and push the current branch with `git push origin HEAD` (currently `streamline-driver-streamer`). Check `git status` first for build folders and recordings (there is no `.gitignore`), and always before saying something is or isn't committed.

Code reaches the Jetson only through git: commit and push here, then `git pull` on the Jetson (`~/Jetson-Event-VIO`, SSH `jetson@jetson.local`; the user gives the login when needed, and it must never be written into the repo). Recordings come back with `scp`. When testing uncommitted code on the Jetson, copy the tree to `/tmp` (e.g. `tar` over SSH) and build there, so the user's checkout and recordings stay untouched; run anything that writes files from that copy, and clean it up afterwards (sudo runs leave root-owned files). Don't change the Jetson's system settings (packages, power mode, sudoers) without asking.

`~/Desktop/flow_playback` (outside the repo, its own `.venv` with numpy and matplotlib) has `play_flow.py`, which plays a `flow_recording.bin`: speed colour map plus arrows, grid size inferred from the file size, `--fps`, `--save out.gif`, space / ←→ to pause and step.
