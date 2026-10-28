# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Visual-inertial odometry for a Jetson Orin NX using a Prophesee GenX320 event camera (320×320). The goal is to estimate velocity from ego-motion in the event stream and send it to a PX4 flight controller. The camera input path, the EVT2.1 decoder, the event histograms and a threaded histogram file logger exist so far, and the camera → histograms path has been validated on hardware (see Hardware findings). Optic flow has started: its types (`include/flow_types.h`) and the histogram filter interface (`src/modules/opticFlow/histogram_filters.h`) are declared but not implemented. Time surfaces, the Lucas-Kanade solve, IMU fusion and the PX4 link are unimplemented.

## Build

There is no top-level build. Each test is its own CMake project. The capture test (`.vscode/settings.json` points CMake Tools at it):

```
cmake -S tests/write_evt21_to_file -B build && cmake --build build
./build/write_evt21_to_file <video-device> <sensor-device>   # writes 1 s of raw EVT2.1 to ./log.raw
```

`tests/histogram_buffer_writer` is a second CMake project: camera → `Evt21Streamer` → `histograms` → `writeHistograms`, logging every window in order (via `dequeueNextHistogramPair()`) to `./histogram_recording.bin` for 100 camera buffers. On the first pair it also writes the older half, so window 1 is logged. It prints the written, dropped and bad-word counts at the end:

```
cmake -S tests/histogram_buffer_writer -B build_hbw && cmake --build build_hbw
./build_hbw/histogram_buffer_writer <video-device> <sensor-device>
```

On the Jetson the tests are usually built in a `build/` folder inside the test directory (`cmake .. && cmake --build .`), so the recording lands in `tests/histogram_buffer_writer/build/histogram_recording.bin`. There is no `.gitignore` yet, so build folders and recordings get staged by `git add -A`.

Both build with C++17 and `-Wall -Wextra -Wpedantic`. Running either test needs the GenX320 V4L2 driver on the Jetson.

Modules include `types.h` / `config.h` by bare name, so compiling them needs `-Iinclude -Iconfig`. To check a single file builds:

```
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude -Iconfig -c src/modules/addressEventCamera/evt21Decoder.cpp
```

Anything including `flow_types.h` or the opticFlow headers also needs Eigen: `-isystem external/eigen -DEIGEN_MPL2_ONLY`, or in CMake `target_include_directories(<target> SYSTEM PRIVATE ../../external/eigen)` plus `target_compile_definitions(<target> PRIVATE EIGEN_MPL2_ONLY)`. `SYSTEM`/`-isystem` keeps `-Wall -Wextra -Wpedantic` quiet about Eigen's own headers. No CMake target uses Eigen yet.

There is no test framework or linter.

The driver, `Evt21Streamer` and the capture test include `<linux/videodev2.h>`, so they only compile on Linux (the Jetson). On macOS they can be syntax-checked against a minimal stub `linux/videodev2.h` passed with `-isystem`, but that does not check them against real V4L2.

## Architecture

Data flows driver → streamer → addressEventCamera (decoder, histograms, time surfaces) → future optic-flow / VIO.

- **`Genx320Driver`** (`src/drivers/genx320/`) is raw V4L2:
  - The EVT-format control (`0x00982903`, value 1 = EVT2.1) is set on the **sensor subdevice**. That fd stays open for the session because some drivers reset controls on close.
  - The MMAP streaming queue lives on the **video device**, opened `O_NONBLOCK`. 8 buffers are requested, but the driver may grant a different count, so code must use `bufferCount()`. A grant of 0 fails in `initialize()`.
  - `initialize()` calls `stop()` first, so it is safe to call twice.
  - It exposes `fd()`, `buffer(index)` (throws `std::out_of_range`) and `bufferCount()`, and leaves DQBUF/QBUF to its owner.
- **`v4l2Util.h`** (`src/drivers/genx320/`, namespace `v4l2_util`) holds the header-only V4L2 helpers shared by the driver, the streamer and the capture test: `xioctl` (retries on EINTR), `fail` (throws `std::system_error` with errno), `makeBuffer(index)` (an MMAP capture `v4l2_buffer`) and `queueBuffer(fd, index)`. Use these rather than writing new copies.
- **`Evt21Streamer`** owns the driver and does DQBUF/QBUF:
  - Its lifecycle is a private `State` enum: `CLOSED` → `INITIALIZED` → `STREAMING` ⇄ `HOLDING`, and `stop()` returns it to `CLOSED`.
  - `getBuffer()` tries `VIDIOC_DQBUF` first and only calls `poll()` on `EAGAIN`, so a buffer that is already waiting costs no poll syscall.
  - The whole `getBuffer()` call has one deadline, `cameraConfig::BUFFER_TIMEOUT_MS` (500 ms, in `config/config.h`). If no buffer arrives in time it returns `nullptr` with nothing held and the stream still running, so the caller just calls it again. From the camera streamer, `nullptr` means "no data yet"; from the file streamer, it means EOF.
  - Empty buffers are requeued and skipped.
  - A buffer with the error flag, or a bad size, is requeued and reported as `std::runtime_error`. Requeue failures are ignored there so the original error is reported, and the stream keeps running.
  - An out-of-range buffer index stops the stream.
- The capture test (`tests/write_evt21_to_file`) drives `Genx320Driver` directly rather than `Evt21Streamer`, with its own 1 s capture deadline. It could now be moved onto `Evt21Streamer`, since `getBuffer()` times out.
- `Evt21StreamerFromFile` replays a recorded `.evt21`/raw file.
- **Shared streamer interface:** `Evt21Streamer` and `Evt21StreamerFromFile` must keep **identical interfaces** so either can feed the pipeline. The interface is `initialize(source, sensor_device)`, `start()`, `stop()`, `getBuffer()` → `evt21_buffer` (`uint64_t*`), `wordCount()`, `releaseBuffer()`. Only one buffer may be held at a time; it must be released before the next `getBuffer()`. A camera buffer is zero-copy mapped memory and is requeued on release. The file streamer returns `nullptr` at EOF.
- **`addressEventCamera`** (`src/modules/addressEventCamera/`):
  - `evt21Decoder` holds every EVT2.1 bit layout constant. Keep new field extraction there rather than in the modules that use it. It provides `get_evt21_word_type`, `get_time_stamp`, `get_time_high` / `get_time_high_duration` (wrap-safe, 28-bit), `get_x` / `get_y` (`size_t`, for indexing), `get_valid_mask` and `is_valid_event`.
  - `histogram_buffer` / `histograms` are implemented: a ring of 5 per-pixel signed event histograms that hands out time-adjacent pairs for optic flow. Their tunables, `HISTOGRAM_BUFFER_COUNT` (5) and `HISTOGRAM_TIME_HIGH_WIDTH` (40), are `static constexpr` members in `histograms.h`, not in `config.h`. Time surfaces aren't built yet: only the `time_surface` type exists in `types.h`, as a raw `timestamp_us[PIXEL_NUM]` C array; make it a `std::array` like `histogram_array` before using it. The histogram files are built only by the `histogram_buffer_writer` test. `histogram_buffer` exposes `getTimeHighStart()` (decoded TIME_HIGH the window began at) for logging.

- **`fileWriter`** (`src/modules/fileWriter/`): `writeHistograms` logs histograms (`uint64_t` TIME_HIGH start + raw 320×320 `int16_t`, fixed-size records, no file header) from a background thread. `tryWrite()` copies into a preallocated 32-slot ring and returns `false` (counting a drop) when full, never blocking the pipeline. Only the ring is shared between threads, so the pipeline stays single-threaded. Linking it needs `-pthread` (`Threads::Threads` in CMake). `tryWrite()` throws `std::logic_error` if the buffer isn't READY/HELD or the writer isn't started, and rethrows a disk error from the writer thread. `stop()` drains the ring, joins, `fdatasync`s and closes. The ring (32 × 204,808 B ≈ 6.5 MB) is allocated once and kept across `stop()`/`initialize()`. The file-only `fail()`/`writeAll()` helpers live in its anonymous namespace, not `v4l2Util.h`, because that header pulls in `<linux/videodev2.h>`.
  - Log every window once: consecutive pairs overlap, so write only `newer` of each pair (plus `older` of the first). Read a recording with `np.fromfile(path, dtype=[('t','<u8'),('hist','<i2',(320,320))])`; consecutive `t` should differ by exactly 40.
  - Full logging is about 80 MB/s (≈390 windows/s × 204,808 B), so record to NVMe, never tmpfs.

- **Eigen** (`external/eigen`) is Eigen 3.4.0, headers only, vendored so neither machine needs a system install; `external/eigen/VERSION` records the source URL and sha256. Use only fixed-size Eigen types (`Vector2f`, `Matrix2f`, …), never `MatrixXf`/`VectorXf`, and `float` rather than `double` (NEON fp32). Keep Eigen out of `types.h` and `config.h` so modules that don't use flow don't depend on it. Eigen types aren't `constexpr`, so compile-time constants such as kernels are `std::array`.
- **Flow types** (`include/flow_types.h`):
  - `vector2d` (`Eigen::Vector2f`) is flow in pixels per second, image frame (x right, y down). `eigenValues2d` / `eigenVectors2d` hold the structure tensor's eigen decomposition.
  - `flow_patch` is one grid point's result (flow, eigenvalues, eigenvectors, `valid`). It is 48 B and 16-byte aligned because of the `Matrix2f`.
  - `flow_field` is the TIME_HIGH start/end plus one `flow_patch` per grid point, row-major. Dense with 3×3 patches it's ≈4.85 MB, so the `histograms` storage rule applies: static storage or a single `make_unique`, never the stack.
  - `grid_mask` is `std::bitset<GRID_POINT_NUM>`, one bit per grid point (≈12.6 KB dense), always passed by reference.
  - `grid_to_pixel(grid_index)` gives a patch centre's pixel coordinate along one axis.
- **Flow grid** (`flowConfig` in `config.h`): `PATCH_WIDTH` (odd, currently 3) and `GRID_STRIDE` (pixel distance between patch centres; 1 = dense) define the grid, and patches overlap when the stride is below the width. `GRID_WIDTH` / `GRID_HEIGHT` / `GRID_POINT_NUM` are derived and keep only centres whose whole patch fits on the sensor, so there is no edge padding (dense 3×3 → 318×318, dense 5×5 → 316×316). 5×5 over a ~2.5 ms window is verified in event-dense scenes. Functions index by `grid_index`, not pixel index; the two only coincide at stride 1. `HISTOGRAM_MIN_THRESH` (2) is method 1's cutoff.
- **Filter constants** (`filters` in `config.h`): Gaussian sigma (placeholder value) and width, Savitzky-Golay (SGF) width, and the 3×3 SGF kernels as `filters::kernel3x3` (`std::array<float, 9>`, row-major, index = row·3 + col, row = y). Use float literals (`1.0f/6`); `1/6` is integer division and evaluates to 0.
- **`opticFlow`** (`src/modules/opticFlow/`), namespace `histogram_filters` in `histogram_filters.h`. These are standalone functions taking `const histogram_array&`, kept separate from the Lucas-Kanade solve, which will get its own header:
  - **Mask pipeline:** `getValidHistMask_method1` (threshold on histogram magnitude) and `getValidHistMask_method2` (Hessian ridge condition) take a `grid_mask&` and return `void`. Each only considers bits already set and clears the ones it rejects, so filters chain: `mask.set()`, then apply them in any order.
  - **Derivatives at a `grid_index`:** `applySGF_XY` / `applySGF_T` (Savitzky-Golay), `applyGuassianConvolution_firstOrder` (alternative to `applySGF_XY`) and `applyGuassianConvolution_secondOrder` (for method 2's Hessian).
  - **Naming vs. physics:** a histogram value is already the intensity change over the window, so `applySGF_T` needs only one histogram (it smooths the change), and the "spatial derivative of the histogram" is physically a mixed space-time derivative of intensity. The naming is known to be loose; leave it unless asked.
  - Callers get the histogram as `*buffer->getHistogram()` after checking it's non-null.

### Current goal: optic flow pipeline

This is the target the user is working to (also in `README.md`). Design new flow code to fit it:

```
histogram_pair -> filter -> spatial + temporal derivative convolution on valid pixels -> LK on valid pixels
```

- **filter** (produces the `grid_mask` of valid grid points), one of:
  - second-order Gaussian convolution with the Hessian ridge criterion (`getValidHistMask_method2`)
  - minimum magnitude threshold (`getValidHistMask_method1`)
  - nothing: `mask.set()`, so the next step processes every pixel
- **spatial derivative convolution** on the newer histogram: SGF (`applySGF_XY`) or first-order Gaussian (`applyGuassianConvolution_firstOrder`).
- **temporal derivative convolution** (`applySGF_T`) on both histograms. Their difference over Δt is the second-order temporal derivative (Rueckauer & Delbruck 2016, eq. 5; histogram values are already intensity changes, so the spatial derivatives are mixed space-time derivatives).
- **LK**: per valid grid point, sum the structure matrix and right-hand side over the `PATCH_WIDTH` patch (eq. 2) and solve the 2×2 system in closed form, keeping the eigenvalues for confidence.
- **Each step can be a CUDA kernel or a CPU process.** Keep the stages interchangeable: each consumes the pair and/or the previous stage's outputs plus the mask and writes its own outputs, so either implementation can be swapped in per stage. Moving data between CPU and GPU between stages costs a transfer (≈58 µs per 800 KB over the desktop's PCIe, much cheaper on the Orin's shared memory), so consecutive GPU stages should keep their data on the device. `std::bitset` can't be used in device code, so the GPU side needs its own mask layout (e.g. `uint8_t` per grid point or packed `uint32_t` words); not decided yet.

**Benchmark (Oct 2026, desktop RTX 5070 Ti, not yet run on the Orin):** dense 3×3 SGF derivatives + LK over a full pair is ≈18.8 MFLOP per frame (≈7.3 GFLOP/s at 390 windows/s). On the GPU both kernels take ≈11 µs; the pair can be requeued once it's copied (≈38 µs), and flow is back on the CPU after ≈110 µs, dominated by PCIe copies. The same algorithm, unoptimized on one CPU core, takes ≈1.5 ms. On the GPU, the pair must stay HELD until its copy (or, with zero-copy, the kernel) completes before `queueHistogramPair()`.

### Histogram memory design

- Each `histogram_buffer` holds its `histogram_array` inline (`std::array<int16_t, 320*320>`, ~200 KB) and allocates nothing. Each pixel holds a signed sum over the window: a positive event adds 1 and a negative event subtracts 1. `int16_t` (±32767) can't overflow within a ~2.5 ms window. `histograms` holds its 5 buffers by value in a `std::array`, so one `histograms` object is a single contiguous ~1 MB block. Don't turn these members into pointers or per-buffer heap allocations, because the count and size are compile-time constants and the buffers live exactly as long as their manager.
- `histogram_buffer` is non-copyable, which makes `histograms` non-copyable too. This prevents accidental 200 KB / 1 MB copies.
- Exactly one `histograms` instance is created by whatever owns the pipeline. Put it in static storage (namespace scope, or a member of a statically stored owner) or create it with a single `std::make_unique` at startup. Never make it a stack local, since 1 MB is too much for the stack, especially on threads. Don't use `static` data members inside the class. Buffer addresses stay fixed for the whole run, so consumers can take a `histograms&` or the `histogram_buffer*` pair from `dequeueHistogramPair()`.
- Each histogram covers `HISTOGRAM_TIME_HIGH_WIDTH = 40` TIME_HIGH ticks, at 64 µs per tick, which is ≈2.56 ms. Buffers are reused, never reallocated: `resetHistogram()` zeroes `hist_` in place.

### Histogram filling and hand-off

- **Windows:** `updateHistograms(buffer, word_count)` keeps its state between calls, so a window can span streamer buffers. Events before the first TIME_HIGH are dropped. When the active window reaches the width, it becomes READY, and `getNextQueuedIndex()` (private) picks the next buffer: a FREE one if there is one, otherwise the oldest READY one, never FILLING or HELD. That buffer is reset, starts at the same tick, and becomes FILLING, so windows have no gaps. TIME_HIGH values are stored decoded (`size_t`), never as raw words.
- **Buffer state:** each `histogram_buffer` has exactly one `buffer_state`: `FREE`, `FILLING`, `READY` or `HELD`. The enum is defined in `histogram_buffer.h`. Only `histograms` changes states; the buffer just stores it (`getState` / `setState`). There is always exactly one FILLING buffer (`active_index_`, kept as a fast path) once the first TIME_HIGH arrives, and at most 2 HELD. `getHistogram()` returns non-null only for READY / HELD. The state is a plain enum for now; it is meant to become `std::atomic<buffer_state>` when the consumer moves to its own thread (release store on FILLING→READY; compare-exchange on READY→HELD vs READY→FILLING).
- **Sequence numbers:** every window gets a `sequence_` from `next_sequence_`, starting at 1, so consecutive windows have consecutive numbers. Ring indexes say nothing about time order, so pairing uses sequence numbers, not indexes.
- **Pair protocol:**
  - `dequeueHistogramPair()` returns `{newer, older}` for the newest ready buffer N and its predecessor N−1.
  - It returns `{nullptr, nullptr}` if N isn't newer than the last pair handed out, or if N−1 was overwritten. So the consumer never gets the same pair twice, but consecutive pairs may overlap (`{C,B}` after `{B,A}`).
  - A consumer that falls behind gets the newest pair, skipping missed windows.
  - `dequeueNextHistogramPair()` returns pairs in order instead: the oldest READY window after the last pair returned whose predecessor is still READY/HELD. Calling it in a loop (queuing after each pair) after every `updateHistograms` consumes every window, as long as one camera buffer completes at most 3 windows (`HISTOGRAM_BUFFER_COUNT` − 1 FILLING − 1 HELD). Beyond that the oldest READY windows are overwritten and it skips to the oldest pair left. Both dequeues share `last_dequeued_seq_` and may be mixed.
  - `queueHistogramPair()` releases only the older buffer. The newer one stays held as the next pair's older half.
  - Taking a new pair moves any HELD buffer that isn't part of it to FREE (the consumer fell behind). At most 3 buffers are ever unavailable (1 FILLING plus 2 HELD), so the writer always has a buffer.
  - Dequeuing twice without queuing, or queuing with nothing held, throws `std::logic_error`.
- **Bad words:** POS/NEG words failing `is_valid_event` (misaligned x, or off the sensor) are skipped and counted in `getBadWordCount()` rather than stopping the stream. `histogram_buffer::updateHistogram` still throws on them as a safety net, so the check currently runs twice per word.
- **Still open:** it isn't thread-safe. `updateHistograms` and dequeue/queue must run on the same thread until the state becomes atomic. Making `dequeueHistogramPair()` lock-free also needs a back-out if the older buffer is reused between taking the newer and the older one.

### Hardware findings (GenX320 on the Jetson, Oct 2026)

- **Startup blackout:** about 200–220 ms of TIME_HIGH goes missing a few windows (≈30 ms) after STREAMON. It is a camera kernel driver quirk at startup, not a pipeline bug, and appeared in both recordings.
- **Startup burst:** the first window after STREAMON had ~55,900 active pixels (over half the sensor) versus a median of ~790 for the rest of a quiet scene. Discard every window up to the end of the blackout (≈250 ms after STREAMON) before optic flow or analysis.
- **Windows per camera buffer:** in a low-event-rate scene each V4L2 buffer held 5–8 ms of events, i.e. 2–3 windows. Quiet scenes make buffers span more time, which is what makes newest-only `dequeueHistogramPair()` skip windows. With in-order `dequeueNextHistogramPair()` a 100-buffer recording (≈850 ms) gave 255 records with every gap exactly 40 ticks apart except the startup blackout. 3 windows per buffer is the limit at `HISTOGRAM_BUFFER_COUNT = 5`; it needs to be at least windows per buffer + 2, so raise it (e.g. to 8, +600 KB) if quieter scenes exceed that.
- Per-pixel window values seen so far range from −4 to +8.

### Open items

- Discard the startup burst/blackout windows in the pipeline.
- Possibly raise `HISTOGRAM_BUFFER_COUNT` (see above).
- Implement `histogram_filters` and the Lucas-Kanade solve, and give the opticFlow module a CMake target with Eigen.
- Spelling: `GUASSIAN` / `KERNAL` in `config.h` and `Guassian` in the `histogram_filters` function names.
- Time surfaces, IMU fusion and the PX4 link.
- Move the capture test onto `Evt21Streamer`; it also doesn't check `V4L2_BUF_FLAG_ERROR`.
- Nits: `Evt21StreamerFromFile::start()` reports "initialize the file before starting it" when it is already started; `evt21Streamer/README.md` misspells `genx320Drvier`.
- Add a `.gitignore` for build folders and `.bin`/`.raw` recordings.

Each module directory has a README stating its intended contract. Read it before changing a module.

## EVT2.1 format

The camera produces 64-bit little-endian words. Bits [63:60] hold the type:
- `0x0` NEG / `0x1` POS: [59:54] timestamp LSBs, [53:43] x (aligned to 32), [42:32] y, [31:0] valid mask for pixels x..x+31.
- `0x8` TIME_HIGH: [59:32] timestamp bits 33..6, so one TIME_HIGH tick is 64 µs.
- Also `0xA` EXT_TRIGGER, `0xE` OTHERS, `0xF` CONTINUED. These are not in the `evt21_type` enum (`config/config.h`), so switches over it need a default case.

Timestamps are in µs and 34 bits wide. `timestamp_us` is `uint32_t` and wraps after about 71 minutes.

## Conventions

Error handling uses exceptions. Syscall failures throw `std::system_error` with errno, misuse throws `std::logic_error`, and bad data throws `std::runtime_error`. V4L2 helpers shared across files live in `v4l2Util.h`; helpers used by only one .cpp stay in that file's anonymous namespace. Classes owning hardware are non-copyable and release resources in a `noexcept` `stop()` that the destructor also calls.

## Workflow

Roshan Patel is currently the only contributor. "Push this" means stage everything, untracked files included (`git add -A`), commit, and push the current branch with `git push origin HEAD`. Commit straight to whatever branch is checked out (currently `streamline-driver-streamer`).

Code reaches the Jetson only through git: commit and push here, then `git pull` on the Jetson (repo at `~/Jetson-Event-VIO`, reachable over SSH as `jetson@jetson.local`). Recordings come back the other way with `scp`. Over Wi-Fi a 50 MB recording takes more than a minute, so use a long timeout. Always check `git status` before saying something is or isn't committed.
