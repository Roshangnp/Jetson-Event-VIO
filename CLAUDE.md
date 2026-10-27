# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Visual-inertial odometry for a Jetson Orin NX using a Prophesee GenX320 event camera (320×320). The goal is to estimate velocity from ego-motion in the event stream and send it to a PX4 flight controller. The camera input path, the EVT2.1 decoder and the event histograms exist so far. Time surfaces, optic flow, IMU fusion and the PX4 link are unimplemented.

## Build

There is no top-level build. Each test is its own CMake project. The capture test (`.vscode/settings.json` points CMake Tools at it):

```
cmake -S tests/write_evt21_to_file -B build && cmake --build build
./build/write_evt21_to_file <video-device> <sensor-device>   # writes 1 s of raw EVT2.1 to ./log.raw
```

`tests/histogram_buffer_writer` is a second CMake project: camera → `Evt21Streamer` → `histograms` → `writeHistograms`, logging the newer histogram of each pair to `./histogram_recording.bin` for 100 camera buffers:

```
cmake -S tests/histogram_buffer_writer -B build_hbw && cmake --build build_hbw
./build_hbw/histogram_buffer_writer <video-device> <sensor-device>
```

Both build with C++17 and `-Wall -Wextra -Wpedantic`. Running it needs the GenX320 V4L2 driver on the Jetson.

Modules include `types.h` / `config.h` by bare name, so compiling them needs `-Iinclude -Iconfig`. To check a single file builds:

```
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude -Iconfig -c src/modules/addressEventCamera/evt21Decoder.cpp
```

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
  - `histogram_buffer` / `histograms` are implemented: a ring of 5 per-pixel signed event histograms that hands out time-adjacent pairs for optic flow. Time surfaces don't exist yet. None of these files are in a CMake target yet.

- **`fileWriter`** (`src/modules/fileWriter/`): `writeHistograms` logs histograms (`uint64_t` TIME_HIGH start + raw 320×320 `int16_t`, fixed-size records, no file header) from a background thread. `tryWrite()` copies into a preallocated 32-slot ring and returns `false` (counting a drop) when full, never blocking the pipeline. Only the ring is shared between threads, so the pipeline stays single-threaded. Linking it needs `-pthread`.

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
  - `queueHistogramPair()` releases only the older buffer. The newer one stays held as the next pair's older half.
  - Taking a new pair moves any HELD buffer that isn't part of it to FREE (the consumer fell behind). At most 3 buffers are ever unavailable (1 FILLING plus 2 HELD), so the writer always has a buffer.
  - Dequeuing twice without queuing, or queuing with nothing held, throws `std::logic_error`.
- **Bad words:** POS/NEG words failing `is_valid_event` (misaligned x, or off the sensor) are skipped and counted in `getBadWordCount()` rather than stopping the stream. `histogram_buffer::updateHistogram` still throws on them as a safety net, so the check currently runs twice per word.
- **Still open:** it isn't thread-safe. `updateHistograms` and dequeue/queue must run on the same thread until the state becomes atomic. Making `dequeueHistogramPair()` lock-free also needs a back-out if the older buffer is reused between taking the newer and the older one.

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

Roshan Patel is currently the only contributor. "Push this" means stage everything, untracked files included (`git add -A`), commit, and push straight to main with `git push origin main`.
