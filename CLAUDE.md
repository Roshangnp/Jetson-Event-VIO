# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Visual-inertial odometry for a Jetson Orin NX using a Prophesee GenX320 event camera (320×320). The goal is to estimate velocity from ego-motion in the event stream and send it to a PX4 flight controller. Only the camera input path exists so far; decoding, histograms/time surfaces, optic flow, IMU fusion and the PX4 link are unimplemented.

## Build

There is no top-level build. The only CMake project is the capture test (`.vscode/settings.json` points CMake Tools at it):

```
cmake -S tests/write_evt21_to_file -B build && cmake --build build
./build/write_evt21_to_file <video-device> <sensor-device>   # writes 1 s of raw EVT2.1 to ./log.raw
```

It builds with C++17 and `-Wall -Wextra -Wpedantic`. Running it needs the GenX320 V4L2 driver on the Jetson.

Modules include `types.h` / `config.h` by bare name, so compiling them needs `-Iinclude -Iconfig`. To check a single file builds:

```
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude -Iconfig -c src/modules/addressEventCamera/evt21Decoder.cpp
```

There is no test framework or linter.

## Architecture

Data flows driver → streamer → addressEventCamera (decoder, histograms, time surfaces) → future optic-flow / VIO.

- **`Genx320Driver`** (`src/drivers/genx320/`) is raw V4L2. The EVT-format control (`0x00982903`, value 1 = EVT2.1) is set on the **sensor subdevice**; the MMAP streaming queue (8 buffers) lives on the **video device**, opened `O_NONBLOCK`. It exposes `fd()` and `buffer(index)` and leaves DQBUF/QBUF to its owner.
- **`Evt21Streamer`** owns the driver and does poll/DQBUF/QBUF. `Evt21StreamerFromFile` replays a recorded `.evt21`/raw file. The two must keep **identical interfaces** so either can feed the pipeline: `initialize(source, sensor_device)`, `start()`, `stop()`, `getBuffer()` → `evt21_buffer` (`uint64_t*`), `wordCount()`, `releaseBuffer()`. Only one buffer may be held at a time; it must be released before the next `getBuffer()`. A camera buffer is zero-copy mapped memory and is requeued on release. The file streamer returns `nullptr` at EOF.
- **`addressEventCamera`** (`src/modules/addressEventCamera/`): `evt21Decoder.cpp` decodes word type and timestamps. `histogram_buffer` / `histograms` (a ring of 5 per-pixel count histograms that hands out ready pairs for optic flow) exist only as headers.

### Histogram memory design

- Each `histogram_buffer` holds its `histogram_array` inline (`std::array<uint16_t, 320*320>`, ~200 KB) and allocates nothing. `histograms` holds its 5 buffers by value in a `std::array`, so one `histograms` object is a single contiguous ~1 MB block. Don't turn these members into pointers or per-buffer heap allocations, because the count and size are compile-time constants and the buffers live exactly as long as their manager.
- `histogram_buffer` is non-copyable, which makes `histograms` non-copyable too. This prevents accidental 200 KB / 1 MB copies.
- Exactly one `histograms` instance is created by whatever owns the pipeline. Put it in static storage (namespace scope, or a member of a statically stored owner) or create it with a single `std::make_unique` at startup. Never make it a stack local, since 1 MB is too much for the stack, especially on threads. Don't use `static` data members inside the class. Buffer addresses stay fixed for the whole run, so consumers can take a `histograms&` or the `histogram_buffer*` pair from `getReadyHistogramPair()`.
- Each histogram covers `HISTOGRAM_TIME_HIGH_WIDTH = 40` TIME_HIGH ticks, at 64 µs per tick, which is ≈2.56 ms. Buffers are reused, never reallocated: `resetHistogram()` zeroes `hist_` in place.
- The consumer (optic flow) holds 2 histograms at a time, and the remaining slots give the producer room to keep filling. Still open: if the consumer runs on a different thread from `updateHistograms`, the `ready_`/`reset_` flags need to be atomic, and the consumer needs a way to hand a pair back before the producer can reuse those slots.

Each module directory has a README stating its intended contract. Read it before changing a module.

## EVT2.1 format

The camera produces 64-bit little-endian words. Bits [63:60] hold the type:
- `0x0` NEG / `0x1` POS: [59:54] timestamp LSBs, [53:43] x (aligned to 32), [42:32] y, [31:0] valid mask for pixels x..x+31.
- `0x8` TIME_HIGH: [59:32] timestamp bits 33..6, so one TIME_HIGH tick is 64 µs.
- Also `0xA` EXT_TRIGGER, `0xE` OTHERS, `0xF` CONTINUED. These are not in the `evt21_type` enum (`config/config.h`), so switches over it need a default case.

Timestamps are in µs and 34 bits wide. `timestamp_us` is `uint32_t` and wraps after about 71 minutes.

## Conventions

Error handling uses exceptions. Syscall failures throw `std::system_error` with errno, misuse throws `std::logic_error`, and bad data throws `std::runtime_error`. Each .cpp keeps its local helpers (`xioctl` retrying on EINTR, `fail`) in an anonymous namespace. Classes owning hardware are non-copyable and release resources in a `noexcept` `stop()` that the destructor also calls.

## Workflow

Roshan Patel is currently the only contributor. "Push this" means stage everything, untracked files included (`git add -A`), commit, and push straight to main with `git push origin main`.
