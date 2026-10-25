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

Each module directory has a README stating its intended contract. Read it before changing a module.

## EVT2.1 format

The camera produces 64-bit little-endian words. Bits [63:60] hold the type:
- `0x0` NEG / `0x1` POS: [59:54] timestamp LSBs, [53:43] x (aligned to 32), [42:32] y, [31:0] valid mask for pixels x..x+31.
- `0x8` TIME_HIGH: [59:32] timestamp bits 33..6.
- Also `0xA` EXT_TRIGGER, `0xE` OTHERS, `0xF` CONTINUED. These are not in the `evt21_type` enum (`config/config.h`), so switches over it need a default case.

Timestamps are in µs and 34 bits wide. `timestamp_us` is `uint32_t` and wraps after about 71 minutes.

## Conventions

Error handling uses exceptions. Syscall failures throw `std::system_error` with errno, misuse throws `std::logic_error`, and bad data throws `std::runtime_error`. Each .cpp keeps its local helpers (`xioctl` retrying on EINTR, `fail`) in an anonymous namespace. Classes owning hardware are non-copyable and release resources in a `noexcept` `stop()` that the destructor also calls.
