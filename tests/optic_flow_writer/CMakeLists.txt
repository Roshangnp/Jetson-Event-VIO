cmake_minimum_required(VERSION 3.18)

# camera -> Evt21Streamer -> histograms -> optic_flow (CPU or CUDA) -> writeFlows
#
# Jetson (Orin, sm_87):  cmake .. && cmake --build .
# Desktop (RTX 5070 Ti): needs CUDA 12.8+ for sm_120, and the glibc 2.43 workaround:
#   cmake .. -DCMAKE_CUDA_COMPILER=/usr/local/cuda-13.1/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120 -DGLIBC_243_WORKAROUND=ON

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 87)
endif()

# JetPack installs CUDA in /usr/local/cuda without putting nvcc on PATH, so fall back to it when no
# compiler is given. A build folder from a configure that already failed here still has incomplete
# compiler detection cached, so delete it once (rm -rf build) rather than reconfiguring it.
if(NOT CMAKE_CUDA_COMPILER AND NOT DEFINED ENV{CUDACXX})
    find_program(NVCC_ON_PATH nvcc)
    if(NOT NVCC_ON_PATH AND EXISTS /usr/local/cuda/bin/nvcc)
        set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc CACHE FILEPATH "CUDA compiler" FORCE)
    endif()
endif()

# Set before project(): CMake's own CUDA compiler check already fails without it.
option(GLIBC_243_WORKAROUND "CUDA 13.1 headers clash with glibc 2.43's rsqrt/rsqrtf declarations" OFF)
if(GLIBC_243_WORKAROUND)
    set(CMAKE_CUDA_FLAGS "-Xcompiler=-U_GNU_SOURCE -D_DEFAULT_SOURCE -D_XOPEN_SOURCE=700 ${CMAKE_CUDA_FLAGS}")
endif()

project(optic_flow_writer LANGUAGES CXX CUDA)

find_package(Threads REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(optic_flow_writer
    main.cpp
    ../../src/drivers/genx320/genx320Driver.cpp
    ../../src/modules/evt21Streamer/evt21Streamer.cpp
    ../../src/modules/addressEventCamera/evt21Decoder.cpp
    ../../src/modules/addressEventCamera/histogram_buffer.cpp
    ../../src/modules/addressEventCamera/histograms.cpp
    ../../src/modules/opticFlow/histogram_filters.cpp
    ../../src/modules/opticFlow/histogram_filters.cu
    ../../src/modules/opticFlow/lucas_kanade.cpp
    ../../src/modules/opticFlow/lucas_kanade.cu
    ../../src/modules/opticFlow/optic_flow.cpp
    ../../src/modules/fileWriter/writeFlows.cpp
)

target_include_directories(optic_flow_writer PRIVATE
    ../../include
    ../../config
    ../../src/drivers/cuda
    ../../src/modules/evt21Streamer
    ../../src/modules/addressEventCamera
    ../../src/modules/opticFlow
    ../../src/modules/fileWriter
)
# SYSTEM so the warning flags don't report Eigen's own headers
target_include_directories(optic_flow_writer SYSTEM PRIVATE ../../external/eigen)

target_compile_features(optic_flow_writer PRIVATE cxx_std_17 cuda_std_17)
target_compile_definitions(optic_flow_writer PRIVATE EIGEN_MPL2_ONLY)
target_compile_options(optic_flow_writer PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
    # constexpr config (std::array kernels, grid_to_pixel) is used from device code; 20012 is
    # Eigen's harmless "__host__ __device__ annotation is ignored" warning
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr -diag-suppress=20012 -Xcompiler=-Wall,-Wextra>
)
target_link_libraries(optic_flow_writer PRIVATE Threads::Threads CUDA::cudart)
