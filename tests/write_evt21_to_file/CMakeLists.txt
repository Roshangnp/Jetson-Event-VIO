cmake_minimum_required(VERSION 3.10)

# Without a build type CMake passes no -O flag, so the code runs unoptimized (several times slower on
# the Jetson).
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

project(write_evt21_to_file LANGUAGES CXX)

add_executable(write_evt21_to_file
    main.cpp
    ../../src/drivers/genx320/genx320Driver.cpp
)

target_compile_features(write_evt21_to_file PRIVATE cxx_std_17)
target_compile_options(write_evt21_to_file PRIVATE -Wall -Wextra -Wpedantic)
