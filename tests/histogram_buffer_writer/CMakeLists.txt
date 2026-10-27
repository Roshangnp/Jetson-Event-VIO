cmake_minimum_required(VERSION 3.10)

project(histogram_buffer_writer LANGUAGES CXX)

find_package(Threads REQUIRED)

add_executable(histogram_buffer_writer
    main.cpp
    ../../src/drivers/genx320/genx320Driver.cpp
    ../../src/modules/evt21Streamer/evt21Streamer.cpp
    ../../src/modules/addressEventCamera/evt21Decoder.cpp
    ../../src/modules/addressEventCamera/histogram_buffer.cpp
    ../../src/modules/addressEventCamera/histograms.cpp
    ../../src/modules/fileWriter/writeHistograms.cpp
)

target_include_directories(histogram_buffer_writer PRIVATE
    ../../include
    ../../config
    ../../src/modules/evt21Streamer
    ../../src/modules/addressEventCamera
    ../../src/modules/fileWriter
)

target_compile_features(histogram_buffer_writer PRIVATE cxx_std_17)
target_compile_options(histogram_buffer_writer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(histogram_buffer_writer PRIVATE Threads::Threads)
