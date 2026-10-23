cmake_minimum_required(VERSION 3.10)

project(write_evt21_to_file LANGUAGES CXX)

add_executable(write_evt21_to_file
    main.cpp
    ../../src/drivers/genx320/genx320Driver.cpp
)

target_compile_features(write_evt21_to_file PRIVATE cxx_std_17)
target_compile_options(write_evt21_to_file PRIVATE -Wall -Wextra -Wpedantic)
