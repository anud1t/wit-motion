cmake_minimum_required(VERSION 3.12)
project(wt901_reader_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(wt901_reader src/main.cpp)
target_compile_options(wt901_reader PRIVATE -Wall -Wextra -O2)


