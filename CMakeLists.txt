cmake_minimum_required(VERSION 3.20)
project(ptit)

set(CMAKE_CXX_STANDARD 20)

#adding executables
add_executable(ptit main.cpp primitives.cpp image-creator.cpp)