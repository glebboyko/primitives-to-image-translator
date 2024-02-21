cmake_minimum_required(VERSION 3.20)
project(ptit)

set(CMAKE_CXX_STANDARD 20)

include_directories(include source)

add_library(${PROJECT_NAME}
        STATIC
        source/primitives.cpp
        source/extract_primitives.cpp
        source/supply.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib_")