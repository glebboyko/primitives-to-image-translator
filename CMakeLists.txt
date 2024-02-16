cmake_minimum_required(VERSION 3.20)
project(ptit)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

add_library(${PROJECT_NAME}
        STATIC
        source/primitives.cpp
        source/image-creator.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib_")