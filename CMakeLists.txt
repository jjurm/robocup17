cmake_minimum_required(VERSION 3.8)
project(flows)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        ai.c
        ai.smp)

add_library(flows SHARED ${SOURCE_FILES})
