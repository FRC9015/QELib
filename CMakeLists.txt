cmake_minimum_required(VERSION 3.16)
project(qelib)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)


# Your library
file(GLOB_RECURSE SOURCES "src/main/native/cpp/*.cpp")
add_library(qelib SHARED ${SOURCES})

target_include_directories(qelib PUBLIC "src/main/native/include")