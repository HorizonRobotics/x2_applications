cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
option(RELEASE_LIB "build version of release" ON)

add_definitions(-DHR_POSIX)
add_definitions(-DHR_LINUX)
add_definitions(-DX2)
list(APPEND CMAKE_C_FLAGS " -march=armv8-a -mcpu=cortex-a53 ")

# 编译模式
if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
else ()
    set(CMAKE_BUILD_TYPE Debug)
endif ()

SET(DEBUG_POSTFIX "d")
set(CMAKE_DEBUG_POSTFIX ${DEBUG_POSTFIX})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
link_directories(${CMAKE_BINARY_DIR}/lib)