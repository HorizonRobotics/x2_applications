cmake_minimum_required(VERSION 2.8)
# set compiler option
option(CPU "build for CPU" OFF)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)

link_directories(
        ${PROJECT_SOURCE_DIR}/lib
)

# test case的所有cpp文件
set(SOURCE_FILES
        gtest_main.cpp
        gtest_msg.cpp
        )
# 添加依赖
## base deps
if (PLATFORM_WIN)
    set(BASE_LIBRARIES gtest.lib hobotlog.lib)
else ()
    set(BASE_DEPS -ldl -lm -lpthread -lrt)
    set(BASE_LIBRARIES ${BASE_LIBRARIES} hbipc_cp libgtest.a libhobotlog.a libprotobuf.a libjsoncpp.a)
    set(BASE_LIBRARIES ${BASE_LIBRARIES} ${BASE_DEPS})
endif ()

## opencv lib & deps
set(OpenCVLib opencv_world)
set(OpenCVDeps libprotobuf libjpeg-turbo libwebp libpng libtiff
        libjasper zlib tegra_hal quirc -ldl -lm -lpthread -lrt)

#set(BASE_LIBRARIES ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})

# hobot_util.cmake已经设置了在build/bin中生成可执行文件，默认应该在test目录里生成
add_executable(unit_test ${SOURCE_FILES})
target_link_libraries(unit_test ${CMAKE_PROJECT_NAME} ${BASE_LIBRARIES})
