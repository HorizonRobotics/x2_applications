cmake_minimum_required(VERSION 2.8)
# set compiler option
option(CPU "build for CPU" OFF)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)
# test case的所有cpp文件
set(SOURCE_FILES
        gtest_main.cc
        gtest_api.cpp
        )
# 添加依赖
## base deps
if (PLATFORM_WIN)
    set(BASE_LIBRARIES gtest.lib hobotlog.lib)
else ()
    set(BASE_DEPS -ldl -lm -lpthread -lrt)
    set(BASE_LIBRARIES ${BASE_LIBRARIES} -lxpluginflow libgtest.a libhobotlog.a libprotobuf.a)
endif ()

set(ARCH_LIBS vio cam fb)
set(BASE_LIBRARIES ${BASE_LIBRARIES} hobotlog vision_type_util jsoncpp ${ARCH_LIBS})

# hobot_util.cmake已经设置了在build/bin中生成可执行文件，默认应该在test目录里生成
add_executable(vioplugin_test ${SOURCE_FILES})
target_link_libraries(vioplugin_test ${CMAKE_PROJECT_NAME} ${BASE_LIBRARIES} ${BASE_DEPS})
#target_link_libraries(vioplugin_test dl pthread)
