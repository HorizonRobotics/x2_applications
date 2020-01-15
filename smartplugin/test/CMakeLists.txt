cmake_minimum_required(VERSION 2.8)
# set compiler option
option(CPU "build for CPU" OFF)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)
# test case的所有cpp文件
set(SOURCE_FILES
        gtest_main.cc
        test_api.cpp
        )
# 添加依赖
## base deps
if (PLATFORM_WIN)
    set(BASE_LIBRARIES gtest.lib hobotlog.lib)
else ()
    set(BASE_DEPS -ldl -lm -lpthread -lrt)
    set(BASE_LIBRARIES ${BASE_LIBRARIES} libgtest.a libhobotlog.a)
    set(BASE_LIBRARIES ${BASE_LIBRARIES} ${BASE_DEPS})
endif ()

## opencv lib & deps
#set(OpenCVLib opencv_world)
#set(OpenCVDeps libprotobuf libjpeg-turbo libwebp libpng libtiff
#        libjasper zlib tegra_hal quirc -ldl -lm -lpthread -lrt)

# xroc lib
#set(XRocLib -Wl,--whole-archive xroc-framework -Wl,--no-whole-archive)
# xroc deps
#set(XRocDeps xroc_predictor xroc_basedata_helper hobotlog jsoncpp configparser pthread dl haes hcrypto)
# Method lib
#set(MethodLib CNNMethod AlphadetMethod)
# Method deps
#set(MethodDeps hobotsdk hobotthread alphadet alpha-det-prediction)

# cnn lib & deps
if (CPU)
    set(CNNLib ${CNNLib} mxnet_mkl.so mklml_intel.so libiomp5.so)
else ()  # mxnet
    set(CNNLib ${CNNLib} mxnet_shared tensorrt_predict nvinfer nvinfer_plugin)
    link_directories(/usr/local/cuda-8.0/lib64/)
    set(CNNLib ${CNNLib} cudnn7 cuda cudart cublas curand culibos cufft nvrtc cusolver)
endif ()

#set(BASE_LIBRARIES ${BASE_LIBRARIES} ${MethodLib} ${MethodDeps})
#set(BASE_LIBRARIES ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})
#set(BASE_LIBRARIES ${BASE_LIBRARIES} ${XRocLib} ${XRocDeps})
#set(BASE_LIBRARIES ${BASE_LIBRARIES} ${CNNLib})

# hobot_util.cmake已经设置了在build/bin中生成可执行文件，默认应该在test目录里生成
add_executable(unit_test ${SOURCE_FILES})
target_link_libraries(unit_test ${CMAKE_PROJECT_NAME} ${BASE_LIBRARIES})