cmake_minimum_required(VERSION 2.8)

include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/include
)

set(BASE_LIBRARIES jsoncpp pthread dl rt)

set(OpenCVLib opencv_world)

# set(OpenCVDeps libprotobuf libjpeg-turbo libwebp libpng libtiff libjasper zlib tegra_hal quirc -ldl -lm -lpthread -lrt)

set(BASE_LIBRARIES hobotlog ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})

set(XROC_LB -Wl,--whole-archive xroc-framework -Wl,--no-whole-archive)

add_executable(FasterRCNNMethod_example
        main.cpp
        method_factory.cpp
        input_util/vio.h
        input_util/camera.h
        dump_util.h
        stopwatch.cpp
        test_model_info.cpp
        test_faster_rcnn_image.cpp
        test_fb_pyramid.cpp
        test_fb_fasterrcnn.cpp
        dump_nv12.cpp
        test_x2_dev_single_pyramid.cpp
        test_x2_dev_dual_pyramid.cpp
        test_x2_dev_faster_rcnn_pyramid.cpp
        test_hbcc_info.cpp
        test_two_fasterrcnn.cpp
        dump_face_result.cpp
        )

target_link_libraries(FasterRCNNMethod_example
        FasterRCNNMethod
        ${XROC_LB}
        ${DYNAMIC_DEPS_LIBS}
        hobotlog
        jsoncpp
        ${OpenCVLib}
        ${OpenCVDeps}
        -lpthread
        -ldl
        -lrt
        )

