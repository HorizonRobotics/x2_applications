set(TEST_SOURCE_SRC
        src/gtest_main.cpp
        src/face_det.cpp
        ../example/method_factory.cpp
        )

set(BASE_LIBRARIES jsoncpp pthread dl rt)

set(OpenCVLib opencv_world)
# set(OpenCVDeps libprotobuf libjpeg-turbo libwebp libpng libtiff libjasper zlib tegra_hal quirc -ldl -lm -lpthread -lrt)

set(BASE_LIBRARIES hobotlog ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})

set(XROC_LB -Wl,--whole-archive xroc-framework -Wl,--no-whole-archive)

add_executable(gtest_fasterrcnn ${TEST_SOURCE_SRC})

target_link_libraries(gtest_fasterrcnn
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
        gtest)
