cmake_minimum_required(VERSION 2.8)

set(BASE_LIBRARIES jsoncpp pthread dl rt)
set(OpenCVLib libopencv_world.so)
# set(OpenCVDeps libprotobuf libjpeg-turbo libwebp libpng libtiff libjasper zlib tegra_hal quirc -ldl -lm -lpthread -lrt)

set(XROC_LB -Wl,--whole-archive xroc-framework -Wl,--no-whole-archive)
set(X2_PREBUILT_LIB bpu_predict libfb.so libcam.so libvio.so libcnn_intf.so)

file(GLOB_RECURSE EXAMPLE_SOURCE_FILES *.cpp)

add_executable(CNNMethod_example
               ${EXAMPLE_SOURCE_FILES}
)
include_directories(include)

target_link_libraries(CNNMethod_example
  CNNMethod
  FasterRCNNMethod
  xroc-imagetools
  ${XROC_LB}
  ${X2_PREBUILT_LIB}
  ${BASE_LIBRARIES}
  ${OpenCVLib}
  ${OpenCVDeps}
  libhobotlog.a
  libturbojpeg.a
  libyuv.a
  hbrt_bernoulli_aarch64
)

#install(TARGETS example
#  DESTINATION ${MY_OUTPUT_ROOT}/bin)
