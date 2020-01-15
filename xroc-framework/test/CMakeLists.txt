include_directories(
        ${PROJECT_SOURCE_DIR}/include
        include
        include/method
        include/hobotxroc
)

set(METHOD_FILES
          src/method/bbox_filter.cpp
          src/method/MultiSourceTestMethod.cpp
   )

set(SOURCE_FILES ${METHOD_FILES}
        gtest_main.cc
        method_factory.cpp
    )

add_executable(xroc_unit_test ${SOURCE_FILES} method_cb.cpp)
target_link_libraries(xroc_unit_test ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

add_executable(cpp_api_test ${SOURCE_FILES}  cpp_api_test.cc)
target_link_libraries(cpp_api_test ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

add_executable(xroc_test ${SOURCE_FILES} xroc_test.cpp)
target_link_libraries(xroc_test  ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

add_executable(node_test ${SOURCE_FILES} node_test.cpp)
target_link_libraries(node_test  ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

add_executable(workflow_test ${SOURCE_FILES} subworkflow_test.cpp)
target_link_libraries(workflow_test  ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

#add_executable(c_api_test c_api_test.cc gtest_main.cc)
#target_link_libraries(c_api_test bbox_filter gtest pthread)

set(THREAD_MODEL_SOURCES ${SOURCE_FILES}
                         thread_model.cpp
                         src/method/bbox_filter.cpp
                         src/data_types/array.cpp
   )
add_executable(xroc_threadmodel_test ${THREAD_MODEL_SOURCES})
target_link_libraries(xroc_threadmodel_test ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

set(THREAD_ORDER_SOURCES ${SOURCE_FILES}
                         thread_order.cpp
                         src/data_types/orderdata.cpp
   )
add_executable(xroc_threadorder_test ${THREAD_ORDER_SOURCES})
target_link_libraries(xroc_threadorder_test ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

set(THREAD_SAFE_SOURCES ${SOURCE_FILES}
                         thread_safe.cpp
                         src/data_types/orderdata.cpp
   )
add_executable(xroc_threadsafe_test ${THREAD_SAFE_SOURCES})
target_link_libraries(xroc_threadsafe_test ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

set(MULTISOURCE_TEST_SOURCES ${SOURCE_FILES}
                         multisource_test.cpp
   )
add_executable(xroc_multisource_test ${MULTISOURCE_TEST_SOURCES})
target_link_libraries(xroc_multisource_test ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

add_executable(config_test ${SOURCE_FILES} config_test.cpp)
target_link_libraries(config_test ${PROJECT_NAME} hobotlog jsoncpp gtest pthread)

