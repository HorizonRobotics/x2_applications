
def xroc_test(name, case):
    native.cc_test(
        name = name,
        deps = [":xroc-framework","@googletest//:gtest","@vision_type//:vision_type","@opencv_centos7//:opencv_centos7"],
        data = native.glob(["test/configs/**", "test/configs_subworkflow/**"]),
        srcs = [":test_src"] + case,
        includes = ["test/include","test","external/vision_type/include"],
)



