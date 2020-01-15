def cnnmethod_test(name, case):
    native.cc_test(
        name = name,
        deps = [":cnnmethod", "@googletest//:gtest", "@vision_type//:vision_type", "@opencv_aarch64//:opencv_aarch64"],
        srcs = [":test_src"] + case,
        includes = ["test"],
)
