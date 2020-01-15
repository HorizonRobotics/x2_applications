/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNConst.h
 * @Brief: definition of the const var
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:52:31
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 16:22:44
 */

#include "CNNMethod/CNNConst.h"
#include <vector>
#include <map>
#include <string>

namespace HobotXRoc {

const std::map<std::string, InputType> g_input_type_map = {
    {"rect", InputType::RECT},
    {"lmk", InputType::LMK_IMG},
    {"img", InputType::IMG}};

const std::map<std::string, PostFun> g_post_fun_map = {
    {"face_feature", PostFun::FACE_ID},
    {"antispoofing", PostFun::ANTI_SPF},
    {"lmk_pose", PostFun::LMK_POSE},
    {"age_gender", PostFun::AGE_GENDER},
    {"face_quality", PostFun::FACE_QUALITY}
};

const std::map<std::string, NormMethod> g_norm_method_map = {
    {"norm_by_width_length", NormMethod::BPU_MODEL_NORM_BY_WIDTH_LENGTH},
    {"norm_by_width_ratio", NormMethod::BPU_MODEL_NORM_BY_WIDTH_RATIO},
    {"norm_by_height_rario", NormMethod::BPU_MODEL_NORM_BY_HEIGHT_RATIO},
    {"norm_by_lside_ratio", NormMethod::BPU_MODEL_NORM_BY_LSIDE_RATIO},
    {"norm_by_height_length", NormMethod::BPU_MODEL_NORM_BY_HEIGHT_LENGTH},
    {"norm_by_lside_length", NormMethod::BPU_MODEL_NORM_BY_LSIDE_LENGTH},
    {"norm_by_lside_square", NormMethod::BPU_MODEL_NORM_BY_LSIDE_SQUARE},
    {"norm_by_diagonal_square", NormMethod::BPU_MODEL_NORM_BY_DIAGONAL_SQUARE},
    {"norm_by_nothing", NormMethod::BPU_MODEL_NORM_BY_NOTHING}
};

const std::map<std::string, FilterMethod> g_filter_method_map = {
    {"out_of_range", FilterMethod::OUT_OF_RANGE},
    {"no_filter", FilterMethod::NO_FILTER}
};

const std::vector<float> g_lmk_template = {38.2946f,
                                           51.6963f,
                                           73.5318f,
                                           51.5014f,
                                           56.0252f,
                                           71.7366f,
                                           41.5493f,
                                           92.3655f,
                                           70.7299f,
                                           92.2041f};

const std::vector<int> g_age_range = {
    1, 6, 7, 12, 13, 18, 19, 28, 29, 35, 36, 45, 46, 55, 56, 100};

}  // namespace HobotXRoc
