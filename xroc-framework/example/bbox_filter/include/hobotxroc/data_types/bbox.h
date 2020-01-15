/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     bbox base data type
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.23
 */

#ifndef HOBOTXROC_DATA_TYPES_BBOX_H_
#define HOBOTXROC_DATA_TYPES_BBOX_H_

#include "hobotxsdk/xroc_data.h"
#include "horizon/vision_type/vision_type.hpp"

namespace HobotXRoc {

typedef XRocData<hobot::vision::BBox> BBox;

}  // namespace HobotXRoc

#endif  // HOBOTXROC_DATA_TYPES_BBOX_H_
