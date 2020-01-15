/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework test Data
 * @author    ronghui.zhang
 * @email     ronghui.zhang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.11.28
 */

#include <stdlib.h>
#include <memory>

#include "hobotxroc/data_types/orderdata.h"

namespace HobotXRoc {

OrderData::OrderData(size_t sequece_id) {
    type_ = "OrderData";
    sequence_id = sequence_id;
}

OrderData::~OrderData() {
}
}  // namespace HobotXRoc

