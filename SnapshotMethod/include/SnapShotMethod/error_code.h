/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Error Code header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.03
 */

#ifndef SNAPSHOTMETHOD_ERROR_CODE_H_
#define SNAPSHOTMETHOD_ERROR_CODE_H_

/// 执行成功返回值
#define XROC_SNAPSHOT_OK 0
/// 参数错误
#define XROC_SNAPSHOT_ERR_PARAM -1
/// 无返回结果
#define XROC_SNAPSHOT_NO_RESULT -2

/// 外部冲刷上报状态码
#define FLUSH_POST_TYPE 0
/// 达到优选数上报状态码
#define READY_POST_TYPE 1

#endif  // SNAPSHOTMETHOD_ERROR_CODE_H_
