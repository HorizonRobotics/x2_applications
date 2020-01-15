
//===----- hbdk_march.h - Runtime definition and helper function ---*- C -*-===//
// Automatically generated. DO NOT EDIT
//
//                     The HBDK Compiler Infrastructure
//
// This file is subject to the terms and conditions defined in file
// 'LICENSE.txt', which is part of this source code package.
//
//===-----------------------------------------------------------------------===//

#ifndef HBDK_MARCH_H
#define HBDK_MARCH_H

#pragma once

#include "hbdk_config.h"
#include <ctype.h>    // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)
#include <stdbool.h>  // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)
#include <stdint.h>   // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)
#include <stdlib.h>   // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)
#include <string.h>   // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)

#ifdef __cplusplus
extern "C" {
#endif

#define MARCH_NAME_LENGTH (3)
#define MAGIC_HEADER_SIZE (16)

typedef enum MARCH {
  MARCH_UNKNOWN = ('?' << 0) + ('?' << 8) + ('?' << 16),
  MARCH_X2 = ('X' << 0) + ('2' << 8) + (' ' << 16),
  MARCH_X2A = ('X' << 0) + ('2' << 8) + ('A' << 16),
} MARCH;

#ifdef __cplusplus
}
#endif

#endif // HBDK_MARCH_H
