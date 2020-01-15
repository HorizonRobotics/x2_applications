//===----- hbdk_error.h - Error handling function ---*- C -*-===//
//
//                     The HBDK Compiler Infrastructure
//
// This file is subject to the terms and conditions defined in file
// 'LICENSE.txt', which is part of this source code package.
//
///===----------------------------------------------------------------------===
/// @file
/// Error handling related features\n
///===----------------------------------------------------------------------===

#ifndef HBDK3_HBDK_ERROR_H
#define HBDK3_HBDK_ERROR_H

#include "hbdk_config.h"

#ifdef __cplusplus
#include <functional>
extern "C" {
#endif

#include <stdio.h>

enum enumhbrtError {

  /// The API call returned with no errors.
  hbrtSuccess = 0,

  /// The file path passed to the API is invalid.
  hbrtErrorCannotOpenFile,

  /// The API is trying to load a hbm to a memory chunk which is already occupied by another hbm.
  hbrtErrorHBMIdIsBusy,

  /// The number of loaded hbm has exceeded limit.
  hbrtErrorHBMSlotIsFull,

  /// The API is trying to load a hbm which is already loaded before.
  hbrtErrorHBMAlreadyLoaded,

  /// The API is trying to laod a hbm compiled for a different march.
  hbrtErrorHBMCCForDifferentMARCH,

  /// The crc32 checksum is not same as what is recorded in it.
  hbrtErrorHBMCRC32VerifyFail,

  /// The malloc api called inside a API returned null_ptr.
  hbrtErrorMemoryAllocationFail,

  /// The memory passed to a API is too small to contain data.
  hbrtErrorMemoryOverflow,

  /// A numeric overflow occurred inside the API
  hbrtErrorNumericOverflow,

  /// The core mask passed to a API is illegal.
  hbrtErrorIllegalCoreMask,

  /// The march passed to a API or decoded from hbm is illegal.
  hbrtErrorIllegalMARCH,

  /// The input hbm to the API is illegal.
  hbrtErrorIllegalHBM,

  /// The hbm handle passed to the API is illegal.
  hbrtErrorIllegalHBMHandle,

  /// The element type passed to the API or decoded from hbm is illegal.
  hbrtErrorIllegalElementType,

  /// The input source passed to the API or decoded from hbm is illegal.
  hbrtErrorIllegalInputSourceType,

  /// The region type passed to the API or decoded from hbm is illegal.
  hbrtErrorIllegalRegionType,

  /// The ri_id passed to the API or grabbed from handle is illegal.
  hbrtErrorIllegalRIID,

  /// The register value decoded from hbm or determined by runtime is illegal.
  hbrtErrorIllegalRegisterValue,

  /// The output region passed to the API or created by runtime is illegal.
  hbrtErrorIllegalOutputRegion,

  /// The intermediate region created by runtime is illegal.
  hbrtErrorIllegalIntermediateRegion,

  /// The heap region created by runtime is illegal.
  hbrtErrorIllegalHeapRegion,

  /// The RI configuration passed to the API or grabbed from handle is illegal.
  hbrtErrorIllegalRIConfig,

  /// The register type decoded from hbm is illegal.
  hbrtErrorIllegalRegisterType,

  /// The CPU operator parameters decoded from hbm is illegal.
  hbrtErrorIllegalCPUOperator,

  /// The layout passed to a API or grabbed from handle is illegal.
  hbrtErrorIllegalLayout,

  /// A illegal memory access occurred inside the API.
  hbrtErrorIllegalMemoryRead,

  /// Try to execute a model which can not be accessed by corresponding API.
  hbrtErrorIllegalModel,

  /// The hbm handle passed to the API is invalid.
  hbrtErrorInvalidHBMHandle,

  /// The input model handle passed to the API is invalid.
  hbrtErrorInvalidModelHandle,

  /// The feature handle passed to the API is invalid.
  hbrtErrorInvalidFeatureHandle,

  /// The model name passed to the API is invalid or the hbm does not contain a model with that name.
  hbrtErrorInvalidModelName,

  /// The input index passed to the API is invalid or exceeded the model input number.
  hbrtErrorInvalidInputIndex,

  /// The output index passed to the API is invalid or exceeded the model input number.
  hbrtErrorInvalidOutputIndex,

  /// The output number exceeds the max number
  hbrtErrorInvalidOutputNumber,

  /// The batch count decoded from hbm is invalid.
  hbrtErrorInvalidBatchCount,

  /// The segment index grabbed from ri or another structure is invalid.
  hbrtErrorInvalidSegmentIndex,

  /// The interrupt number passed to the API is invalid.
  hbrtErrorInvalidInterruptNum,

  /// The resizer parameters passed to the API is invalid.
  hbrtErrorInvalidResizerParam,

  /// The data work passed to the API is invalid.
  hbrtErrorInvalidDataWork,

  /// The address passed to the API or grabbed from other structures is invalid.
  hbrtErrorInvalidAddress,

  /// An error occurred during RLE decoding.
  hbrtErrorInvalidRle,

  /// The ROI passed to RoiAlign is invalid.
  hbrtErrorInvalidRoi,

  /// The memory pool is broken somewhere.
  hbrtErrorInvalidMemoryPool,

  /// The global config is unset or invalid.
  hbrtErrorInvalidConfig,

  /// The bpu_memcpy returned non-zero value inside the API.
  hbrtErrorBPUCPUMemcpyFail,

  /// The bpu_mem_alloc returned null_ptr inside the API.
  hbrtErrorBPUMemAllocFail,

  /// The bpu_cpumem_alloc returned null_ptr inside the API.
  hbrtErrorBPUCPUMemAllocFail,

  /// The ri id passed to the API is not bound with any model
  hbrtErrorRiIsNotInUse,

  /// The API is trying to start a ri with ri id which is not destroyed yet.
  hbrtErrorRiIsInUse,

  /// The memory to store function call is to small to contain generated function calls.
  hbrtErrorFunccallSlotNotEnough,

  /// The API is not implemented in current version yet.
  hbrtErrorFunctionNotImplemented,

  /// The version is incompatible
  hbrtErrorIncompatibleVersion,

  // The license is not match
  hbrtErrorInvalidLicense,
};

typedef enum enumhbrtError hbrt_error_t;

static inline void _hbdk_error_print_to_stderr(const char* msg) {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
  fprintf(stderr, "%s\n", msg);
}

static inline void _hbdk_int_print_to_stderr(int num) {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
  fprintf(stderr, "%d\n", num);
}

inline static void _hbrtBreakInClion() {}

#define HBRT_ERROR(...)             \
  do {                              \
    fprintf(stderr, "error: ");     \
    fprintf(stderr, ##__VA_ARGS__); \
    abort();                        \
  } while (false)

#define HBRT_ERROR_IF_NOT(cond, ...) \
  do {                               \
    if (!cond) {                     \
      HBRT_ERROR(__VA_ARGS__);       \
    }                                \
  } while (false)

#define CHECK_HBRT_ERROR(return_val)                                \
  do {                                                              \
    hbrt_error_t __hbrt_status = return_val;                        \
    if (__hbrt_status) {                                            \
      _hbdk_error_print_to_stderr(hbrtGetErrorName(__hbrt_status)); \
      _hbdk_error_print_to_stderr(__FILE__);                        \
      _hbdk_int_print_to_stderr(__LINE__);                          \
      abort();                                                      \
    }                                                               \
  } while (0)

#ifdef FOR_DEV_USE
#define RETURN_HBRT_ERROR(return_val)                               \
  do {                                                              \
    hbrt_error_t __hbrt_status = return_val;                        \
    if (__hbrt_status) {                                            \
      _hbdk_error_print_to_stderr(hbrtGetErrorName(__hbrt_status)); \
      _hbdk_error_print_to_stderr(__FILE__);                        \
      _hbdk_int_print_to_stderr(__LINE__);                          \
      abort();                                                      \
      return __hbrt_status;                                         \
    }                                                               \
  } while (0)
#else
#define RETURN_HBRT_ERROR(return_val)                               \
  do {                                                              \
    hbrt_error_t __hbrt_status = return_val;                        \
    if (__hbrt_status) {                                            \
      _hbdk_error_print_to_stderr(hbrtGetErrorName(__hbrt_status)); \
      _hbdk_error_print_to_stderr(__FILE__);                        \
      _hbdk_int_print_to_stderr(__LINE__);                          \
      return __hbrt_status;                                         \
    }                                                               \
  } while (0)
#endif

#define RETURN_HBRT_ERROR_IF_NOT(cond, return_val) \
  do {                                             \
    if (!(cond)) {                                 \
      RETURN_HBRT_ERROR(return_val);               \
    }                                              \
  } while (0)

// goto the label "goto_label:" within the current function if return_val indicates an hbrt error
// The return_val is stored in the variable hbrt_error_t "err_var"
#define HBRT_GOTO_ON_ERROR(return_val, goto_label, err_var) \
  do {                                                      \
    (err_var) = return_val;                                 \
    if (err_var) {                                          \
      _hbrtBreakInClion();                                  \
      goto goto_label;                                      \
    }                                                       \
  } while (0)

// goto the label "done:" within the current function if return_val indicates an hbrt error
// Use this macro if you want to write less code to free resource on HBRT error.
// Require to predefine "hbrt_error_t err;" because the return_val will be stored in that variable.
#define HBRT_GOTO_DONE_ON_ERROR(return_val) HBRT_GOTO_ON_ERROR(return_val, done, err)

// same as HBRT_GOTO_DONE_ON_ERROR if cond is false
#define HBRT_GOTO_DONE_ON_ERROR_IF_NOT(cond, return_val) \
  do {                                                   \
    if (!(cond)) {                                       \
      HBRT_GOTO_DONE_ON_ERROR(return_val);               \
    }                                                    \
  } while (0)

static inline const char* hbrtGetErrorName(hbrt_error_t error) {
  switch (error) {
    case hbrtSuccess:
      return "hbrtSuccess";
    case hbrtErrorMemoryAllocationFail:
      return "hbrtErrorMemoryAllocationFail";
    case hbrtErrorCannotOpenFile:
      return "hbrtErrorCannotOpenFile";
    case hbrtErrorHBMAlreadyLoaded:
      return "hbrtErrorHBMAlreadyLoaded";
    case hbrtErrorIllegalCoreMask:
      return "hbrtErrorIllegalCoreMask";
    case hbrtErrorHBMIdIsBusy:
      return "hbrtErrorHBMIdIsBusy";
    case hbrtErrorHBMSlotIsFull:
      return "hbrtErrorHBMSlotIsFull";
    case hbrtErrorIllegalHBM:
      return "hbrtErrorIllegalHBM";
    case hbrtErrorHBMCRC32VerifyFail:
      return "hbrtErrorHBMCRC32VerifyFail";
    case hbrtErrorIllegalHBMHandle:
      return "hbrtErrorIllegalHBMHandle";
    case hbrtErrorBPUCPUMemcpyFail:
      return "hbrtErrorBPUCPUMemcpyFail";
    case hbrtErrorInvalidHBMHandle:
      return "hbrtErrorInvalidHBMHandle";
    case hbrtErrorInvalidModelHandle:
      return "hbrtErrorInvalidModelHandle";
    case hbrtErrorInvalidFeatureHandle:
      return "hbrtErrorInvalidFeatureHandle";
    case hbrtErrorInvalidModelName:
      return "hbrtErrorInvalidModelName";
    case hbrtErrorIllegalElementType:
      return "hbrtErrorIllegalElementType";
    case hbrtErrorIllegalInputSourceType:
      return "hbrtErrorIllegalInputSourceType";
    case hbrtErrorIllegalRegionType:
      return "hbrtErrorIllegalRegionType";
    case hbrtErrorIllegalRIID:
      return "hbrtErrorIllegalRIID";
    case hbrtErrorRiIsNotInUse:
      return "hbrtErrorRiIsNotInUse";
    case hbrtErrorRiIsInUse:
      return "hbrtErrorRiIsInUse";
    case hbrtErrorHBMCCForDifferentMARCH:
      return "hbrtErrorHBMCCForDifferentMARCH";
    case hbrtErrorBPUMemAllocFail:
      return "hbrtErrorBPUMemAllocFail";
    case hbrtErrorBPUCPUMemAllocFail:
      return "hbrtErrorBPUCPUMemAllocFail";
    case hbrtErrorInvalidInputIndex:
      return "hbrtErrorInvalidInputIndex";
    case hbrtErrorInvalidOutputIndex:
      return "hbrtErrorInvalidOutputIndex";
    case hbrtErrorInvalidOutputNumber:
      return "hbrtErrorInvalidOutputNumber";
    case hbrtErrorInvalidBatchCount:
      return "hbrtErrorInvalidBatchCount";
    case hbrtErrorIllegalRegisterValue:
      return "hbrtErrorIllegalRegisterValue";
    case hbrtErrorInvalidSegmentIndex:
      return "hbrtErrorInvalidSegmentIndex";
    case hbrtErrorFunccallSlotNotEnough:
      return "hbrtErrorFunccallSlotNotEnough";
    case hbrtErrorInvalidInterruptNum:
      return "hbrtErrorInvalidInterruptNum";
    case hbrtErrorInvalidResizerParam:
      return "hbrtErrorInvalidResizerParam";
    case hbrtErrorIllegalOutputRegion:
      return "hbrtErrorIllegalOutputRegion";
    case hbrtErrorIllegalIntermediateRegion:
      return "hbrtErrorIllegalIntermediateRegion";
    case hbrtErrorIllegalHeapRegion:
      return "hbrtErrorIllegalHeapRegion";
    case hbrtErrorIllegalRIConfig:
      return "hbrtErrorIllegalRIConfig";
    case hbrtErrorFunctionNotImplemented:
      return "hbrtErrorFunctionNotImplemented";
    case hbrtErrorInvalidDataWork:
      return "hbrtErrorInvalidDataWork";
    case hbrtErrorIllegalRegisterType:
      return "hbrtErrorIllegalRegisterType";
    case hbrtErrorIllegalCPUOperator:
      return "hbrtErrorIllegalCPUOperator";
    case hbrtErrorInvalidAddress:
      return "hbrtErrorInvalidAddress";
    case hbrtErrorInvalidRle:
      return "hbrtErrorInvalidRle";
    case hbrtErrorIllegalLayout:
      return "hbrtErrorIllegalLayout";
    case hbrtErrorMemoryOverflow:
      return "hbrtErrorMemoryOverflow";
    case hbrtErrorIllegalMARCH:
      return "hbrtErrorIllegalMARCH";
    case hbrtErrorIllegalMemoryRead:
      return "hbrtErrorIllegalMemoryRead";
    case hbrtErrorInvalidRoi:
      return "hbrtErrorInvalidRoi";
    case hbrtErrorNumericOverflow:
      return "hbrtErrorNumericOverflow";
    case hbrtErrorIllegalModel:
      return "hbrtErrorIllegalModel";
    case hbrtErrorInvalidMemoryPool:
      return "hbrtErrorInvalidMemoryPool";
    case hbrtErrorInvalidConfig:
      return "hbrtErrorInvalidConfig";
    case hbrtErrorIncompatibleVersion:
      return "hbrtErrorIncompatibleVersion";
    case hbrtErrorInvalidLicense:
      return "hbrtErrorInvalidLicense";
  }
  return "unregistered error code.";
}

#ifdef __cplusplus
}
#endif

#endif  // HBDK3_HBDK_ERROR_H
