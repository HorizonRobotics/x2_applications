//===----- hbdk_hbrt.h - Runtime definition and helper function ---*- C -*-===//
//
//                     The HBDK Compiler Infrastructure
//
// This file is subject to the terms and conditions defined in file
// 'LICENSE.txt', which is part of this source code package.
//

/**
 * @mainpage
 * HBDK runtime (alias HBRT) is a library containing a series of APIs which application developers can use to control
 * the BPU chip. This library serves like CUDA for NVIDIA GPU and it depends on a specific horizon BPU system software,
 * which is more like GPU's driver.
 *
 * @section hbrtcando What HBRT can do
 *
 * A HBM file may contain several models. When given a specific model name,
 * the major task for runtime is to find out where the compiled binary of this model is.
 *
 * Including instructions of BPU, binary parameters, etc.
 * the runtime will figure out such kind of information and organize them as structural data.
 *
 * @section hbrtcannotdo What HBRT can NOT do
 *
 * Runtime does not implement any function for memory management. Such functions are provided by BPU system software.
 * Runtime can not do model-level schedule or thread-level schedule.
 */

//===----------------------------------------------------------------------===//
/// @file
/// This file defines structs and APIs used to run a running instance (RI),
/// which composed of a model and its memory context.
///
//===----------------------------------------------------------------------===//
#pragma once

#include "hbdk_config.h"
#include "hbdk_error.h"
#include "hbdk_layout.h"
#include "hbdk_march.h"
#include <stddef.h>  // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)

#ifndef bpu_addr_t
#define bpu_addr_t uint64_t
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * We use 64-bit handle to identify a model. This handle can be used as model identification
 * to get information such as input feature number, output feature byte size, etc.
 */
typedef struct {
  uint64_t handle;
} hbrt_model_handle_t;

/**
 * Similar to model handle. This handle can be used as hbm identification to get information such
 * as number of models in the hbm, etc.
 */
typedef struct {
  uint64_t handle;
} hbrt_hbm_handle_t;

/**
 * This struct stores the hbrt version info
 */
typedef struct {
  uint32_t major;
  uint32_t minor;
  uint32_t patch;
  char version[64];
  uint32_t _reserved[3];
} hbrt_version_info_t;

/**
 * Similar to model handle. This handle can be used as feature identification
 * to get information such as feature dimensions, element type, etc.
 */
typedef struct {
  uint64_t handle;
} hbrt_feature_handle_t;

/**
 * Maximum core number of all supported marches.
 */
#define MAX_CORE_NUMBER (2U)
#define MAX_HBM_NUMBER_ON_SINGLE_CORE 8
#define MAX_CORE_MASK (1U << MAX_CORE_NUMBER)
#define MAX_OUTPUT_NUMBER (32U)
#define MAX_RI_NUMBER (256U)

typedef struct {
  MARCH march;

  bpu_addr_t (*bpu_malloc)(int, int);  /// return bpu address
  bpu_addr_t (*cpu_malloc)(int, int);  /// return cpu address
  void (*bpu_free)(bpu_addr_t);        /// provide bpu address
  void (*cpu_free)(bpu_addr_t);        /// provide cpu address
  int (*bpu_memcpy)(bpu_addr_t, bpu_addr_t, unsigned, int);

  bool enable_hbrt_memory_pool;
  uint32_t ri_num_to_delay_free;
} hbrt_global_config_t;

#define HBRT_GLOBAL_CONFIG_INITIALIZER \
  { MARCH_UNKNOWN, bpu_mem_alloc, bpu_cpumem_alloc, bpu_mem_free, bpu_cpumem_free, bpu_memcpy, true, 64 }

HBDK_PUBLIC extern hbrt_error_t hbrtSetGlobalConfig(const hbrt_global_config_t *config);

/**
 * @brief Load a hbm binary from given address
 * @param [out] hbm_handle handle of this HBM, can be used as parameter for other APIs to obtain model
 * information.
 * @param [in] hbm_address address of memory containing a binary HBM. This address must be accessible for DMA.
 * @param [in] hbm_byte_size byte size of the HBM binary.
 * @return ::hbrtSuccess, ::hbrtErrorIllegalHBM, ::hbrtErrorHBMCCForDifferentMARCH, ::hbrtErrorHBMSlotIsFull,
 * ::hbrtErrorIllegalCoreMask, ::hbrtErrorBPUCPUMemcpyFail, ::hbrtErrorMemoryAllocationFail, ::hbrtIncompatibleVersion
 */
HBDK_PUBLIC extern hbrt_error_t hbrtLoadHBMFromAddr(hbrt_hbm_handle_t *hbm_handle, void *hbm_address,
                                                    size_t hbm_byte_size);

/**
 * @brief Load a hbm binary from given file path
 * @param [out] hbm_handle  handle of this HBM, can be used as parameter for other APIs to obtain model information.
 * @param [in] hbm_path  path to an HBM file.
 * @return ::hbrtSuccess, ::hbrtErrorCannotOpenFile, ::hbrtErrorIllegalHBM, ::hbrtErrorHBMCCForDifferentMARCH,
 * ::hbrtErrorHBMSlotIsFull, ::hbrtErrorIllegalCoreMask, ::hbrtErrorBPUCPUMemcpyFail, ::hbrtErrorMemoryAllocationFail,
 * ::hbrtIncompatibleVersion
 */
HBDK_PUBLIC extern hbrt_error_t hbrtLoadHBMFromFile(hbrt_hbm_handle_t *hbm_handle, const char *hbm_path);

/**
 * @brief Release memory occupied by a HBM.
 * @param [in] hbm_handle handle of the HBM to offload.
 * @return ::hbrtSuccess, ::hbrtErrorIllegalHBM
 * @note User must destroy all running instances which are bound with a model in this HBM explicitly.
 * Once a HBM is offloaded, other APIs would not be able to access any information of models in this HBM.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtOffloadHBM(hbrt_hbm_handle_t hbm_handle);

/**
 * @brief check if the CRC32 recorded in HBM is the same as in-time calculated CRC32 of corresponding memory.
 * @param [in] hbm_address address containing a HBM binary.
 * @return ::hbrtSuccess, ::hbrtErrorHBMCRC32VerifyFail, ::hbrtErrorIllegalHBM
 * @note The hbm_address must be verified before calling hbrtLoadFromAddr, i.e.,
 *
 *  1.	hbm_address = OpenFile(HBM);
 *  2.	hbrtVerifyHBM(hbm_address);
 *  3.	hbrtLoadFromAddr(hbm_address);
 *
 *  The verification will fail if it’s called after hbrtLoadHBMFromAddr, as hbrtLoadHBMFromAddr would relocate some
 * pointers in the binary.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtVerifyHBM(const void *hbm_address);

/*
 * APIs for model
 */

/**
 * @brief Know how many models are in a given HBM.
 * @param [out] model_number number of models in the HBM
 * @param [in] hbm_handle handle of a HBM
 * @return ::hbrtSuccess, ::hbrtErrorIllegalHBMHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelNumberInHBM(uint32_t *model_number, hbrt_hbm_handle_t hbm_handle);

/**
 * @brief Know what models are in a given HBM
 * @param [out] model_names names of models in the HBM
 * @param [in] hbm_handle handle of a HBM
 * @return ::hbrtSuccess, ::hbrtErrorIllegalHBMHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelNamesInHBM(const char ***model_names, hbrt_hbm_handle_t hbm_handle);

/**
 * @brief Get the handle of a given model
 * @param [out] model_handle handle of the given model
 * @param [in] hbm_handle handle of a HBM
 * @param [in] model_name name of a model
 * @return ::hbrtSuccess, ::hbrtErrorIllegalHBMHandle, ::hbrtErrorInvalidModelName, ::hbrtErrorIncompatibleVersion
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelHandle(hbrt_model_handle_t *model_handle, hbrt_hbm_handle_t hbm_handle,
                                                   const char *model_name);

/**
 * @brief Get the name of a given model
 * @param [out] model_name name of the given model
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtErrorIllegalHBMHandle, ::hbrtErrorInvalidModelName
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelName(const char **model_name, hbrt_model_handle_t model_handle);

/**
 * @brief Get the march of a given model
 * @param [out] march march of the given model
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelMarch(MARCH *march, hbrt_model_handle_t model_handle);

/**
 * @brief Get the core number of a given model
 * @param [out] core_num core number of the given model
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 * @note The core number of a model is determined during compilation. A two-core model must occupy two cores to finish
 * execution.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelCoreNumber(uint8_t *core_num, hbrt_model_handle_t model_handle);

/**
 * @brief Get the pe number of a given model
 * @param [out] pe_num pe number of the given model
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 * @note The pe number of a model is determined during compilation. A two-pe model must occupy two PEs to finish
 * execution.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelPeNumber(uint8_t *pe_num, hbrt_model_handle_t model_handle);

/**
 * @brief Get the description of a given model
 * @param [out] model_name name of the given model
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtErrorIllegalHBMHandle, ::hbrtErrorInvalidModelName
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelDescription(const char **description, hbrt_model_handle_t model_handle);

/**
 * @brief Get the estimated latency in microsecond of given model.
 * @param latency estimated latency in microsecond
 * @param model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtErrorIllegalHBMHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelEstimatedLatency(uint32_t *latency, hbrt_model_handle_t model_handle);

/**
 * @brief Know if the model has only one segment
 * @param [out] is_one_segment_model True means the model has only one segment.
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 * @note Calling hbrtRiContinue is unnecessary if the model has only one segment.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtIsOneSegmentModel(bool *is_one_segment_model, hbrt_model_handle_t model_handle);

// input
/**
 * @brief Get the input feature handles of a given model.
 * @param [out] feature_handle input feature handles
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetInputFeatureHandles(const hbrt_feature_handle_t **feature_handle,
                                                           hbrt_model_handle_t model_handle);

/**
 * @brief Get the input feature number of a given model.
 * @param [out] input_number input feature number
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetInputFeatureNumber(uint32_t *input_number, hbrt_model_handle_t model_handle);

// output
/**
 * @brief Get the output feature handles of a given model.
 * @param [out] feature_handle output feature handles
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetOutputFeatureHandles(const hbrt_feature_handle_t **feature_handle,
                                                            hbrt_model_handle_t model_handle);

/**
 * @brief Get the output feature number of a given model.
 * @param [out] output_number output feature number
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetOutputFeatureNumber(uint32_t *output_number, hbrt_model_handle_t model_handle);

/**
 * @brief Get the total byte size of all output features of a given model.
 * @param [out] size output feature size
 * @param [in] model_handle handle of a model
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetOutputFeatureTotalSize(uint32_t *size, hbrt_model_handle_t model_handle);

/*
 * APIs for feature
 */
/**
 * @brief Get the name of a given feature
 * @param [out] name name of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureName(const char **name, hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the definer's operator type of a given feature
 * @param [out] type operator type of the definer of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 * @note For operators like RCNNPostProcess, DetectionPostProcess, the outputs are organized in corresponding format.
 * See ::bernoulli_hw_detection_post_process_bbox_type_t, ::cpu_op_rcnn_post_process_bbox_float_type_t for more details.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureOperatorType(hbrt_output_operator_type_t *type,
                                                           hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the description of a given feature
 * @param [out] desc description of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureDescription(const char **desc, hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the aligned dimensions of a given feature
 * @param [out] dim aligned dimensions of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 * @note To align to the bit-width of BPU, a feature may need to pad at some dimension(s). The memory byte size occupied
 * by this feature should by calculated by aligned dimensions.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureAlignedDimension(hbrt_dimension_t *dim,
                                                               hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the valid dimensions of a given feature
 * @param [out] dim valid dimensions of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureValidDimension(hbrt_dimension_t *dim,
                                                             hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the resizer roi alignment of a given feature
 * @param [out] height_alignment recommended height alignment for the resizer ROI (bottom - top)
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetResizerRoiHeightAlignment(uint32_t *height_alignment,
                                                                 hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the resizer roi alignment of a given feature
 * @param [out] h_alignment recommended width alignment for the resizer ROI (right - left)
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetResizerRoiWidthAlignment(uint32_t *width_alignment,
                                                                hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the aligned byte size of a given feature
 * @param [out] size byte size of the aligned feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureAlignedTotalByteSize(uint32_t *size,
                                                                   hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the valid byte size of a given feature
 * @param [out] size byte size of the valid feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureValidTotalByteSize(uint32_t *size, hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the shift value(s) of a given feature
 * @param [out] shift shift of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureShiftValues(const uint8_t **shift, hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the length of shift value(s) of a given feature
 * @param [out] num length of shift value(s)
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureShiftValueNumber(uint32_t *num, hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the element type of a given feature
 * @param [out] element_type element type of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 * @note DetectionPostProcess and RCNNPostProcess have structural output, users should parse the output with
 * corresponding structure instead of element type got from this API. See
 * ::bernoulli_hw_detection_post_process_bbox_type_t,
 * ::cpu_op_rcnn_post_process_bbox_float_type_t for more details.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureElementType(hbrt_element_type_t *element_type,
                                                          hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the data layout of a given feature
 * @param [out] layout data layout of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetFeatureLayoutType(hbrt_layout_type_t *layout,
                                                         hbrt_feature_handle_t feature_handle);

/**
 * @brief Know if the feature is in big endian
 * @param [out] isBigEndian True means this feature is stored in big endian
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtFeatureIsBigEndian(bool *isBigEndian, hbrt_feature_handle_t feature_handle);

/**
 * Feature input source.
 * An input feature may come from DDR, resizer or pyramid.
 */
typedef enum {
  INPUT_FROM_DDR = 0,
  INPUT_FROM_RESIZER,
  INPUT_FROM_PYRAMID,
  INPUT_SOURCE_NUMBER,
} hbrt_input_source_t;

/**
 * @brief Get input source of a given feature.
 * @param [out] input_source input source of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetInputFeatureSource(hbrt_input_source_t *input_source,
                                                          hbrt_feature_handle_t feature_handle);

/**
 * @brief Get the name of an input source enum
 * @param [out] name name of the input source
 * @param [in] source enum type of input source
 * @return ::hbrtSuccess, ::hbrtErrorIllegalInputSourceType
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetInputSourceName(const char **name, hbrt_input_source_t source);

/**
 * @brief Get input pyramid stride of a given feature.
 * @param [out] stride pyramid stride of the feature
 * @param [in] feature_handle handle of a feature
 * @return ::hbrtSuccess, ::hbrtInvalidModelHandle, ::hbrtInvalidFeatureHandle
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetInputPyramidStride(uint32_t *stride, hbrt_feature_handle_t feature_handle);

typedef enum {
  EMPTY_REGION = 0,     ///< Unallocated region
  INPUT_REGIONS,        ///< Each input feature has its own region
  OUTPUT_REGIONS,       ///< Each output feature has its own region
  INTERMEDIATE_REGION,  ///< Transfer data between segments, can be overwritten after the entire model completes
  HEAP_REGION,          ///< Transfer data within a segment, can be overwritten after a segment completes
  RAM_REGION,           ///< A faster region, usually used for transient data in heap, can be overwritten after segment
  PARAM_REGION,         ///< Holding weight and bias
  ZERO_REGION,          ///< Absolute address
  REGION_NUMBER,
} hbrt_region_type_t;

/*
 * API to run a model
 */

/**
 * This struct represents the input information for the running instance.
 * Refer to the links below for more information about YUV.
 *
 *   - http://notes.maxwi.com/2017/12/05/yuv/
 *   - https://en.wikipedia.org/wiki/YUV
 */
typedef struct {
  hbrt_input_source_t input_source;  ///< input can come from DDR, pyramid or resizer

  hbrt_feature_handle_t feature_handle;  ///< an input data should be bound with an input feature

  bpu_addr_t feature_ptr;  ///< feature data address, should be a bpu address returned by bpu_mem_alloc. This field is
                           ///< valid for DDR input only

  bpu_addr_t y_ptr;   ///< point to input containing Y, null otherwise
  bpu_addr_t uv_ptr;  ///< point to input containing UV, null otherwise

  uint32_t resizer_img_height;       /// base image height for resizer
  uint32_t resizer_img_width;        /// base image width for resizer
  hbrt_pad_mode_t resizer_pad_mode;  /// pad method for resizer
  uint32_t img_stride;               /// image stride for resizer/pyramid input
  hbrt_roi_t roi;                    /// roi for resizer or pyramid input
} hbrt_ri_input_info_t;

#define HBRT_RI_INPUT_INFO_INITIALIZER \
  { INPUT_SOURCE_NUMBER, {0}, 0, 0, 0, 0, 0, HBRT_PAD_ZERO, 0, HBRT_ROI_INITIALIZER }

/**
 * This struct represents the memory context where RI is running.
 */
typedef struct {
  uint32_t core_mask;  ///< bit mask representing on which cores the model will run. e.g. 1(0x01) means the model runs
  ///< on core 0 while 3(0x11) means the model runs on both core0 and core1

  bpu_addr_t combined_output_region_ptr;  ///< if nullptr, separate output regions must have values
  uint32_t combined_output_region_size;

  struct {
    bpu_addr_t one_output_ptr;
    uint32_t one_output_memory_size;
    hbrt_feature_handle_t one_output_handle;
  } separate_output_regions[MAX_OUTPUT_NUMBER];

} hbrt_ri_config_t;

#define HBRT_RI_CONFIG_INITIALIZER \
  {                                \
    0, 0, 0, {                     \
      {                            \
        0, 0, { 0 }                \
      }                            \
    }                              \
  }

/**
 * Create a running instance (RI) with the specified ri_id, generate funccalls to the specified buffer.
 * @param [out] p_funccall_buffer          generated funccall(s) will be written to this address
 * @param [out] generated_funccall_number  number of generated funccall(s)
 * @param [in]  model_handle               handle of the model to run
 * @param [in]  input_infos                input infos of the model
 * @param [in]  ri_config                  configuration of this ri
 * @param [in]  ri_id                      the id to assign to this running instance
 * @param [in]  interrupt_number  raise the specified interrupt after the last generated funccall
 * @return ::hbrtSuccess, ::hbrtErrorRiIsInUse, ::hbrtErrorInvalidModelHandle, ::hbrtErrorIllegalCoreMask,
 * ::hbrtErrorIllegalOutputRegion, ::hbrtErrorIllegalRIConfig, ::hbrtErrorInvalidResizerParam,
 * ::hbrtErrorInvalidSegmentIndex, ::hbrtErrorMemoryAllocationFail, ::hbrtErrorInvalidInterruptNum,
 * ::hbrtErrorIllegalMARCH, ::hbrtErrorInvalidInputIndex, ::hbrtErrorFunccallSlotNotEnough
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiStart(void **p_funccall_buffer, uint32_t *generated_funccall_num,
                                            hbrt_model_handle_t model_handle, const hbrt_ri_input_info_t *input_infos,
                                            const hbrt_ri_config_t *ri_config, uint32_t ri_id,
                                            uint32_t interrupt_number);

#define MAX_BATCH_SIZE 4096

/**
 * Similar to hbrtRiStart, but can handle batch input.
 * @param [out] p_funccall_buffer          generated funccall(s) will be written to this address
 * @param [out] generated_funccall_number  number of generated funccall(s)
 * @param [in]  model_handle               handle of the model to run
 * @param [in]  model_input_infos          a secondary pointer to the input infos of one batch data. The first dimension
 * traverses model input number and the second dimension traverses the batch size of this input
 * @param [in]  model_input_batch_size     a pointer to the batch size array. All model inputs should have batch size of
 * 1 or equal to a same number.
 * @param [in]  ri_config                  configuration of this ri
 * @param [in]  ri_id                      the id to assign to this running instance
 * @param [in]  interrupt_number  raise the specified interrupt after the last generated funccall
 * @return ::hbrtSuccess, ::hbrtErrorRiIsInUse, ::hbrtErrorInvalidModelHandle, ::hbrtErrorIllegalCoreMask,
 * ::hbrtErrorIllegalOutputRegion, ::hbrtErrorIllegalRIConfig, ::hbrtErrorInvalidResizerParam,
 * ::hbrtErrorInvalidSegmentIndex, ::hbrtErrorMemoryAllocationFail, ::hbrtErrorInvalidInterruptNum,
 * ::hbrtErrorIllegalMARCH, ::hbrtErrorInvalidInputIndex, ::hbrtErrorFunccallSlotNotEnough
 * @note For multi-input models, you can broadcast some inputs by setting the corresponding input batch size as 1. Only
 * cpu-operator-free models can be started with this API.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiBatchStart(void **p_funccall_buffer, uint32_t *generated_funccall_num,
                                                 hbrt_model_handle_t model_handle,
                                                 const hbrt_ri_input_info_t **model_input_infos,
                                                 const uint32_t *model_input_batch_size,
                                                 const hbrt_ri_config_t *ri_config, uint32_t ri_id,
                                                 uint32_t interrupt_number);

/**
 * Continue the execution of the RI specified by ri_id, after the previous funccalls finished (e.g., call this in
 * interrupt handler).
 * @param [out] funccall_buffer          generated funccall(s) will be written to this address
 * @param [out] generated_funccall_number  number of generated funccall(s)
 * @param [in]  ri_id                     id of the ri
 * @param [in]  interrupt_number  raise the specified interrupt after the last generated funccall
 * @return ::hbrtSuccess, ::hbrtErrorRiIsInUse, ::hbrtErrorInvalidModelHandle, ::hbrtErrorIllegalCoreMask,
 * ::hbrtErrorIllegalOutputRegion, ::hbrtErrorIllegalRIConfig, ::hbrtErrorInvalidResizerParam,
 * ::hbrtErrorInvalidSegmentIndex, ::hbrtErrorMemoryAllocationFail, ::hbrtErrorInvalidInterruptNum,
 * ::hbrtErrorIllegalMARCH, ::hbrtErrorInvalidInputIndex, ::hbrtErrorIllegalLayout, ::hbrtErrorInvalidRoi,
 * ::hbrtErrorFunccallSlotNotEnough, ::hbrtErrorIllegalCPUOperator
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiContinue(void **funccall_buffer, uint32_t *generated_funccall_num, uint32_t ri_id,
                                               uint32_t interrupt_number);

/**
 * Destroy the RI specified by ri_id. This should be called after the entire RI completes
 * (i.e., previous HbrtRiContinue returned 0).
 * @param [in]  ri_id                     id of the ri
 * @return ::hbrtSuccess, ::hbrtErrorRiIsNotInUse, ::hbrtErrorIllegalRIID
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiDestroy(uint32_t ri_id);

/**
 * Determine if the model bound with a given ri is fully executed
 * @param [out] is_done true means the model is fully executed
 * @param [in] ri_id index of the ri
 * @return ::hbrtSuccess, ::hbrtErrorRiIsNotInUse, ::hbrtErrorIllegalRIID
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiIsDone(bool *is_done, uint32_t ri_id);

/**
 * Determine if the next call of hbrtRiContinue for a given ri involves cpu computation
 * @param [out] involve_cpu true means the next call of hbrtRiContinue for a given ri involves cpu computation
 * @param [in] ri_id index of the ri
 * @return ::hbrtSuccess, ::hbrtErrorRiIsNotInUse, ::hbrtErrorIllegalRIID
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiNextSegmentInvolveCpu(bool *involve_cpu, uint32_t ri_id);

/**
 * Use just-in-time generated instructions to crop and resize a ROI of image to given size
 * @param [out] funccall_buffer generated function call to do the crop and resize.
 * @param [out] generated_funccall_num  generated function call number.
 * @param [in] image_y_addr address of the Y channel of image.
 * @param [in] image_uv_addr address of the UV channel of image.
 * @param [in] image_height height of the image.
 * @param [in] image_width width of the image.
 * @param [in] image_h_stride byte size of one line of image
 * @param [in] uv_enable whether to crop and resize uv channel
 * @param [in] pad_mode method to pad for box exceeding image boundary
 * @param [in] p_rois pointers to ROIs.
 * @param [in] roi_num number of ROIs.
 * @param [in] dest_w the ROI will be resized to this width.
 * @param [in] dest_h the ROI will be resized to this height.
 * @param [in] output_layout layout of output.
 * @param [in] output_bpu_addr address of output.
 * @param [in] ri_id the id of running instance to execute generated function call.
 * @param [in] march see note.
 * @param [in] interrupt_num the interrupt number to report when function call execution finished.
 * @return ::hbrtSuccess, ::hbrtErrorMemoryAllocationFail, ::hbrtErrorInvalidRoi,
 * ::hbrtErrorIllegalMARCH.
 * @note
 *       1. the input image should be in YUV420NV12 format and the output image will be in YUV444 format.
 *       2. this api will generate BPU instructions and function call(s) for platform specified by march.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtBilinearRoiResizeImage(
    void **funccall_buffer, uint32_t *generated_funccall_num, bpu_addr_t image_y_addr, bpu_addr_t image_uv_addr,
    uint32_t image_height, uint32_t image_width, uint32_t image_h_stride, bool uv_enable, hbrt_pad_mode_t pad_mode,
    const hbrt_roi_t *p_rois, uint32_t roi_num, uint32_t dest_w, uint32_t dest_h, hbrt_layout_type_t output_layout,
    bpu_addr_t output_bpu_addr, uint32_t ri_id, MARCH march, uint32_t interrupt_num);

/**
 * Same as hbrtBilinearRoiResizeImage, but keep the aspect ratio of the original ROI
 * @param [out] funccall_buffer generated function call to do the crop and resize.
 * @param [out] generated_funccall_num  generated function call number.
 * @param [in] image_y_addr address of the Y channel of image.
 * @param [in] image_uv_addr address of the UV channel of image.
 * @param [in] image_height height of the image.
 * @param [in] image_width width of the image.
 * @param [in] image_h_stride byte size of one line of image
 * @param [in] uv_enable whether to crop and resize uv channel
 * @param [in] pad_mode method to pad for box exceeding image boundary
 * @param [in] p_rois pointers to ROIs.
 * @param [in] roi_num number of ROIs.
 * @param [in] dest_w the ROI will be resized to this width.
 * @param [in] dest_h the ROI will be resized to this height.
 * @param [in] output_layout layout of output.
 * @param [in] output_bpu_addr address of output.
 * @param [in] ri_id the id of running instance to execute generated function call.
 * @param [in] march see note.
 * @param [in] interrupt_num the interrupt number to report when function call execution finished.
 * @return ::hbrtSuccess, ::hbrtErrorMemoryAllocationFail, ::hbrtErrorInvalidRoi,
 * ::hbrtErrorIllegalMARCH.
 * @note
 *       1. the input image should be in YUV420NV12 format and the output image will be in YUV444 format.
 *       2. this api will generate BPU instructions and function call(s) for platform specified by march.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtBilinearRoiResizeImageKeepRatio(
    void **funccall_buffer, uint32_t *generated_funccall_num, bpu_addr_t image_y_addr, bpu_addr_t image_uv_addr,
    uint32_t image_height, uint32_t image_width, uint32_t image_h_stride, bool uv_enable, hbrt_pad_mode_t pad_mode,
    const hbrt_roi_t *p_rois, uint32_t roi_num, uint32_t dest_w, uint32_t dest_h, hbrt_layout_type_t output_layout,
    bpu_addr_t output_bpu_addr, uint32_t ri_id, MARCH march, uint32_t interrupt_num);

/**
 * Get model handle by ri id.
 * @param [out] model_handle model handle bound with the ri.
 * @param [in] ri_id id of the ri.
 * @return ::hbrtSuccess, ::hbrtErrorIllegalRIID, ::hbrtErrorRiIsNotInUse
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiGetModelHandle(hbrt_model_handle_t *model_handle, uint32_t ri_id);

/**
 * Get configuration of the ri.
 * @param [out] ri_config configuration of the ri
 * @param [in] ri_id id of the ri
 * @return ::hbrtSuccess, ::hbrtErrorIllegalRIID, ::hbrtErrorRiIsNotInUse
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiGetConfig(const hbrt_ri_config_t **ri_config, uint32_t ri_id);

/**
 * Get input infos by ri id.
 * @param [out] input_info input infos of the model bound with the ri.
 * @param [in] ri_id id of the ri.
 * @return ::hbrtSuccess, ::hbrtErrorIllegalRIID, ::hbrtErrorRiIsNotInUse
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiGetInputInfo(const hbrt_ri_input_info_t **input_info, uint32_t ri_id);

/**
 * An output feature may have been generated before the entire RI completes. (e.g., a large model with a few
 * outputs) Call this function to check which output feature is ready for use. The status is a bitmap. (output 0 is
 * ready if bit 0 is set, output 1 is ready if bit 1 is set, etc.)
 * @param [out] status bitmap of output status.
 * @param [in] ri_id id of the ri.
 * @return ::hbrtSuccess, ::hbrtErrorIllegalRIID, ::hbrtErrorRiIsNotInUse
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiGetOutputStatus(uint64_t *status, uint32_t ri_id);

/**
 * Return the CPU address of the specified feature.
 * if "WORK_CPUMALLOC_RAW", the returned pointer should be freed by bpu_cpumem_free
 * for other work, the returned pointer should be freed by <free>
 */
typedef enum {
  WORK_CPUMALLOC_RAW = 0,  /// return address allocated by bpu_cpumem_alloc.
  WORK_MALLOC_NATIVE,  /// return address allocated by malloc in stdlib. The data shall be converted to native layout in
                       /// order NHWC
  WORK_MALLOC_NATIVE_NOPADDING,  /// return address allocated by malloc in stdlib. The data shall be in native layout
                                 /// (NHWC) without padding
  WORK_MALLOC_NATIVE_NOPADDING_INT32,  /// return address allocated by malloc in stdlib. The data shall be in native
                                       /// layout (NHWC) without padding and each element shall be converted to int32
  WORK_MALLOC_NATIVE_NOPADDING_FLOAT,  /// return address allocated by malloc in stdlib. The data shall be in native
                                       /// layout (NHWC) without padding and each element shall be converted to float
  WORK_BPU_RAW,                        /// return BPU address. No data transfer shall happen between BPU and CPU.
} DATA_WORK;

/**
 * Get BPU address of a feature at a specified ri
 * @param [out] addr BPU address of the feature
 * @param [in] ri_id id of the ri
 * @param [in] fh handle of the feature
 * @return ::hbrtSuccess, ::hbrtErrorInvalidFeatureHandle, ::hbrtErrorIllegalRIID, ::hbrtErrorRiIsNotInUse,
 *  ::hbrtErrorIllegalMemoryRead
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiGetFeatureBpuAddress(bpu_addr_t *addr, uint32_t ri_id, hbrt_feature_handle_t fh);

/**
 * Get the data of a feature at a specified ri
 * @param [out] data data of the feature
 * @param [in] ri_id id of the ri
 * @param [in] fh  handle of the feature
 * @param [in] work the way to process the data. see ::DATA_WORK
 * @return ::hbrtSuccess, ::hbrtErrorInvalidFeatureHandle, ::hbrtErrorIllegalRIID, ::hbrtErrorRiIsNotInUse,
 * ::hbrtErrorBPUCPUMemAllocFail, ::hbrtErrorMemoryOverflow, ::hbrtErrorInvalidDataWork
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiGetFeatureData(void **data, uint32_t ri_id, hbrt_feature_handle_t fh,
                                                     DATA_WORK work);

/**
 * Similar to HbrtRiGetFeatureData, but give an output index instead of feature handle
 * @param [out] data data of the feature
 * @param [in] ri_id id of the ri
 * @param [in] output_index index of the output
 * @param [in] work the way to process the data. see ::DATA_WORK
 * @return ::hbrtSuccess, ::hbrtErrorInvalidFeatureHandle, ::hbrtErrorIllegalRIID, ::hbrtErrorRiIsNotInUse,
 * ::hbrtErrorBPUCPUMemAllocFail, ::hbrtErrorMemoryOverflow, ::hbrtErrorInvalidDataWork
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRiGetOutputData(void **data, uint32_t ri_id, uint32_t output_index, DATA_WORK work);

/**
 * Inverse-quantize data
 * @param [out] to_float_data inverse-quantized float data will be written to this address
 * @param [in] from_int_element_type the source data's element type
 * @param [in] dim the dimensions of source data. should be 4-element uint32 array.
 * @param [in] shifts shift value of the data.
 * @param [in] from_int_data address of the source data.
 * @return ::hbrtSuccess, ::hbrtErrorInvalidFeatureHandle, ::hbrtErrorInvalidAddress, ::hbrtErrorIllegalElementType
 */
HBDK_PUBLIC extern hbrt_error_t hbrtUnquantize(float *to_float_data, hbrt_element_type_t from_int_element_type,
                                               hbrt_dimension_t dim, const uint8_t *shifts, const void *from_int_data);

/**
 * Quantize data
 * @param [out] to_int_data quantized int data will be written to this address
 * @param [in] to_int_element_type target element type
 * @param [in] dim the dimensions of source data. should be 4-element uint32 array.
 * @param [in] shifts shift value of the data.
 * @param [in] from_float_data address of the source data.
 * @return ::hbrtSuccess, ::hbrtErrorInvalidFeatureHandle, ::hbrtErrorInvalidAddress, ::hbrtErrorIllegalElementType
 */
HBDK_PUBLIC extern hbrt_error_t hbrtQuantize(void *to_int_data, hbrt_element_type_t to_int_element_type,
                                             hbrt_dimension_t dim, const uint8_t *shifts, const float *from_float_data);

/**  APIs for debug  **/

/**
 * @brief Set runtime log level by read the environment variable 'HBRT_LOG_LEVEL'.
 * @return ::hbrtSuccess
 * @note The higher the log level is, the more detailed the log will be.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtSetLogLevel();

/**
 * Print the function call to console for debug purpose.
 * @param [in] funccall the address of a buffer containing function call.
 * @return ::hbrtSuccess
 * @note the buffer can contain multiple function calls and each function call shall be printed in order.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtPrintFunccall(const void *funccall);

/**
 * Dump model output by a standardised format
 * @param path path to store dumped files
 * @param filename_prefix prefix of dumped files
 * @param ri_id  id of the ri to dump
 * @return ::hbrtSuccess
 */
HBDK_PUBLIC extern hbrt_error_t hbrtDumpModelOutputToFile(const char *path, const char *filename_prefix,
                                                          uint32_t ri_id);

/**
 * The the version of the running hbrt
 * @param [in] version The current hbrt version
 * @return ::hbrtSuccess
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetVersion(hbrt_version_info_t *version);

/**
 * Get the hbrt version of hbm
 * @param [out] version The hbrt version of hbm
 * @param [in] hbm_handle The hbm handle
 * @return ::hbrtSuccess
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetHbmHbrtVersion(hbrt_version_info_t *version, hbrt_hbm_handle_t hbm_handle);

/**
 * Get the HBM tag
 * @param [out] tag The HBM tag
 * @param [in] hbm_handle The hbm handle
 * @return ::hbrtSuccess
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetHbmTag(const char **tag, hbrt_hbm_handle_t hbm_handle);

/**
 * Get the hbrt version of model
 * @param [out] version The hbrt version of hbm
 * @param [in] model_handle The modle handle
 * @return ::hbrtSuccess
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetModelHbrtVersion(hbrt_version_info_t *version, hbrt_model_handle_t model_handle);

/**
 * Check if two versions are compatible. Only versions with same major and minor number are compatible
 * @param v1 version1
 * @param v2 version2
 * @return ::hbrtSuccess, ::hbrtIncompatibleVersoin
 */
HBDK_PUBLIC extern hbrt_error_t hbrtIsCompatibleVersion(hbrt_version_info_t v1, hbrt_version_info_t v2);
/*
 * Check if the hbrt header is compatible to the hbrt library.
 * @return ::hbrtSuccess, ::hbrtErrorIncompatibleVersion
 */
static inline hbrt_error_t hbrtIsCompatibleHeader() {
  hbrt_version_info_t ver;
  hbrt_error_t err = hbrtGetVersion(&ver);
  if (err) {
    return err;
  }
  if (ver.major != HBRT_VERSION_MAJOR || ver.minor != HBRT_VERSION_MINOR) {
    return hbrtErrorIncompatibleVersion;
  }
  return hbrtSuccess;
}

#ifdef __cplusplus
}
#endif
