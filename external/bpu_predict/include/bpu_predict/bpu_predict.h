/*
 *  Copyright (c) 2019 by Horizon
 * \file bpu_predict.h
 * \brief BPU predict API for Horizon BPU Platform.
 */

#ifndef BPU_PREDICT_H_
#define BPU_PREDICT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define BPU_PREDICT_VER_MAJOR 3
#define BPU_PREDICT_VER_MINOR 2
#define BPU_PREDICT_VER_PATCH 0

typedef void* BPUHandle;

/*
 * \brief load libmodel.so from file system
 * \return 0 for everything is ok, -1 for encounter a problem.
 */
int BPU_loadModel(const char* model_file_name, BPUHandle *handle, const char *config_file_name = nullptr);

/*
 * \brief release all data structure that allocated by BPU service.
 *  and do not need bpu service any more.
 * \return 0 for everything is ok, -1 for encounter a problem.
 */
int BPU_release(BPUHandle handle);

/*
 * \brief get the version string of libmodel.so
 */
const char* BPU_getVersion(BPUHandle handle);

/*
 * \brief get error description string of the last encouter problem. if no problem has been happened, just return "OK".
 */
const char* BPU_getLastError(BPUHandle handle);

/*
 * \brief get a list of string that is the name of all available in libmodel.so
 *  the name will be need on calling for rumModel.
 * \return:
 *  name_list, pointer of char* array, the memory was allocated by lib, and do not write or free them
 *  name_list_cnt, number of name in name_list.
 */
int BPU_getModelNameList(BPUHandle handle, const char*** name_list, int *name_list_cnt);

enum BPUDtype {
  BPU_DTYPE_FLOAT32 = 0,
  BPU_DTYPE_INT8
};

struct BPUModelInfo {
  int num;  // num of data
  int *ndim_array;  // ndim array
  int *aligned_shape_array;  // shape array
  int *valid_shape_array;  // valid shape array
  int *dtype_array;  // dtype array
  int *size_array;  // the size array that needed by each output buffer
  int *output_operator_type;  // output operator type
  uint8_t **shift_value;  // shift value
  const char** name_list;  // name list of output data
};

/*
 * use example of ndim, shape and dtype
 * suppose num of data is 2, and input1's shape is (1, 224, 224, 3), input2's shape is (2,6) then:
 * num is 2
 * ndim_array is [0, 4, 6], the 0 is input1's shape start, 4 is input1's shape end and input2's shape start
 * shape_array is [1, 224, 224, 3, 2, 6], you can use ndim as shape index:
 * input1's shape is shape_array[ndim_array[0]] ~ shape_array[ndim_array[1]]
 * input2's shape is shape_array[ndim_array[1]] ~ shape_array[ndim_array[2]]
 *
 * dtype_array represent for each output data type:
 * out1: dtype_array[0]
 * out2: dtype_array[1]
 * ...
 *
 * so as to dtype_array
 *
 */

/*
 * \brief get input info of model with model_name.
 *  num is for number of input-data
 *  ndim_array is for input-data's shape ndim.
 *  shape_array is for input-data's shapes
 *  dtype_array is for input-data's dtype
 *  name_list is for input-data's name
 */
int BPU_getModelInputInfo(BPUHandle handle, const char* model_name, BPUModelInfo *info);

/*
 * \brief get output info of model with model_name.
 *  usually the output do not has name, so info->name_list is null
 */
int BPU_getModelOutputInfo(BPUHandle handle, const char* model_name, BPUModelInfo *info);

/*
 * BPU buffer operations
 */
typedef void* BPU_Buffer_Handle;

/*
 * get pointer of BPU Buffer data. the return pointer is void*, and can be cast to other data type
 */
void* BPU_getRawBufferPtr(BPU_Buffer_Handle buff);

/*
 * get buffer size of BPU Buffer
 */
int BPU_getRawBufferSize(BPU_Buffer_Handle buff);

/*
 * construct a BPU Buffer from exist buffer pointer
 */
BPU_Buffer_Handle BPU_createBPUBuffer(void* buff, int size);

/*
 * create an empty buffer, that can be used for getModuleOutput
 */
BPU_Buffer_Handle BPU_createEmptyBPUBuffer();

/*
 *
 */
int BPU_freeBPUBuffer(BPU_Buffer_Handle buff);

/*
 * describe data buffer that come from pyramid module.
 */
typedef void* BPUPyramidBuffer;

struct BPUFakeImage {
  BPU_Buffer_Handle data;
  int height;
  int width;
  int image_id;
  uint64_t timestamp;
};

struct BPUBBox {
  float x1;
  float y1;
  float x2;
  float y2;
  float score;
  int type;
  bool resizable; //used by runModelFromResizer
};

/*
 * BPU run model operations
 */
typedef void* BPUModelHandle;

/*
 * \brief wait to get BPU output data.
 *  when the model is running, this function will cause current thread to wait for running done.
 *  when the model has already done, this function will return immediately.
 */
int BPU_getModelOutput(BPUHandle handle, BPUModelHandle model_handle);

/*
 * \brief get model running error string.
 */
const char* BPU_getModelLastError(BPUHandle handle, BPUModelHandle model_handle);

/*
 * \brief release model handle and relative data structure.
 *  if model is running, then stop running and release data.
 */
int BPU_releaseModelHandle(BPUHandle handle, BPUModelHandle model_handle);

/*
 * \brief run model with input data that come from pyramid module
 */
int BPU_runModelFromPyramid(BPUHandle handle, const char* model_name,
                            BPUPyramidBuffer input,
                            int pyr_level,
                            BPU_Buffer_Handle output[],
                            int nOutput,
                            BPUModelHandle *model_handle,
                            BPU_Buffer_Handle *extra_input = nullptr,
                            int extra_input_size = 0,
                            int core_id = -1);

/*
 * \brief run model with input data that come from input image data
 */
int BPU_runModelFromImage(BPUHandle handle, const char* model_name,
                          BPUFakeImage *input,
                          BPU_Buffer_Handle output[],
                          int nOutput,
                          BPUModelHandle *model_handle,
                          BPU_Buffer_Handle *extra_input = nullptr,
                          int extra_input_size = 0,
                          int core_id = -1);

/*
 * \brief run model from ddr, which had been format into 8P8C before call this function.
 */
int BPU_runModelFromDDR(BPUHandle handle, const char* model_name,
                        BPU_Buffer_Handle input[], int nInput,
                        BPU_Buffer_Handle output[], int nOutput,
                        BPUModelHandle *model_handle,
                        int core_id = -1);

/*
 * \brief run model from ddr, which had been convert layout in this function.
 */
int BPU_runModelFromDDRWithConvertLayout(BPUHandle handle, const char* model_name,
                                         BPU_Buffer_Handle input[], int nInput,
                                         BPU_Buffer_Handle output[], int nOutput,
                                         BPUModelHandle *model_handle,
                                         int core_id = -1);
/*
 * \brief run model with input data and roi
 *  this function will use roi to select one of input image, crop and resize to proper size of model's input-data
 *  and then run model from the resized input-data
 *  ATTENTION: bbox may be change in this function! output is the noramlized bbox
 */
int BPU_runModelFromResizer(BPUHandle handle, const char* model_name,
                            BPUPyramidBuffer input,
                            BPUBBox *bbox,
                            int nBox,
                            int *resizable_cnt,
                            BPU_Buffer_Handle output[],
                            int nOutput,
                            BPUModelHandle *model_handle,
                            int core_id = -1);

/*
 * \brief run model from pyramid, this function can crop input data.
 */
int BPU_runModelCropPyramid(BPUHandle handle, const char *model_name,
                            BPUPyramidBuffer input,
                            int pyr_level,
                            int start_x,
                            int start_y,
                            BPU_Buffer_Handle output[],
                            int nOutput,
                            BPUModelHandle *model_handle,
                            BPU_Buffer_Handle *extra_input = nullptr,
                            int extra_input_size = 0,
                            int core_id = -1);

/*
 * \brief BPU group operations
 */

/*
 * \brief create a group on bpu
 * \return group id.
 */
int BPU_createGroup(BPUHandle handle);

/*
 * \brief set model to group
 * when you do not want model belong to some group, can set group id to zero.
 * group id is zero means the default group.
 */
int BPU_setModelGroup(BPUHandle handle, const char* model_name, int group_id);

/*
 * \brief set proportion of group. The proportion must in range [0, 100], and is calculated as proportion / 100
 */
int BPU_setGroupProportion(BPUHandle handle, int group_id, int proportion);

enum BPULayoutType {
  LAYOUT_1HW1 = 0,
  LAYOUT_111C,
  LAYOUT_1111,
  LAYOUT_NHWC
};
/*
 * \brief convert layout.
 */
int BPU_convertLayout(BPUHandle handle, void *to_data, const void *from_data,
                      const char *model_name, BPULayoutType layout_type,
                      uint32_t layer_index, uint32_t n_index,
                      uint32_t h_index, uint32_t w_index, uint32_t c_index);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // BPU_PREDICT_H_
