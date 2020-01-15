/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2016-2019 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/

#ifndef HBDK_HBDK_IR_HPP
#define HBDK_HBDK_IR_HPP

#pragma once

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace hbdk {
namespace hbir {

using Coord = std::vector<int32_t>;           // for tensor ROI coord
using Shape = std::vector<uint32_t>;          // for tensor dimension, ROI size
using Shape2D = std::pair<int32_t, int32_t>;  // for stride, padding, etc

template <typename T>
struct ElementType;

template <typename T>
void CheckVersion(const T *, std::uint32_t version) {
  if (version != cereal::detail::Version<T>::version) {
    std::cerr << "The class " << cereal::detail::binding_name<T>::name() << " version is "
              << cereal::detail::Version<T>::version << ", but the loaded version is " << version << std::endl;
    std::abort();
  }
}

template <typename T>
void AbortOnVersion(const T *, std::uint32_t version) {
  if (version != cereal::detail::Version<T>::version) {
    std::cerr << "The class " << cereal::detail::binding_name<T>::name() << " with version "
              << cereal::detail::Version<T>::version << " cannot deserialize incompatible version " << version
              << std::endl;
  } else {
    std::cerr << "The latest version has not been processed, serialize error in "
              << cereal::detail::binding_name<T>::name() << std::endl;
  }
  std::abort();
}

/**
 * Tensor used by Layer inputs and outputs
 */
struct Tensor {
 public:
  enum class element_type_t {
    UNKNOWN,
    S8,
    S16,
    S32,
    S64,
    U8,
    U16,
    U32,
    U64,
    F32,
    F64,
  };

  Tensor() = default;
  virtual ~Tensor() noexcept = default;
  /**
   * Construct a Tensor,
   * different type tensor may has different shape dimension, element type, number of shift value,
   * see Layer description
   *
   * @param name tensor name (globally unique)
   * @param shape tensor shape
   * @param element_type
   * @param shift tensor data shift values
   * @param is_model_output mark the tensor as model output
   */
  Tensor(std::string name, Shape shape, element_type_t element_type = element_type_t::S8,
         std::vector<int32_t> shift = {}, bool is_model_output = false)
      : name(std::move(name)),
        shape(std::move(shape)),
        shift(std::move(shift)),
        is_model_output(is_model_output),
        element_type(element_type) {}

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(CEREAL_NVP(name), CEREAL_NVP(shape), CEREAL_NVP(element_type), CEREAL_NVP(shift), CEREAL_NVP(is_model_output),
         CEREAL_NVP(data));
    } else {
      AbortOnVersion(this, version);
    }
  }

 public:
  std::string name;
  Shape shape;
  std::vector<int32_t> shift;
  bool is_model_output = false;

  // ------------------------------------------------
  // ---------- tensor data & element type ----------

  element_type_t GetElementType() const { return element_type; }
  void SetElementType(element_type_t type) { element_type = type; }

  template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value>::type>
  void SetData(const T *ptr, size_t size) {
    if (ElementType<T>::value != element_type) {
      std::cerr << "cannot set data with element type " << static_cast<uint32_t>(ElementType<T>::value) << " to tensor "
                << name << " with element type " << static_cast<uint32_t>(element_type) << std::endl;
    }
    data.assign(reinterpret_cast<const char *>(ptr), reinterpret_cast<const char *>(ptr) + size * sizeof(T));
  }

  template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value>::type>
  uint32_t GetDataSize() const {
    if (ElementType<T>::value != element_type) {
      std::cerr << "cannot get size of data with element type " << static_cast<uint32_t>(ElementType<T>::value)
                << " from tensor " << name << " with element type " << static_cast<uint32_t>(element_type) << std::endl;
    }
    return data.size() / sizeof(T);
  }

  template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value>::type>
  const T *GetData() const {
    if (ElementType<T>::value != element_type) {
      std::cerr << "cannot get data with element type " << static_cast<uint32_t>(ElementType<T>::value)
                << " from tensor " << name << " with element type " << static_cast<uint32_t>(element_type) << std::endl;
    }
    return reinterpret_cast<const T *>(data.data());
  }

  bool HasData() const { return data.size() > 0; }

 private:
  element_type_t element_type = element_type_t::UNKNOWN;
  std::vector<char> data;

  // ---------- tensor data & element type ----------
  // ------------------------------------------------
};

/**
 * Layer used to describe the model
 */
struct Layer {
 public:
  enum class layer_type_t {
    UNKNOWN,
    QUANTI_INPUT,
    CONVOLUTION,
    POOLING_AVG,
    POOLING_MAX,
    POOLING_AVG_GLOBAL,
    POOLING_MAX_GLOBAL,
    RELU,
    ELEMENTWISE_MUL,
    ELEMENTWISE_ADD,
    ELEMENTWISE_MULADD,  // deprecated.
    SPLIT,
    CONCAT,
    CHANNEL_MAX,
    SOFTMAX,
    WARPING,
    DETECTION_POST_PROCESS,
    RCNN_POST_PROCESS,
    ROI_ALIGN,
    ROI_RESIZE,
    LUT,
    CHANNEL_SUM,
    FILTER,
    SLICE,
    ELEMENTWISE_SUB,
    RESHAPE,
    QUANTI_FLATTEN,
    DECONVOLUTION,
    BPUCONVOLUTION
  };

  Layer() = default;
  virtual ~Layer() noexcept = default;

  /**
   * Layer base, cannot be constructed
   *
   * @param name unique layer name
   * @param input_tensors input tensors of the layer
   * @param output_tensors output tensors of the layer
   */
  Layer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
        std::vector<std::shared_ptr<Tensor>> output_tensors)
      : name(std::move(name)), input_tensors(std::move(input_tensors)), output_tensors(std::move(output_tensors)) {}
  virtual layer_type_t GetLayerType() const = 0;

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(CEREAL_NVP(name), CEREAL_NVP(input_tensors), CEREAL_NVP(output_tensors), CEREAL_NVP(desc));
    } else {
      AbortOnVersion(this, version);
    }
  }

  std::string name;
  std::vector<std::shared_ptr<Tensor>> input_tensors;
  std::vector<std::shared_ptr<Tensor>> output_tensors;
  std::string desc;  // user-defined information, e.g. how to use the output of this layer.
                     // the compiler does not parse it
};

/**
 * Quantized Input layer
 *
 * has 1 input (fake) and 1 output (int8).
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 *
 * NOTE: the input and output should has same shape.
 *       the compiler just use output as model input, the input will be ignored.
 *
 */
struct QuantiInputLayer : public Layer {
 public:
  using Layer::Layer;
  ~QuantiInputLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::QUANTI_INPUT; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

/**
 * Convolution Layer
 *
 * inputs [feature, weight, bias, sumin]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 * 2. weight shape is [N, H, W, C], has 1 or N shift values
 * 3. bias shape is [N], has 1 or N shift values, can be nullptr
 * 4. sumin shape is [N, H, W, C], has 1 or C shift values, can be nullptr
 *
 * output
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 *
 * output = feature * weight + bias + sumin
 */
struct ConvolutionLayer : public Layer {
 public:
  ConvolutionLayer() = default;  // for windows build
  ConvolutionLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                   std::vector<std::shared_ptr<Tensor>> output_tensors, Shape2D stride, Shape2D padding)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        stride(std::move(stride)),
        padding(std::move(padding)) {}
  ~ConvolutionLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::CONVOLUTION; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(stride), CEREAL_NVP(padding));
    } else {
      AbortOnVersion(this, version);
    }
  }

  Shape2D stride;   // [H, W]
  Shape2D padding;  // [H, W]
};

/**
 * Deconvolution Layer
 *
 * inputs [feature, weight, bias, sumin]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 * 2. weight shape is [N, H, W, C], has 1 or N shift values
 * 3. bias shape is [N], has 1 or N shift values, can be nullptr
 * 4. sumin is at present not supported
 *
 * output
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 *
 * output = feature * weight + bias + sumin
 */
struct DeconvolutionLayer : public Layer {
 public:
  DeconvolutionLayer() = default;
  DeconvolutionLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                     std::vector<std::shared_ptr<Tensor>> output_tensors, Shape2D stride, Shape2D padding,
                     Shape2D output_padding)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        stride(std::move(stride)),
        padding(std::move(padding)),
        output_padding(std::move(output_padding)) {}
  ~DeconvolutionLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::DECONVOLUTION; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(stride), CEREAL_NVP(padding), CEREAL_NVP(output_padding));
    } else {
      AbortOnVersion(this, version);
    }
  }
  Shape2D stride;
  Shape2D padding;
  Shape2D output_padding;
};

/**
 * Average Pooling Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct AvgPoolingLayer : public Layer {
 public:
  AvgPoolingLayer() = default;
  AvgPoolingLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                  std::vector<std::shared_ptr<Tensor>> output_tensors, Shape2D kernel, Shape2D stride, Shape2D padding)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        kernel(kernel),
        stride(stride),
        padding(padding) {}
  ~AvgPoolingLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::POOLING_AVG; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(kernel), CEREAL_NVP(stride), CEREAL_NVP(padding),
         CEREAL_NVP(result_right_shift));
    } else {
      AbortOnVersion(this, version);
    }
  }

  Shape2D kernel;                   // [H, W]
  Shape2D stride;                   // [H, W]
  Shape2D padding;                  // [H, W]
  uint32_t result_right_shift = 9;  // result sum will right shift this
};

/**
 * Max Pooling Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct MaxPoolingLayer : public Layer {
 public:
  MaxPoolingLayer() = default;
  MaxPoolingLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                  std::vector<std::shared_ptr<Tensor>> output_tensors, Shape2D kernel, Shape2D stride, Shape2D padding)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        kernel(kernel),
        stride(stride),
        padding(padding) {}
  ~MaxPoolingLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::POOLING_MAX; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(kernel), CEREAL_NVP(stride), CEREAL_NVP(padding),
         CEREAL_NVP(result_right_shift));
    } else {
      AbortOnVersion(this, version);
    }
  }

  Shape2D kernel;
  Shape2D stride;
  Shape2D padding;
  uint32_t result_right_shift = 9;  // result sum will right shift this
};

/**
 * Global Average Pooling Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct GlobalAvgPoolingLayer : public Layer {
 public:
  using Layer::Layer;
  ~GlobalAvgPoolingLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::POOLING_AVG_GLOBAL; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 2) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(result_right_shift), CEREAL_NVP(sum_right_shift));
    } else {
      AbortOnVersion(this, version);
    }
  }

  uint32_t result_right_shift = 9;  // result sum will right shift this
  uint32_t sum_right_shift = 0;     // partial sum will right shift this
};

/**
 * Global Max Pooling Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct GlobalMaxPoolingLayer : public Layer {
 public:
  using Layer::Layer;
  ~GlobalMaxPoolingLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::POOLING_MAX_GLOBAL; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

///**
// * Upscaling Layer
// *
// * input [feature]
// * 1. feature shape is [N, H, W, C], has 1 or C shift values
// *
// * output [output]
// * 1. output shape is [N, H, W, C], has 1 or C shift values
// */
// struct UpscalingLayer : public Layer {
//  using Layer::Layer;
//  ~UpscalingLayer() noexcept override = default;
//  layer_type_t GetLayerType() const override { return layer_type_t::UPSCALING; }
//
//  template <class Archive>
//  void serialize(Archive &ar, std::int32_t const version) {
//    CheckVersion(this, version);
//    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
//    ar(cereal::base_class<Layer>(this));
//  }
//};

/**
 * RoiResize Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 */
struct RoiResizeLayer : public Layer {
  using Layer::Layer;
  struct Roi;
  enum class PadMode;
  enum class AlignMode;
  RoiResizeLayer() = default;
  RoiResizeLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                 std::vector<std::shared_ptr<Tensor>> output_tensors, RoiResizeLayer::Roi roi,
                 RoiResizeLayer::PadMode pad_mode, AlignMode align_mode)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        roi_(roi),
        pad_mode_(pad_mode),
        align_mode_(align_mode) {
    if (roi_.left == 0 && roi_.right == 0 && roi_.top == 0 && roi_.bottom == 0) {
      roi_.right = this->input_tensors.at(0)->shape[2];
      roi_.bottom = this->input_tensors.at(0)->shape[1];
    }
  }
  ~RoiResizeLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::ROI_RESIZE; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 2) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(roi_));
    } else if (version == 3) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(roi_), CEREAL_NVP(pad_mode_));
    } else if (version == 4) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(roi_.left), CEREAL_NVP(roi_.top), CEREAL_NVP(roi_.right),
         CEREAL_NVP(roi_.bottom), CEREAL_NVP(pad_mode_), CEREAL_NVP(align_mode_));
    } else {
      AbortOnVersion(this, version);
    }
  }
  struct Roi {
    int32_t left = 0;
    int32_t top = 0;
    int32_t right = 0;   // exclusive
    int32_t bottom = 0;  // exclusive
    template <class Archive>
    void serialize(Archive &ar) {
      // DO NOT CHANGE THIS CODE, IT IS USED BY VERSION 2,3
      ar(CEREAL_NVP(left), CEREAL_NVP(top), CEREAL_NVP(right), CEREAL_NVP(bottom));
    }
  } roi_;

  enum class PadMode { ZERO, BOUNDARY } pad_mode_ = PadMode::ZERO;
  enum class AlignMode { ORIGIN, CENTER } align_mode_ = AlignMode::ORIGIN;
};

/**
 * Relu Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct ReluLayer : public Layer {
  using Layer::Layer;
  ~ReluLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::RELU; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

/**
 * Elementwise Multiply Layer
 *
 * input [feature1, feature2]
 * 1. feature1 shape is [N, H, W, C], has 1 or C shift values
 * 1. feature2 shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct ElementwiseMul : public Layer {
  using Layer::Layer;
  ~ElementwiseMul() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::ELEMENTWISE_MUL; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

/**
 * Elementwise Add Layer
 *
 * input [feature1, feature2]
 * 1. feature1 shape is [N, H, W, C], has 1 or C shift values
 * 1. feature2 shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct ElementwiseAdd : public Layer {
  using Layer::Layer;
  ~ElementwiseAdd() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::ELEMENTWISE_ADD; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

/* ===============================================
 * NOTE:
 * This is no ElementwiseMulAdd layer.
 * To simulate an ElementwiseMulAdd layer,
 * you need to create two layers. One ElementwiseMul and One ElementwiseAdd.
 * The output of ElementwiseMul should be used by ElementwiseAdd, and not the model output.
 * The shift of ElementwiseMul output should be the sum of its inputs shifts.
 * The element size of ElementwiseMul output should be >= the sum of its input element size.
 * (Otherwise overflow could occur and we cannot fuse them).
 * ===============================================
 */

/**
 * Elementwise Subtract Layer
 *
 * input [feature1, feature2]
 * 1. feature1 shape is [N, H, W, C], has 1 or C shift values
 * 1. feature2 shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct ElementwiseSub : public Layer {
  using Layer::Layer;
  ~ElementwiseSub() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::ELEMENTWISE_SUB; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

/**
 * Split Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output1, output2, ...]
 * 1~n. output shape is [N, H, W, C], has 1 or C shift values
 *
 * outputs can be split in any number instead of split equally
 */
struct SplitLayer : public Layer {
  using Layer::Layer;
  ~SplitLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::SPLIT; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

/**
 * Concat Layer
 *
 * input [feature1, feature2, ...]
 * 1~n. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, C], has 1 or C shift values
 */
struct ConcatLayer : public Layer {
  using Layer::Layer;
  ~ConcatLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::CONCAT; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

/**
 * Channel Max Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N,H,W,1 or 2], has 1 or 2 shift values
 *
 * NOTE: when keep_score is true: channel max output 2 channels, the first is the index in feature channel, the second
 * is the max value, when keep_score is false: channel max output 1 channel, keep index
 */
struct ChannelMaxLayer : public Layer {
  ChannelMaxLayer() = default;
  ChannelMaxLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                  std::vector<std::shared_ptr<Tensor>> output_tensors, bool keep_score, bool run_length_encoding,
                  int32_t class_offset, int32_t group_number)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        keep_score(keep_score),
        run_length_encoding(run_length_encoding),
        class_offset(class_offset),
        group_number(group_number) {}
  ~ChannelMaxLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::CHANNEL_MAX; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 2) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(keep_score), CEREAL_NVP(run_length_encoding),
         CEREAL_NVP(class_offset), CEREAL_NVP(group_number));
    } else {
      AbortOnVersion(this, version);
    }
  }

  bool keep_score;
  bool run_length_encoding;
  int32_t class_offset;
  int32_t group_number;
};

/**
 * Channel Sum Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values
 *
 * output [output]
 * 1. output shape is [N, H, W, 1], has 1 shift value
 */
struct ChannelSumLayer : public Layer {
  using Layer::Layer;
  ~ChannelSumLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::CHANNEL_SUM; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

/**
 * Filter Layer
 *
 * input [?]
 *
 * output [?]
 */
struct FilterLayer : public Layer {
  FilterLayer() = default;
  FilterLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
              std::vector<std::shared_ptr<Tensor>> output_tensors, int32_t anchor_num, int32_t thresh,
              uint32_t max_box_n, uint32_t start_c, uint32_t end_c)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        anchor_dim(anchor_num),
        threshold(thresh),
        max_box_num(max_box_n),
        start_channel(start_c),
        end_channel(end_c) {}
  ~FilterLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::FILTER; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    if (version == 1) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(anchor_dim), CEREAL_NVP(threshold), CEREAL_NVP(max_box_num),
         CEREAL_NVP(start_channel), CEREAL_NVP(end_channel));
    } else {
      AbortOnVersion(this, version);
    }
  }

  int32_t anchor_dim;
  int32_t threshold;
  uint32_t max_box_num;
  uint32_t start_channel;
  uint32_t end_channel;
};

/**
 * Softmax Layer
 *
 * input [?]
 *
 * output [?]
 */
struct SoftmaxLayer : public Layer {
  using Layer::Layer;
  SoftmaxLayer() = default;
  SoftmaxLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
               std::vector<std::shared_ptr<Tensor>> output_tensors, bool preserve_shape)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        preserve_shape(std::move(preserve_shape)) {}
  ~SoftmaxLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::SOFTMAX; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 2) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(preserve_shape));
    } else {
      AbortOnVersion(this, version);
    }
  }

  bool preserve_shape = false;
};

/**
 * Warping Layer
 *
 * input [?]
 *
 * output [?]
 */
struct WarpingLayer : public Layer {
  using Layer::Layer;
  ~WarpingLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::WARPING; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 3) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(mode), CEREAL_NVP(stride), CEREAL_NVP(mapping_offset),
         CEREAL_NVP(padding_value), CEREAL_NVP(is_mapping_y_then_x));
    } else {
      AbortOnVersion(this, version);
    }
  }

  enum class warping_mode_t { NORMAL, YUV, UV };
  warping_mode_t mode = warping_mode_t::NORMAL;
  Shape2D stride;
  Shape2D mapping_offset;
  int32_t padding_value = 0;
  bool is_mapping_y_then_x = false;  // motion vector channel 0 is y, and channel 1 is x ?
};

/**
 * Detection Post Process Layer
 *
 * input [feature1, feature2, ..., anchor_table, exp_table, im_info]
 * 1~n. feature shape is [N, H, W, C], has 1 or C shift values, type is int8
 * n+1. anchor_table shape is [num_anchor, 4], no shift values, type is int32
 *      one anchor has 4 indexes in 4D shape
 * n+2. exp_table shape is [256], has 1 shift value, type is int32
 * n+3. im_info shape is [N, 1, 1, 2], no shift values, type is int32, can be nullptr
 *
 * output [bbox_tensor]
 * 1. bbox_tensor shape is [num_batch, num_bbox, 6], no shift, type is int32
 *    one bbox has 6 numbers [x1, y1, x2, y2, score, class_index]
 */
struct DetectionPostProcessLayer : public Layer {
  struct Attr {
    int32_t padding_value = -1;   // padding value for invalid boxes
    int32_t nms_threshold = 128;  // The overlap threshold that nms uses to suppress boxes (0-1, 8bit quantized)
    // The final number of output boxes, pad if it's fewer than it.
    int32_t nms_output_bbox_num = 300;
    // The score margin to prevent boxes to be suppressed in nms
    int32_t nms_margin = 0;
    // The shift number of the regression box_deltas and score. Usually 5/6 is a good choice.
    // This attribute is called "input_shift" in MXNET.
    int32_t score_shift = 6;
    // The threshold used to filter out low-score boxes (quantized by the input_shift)
    int32_t box_filter_threshold = 0;
    int32_t image_h = 1024;  // The original input image height.
    int32_t image_w = 1024;  // The original input image width.
    // The initial seed for the LRSF random number generator for boxes overflowing replacement.
    int32_t initial_seed = 1;
    // Whether clip the result box into [0, image_h or w - 1]
    std::vector<bool> use_clipping_list;
    // The anchor box start address for each branch in the 1-D input data: anchor"
    std::vector<int32_t> anchor_start_addr;
    // The number of classes to be classified for each branch. Used for InferShape checking.
    std::vector<int32_t> class_num_list;
    // The offset of classes that handled by one branch in the total number of classes
    std::vector<int32_t> class_offset_list;
    // Class indexes with custom thresholds, in corespondent with the class_threshold_values
    std::vector<int32_t> class_threshold_indexes;
    // Class thresholds, in corespondent with the class_threshold_indexes
    std::vector<int32_t> class_threshold_values;
    // The stride_w of the RPN layer. Usually 16 in Faster RCNN.
    std::vector<int32_t> feature_stride_h;
    // The stride_h of the RPN layer. Usually 16 in Faster RCNN.
    std::vector<int32_t> feature_stride_w;
    // The number of anchors used for each branch.
    std::vector<int32_t> num_anchors;
    // how to slicing feature map in H dimension
    std::vector<int32_t> __h_block_size_list__ = {-1, -1, -1, -1, -1};
    // how to slicing feature map in W dimension
    std::vector<int32_t> __w_block_size_list__ = {-1, -1, -1, -1, -1};
    // whether to skip nms in the final dpp output
    bool skip_nms = false;
    // Whether to do stable or unstable sort
    bool stable_sort = false;
    // Whether the image size is fixed. If not fixed, an extra input im_info needs to be provided.
    bool image_size_fixed = true;

    template <class Archive>
    void serialize(Archive &ar) {
      // DO NOT CHANGE THIS CODE, IT IS USED BY VERSION 3
      ar(CEREAL_NVP(padding_value), CEREAL_NVP(nms_threshold), CEREAL_NVP(nms_output_bbox_num), CEREAL_NVP(nms_margin),
         CEREAL_NVP(score_shift), CEREAL_NVP(box_filter_threshold), CEREAL_NVP(image_h), CEREAL_NVP(image_w),
         CEREAL_NVP(initial_seed), CEREAL_NVP(use_clipping_list), CEREAL_NVP(anchor_start_addr),
         CEREAL_NVP(class_num_list), CEREAL_NVP(class_offset_list), CEREAL_NVP(class_threshold_indexes),
         CEREAL_NVP(class_threshold_values), CEREAL_NVP(feature_stride_h), CEREAL_NVP(feature_stride_w),
         CEREAL_NVP(num_anchors), CEREAL_NVP(__h_block_size_list__), CEREAL_NVP(__w_block_size_list__),
         CEREAL_NVP(skip_nms), CEREAL_NVP(stable_sort), CEREAL_NVP(image_size_fixed));
    }
  };

  DetectionPostProcessLayer() = default;
  DetectionPostProcessLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                            std::vector<std::shared_ptr<Tensor>> output_tensors, DetectionPostProcessLayer::Attr attr)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)), attr(std::move(attr)) {}
  ~DetectionPostProcessLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::DETECTION_POST_PROCESS; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 3) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(attr));
    } else if (version == 4) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(attr.padding_value), CEREAL_NVP(attr.nms_threshold),
         CEREAL_NVP(attr.nms_output_bbox_num), CEREAL_NVP(attr.nms_margin), CEREAL_NVP(attr.score_shift),
         CEREAL_NVP(attr.box_filter_threshold), CEREAL_NVP(attr.image_h), CEREAL_NVP(attr.image_w),
         CEREAL_NVP(attr.initial_seed), CEREAL_NVP(attr.use_clipping_list), CEREAL_NVP(attr.anchor_start_addr),
         CEREAL_NVP(attr.class_num_list), CEREAL_NVP(attr.class_offset_list), CEREAL_NVP(attr.class_threshold_indexes),
         CEREAL_NVP(attr.class_threshold_values), CEREAL_NVP(attr.feature_stride_h), CEREAL_NVP(attr.feature_stride_w),
         CEREAL_NVP(attr.num_anchors), CEREAL_NVP(attr.__h_block_size_list__), CEREAL_NVP(attr.__w_block_size_list__),
         CEREAL_NVP(attr.skip_nms), CEREAL_NVP(attr.stable_sort), CEREAL_NVP(attr.image_size_fixed));
    } else {
      AbortOnVersion(this, version);
    }
  }

  DetectionPostProcessLayer::Attr attr;
};

/**
 * RCNN Post Process Layer
 *
 * input [bbox_tensor, bbox_score, bbox_deltas]
 * 1. bbox_tensor shape is [num_batch, num_bbox, 6], no shift, type is int32
 *    one bbox has 6 numbers [x1, y1, x2, y2, score, class_index]
 * 2. bbox_score shape is [num_batch * num_bbox, 1, 1, (num_class + 1)], no shift, type is float32
 * 3. bbox_deltas shape is [num_batch * num_bbox, 1, 1, (num_class + 1) * 4], no shift, type is float32
 * 4. im_info shape is [N, 1, 1, 2], no shift values, type is int32, can be nullptr
 *
 * output [output0, output1]
 * 1. output0 shape is [num_batch, nms_top_n, 6], no shift, type is int32
 *    one bbox has 6 numbers [x1, y1, x2, y2, score, class_index]
 * 2. output1 shape is [num_batch, nms_top_n, 6], no shift, type is float32
 *    one bbox has 6 numbers [x1, y1, x2, y2, score, class_index]
 */
struct RcnnPostProcessLayer : public Layer {
  struct Attr;
  RcnnPostProcessLayer() = default;
  RcnnPostProcessLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                       std::vector<std::shared_ptr<Tensor>> output_tensors, Attr attr)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)), attr(std::move(attr)) {}
  ~RcnnPostProcessLayer() override = default;
  layer_type_t GetLayerType() const override { return layer_type_t ::RCNN_POST_PROCESS; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 2) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(attr));
    } else if (version == 3) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(attr.original_img_h), CEREAL_NVP(attr.original_img_w),
         CEREAL_NVP(attr.nms_threshold), CEREAL_NVP(attr.class_number), CEREAL_NVP(attr.nms_top_n),
         CEREAL_NVP(attr.score_threshold), CEREAL_NVP(attr.bbox_delta_mean), CEREAL_NVP(attr.bbox_delta_std),
         CEREAL_NVP(attr.image_size_fixed));
    } else {
      AbortOnVersion(this, version);
    }
  }

  struct Attr {
    uint32_t original_img_h = 0;
    uint32_t original_img_w = 0;
    float nms_threshold = 0.0;
    uint32_t class_number = 0;
    uint32_t nms_top_n = 0;
    float score_threshold = 0.0;
    std::vector<float> bbox_delta_mean;  // array[4]
    std::vector<float> bbox_delta_std;   // array[4]
    // Whether the image size is fixed. If not fixed, an extra input im_info needs to be provided.
    bool image_size_fixed = true;

    template <class Archive>
    void serialize(Archive &ar) {
      // DO NOT CHANGE THIS CODE, IT IS USED BY VERSION 2
      ar(CEREAL_NVP(original_img_h), CEREAL_NVP(original_img_w), CEREAL_NVP(nms_threshold), CEREAL_NVP(class_number),
         CEREAL_NVP(nms_top_n), CEREAL_NVP(score_threshold), CEREAL_NVP(bbox_delta_mean), CEREAL_NVP(bbox_delta_std),
         CEREAL_NVP(image_size_fixed));
    }
  } attr;
};

/**
 * ROI Align Layer
 *
 * ========== roi align mode ==========
 *
 * input [feature1, feature2, ..., bbox_tensor]
 * 1~n. feature shape is [N, H, W, C], has 1 or C shift values, type is int8
 * n+1. bbox_tensor shape is [1, num_bbox, 6], no shift, type is int32
 *      one bbox has 6 numbers [left, top, right, bottom, score, class_index]
 *
 * output [output]
 * 1. output shape is [num_bbox, H, W, C], has 1 or C shift values, type is int8
 */
struct ROIAlignLayer : public Layer {
  struct Attr;
  ROIAlignLayer() = default;
  ROIAlignLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
                std::vector<std::shared_ptr<Tensor>> output_tensors, Attr attr)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)), attr(std::move(attr)) {}
  ~ROIAlignLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::ROI_ALIGN; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 2) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(attr));
    } else if (version == 3) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(attr.output_shape), CEREAL_NVP(attr.feature_stride_h),
         CEREAL_NVP(attr.feature_stride_w), CEREAL_NVP(attr.base_image_scale), CEREAL_NVP(attr.middle_layer_id),
         CEREAL_NVP(attr.ignore_score_value), CEREAL_NVP(attr.num_pooling), CEREAL_NVP(attr.max_num_pooling),
         CEREAL_NVP(attr.pool_method), CEREAL_NVP(attr.padding_mode), CEREAL_NVP(attr.area_index),
         CEREAL_NVP(attr.clip_box), CEREAL_NVP(attr.box_augmentation), CEREAL_NVP(attr.box_augmentation_params));
    } else {
      AbortOnVersion(this, version);
    }
  }

  struct Attr {
    Shape2D output_shape;  // ROI output shape (h,w), it usually varies from different tasks, like (7, 7) for detection

    std::vector<uint16_t> feature_stride_h = {1u};  // The corresponding stride_h for data1-5
    std::vector<uint16_t> feature_stride_w = {1u};  // The corresponding stride_w for data1-5

    // box_layer_id = floor(middle_layer_id + log2(sqrt(box_w * box_h) / base_image_scale))
    int32_t base_image_scale = 224;
    int32_t middle_layer_id = 0;  // It can usually be calculated as 4 - log2(min(stride_list))

    float ignore_score_value = -128;  // the score value for ignored boxes

    // when num_pooling >= 0, this Op will always does certain number of pooling,
    // and the crop_resize target size is also fixed
    int32_t num_pooling = -1;
    int32_t max_num_pooling = -1;  // when it >= 0, this Op will have at most `max_num_pooling` times of poolings
    enum class PoolMethod { AVG, MAX } pool_method = PoolMethod::AVG;  // the pooling method used after crop_and_resize

    // - shrink: make sure the step size is small enough to avoid the border
    // - zero: pad with 0
    // - nearest: pad with border value
    // - border: ?
    enum class PaddingMode { SHRINK, ZERO, NEAREST, BORDER } padding_mode = PaddingMode::SHRINK;

    int32_t area_index = -1;  // if area_index >= 0, it will get the area data from input, instead of calculation

    bool clip_box = false;          // when true, the box will be clipped when use_box_augmentation is true.
    bool box_augmentation = false;  // when true, the box will be augmented when cropping (mostly for skeleton)
    std::vector<float> box_augmentation_params = {1.0, 0, 0, 0, 0};  // array<5>, representing a, r1, r2, r3, r4

    template <class Archive>
    void serialize(Archive &ar) {
      // DO NOT CHANGE THIS CODE, IT IS USED BY VERSION 2
      ar(CEREAL_NVP(output_shape), CEREAL_NVP(feature_stride_h), CEREAL_NVP(feature_stride_w),
         CEREAL_NVP(base_image_scale), CEREAL_NVP(middle_layer_id), CEREAL_NVP(ignore_score_value),
         CEREAL_NVP(num_pooling), CEREAL_NVP(max_num_pooling), CEREAL_NVP(pool_method), CEREAL_NVP(padding_mode),
         CEREAL_NVP(area_index), CEREAL_NVP(clip_box), CEREAL_NVP(box_augmentation),
         CEREAL_NVP(box_augmentation_params));
    }
  } attr;
};

/**
 * Slice Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C], has 1 or C shift values,
 *
 * output [feature]
 * 1. output shape is [N, H, W, C], has 1 or C shift values,
 */
struct SliceLayer : public Layer {
  struct Attr;
  SliceLayer() = default;
  SliceLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
             std::vector<std::shared_ptr<Tensor>> output_tensors, std::vector<uint32_t> begin,
             std::vector<uint32_t> end, std::vector<uint32_t> step)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)),
        begin(std::move(begin)),
        end(std::move(end)),
        step(std::move(step)) {}
  ~SliceLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::SLICE; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(begin), CEREAL_NVP(step), CEREAL_NVP(end));
    } else {
      AbortOnVersion(this, version);
    }
  }

  std::vector<uint32_t> begin;  // can be [N] or [N, H] or [N, H, W] or [N, H, W, C]
  std::vector<uint32_t> end;    // can be [N] or [N, H] or [N, H, W] or [N, H, W, C]
  std::vector<uint32_t> step;   // can be [N] or [N, H] or [N, H, W] or [N, H, W, C]
};

/**
 * Lookup Table Layer
 *
 * input [?]
 *
 * output [?]
 */
struct LutLayer : public Layer {
  struct Attr;
  LutLayer() = default;
  LutLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
           std::vector<std::shared_ptr<Tensor>> output_tensors, Attr attr)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)), attr(std::move(attr)) {}
  ~LutLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::LUT; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(attr));
    } else if (version == 2) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(attr.dense_shift), CEREAL_NVP(attr.dense_scale),
         CEREAL_NVP(attr.dense_beta), CEREAL_NVP(attr.dense_min), CEREAL_NVP(attr.dense_max),
         CEREAL_NVP(attr.dense_table), CEREAL_NVP(attr.sparse_shift), CEREAL_NVP(attr.sparse_scale),
         CEREAL_NVP(attr.sparse_beta), CEREAL_NVP(attr.sparse_min), CEREAL_NVP(attr.sparse_max),
         CEREAL_NVP(attr.sparse_table), CEREAL_NVP(attr.left_shift), CEREAL_NVP(attr.left_scale),
         CEREAL_NVP(attr.left_beta), CEREAL_NVP(attr.right_shift), CEREAL_NVP(attr.right_scale),
         CEREAL_NVP(attr.right_beta), CEREAL_NVP(attr.x_min), CEREAL_NVP(attr.x_max), CEREAL_NVP(attr.enable_symmetry),
         CEREAL_NVP(attr.symmetry_k), CEREAL_NVP(attr.symmetry_b), CEREAL_NVP(attr.idx_bits));
    } else {
      AbortOnVersion(this, version);
    }
  }

  enum class LutType {
    SIGMOID = 0,
    TANH = 1,
    EXP = 2,
    LOG = 3,
  };

  struct Attr {
    int32_t dense_shift;
    int32_t dense_scale;
    int32_t dense_beta;
    int32_t dense_min;
    int32_t dense_max;
    std::vector<int32_t> dense_table;
    int32_t sparse_shift;
    int32_t sparse_scale;
    int32_t sparse_beta;
    int32_t sparse_min;
    int32_t sparse_max;
    std::vector<int32_t> sparse_table;
    int32_t left_shift;
    int32_t left_scale;
    int32_t left_beta;
    int32_t right_shift;
    int32_t right_scale;
    int32_t right_beta;
    int32_t x_min;
    int32_t x_max;
    bool enable_symmetry;
    int32_t symmetry_k;
    int32_t symmetry_b;
    int32_t idx_bits;

    template <class Archive>
    void serialize(Archive &ar) {
      // DO NOT CHANGE THIS CODE, IT IS USED BY VERSION 1
      ar(CEREAL_NVP(dense_shift), CEREAL_NVP(dense_scale), CEREAL_NVP(dense_beta), CEREAL_NVP(dense_min),
         CEREAL_NVP(dense_max), CEREAL_NVP(dense_table), CEREAL_NVP(sparse_shift), CEREAL_NVP(sparse_scale),
         CEREAL_NVP(sparse_beta), CEREAL_NVP(sparse_min), CEREAL_NVP(sparse_max), CEREAL_NVP(sparse_table),
         CEREAL_NVP(left_shift), CEREAL_NVP(left_scale), CEREAL_NVP(left_beta), CEREAL_NVP(right_shift),
         CEREAL_NVP(right_scale), CEREAL_NVP(right_beta), CEREAL_NVP(x_min), CEREAL_NVP(x_max),
         CEREAL_NVP(enable_symmetry), CEREAL_NVP(symmetry_k), CEREAL_NVP(symmetry_b), CEREAL_NVP(idx_bits));
    }
  } attr;
};

/**
 * Reshape Layer
 *
 * input
 * 1. feature shape is [N, H, W, C]
 *
 * output
 * 1. feature shape is [N, H, W, C]
 */
struct ReshapeLayer : public Layer {
  ReshapeLayer() = default;
  ReshapeLayer(std::string name, std::vector<std::shared_ptr<Tensor>> input_tensors,
               std::vector<std::shared_ptr<Tensor>> output_tensors)
      : Layer(std::move(name), std::move(input_tensors), std::move(output_tensors)) {}
  ~ReshapeLayer() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::RESHAPE; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this), CEREAL_NVP(mode), CEREAL_NVP(factor));
    } else {
      AbortOnVersion(this, version);
    }
  }

  enum class reshape_mode_t {
    STACK_NEIGHBOR = 0,
    REORDER_UPSCALE_CONSECUTIVE,  // channels are 00000111112222233333 before reorder_upscale
    REORDER_UPSCALE_UNFOLD,       // channels are 01230123012301230123 before reorder_upscale
  };

  reshape_mode_t mode;
  uint8_t factor;
};

/**
 * Quanti Flatten Layer
 *
 * input [feature]
 * 1. feature shape is [N, H, W, C]
 *
 * output [output]
 * 1. output shape is [N, 1, 1, H*W*C]
 */
struct QuantiFlatten : public Layer {
  using Layer::Layer;
  ~QuantiFlatten() noexcept override = default;
  layer_type_t GetLayerType() const override { return layer_type_t::QUANTI_FLATTEN; }

  template <class Archive>
  void serialize(Archive &ar, std::int32_t const version) {
    // Don't forget to update the version number using HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION in the following
    if (version == 1) {
      ar(cereal::base_class<Layer>(this));
    } else {
      AbortOnVersion(this, version);
    }
  }
};

}  // namespace hbir
}  // namespace hbdk

#define HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(type, version) \
  CEREAL_REGISTER_TYPE(type);                                      \
  CEREAL_CLASS_VERSION(type, version);

// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::Tensor, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::Layer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::QuantiInputLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ConvolutionLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::AvgPoolingLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::MaxPoolingLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::GlobalAvgPoolingLayer, 2);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::GlobalMaxPoolingLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::RoiResizeLayer, 4);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ReluLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ElementwiseMul, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ElementwiseAdd, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::SplitLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ConcatLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ChannelMaxLayer, 2);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::SoftmaxLayer, 2);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::WarpingLayer, 3);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::DetectionPostProcessLayer, 4);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::RcnnPostProcessLayer, 3)
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ROIAlignLayer, 3);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ChannelSumLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::FilterLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::SliceLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::LutLayer, 2);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ElementwiseSub, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::ReshapeLayer, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::QuantiFlatten, 1);
// NOLINTNEXTLINE
HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION(hbdk::hbir::DeconvolutionLayer, 1);
#undef HBDK_HBIR_CEREAL_REGISTER_TYPE_WITH_VERSION

namespace hbdk {
namespace hbir {

struct Serializer {
  template <typename... T>
  static void ToJsonFile(const std::string &filename, T &... objects) {
    std::ofstream ofs(filename);
    cereal::JSONOutputArchive oarchive(ofs);
    oarchive(std::forward<T>(objects)...);
  }

  template <typename... T>
  static void FromJsonFile(const std::string &filename, T &... objects) {
    std::ifstream ifs(filename);
    cereal::JSONInputArchive iarchive(ifs);
    iarchive(std::forward<T>(objects)...);
  }

  template <typename... T>
  static void ToBinaryFile(const std::string &filename, T &... objects) {
    std::ofstream ofs(filename, std::ios::binary);
    cereal::BinaryOutputArchive oarchive(ofs);
    oarchive(std::forward<T>(objects)...);
  }

  template <typename... T>
  static void FromBinaryFile(const std::string &filename, T &... objects) {
    std::ifstream ifs(filename, std::ios::binary);
    cereal::BinaryInputArchive iarchive(ifs);
    iarchive(std::forward<T>(objects)...);
  }
};

template <>
struct ElementType<int8_t> {
  static constexpr auto value = Tensor::element_type_t::S8;
};
template <>
struct ElementType<int16_t> {
  static constexpr auto value = Tensor::element_type_t::S16;
};
template <>
struct ElementType<int32_t> {
  static constexpr auto value = Tensor::element_type_t::S32;
};
template <>
struct ElementType<int64_t> {
  static constexpr auto value = Tensor::element_type_t::S64;
};
template <>
struct ElementType<uint8_t> {
  static constexpr auto value = Tensor::element_type_t::U8;
};
template <>
struct ElementType<uint16_t> {
  static constexpr auto value = Tensor::element_type_t::U16;
};
template <>
struct ElementType<uint32_t> {
  static constexpr auto value = Tensor::element_type_t::U32;
};
template <>
struct ElementType<uint64_t> {
  static constexpr auto value = Tensor::element_type_t::U64;
};
template <>
struct ElementType<float> {
  static constexpr auto value = Tensor::element_type_t::F32;
};
template <>
struct ElementType<double> {
  static constexpr auto value = Tensor::element_type_t::F64;
};

#define HBIR_TENSOR_ELEMENT_TYPE_SWITCH(element_type, DType, ...) \
  switch (element_type) {                                         \
    case hbir::Tensor::element_type_t::S8: {                      \
      using DType = int8_t;                                       \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::S16: {                     \
      using DType = int16_t;                                      \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::S32: {                     \
      using DType = int32_t;                                      \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::S64: {                     \
      using DType = int64_t;                                      \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::U8: {                      \
      using DType = uint8_t;                                      \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::U16: {                     \
      using DType = uint16_t;                                     \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::U32: {                     \
      using DType = uint32_t;                                     \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::U64: {                     \
      using DType = uint64_t;                                     \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::F32: {                     \
      using DType = float;                                        \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    case hbir::Tensor::element_type_t::F64: {                     \
      using DType = double;                                       \
      { __VA_ARGS__ }                                             \
    } break;                                                      \
    default:                                                      \
      std::cerr << "unknown hbir tensor element type";            \
  }

}  // namespace hbir
}  // namespace hbdk

#endif  // HBDK_HBDK_IR_HPP
