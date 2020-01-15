/**
 *  Copyright (c) 2016 Horizon Robotics. All rights reserved.
 *  @file vision_msg.hpp
 *  \~English @brief this header file defines the vision related messages that are used in
 * HOBOT framework, including face, hand
 *
 */
#ifndef VISION_TYPE_VISION_MSG_HPP_
#define VISION_TYPE_VISION_MSG_HPP_

#ifdef __ANDROID__
#include "opencv2/core/core.hpp"
#include "opencv2/core/core_c.h"
#else
#include <opencv2/core/core.hpp>
#endif

#include "hobot/hobot.h"
#include "horizon/vision_type/vision_type.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace hobot {
namespace vision {
struct Header {
  int64_t seq_;
  /// \~English seconds
  int64_t stamp_secs_;
  /// \~English nanoseconds
  int64_t stamp_nsecs_;
  Header() : seq_(0), stamp_secs_(0), stamp_nsecs_(0) {}
  Header(const Header &header) {
    seq_ = header.seq_;
    stamp_secs_ = header.stamp_secs_;
    stamp_nsecs_ = header.stamp_nsecs_;
  }
  Header &operator=(const Header &header) {
    seq_ = header.seq_;
    stamp_secs_ = header.stamp_secs_;
    stamp_nsecs_ = header.stamp_nsecs_;
    return *this;
  }
  int64_t seq() const { return seq_; }
  double stamp() const {
    double stamp = stamp_secs_ + stamp_nsecs_ * 1e-9;
    return stamp;
  }
};

/**
 * \~English @brief Define message used for CV Mat object
 */
class CvMatMessage : public hobot::Message {
 public:
  CvMatMessage() {}

  ~CvMatMessage() {}

  cv::Mat &GetImage() { return image_; }

 public:
  Header header_;

 private:
  cv::Mat image_;
};
typedef std::shared_ptr<CvMatMessage> spCvMatMessage;

/**
 * \~English @brief Define message used for a BBox vector
 */
class RectsMessage : public hobot::Message {
 public:
  RectsMessage() {}

  ~RectsMessage() {}

  std::vector<BBox> &GetRects() { return rects_; }

  std::vector<int> &GetIdxList() { return idx_list_; }

 public:
  Header header_;

 private:
  std::vector<BBox> rects_;

  std::vector<int> idx_list_;
};
typedef std::shared_ptr<RectsMessage> spRectsMessage;

/**
 * \~English @brief Define message used for Landmarks vector
 */
class LandmarksMessage : public hobot::Message {
 public:
  LandmarksMessage() {}

  ~LandmarksMessage() {}

  std::vector<Landmarks> &GetLandmarks() { return landmarks_; }

  std::vector<int> &GetIdxList() { return idx_list_; }

  float &GetPose() { return pose_; }

 public:
  Header header_;

 private:
  std::vector<Landmarks> landmarks_;
  float pose_;
  std::vector<int> idx_list_;
};
typedef std::shared_ptr<LandmarksMessage> spLandmarksMessage;

/**
 * \~English @brief Define message used for a attribute vector
 */
class AttributesMessage : public hobot::Message {
 public:
  AttributesMessage() {}

  ~AttributesMessage() {}

  std::vector<std::vector<Attribute<int32_t>>> &GetAttributes() {
    return attributes_;
  }

  std::vector<int> &GetIdxList() { return idx_list_; }

 public:
  Header header_;

 private:
  std::vector<std::vector<Attribute<int32_t>>> attributes_;

  std::vector<int> idx_list_;
};
typedef std::shared_ptr<AttributesMessage> spAttributesMessage;

/**
 * \~English @brief Define message used for face metric features
 */
class FaceMetricFeaturesMessage : public hobot::Message {
 public:
  FaceMetricFeaturesMessage() {}

  ~FaceMetricFeaturesMessage() {}

  std::vector<std::vector<float>> &GetFeatures() { return metric_feature_; }

  std::vector<bool> &GetHasFeature() { return has_metric_feature_; }

  std::vector<int> &GetIdxList() { return idx_list_; }

 public:
  Header header_;

 private:
  std::vector<std::vector<float>> metric_feature_;

  std::vector<bool> has_metric_feature_;

  std::vector<int> idx_list_;
};
typedef std::shared_ptr<FaceMetricFeaturesMessage> spFaceMetricFeaturesMessage;
typedef FaceMetricFeaturesMessage ReidFeatureMessage;
typedef std::shared_ptr<ReidFeatureMessage> spReidFeatureMessage;

class MessageListMessage : public hobot::Message {
 public:
  MessageListMessage() {}
  ~MessageListMessage() {}
  const std::vector<spMessage> &GetMessageList() { return message_list_; }
  const int GetMessageListLength() { return message_list_.size(); }
  void AddMessage(spMessage message) { this->message_list_.push_back(message); }
  void ClearMessageList() { message_list_.clear(); }

 private:
  std::vector<spMessage> message_list_;
};
typedef std::shared_ptr<MessageListMessage> spMessageListMessage;

}  // namespace vision
}  // namespace hobot

#endif  // VISION_TYPE_VISION_MSG_HPP_
