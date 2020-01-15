//
// Created by yaoyao.sun on 2019-04-23.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include "dump.h"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "horizon/vision_type/vision_type.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "hobotxsdk/xroc_sdk.h"
#include "hobotlog/hobotlog.hpp"

#include "result.h"

using HobotXRoc::BaseData;
using HobotXRoc::BaseDataPtr;
using HobotXRoc::BaseDataVector;
using HobotXRoc::XRocData;

using hobot::vision::BBox;
using hobot::vision::Segmentation;
using hobot::vision::Landmarks;

namespace faster_rcnn_method {

static std::vector<cv::Scalar> cls_colors = \
  {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0),
   cv::Scalar(0, 0, 255), {255, 255, 0},
   cv::Scalar(255, 0, 255), {0, 255, 255}};

void ShowBBox(cv::Mat frame, const BBox &bbox, const cv::Scalar &color) {
  rectangle(frame, cv::Point(bbox.x1, bbox.y1), cv::Point(bbox.x2, bbox.y2),
            color, 2, 8);
}

void DumpSkeleton(cv::Mat &frame,
                  const std::shared_ptr<BaseDataVector> &skeletons) {
  float kps_thresh = 0.2;
  float kps_thresh_max = 3.0;

  for (const auto &skeleton : skeletons->datas_) {
    auto p_kps = std::static_pointer_cast<XRocData<Landmarks>>(skeleton);

    auto &points = p_kps->value.values;
    if (points[15].score > kps_thresh && points[15].score < kps_thresh_max &&
        points[13].score > kps_thresh && points[13].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[15].x, points[15].y),
               cv::Point(points[13].x, points[13].y), CV_RGB(255, 0, 0), 2);
    }
    if (points[13].score > kps_thresh && points[13].score < kps_thresh_max &&
        points[11].score > kps_thresh && points[11].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[13].x, points[13].y),
               cv::Point(points[11].x, points[11].y), CV_RGB(255, 85, 0), 2);
    }
    if (points[16].score > kps_thresh && points[16].score < kps_thresh_max &&
        points[14].score > kps_thresh && points[14].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[16].x, points[16].y),
               cv::Point(points[14].x, points[14].y), CV_RGB(255, 170, 0), 2);
    }
    if (points[14].score > kps_thresh && points[14].score < kps_thresh_max &&
        points[12].score > kps_thresh && points[12].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[14].x, points[14].y),
               cv::Point(points[12].x, points[12].y), CV_RGB(255, 170, 0), 2);
    }
    if (points[11].score > kps_thresh && points[11].score < kps_thresh_max &&
        points[12].score > kps_thresh && points[12].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[11].x, points[11].y),
               cv::Point(points[12].x, points[12].y), CV_RGB(170, 255, 0), 2);
    }
    if (points[5].score > kps_thresh && points[5].score < kps_thresh_max &&
        points[11].score > kps_thresh && points[11].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[5].x, points[5].y),
               cv::Point(points[11].x, points[11].y), CV_RGB(85, 255, 0), 2);
    }
    if (points[6].score > kps_thresh && points[6].score < kps_thresh_max &&
        points[12].score > kps_thresh && points[12].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[6].x, points[6].y),
               cv::Point(points[12].x, points[12].y), CV_RGB(0, 255, 0), 2);
    }
    if (points[5].score > kps_thresh && points[5].score < kps_thresh_max &&
        points[6].score > kps_thresh && points[6].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[5].x, points[5].y),
               cv::Point(points[6].x, points[6].y), CV_RGB(0, 255, 85), 2);
    }
    if (points[5].score > kps_thresh && points[5].score < kps_thresh_max &&
        points[7].score > kps_thresh && points[7].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[5].x, points[5].y),
               cv::Point(points[7].x, points[7].y), CV_RGB(0, 255, 170), 2);
    }
    if (points[6].score > kps_thresh && points[6].score < kps_thresh_max &&
        points[8].score > kps_thresh && points[8].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[6].x, points[6].y),
               cv::Point(points[8].x, points[8].y), CV_RGB(0, 255, 255), 2);
    }
    if (points[7].score > kps_thresh && points[7].score < kps_thresh_max &&
        points[9].score > kps_thresh && points[9].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[7].x, points[7].y),
               cv::Point(points[9].x, points[9].y), CV_RGB(0, 170, 255), 2);
    }
    if (points[8].score > kps_thresh && points[8].score < kps_thresh_max &&
        points[10].score > kps_thresh && points[10].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[8].x, points[8].y),
               cv::Point(points[10].x, points[10].y), CV_RGB(0, 85, 255), 2);
    }
    if (points[1].score > kps_thresh && points[1].score < kps_thresh_max &&
        points[2].score > kps_thresh && points[2].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[1].x, points[1].y),
               cv::Point(points[2].x, points[2].y), CV_RGB(0, 0, 255), 2);
    }
    if (points[0].score > kps_thresh && points[0].score < kps_thresh_max &&
        points[1].score > kps_thresh && points[1].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[0].x, points[0].y),
               cv::Point(points[1].x, points[1].y), CV_RGB(85, 0, 255), 2);
    }
    if (points[0].score > kps_thresh && points[0].score < kps_thresh_max &&
        points[2].score > kps_thresh && points[2].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[0].x, points[0].y),
               cv::Point(points[2].x, points[2].y), CV_RGB(170, 0, 255), 2);
    }
    if (points[1].score > kps_thresh && points[1].score < kps_thresh_max &&
        points[3].score > kps_thresh && points[3].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[1].x, points[1].y),
               cv::Point(points[3].x, points[3].y), CV_RGB(255, 0, 255), 2);
    }
    if (points[2].score > kps_thresh && points[2].score < kps_thresh_max &&
        points[4].score > kps_thresh && points[4].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[2].x, points[2].y),
               cv::Point(points[4].x, points[4].y), CV_RGB(255, 0, 170), 2);
    }
    if (points[3].score > kps_thresh && points[3].score < kps_thresh_max &&
        points[5].score > kps_thresh && points[5].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[3].x, points[3].y),
               cv::Point(points[5].x, points[5].y), CV_RGB(255, 0, 85), 2);
    }
    if (points[4].score > kps_thresh && points[4].score < kps_thresh_max &&
        points[6].score > kps_thresh && points[6].score < kps_thresh_max) {
      cv::line(frame, cv::Point(points[4].x, points[4].y),
               cv::Point(points[6].x, points[6].y), CV_RGB(0, 0, 255), 2);
    }
  }
}

void DumpLandmark(cv::Mat &frame,
                  const std::shared_ptr<BaseDataVector> &landmarks) {
  for (const auto &landmark : landmarks->datas_) {
    auto p_landmark = std::static_pointer_cast<XRocData<Landmarks>>(landmark);

    auto &points = p_landmark->value.values;
    for (auto &point : points) {
      cv::rectangle(frame, cv::Point(point.x - 2, point.y - 2),
                    cv::Point(point.x + 2, point.y + 2), CV_RGB(255, 0, 0), 2);
    }
  }
}

void DumpMask(cv::Mat &frame, const std::shared_ptr<BaseDataVector> &masks,
              const std::shared_ptr<BaseDataVector> &body_boxes) {
  if (masks->datas_.empty()) {
    LOGI << "masks empty.";
    return;
  }
  size_t body_box_size = body_boxes->datas_.size();
  HOBOT_CHECK(masks->datas_.size() == body_box_size);

  // std::ofstream ofs("mask.txt");

  for (size_t i = 0; i < body_box_size; ++i) {
    auto xroc_body_box =
        std::static_pointer_cast<XRocData<BBox>>((body_boxes->datas_)[i]);
    auto body_box = xroc_body_box->value;
    int x1 = body_box.x1;
    int y1 = body_box.y1;
    int x2 = body_box.x2;
    int y2 = body_box.y2;

    auto xroc_mask =
        std::static_pointer_cast<XRocData<Segmentation>>((masks->datas_)[i]);
    auto mask = xroc_mask->value;
    //    ofs << "(";
    //    for (const auto &value : mask.values) {
    //      ofs << value << ",";
    //    }
    //    ofs << ")" << std::endl;

    int h_w = sqrt(mask.values.size());
    LOGI << "mask h_w: " << h_w;

    cv::Mat mask_mat;
    mask_mat.create(h_w, h_w, CV_32F);

    for (int h = 0; h < h_w; ++h) {
      float *ptr = mask_mat.ptr<float>(h);
      for (int w = 0; w < h_w; ++w) {
        *(ptr + w) = (mask.values)[h * h_w + w];
      }
    }
    int width = x2 - x1;
    int height = y2 - y1;
    cv::resize(mask_mat, mask_mat, cv::Size(width, height));

    cv::Mat frame_rect = frame(cv::Rect(x1, y1, width, height));

    float mask_color_scale = 0.4;
    float mask_color_scale_other = 1 - mask_color_scale;
    const cv::Scalar mask_color_3 = cv::Scalar(0, 0, 255);
    int mask_color = mask_color_3[2] * mask_color_scale;
    cv::Mat mask_gray;
    mask_gray.create(height, width, CV_8UC1);
    mask_gray.setTo(0);
    for (int h = 0; h < height; ++h) {
      uchar *p_rect = frame_rect.ptr<uchar>(h);
      uchar *p_gray = mask_gray.ptr<uchar>(h);
      const float *p_mask = mask_mat.ptr<float>(h);
      for (int w = 0; w < width; ++w) {
        if (p_mask[w] > 0.5) {
          // 这个点在人体内
          p_rect[w * 3 + 1] =
              (mask_color_scale_other * p_rect[w * 3 + 1]) + mask_color;
          p_rect[w * 3 + 2] =
              (mask_color_scale_other * p_rect[w * 3 + 2]) + mask_color;
          p_gray[w] = 1;
        } else {
          // p_rect[w*3 + 0] = (0.4 * p_rect[w*3 + 0]) + 204*0.6;
          // p_rect[w*3 + 1] = (0.4 * p_rect[w*3 + 1]) + 187*0.6;
        }
      }
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_gray, contours, cv::noArray(), cv::RETR_CCOMP,
                     cv::CHAIN_APPROX_NONE);
    cv::drawContours(frame_rect, contours, -1, cv::Scalar::all(255), 2);
  }
}

void DumpRects(cv::Mat &frame, const std::shared_ptr<BaseDataVector> &rects,
               int class_id) {
  // std::string save_path = std::to_string(class_id) + "_rect.txt";
  // std::ofstream ofs(save_path);
  for (auto &in_rect : rects->datas_) {
    auto rect = std::static_pointer_cast<XRocData<BBox>>(in_rect);
    LOGD << rect->value;
    ShowBBox(frame, rect->value, cls_colors[class_id]);
  }
}

static std::vector<std::string> vehicle_box = {
    "vehicle_box", "plate_box", "non_motor_box", "pedestrian_box",
    "front_window_box"};

void DumpRectsWithInfo(cv::Mat &frame,
                       const std::shared_ptr<BaseDataVector> &rects,
                       int class_id, std::string image_name) {
  for (auto &in_rect : rects->datas_) {
    auto rect = std::static_pointer_cast<XRocData<BBox>>(in_rect);
    std::cout << image_name << " " << vehicle_box[class_id] << " "
              << rect->value << std::endl;
    ShowBBox(frame, rect->value, cls_colors[class_id]);
  }
}

static std::vector<std::string> plate_colors = {"black", "blue",   "green",
                                                "white", "yellow", "other"};

static std::vector<std::string> plate_rows = {"single", "dual", "trible",
                                              "quater"};
void DumpPlateWithInfo(cv::Mat &frame,
                       const std::shared_ptr<BaseDataVector> &plate_color,
                       const std::shared_ptr<BaseDataVector> &plate_row,
                       int class_id, std::string image_name) {
  HOBOT_CHECK(plate_color->datas_.size() == plate_row->datas_.size());
  for (uint32_t index = 0; index < plate_color->datas_.size(); ++index) {
    auto color_data = std::static_pointer_cast<XRocData<Attribute<int>>>(
        plate_color->datas_[index]);
    auto row_data = std::static_pointer_cast<XRocData<Attribute<int>>>(
        plate_row->datas_[index]);
    std::cout << image_name << " plate_index " << index << " color "
              << plate_colors[color_data->value.value] << " score "
              << color_data->value.score << " row "
              << plate_rows[row_data->value.value] << " score "
              << row_data->value.score << std::endl;
  }
}

}  // namespace faster_rcnn_method
