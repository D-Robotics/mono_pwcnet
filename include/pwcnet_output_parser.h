// Copyright (c) 2024，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PWCNET_OUTPUT_PARSER_H_
#define PWCNET_OUTPUT_PARSER_H_

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "dnn_node/dnn_node_data.h"
#include "rclcpp/rclcpp.hpp"

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::Model;

struct OpticalFlow;
enum class ImgType;

class PwcNetOutputParser {
 public:
  PwcNetOutputParser(int output_height, int output_width, int num_classes, float mask_threshold) {
    output_height_ = output_height;
    output_width_ = output_width;
    num_classes_ = num_classes;
    mask_threshold_ = mask_threshold;
  }
  ~PwcNetOutputParser() {}

  int32_t Parse(
      std::shared_ptr<OpticalFlow> &result,
      const int resized_img_h,
      const int resized_img_w,
      const int model_h,
      const int model_w,
      std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
      float ratio = 1.0);

  int32_t PostProcess(
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    const int resized_img_h,
    const int resized_img_w,
    const int model_h,
    const int model_w,
    std::shared_ptr<OpticalFlow> &result,
    float ratio = 1.0);

 private:
  int num_classes_ = 5;
  int output_height_ = 96;
  int output_width_ = 96;
  float mask_threshold_ = 0.1;
};

#endif  // MIBILESAM_OUTPUT_PARSER_H_
