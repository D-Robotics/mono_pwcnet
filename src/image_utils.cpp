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

#include "include/pwcnet_node.h"

#include "include/image_utils.h"

double const PI = 3.14159265;

int ImageUtils::Render(std::shared_ptr<PwcNetOutput> out, OpticalFlow &flow, cv::Mat &output) {
  int parsing_width = flow.valid_w;
  int parsing_height = flow.valid_h;
  int stride = parsing_width * parsing_height;
  float *flow_data = flow.data.data();
  std::vector<float> flow_x;
  flow_x.reserve(stride);
  std::vector<float> flow_y;
  flow_y.reserve(stride);

  for (int i = 0; i < stride; ++i) {
    flow_x.emplace_back(flow_data[i] * 4.0);
    flow_y.emplace_back(flow_data[stride + i] * 4.0);
  }
  cv::Mat magnitude, angle;
  cv::cartToPolar(flow_x, flow_y, magnitude, angle);
  float *magnitude_data = reinterpret_cast<float *>(magnitude.data);
  if (magnitude_data == nullptr) {
    RCLCPP_INFO(rclcpp::get_logger("mono_pwcnet"), 
                "magnitude_data is null pointer");
    return -1;
  }
  float *angle_data = reinterpret_cast<float *>(angle.data);
  // 1. magnitude: nan check
  for (int i = 0; i < stride; ++i) {
    magnitude_data[i] =
        std::isnan(magnitude_data[i]) ? 0.0f : magnitude_data[i];
  }
  // 2. magnitude: normalize
  cv::normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX);
  // 3. flow2img
  cv::Mat flow_img(parsing_height, parsing_width, CV_8UC3);
  int id = 0;
  for (int h = 0; h < parsing_height; ++h) {
    for (int w = 0; w < parsing_width; ++w) {
      // int stride = h * width + w;
      flow_img.at<cv::Vec3b>(h, w)[0] = angle_data[id] * 180.0 / PI / 2.0;
      flow_img.at<cv::Vec3b>(h, w)[1] = magnitude_data[id];
      flow_img.at<cv::Vec3b>(h, w)[2] = 255;
      ++id;
    }
  }
  // 4. hsv2rgb
  cv::cvtColor(flow_img, flow_img, cv::COLOR_HSV2BGR);

  cv::Mat mat = out->img_mat;
  if (out->img_type == ImgType::YUV444) {
    int height = mat.rows;
    int width = mat.cols;
    cv::Mat bgr_mat(out->resized_h, out->resized_w, CV_8UC3);
    auto *data = bgr_mat.ptr<uint8_t>();

    for (int h = 0; h < out->resized_h; ++h) {
      auto *y_data = mat.ptr<int8_t>() + h * width;
      auto *u_data = y_data + height * width;
      auto *v_data = u_data + height * width;
      for (int w = 0; w < out->resized_w; ++w) {
        *data++ = static_cast<uint8_t>(std::max(std::min(*y_data + 128 + 2.03211 * *u_data, 255.), 0.));
        *data++ = static_cast<uint8_t>(std::max(std::min(*y_data + 128 - 0.39465 * *u_data - 0.58060 * *v_data, 255.), 0.));
        *data++ = static_cast<uint8_t>(std::max(std::min(*y_data + 128 + 1.13983 * *v_data, 255.), 0.));
        ++y_data;
        ++u_data;
        ++v_data;
      }
    }
    mat = std::move(bgr_mat);
  }

  cv::resize(flow_img, flow_img, mat.size());

  int width = flow.width;
  int height = flow.height;
  output = cv::Mat::zeros(height * 2, width, CV_8UC3);
  mat.copyTo(output(cv::Rect(0, 0, mat.cols, mat.rows)));
  flow_img.copyTo(output(cv::Rect(0, mat.rows, flow_img.cols, flow_img.rows)));

  // RCLCPP_INFO(rclcpp::get_logger("MobileSam"),
  //             "Draw result to file: %s",
  //             saving_path.c_str());
  // cv::imwrite("flow.jpeg", flow_img);
  return 0;
}