
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

#include "include/pwcnet_output_parser.h"

double const PI = 3.14159265;

int32_t PwcNetOutputParser::Parse(
    std::shared_ptr<DnnParserResult> &result,
    const int resized_img_h,
    const int resized_img_w,
    const int model_h,
    const int model_w,
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    float ratio) {
  if (!result) {
    result = std::make_shared<DnnParserResult>();
  }
  int ret = PostProcess(output_tensors, 
                        resized_img_h,
                        resized_img_w,
                        model_h,
                        model_w,
                        result->perception,
                        ratio);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PwcNetOutputParser"),
                "postprocess return error, code = %d",
                ret);
  }

  return ret;
}

int32_t PwcNetOutputParser::PostProcess(
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    const int resized_img_h,
    const int resized_img_w,
    const int model_h,
    const int model_w,
    Perception& perception,
    float ratio) {
  if (output_tensors.size() == 0) {
    return -1;
  }
  perception.type = Perception::FLOW;
  hbSysFlushMem(&(output_tensors[0]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

  // NCHW
  int height = output_tensors[0]->properties.alignedShape.dimensionSize[2];
  int width = output_tensors[0]->properties.alignedShape.dimensionSize[3];
  int channel = output_tensors[0]->properties.alignedShape.dimensionSize[1];
  auto scale = output_tensors[0]->properties.scale.scaleData;
  
  std::stringstream ss;
  ss << "[channel]: " << channel << ";  [height]: " << height
      << "; [width]: " << width
      << "; [quantiType]: " << output_tensors[0]->properties.quantiType
      << "; scale[0]: " << scale[0]
      << "; scale[1]: " << scale[1];
  RCLCPP_INFO(rclcpp::get_logger("mono_pwcnet"), "%s", ss.str().c_str());

  float valid_h_ratio = static_cast<float>(resized_img_h) / static_cast<float>(model_h);
  float valid_w_ratio = static_cast<float>(resized_img_w) / static_cast<float>(model_w);
  int valid_h = static_cast<int>(valid_h_ratio * height);
  int valid_w = static_cast<int>(valid_w_ratio * width);
  int stride = valid_w * valid_h;


  int32_t *data = reinterpret_cast<int32_t *>(output_tensors[0]->sysMem[0].virAddr);
  perception.flow.data.clear();
  perception.flow.data.resize(stride * 2);
  perception.flow.valid_w = valid_w;
  perception.flow.valid_h = valid_h;
  perception.flow.width = resized_img_w;
  perception.flow.height = resized_img_h;

  size_t x_offset = 0;
  size_t y_offset = static_cast<size_t>(stride);
  // parsing output data
  for (int h = 0; h < valid_h; ++h) {
    for (int w = 0; w < valid_w; ++w) {
      size_t x_id = h * width + w;
      size_t y_id = x_id + width * height;
      perception.flow.data[x_offset] = data[x_id] * scale[0];
      perception.flow.data[y_offset] = data[y_id] * scale[1];
      ++x_offset;
      ++y_offset;
    }
  }
  
  return 0;
}


int GetCombine(std::shared_ptr<PwcNetOutput> out, OpticalFlow &flow, cv::Mat &output) {
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

  // // 5. upx4
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