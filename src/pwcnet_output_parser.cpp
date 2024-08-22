
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

  // RCLCPP_ERROR(rclcpp::get_logger("MyDebug"),
  //              "resized_img_h: %d, resized_img_w: %d, model_h: %d, model_w: %d",
  //             resized_img_h, resized_img_w, model_h, model_w);

  size_t x_offset = 0;
  size_t y_offset = static_cast<size_t>(stride);
  // parsing output data
  for (int h = 0; h < valid_h; ++h) {
    for (int w = 0; w < valid_w; ++w) {
      // int offset = c * (height * width) + h * width + w;
      // // perception->pt.point.push_back(data[offset] * scale[c] * 4.0);
      // perception.flow.data[x_offset] = {data[x_offset] * scale[0], data[y_offset] * scale[1]};
      size_t x_id = h * width + w;
      size_t y_id = x_id + width * height;
      perception.flow.data[x_offset] = data[x_id] * scale[0];
      perception.flow.data[y_offset] = data[y_id] * scale[1];
      // RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
      //   "x_vector: %f, y_vector: %f",
      //   perception.flow.data[x_offset], perception.flow.data[y_offset]);
      ++x_offset;
      ++y_offset;
    }
  }
  // ss.clear();
  // ss << "PwcNet parse finished, predict result: "
  //    << perception;
  
  return 0;
}

// int32_t PwcNetOutputParser::GenMask(const float* mask,
//                                         const int resized_img_h,
//                                         const int resized_img_w,
//                                         const int model_h,
//                                         const int model_w,
//                                         Perception& perception,
//                                         float ratio) {

//   perception.type = Perception::SEG;

//   int channel = 4;

//   float valid_h_ratio = static_cast<float>(resized_img_h) / static_cast<float>(model_h);
//   float valid_w_ratio = static_cast<float>(resized_img_w) / static_cast<float>(model_w);

//   int valid_h = static_cast<int>(valid_h_ratio * output_height_);
//   int valid_w = static_cast<int>(valid_w_ratio * output_width_);

//   int stride = channel * output_height_ * output_width_;
//   std::vector<cv::Mat> parsing_imgs;
//   for (int n = 0; n < num_classes_; n++) {
//     cv::Mat parsing_img(valid_h, valid_w, CV_32FC1);
//     float *parsing_img_ptr = parsing_img.ptr<float>();
//     for (int h = 0; h < valid_h; h++) {
//       for (int w = 0; w < valid_w; w++) {
//         int offect = h * output_width_ + w;
//         const float* data = mask + n * stride + offect;
//         *parsing_img_ptr++ = data[0];
//       }
//     }
//     parsing_imgs.push_back(parsing_img);
//   }

//   valid_h = static_cast<int>(static_cast<float>(resized_img_h) * ratio);
//   valid_w = static_cast<int>(static_cast<float>(resized_img_w) * ratio);
//   cv::Size size(valid_w, valid_h);

//   for (auto &parsing_img: parsing_imgs) {
//     // resize parsing image
//     cv::resize(parsing_img, parsing_img, size, 0, 0, cv::INTER_LINEAR);
//   }

//   perception.seg.data.resize(valid_h * valid_w);
//   perception.seg.seg.resize(valid_h * valid_w);

//   perception.seg.valid_h = valid_h;
//   perception.seg.valid_w = valid_w;
//   perception.seg.height = static_cast<int>(model_h * valid_h_ratio);
//   perception.seg.width = static_cast<int>(model_w * valid_w_ratio);
//   perception.seg.channel = channel;
//   perception.seg.num_classes = num_classes_ + 1;

//   for (int n = 0; n < num_classes_; n++) {
//     auto &parsing_img = parsing_imgs[n];
//     float *parsing_img_ptr = parsing_img.ptr<float>();
//     for (int h = 0; h < valid_h; h++) {
//       for (int w = 0; w < valid_w; w++) {
//         int offect = h * valid_w + w;
//         int top_index = -1;
//         if (n == 0) {
//           top_index = 0;
//         }
//         if (*parsing_img_ptr++ > mask_threshold_) {
//           top_index = n + 1;
//         }
//         if (top_index != -1) {
//           perception.seg.seg[h * valid_w + w] = top_index;
//           perception.seg.data[h * valid_w + w] = static_cast<float>(top_index);
//         }  
//       }
//     }
//   }
//   return 0;
// }


// int RenderSeg(cv::Mat &mat, OpticalFlow &flow, std::string& saving_path) {
//   static uint8_t bgr_putpalette[] = {
//       0, 0, 0, 128, 64,  128, 244, 35,  232, 70,  70,  70,  102, 102, 156, 190, 153, 153,
//       153, 153, 153, 250, 170, 30,  220, 220, 0,   107, 142, 35,  152, 251, 152,
//       0,   130, 180, 220, 20,  60,  255, 0,   0,   0,   0,   142, 0,   0,   70,
//       0,   60,  100, 0,   80,  100, 0,   0,   230, 119, 11,  32};

//   int parsing_width = flow.valid_w;
//   int parsing_height = flow.valid_h;
//   int stride = parsing_width * parsing_height;
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place1, stride: %d",
//     stride);
//   float *flow_data = flow.data.data();
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place1.1");
//   std::vector<float> flow_x;
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place1.11");
//   flow_x.reserve(stride);
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place1.2");
//   std::vector<float> flow_y;
//   flow_y.reserve(stride);
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place1.3");
//   for (int i = 0; i < stride; ++i) {
//     flow_x.emplace_back(flow_data[i] * 4.0);
//     flow_y.emplace_back(flow_data[stride + i] * 4.0);
//   }
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place2");
//   cv::Mat magnitude, angle;
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place2.1, size_x: %d, size_y: %d",
//     flow_x.size(), flow_y.size());
//   cv::cartToPolar(flow_x, flow_y, magnitude, angle);
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place2.2");
//   float *magnitude_data = reinterpret_cast<float *>(magnitude.data);
//   if (magnitude_data == nullptr) {
//     RCLCPP_INFO(rclcpp::get_logger("mono_pwcnet"), 
//                 "magnitude_data is null pointer");
//     return -1;
//   }
//   float *angle_data = reinterpret_cast<float *>(angle.data);
//   // 1. magnitude: nan check
//   for (int i = 0; i < stride; ++i) {
//     magnitude_data[i] =
//         std::isnan(magnitude_data[i]) ? 0.0f : magnitude_data[i];
//   }
//   // 2. magnitude: normalize
//   cv::normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX);
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place3");
//   // 3. flow2img
//   cv::Mat flow_img(parsing_height, parsing_width, CV_8UC3);
//   int id = 0;
//   for (int h = 0; h < parsing_height; ++h) {
//     for (int w = 0; w < parsing_width; ++w) {
//       // int stride = h * width + w;
//       flow_img.at<cv::Vec3b>(h, w)[0] = angle_data[id] * 180.0 / PI / 2.0;
//       flow_img.at<cv::Vec3b>(h, w)[1] = magnitude_data[id];
//       flow_img.at<cv::Vec3b>(h, w)[2] = 255;
//       ++id;
//     }
//   }
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place4");
//   // 4. hsv2rgb
//   cv::cvtColor(flow_img, flow_img, cv::COLOR_HSV2BGR);
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place5");
//   // 5. upx4
//   cv::resize(flow_img, flow_img, mat.size());
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place6");

//   int width = flow.width;
//   int height = flow.height;
//   cv::Mat render_img = cv::Mat::zeros(height * 2, width, CV_8UC3);
//   mat.copyTo(render_img(cv::Rect(0, 0, mat.cols, mat.rows)));
//   flow_img.copyTo(render_img(cv::Rect(0, mat.rows, flow_img.cols, flow_img.rows)));
//   RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
//     "place7");

//   RCLCPP_INFO(rclcpp::get_logger("MobileSam"),
//               "Draw result to file: %s",
//               saving_path.c_str());
//   cv::imwrite(saving_path, render_img);
//   return 0;
// }


int GetCombine(cv::Mat &mat, OpticalFlow &flow, cv::Mat &output) {
  int parsing_width = flow.valid_w;
  int parsing_height = flow.valid_h;
  int stride = parsing_width * parsing_height;
  float *flow_data = flow.data.data();
  // RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
  //   "test1: %f",
  //   flow_data[0]);
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
  // RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
  //   "test1: %f",
  //   angle_data[0]);
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
  // 5. upx4
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