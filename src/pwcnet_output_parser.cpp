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

int32_t PwcNetOutputParser::Parse(
    std::shared_ptr<OpticalFlow> &result,
    const int resized_img_h,
    const int resized_img_w,
    const int model_h,
    const int model_w,
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    float ratio) {
  if (!result) {
    result = std::make_shared<OpticalFlow>();
  }
  int ret = PostProcess(output_tensors, 
                        resized_img_h,
                        resized_img_w,
                        model_h,
                        model_w,
                        result,
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
    std::shared_ptr<OpticalFlow> &result,
    float ratio) {
  if (output_tensors.size() == 0) {
    return -1;
  }
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
  result->data.clear();
  result->data.resize(stride * 2);
  result->valid_w = valid_w;
  result->valid_h = valid_h;
  result->width = resized_img_w;
  result->height = resized_img_h;

  size_t x_offset = 0;
  size_t y_offset = static_cast<size_t>(stride);
  // parsing output data
  for (int h = 0; h < valid_h; ++h) {
    for (int w = 0; w < valid_w; ++w) {
      size_t x_id = h * width + w;
      size_t y_id = x_id + width * height;
      result->data[x_offset] = data[x_id] * scale[0];
      result->data[y_offset] = data[y_id] * scale[1];
      ++x_offset;
      ++y_offset;
    }
  }
  
  return 0;
}