// Copyright (c) 2024，D-Robotics.
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
#include "include/image_proc.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "hobot_cv/hobotcv_imgproc.h"

// 使用hobotcv resize nv12格式图片，固定图片宽高比
int ImageProc::ResizeNV12Img(const char *in_img_data,
                  const int &in_img_height,
                  const int &in_img_width,
                  int &resized_img_height,
                  int &resized_img_width,
                  const int &scaled_img_height,
                  const int &scaled_img_width,
                  cv::Mat &out_img,
                  float &ratio) {
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void *)(in_img_data));
  float ratio_w =
      static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h =
      static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w) {
    resized_width = scaled_img_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  } else if (dst_ratio == ratio_h) {
    resized_width = static_cast<float>(in_img_width) / dst_ratio;
    resized_height = scaled_img_height;
  }
  // hobot_cv要求输出宽度为16的倍数
  int remain = resized_width % 16;
  if (remain != 0) {
    //向下取16倍数，重新计算缩放系数
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  //高度向下取偶数
  resized_height =
      resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  resized_img_height = resized_height;
  resized_img_width = resized_width;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

int32_t ImageProc::BGRToYUV444(cv::Mat &bgr_mat, cv::Mat &img_yuv444, int offset) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  if (height % 2 || width % 2) {
    std::cerr << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv420sp_mat;
  cv::cvtColor(bgr_mat, yuv420sp_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv420sp_mat.data == nullptr) {
    std::cerr << "yuv420sp_mat.data is null pointer" << std::endl;
    return -1;
  }
  // 分别提取YUV420SP中的Y, U, 和V分量
  auto *yuv = yuv420sp_mat.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  uint8_t *y_data = yuv;
  uint8_t *u_data = yuv + height * width;
  uint8_t *v_data = u_data + uv_height * uv_width;

  img_yuv444.create(cv::Size(width, height), CV_8SC3);
  // 合并上采样后的U和V分量到Y分量
  auto *y_444 = img_yuv444.ptr<int8_t>();
  auto *u_444 = y_444 + height * width;
  auto *v_444 = u_444 + height * width;
  for (int32_t h = 0; h < height; ++h) {
    for (int32_t w = 0; w < width; ++w) {
      *y_444++ = static_cast<int8_t>(static_cast<int>(*y_data++) + offset);
      auto id = ((h / 2) * uv_width + w / 2);
      *u_444++ = static_cast<int8_t>(static_cast<int>(u_data[id]) + offset);
      *v_444++ = static_cast<int8_t>(static_cast<int>(v_data[id]) + offset);
    }
  }

  return 0;
}

int32_t ImageProc::Nv12ToYUV444(cv::Mat &img_nv12, 
                                const int &height, 
                                const int &width,
                                cv::Mat &img_yuv444, 
                                int offset) {

  if (height % 2 || width % 2) {
    std::cerr << "input img height and width must aligned by 2!";
    return -1;
  }

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  uint8_t *y_data = img_nv12.data;
  uint8_t *uv_data = y_data + height * width;
  // uint8_t *v_data = u_data + uv_height * uv_width;

  img_yuv444.create(cv::Size(width, height), CV_8SC3);
  // // 合并上采样后的U和V分量到Y分量
  auto *y_444 = img_yuv444.ptr<int8_t>();
  auto *u_444 = y_444 + height * width;
  auto *v_444 = u_444 + height * width;
  for (int32_t h = 0; h < height; ++h) {
    for (int32_t w = 0; w < width; ++w) {
      *y_444++ = static_cast<int8_t>(static_cast<int>(*y_data++) + offset);
      auto id = ((h / 2) * uv_width + w / 2) * 2;
      *u_444++ = static_cast<int8_t>(static_cast<int>(uv_data[id]) + offset);
      *v_444++ = static_cast<int8_t>(static_cast<int>(uv_data[id + 1]) + offset);
    }
  }
  
  return 0;
}