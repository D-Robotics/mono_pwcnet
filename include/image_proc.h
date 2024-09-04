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

#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include <memory>
#include <string>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

class ImageProc {
    public:

    static int ResizeNV12Img(const char *in_img_data,
                  const int &in_img_height,
                  const int &in_img_width,
                  int &resized_img_height,
                  int &resized_img_width,
                  const int &scaled_img_height,
                  const int &scaled_img_width,
                  cv::Mat &out_img,
                  float &ratio);

    static int32_t BGRToYUV444(cv::Mat &bgr_mat, cv::Mat &img_yuv444, int offset = 0);

    static int32_t Nv12ToYUV444(cv::Mat &img_nv12, 
                        const int &height, 
                        const int &width,
                        cv::Mat &img_yuv444, 
                        int offset);
};



#endif