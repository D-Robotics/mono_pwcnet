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

#include <fstream>
#include <math.h>
#include <memory>
#include <unistd.h>
#include <utility>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "dnn/hb_sys.h"
#include "include/pwcnet_node.h"

builtin_interfaces::msg::Time ConvertToRosTime(
    const struct timespec& time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start,
                      const builtin_interfaces::msg::Time& end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}


template<typename T>
int readbinary(const std::string &filename, const T* &dataOut) {
  std::cout << "readbinary: " << filename + ".bin" << std::endl;
  std::ifstream ifs(filename + ".bin", std::ios::in | std::ios::binary);
  if (!ifs) {
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  int len = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char* data = new char[len];
  ifs.read(data, len);
  dataOut = reinterpret_cast<const T *>(data);
  return len / sizeof(T);
}

PwcNetNode::PwcNetNode(const std::string& node_name,
                               const NodeOptions& options)
    : DnnNode(node_name, options) {
  this->declare_parameter<int>("cache_img_limit", cache_img_limit_);
  this->declare_parameter<int>("cache_task_limit", cache_task_limit_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<int>("image_type", image_type_);
  this->declare_parameter<std::vector<std::string>>("image_file", image_file_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name",
                                       ai_msg_pub_topic_name_);
  this->declare_parameter<std::string>("ros_img_sub_topic_name",
                                       ros_img_sub_topic_name_);
  this->declare_parameter<std::string>("flow_img_pub_topic_name",
                                       flow_img_pub_topic_name_);

  this->get_parameter<int>("cache_img_limit", cache_img_limit_);
  this->get_parameter<int>("cache_task_limit", cache_task_limit_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::vector<std::string>>("image_file", image_file_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name",
                                   ai_msg_pub_topic_name_);
  this->get_parameter<std::string>("ros_img_sub_topic_name",
                                   ros_img_sub_topic_name_);
  this->get_parameter<std::string>("flow_img_pub_topic_name",
                                   flow_img_pub_topic_name_);

  std::stringstream ss;
  ss << "Parameter:"
     << "\n cache_img_limit: " << cache_img_limit_
     << "\n cache_task_limit: " << cache_task_limit_
     << "\n dump_render_img: " << dump_render_img_
     << "\n feed_type(0:local, 1:sub): " << feed_type_
     << "\n image_size: " << image_file_.size()
     << "\n is_shared_mem_sub: " << is_shared_mem_sub_
     << "\n is_sync_mode: " << is_sync_mode_
     << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_
     << "\n ros_img_sub_topic_name: " << ros_img_sub_topic_name_
     << "\n flow_img_pub_topic_name_: " << flow_img_pub_topic_name_;
  RCLCPP_WARN(rclcpp::get_logger("mono_pwcnet"), "%s", ss.str().c_str());

  if(SetNodePara() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), "Init Node Para Failed!");
    return;
  }
  // 使用基类接口初始化，加载模型
  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_yolo_world"), "Init failed!");
    rclcpp::shutdown();
    return;
  }
  // 未指定模型名，从加载的模型中查询出模型名
  if (model_name_.empty()) {
    if (!GetModel()) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_yolo_world"), "Get model fail.");
    } else {
      model_name_ = GetModel()->GetName();
      RCLCPP_WARN(rclcpp::get_logger("hobot_yolo_world"), "Get model name: %s from load model.", model_name_.c_str());
    }
  }
  // 加载模型后查询模型输入分辨率
  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Get model input size fail!");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "The model input width is %d and height is %d",
                model_input_width_,
                model_input_height_);
  }

  if (0 == feed_type_) {
    dump_render_img_ = 1;
    FeedFromLocal();
  } else {
    // ai_msg_manage_ = std::make_shared<AiMsgManage>();
    // ai_msg_subscription_ =
    //     this->create_subscription<ai_msgs::msg::PerceptionTargets>(
    //         ai_msg_sub_topic_name_,
    //         10,
    //         std::bind(
    //             &PwcNetNode::AiMsgProcess, this, std::placeholders::_1));
                
    msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        ai_msg_pub_topic_name_, 10);
    
    img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        flow_img_pub_topic_name_, 10);

    predict_task_ = std::make_shared<std::thread>(
        std::bind(&PwcNetNode::RunPredict, this));

    if (is_shared_mem_sub_) {
#ifdef SHARED_MEM_ENABLED
      RCLCPP_WARN(rclcpp::get_logger("mono_pwcnet"),
                  "Create hbmem_subscription with topic_name: %s",
                  sharedmem_img_topic_name_.c_str());
      sharedmem_img_subscription_ =
          this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
              sharedmem_img_topic_name_,
              rclcpp::SensorDataQoS(),
              std::bind(&PwcNetNode::SharedMemImgProcess,
                        this,
                        std::placeholders::_1));
#else
      RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), "Unsupport shared mem");
#endif
    } else {
      RCLCPP_WARN(rclcpp::get_logger("mono_pwcnet"),
                  "Create subscription with topic_name: %s",
                  ros_img_sub_topic_name_.c_str());
      ros_img_subscription_ =
          this->create_subscription<sensor_msgs::msg::Image>(
              ros_img_sub_topic_name_,
              10,
              std::bind(
                  &PwcNetNode::RosImgProcess, this, std::placeholders::_1));
    }
  }

}

PwcNetNode::~PwcNetNode() {
  for (auto &packed_dnn_handle: packed_dnn_handles_) {
    hbDNNRelease(packed_dnn_handle);
  }
}


int PwcNetNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("hobot_yolo_world"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type =
      hobot::dnn_node::ModelTaskType::ModelInferType;
  dnn_node_para_ptr_->task_num = task_num_;

  RCLCPP_WARN(rclcpp::get_logger("mono_pwcnet"),
              "model_file_name_: %s, task_num: %d",
              model_file_name_.data(),
              dnn_node_para_ptr_->task_num);

  return 0;
}

int PwcNetNode::RunMulti(std::vector<std::shared_ptr<DNNTensor>>& inputs,
                      const std::shared_ptr<DnnNodeOutput> &output,
                      const bool is_sync_mode) {

  output->rt_stat = std::make_shared<hobot::dnn_node::DnnNodeRunTimeStat>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  output->rt_stat->infer_timespec_start = time_now;

  int32_t const input_size{static_cast<int32_t>(inputs.size())};
  std::vector<hbDNNTensor> input_dnn_tensors;
  input_dnn_tensors.resize(static_cast<size_t>(input_size));     
  for (int32_t i{0}; i < input_size; ++i) {
    if (inputs[i] == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), 
        "inputs[%d] is null", i);
      return -1;
    }
    input_dnn_tensors[i] = static_cast<hbDNNTensor>(*(inputs[i]));
  }

  // 1. mobile pwcnet encode
  std::vector<hbDNNTensor> encode_inputs;
  encode_inputs.push_back(input_dnn_tensors[0]);
  std::vector<hbDNNTensor> encode_outputs;
  Infer(encode_inputs, encode_outputs, 0);

  // 2 mobile pwcnet decode
  std::vector<hbDNNTensor> decode_inputs;
  std::vector<hbDNNTensor> decode_outputs;
  decode_inputs.push_back(input_dnn_tensors[1]);
  decode_inputs.push_back(encode_outputs[0]);
  Infer(decode_inputs, decode_outputs, 1);
  hbSysFreeMem(&(encode_outputs[0].sysMem[0]));

  std::vector<hbDNNTensor> output_dnn_tensors = decode_outputs;
  int32_t const output_size{static_cast<int32_t>(output_dnn_tensors.size())};
  output->output_tensors.resize(static_cast<size_t>(output_size));     
  for (int32_t i{0}; i < output_size; ++i) {
    output->output_tensors[i] = std::make_shared<DNNTensor>(output_dnn_tensors[i]);
  }

  clock_gettime(CLOCK_REALTIME, &time_now);
  output->rt_stat->infer_timespec_end = time_now;
  output->rt_stat->fps_updated = true;
  auto time_start = ConvertToRosTime(output->rt_stat->infer_timespec_start);
  auto time_end = ConvertToRosTime(output->rt_stat->infer_timespec_end);
  output->rt_stat->infer_time_ms = CalTimeMsDuration(time_start, time_end);

  if (is_sync_mode) {
    threadPool.enqueue([&, output] {
      PostProcess(output);
    });
  } else {
    PostProcess(output);
  }
  return 0;
}


int PwcNetNode::Infer(std::vector<hbDNNTensor>& inputs,
                          std::vector<hbDNNTensor>& outputs,
                          int idx) {
  // 1. 获取dnn_handle
  hbDNNHandle_t dnn_handle = models_[idx]->GetDNNHandle();

  // 2. 准备模型输出数据的空间
  int output_count;
  hbDNNGetOutputCount(&output_count, dnn_handle);
  for (int i = 0; i < output_count; i++) {
    hbDNNTensor output;
    hbDNNTensorProperties &output_properties = output.properties;
    hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);
    int out_aligned_size = output_properties.alignedByteSize;
    hbSysMem &mem = output.sysMem[0];
    hbSysAllocCachedMem(&mem, out_aligned_size);
    outputs.push_back(output);
  }

  auto *output_ptr{outputs.data()};
  // 3. 推理模型
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
  hbDNNInfer(&task_handle,
              &output_ptr,
              inputs.data(),
              dnn_handle,
              &infer_ctrl_param);

  // 4. 等待任务结束
  hbDNNWaitTaskDone(task_handle, 0);

  // 释放任务
  hbDNNReleaseTask(task_handle);

  return 0;
}


int PwcNetNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  // RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
  //   "In PostProcess");

  auto parser_output = std::dynamic_pointer_cast<PwcNetOutput>(node_output);
  if (!parser_output) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), "Invalid node output");
    return -1;
  }

  // 1. 获取后处理开始时间
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);

  // 2. 模型后处理解析
  auto det_result = std::make_shared<DnnParserResult>();
  output_parser_->Parse(det_result,
                parser_output->resized_h,
                parser_output->resized_w,
                model_input_height_,
                model_input_width_,
                parser_output->output_tensors);
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  pub_data->header.set__stamp(parser_output->msg_header->stamp);
  pub_data->header.set__frame_id(parser_output->msg_header->frame_id);
  // 如果开启了渲染，本地渲染并存储图片
  cv::Mat combined_img;
  if (!parser_output->bgr_mat.empty()) {
    GetCombine(parser_output->bgr_mat, det_result->perception.flow, combined_img);
    if (dump_render_img_ == 1) {
      std::string saving_path = "render_pwcnet_" + pub_data->header.frame_id + "_" +
                            std::to_string(pub_data->header.stamp.sec) + "_" +
                            std::to_string(pub_data->header.stamp.nanosec) +
                            ".jpeg";
      RCLCPP_INFO(rclcpp::get_logger("MobileSam"),
              "Draw result to file: %s",
              saving_path.c_str());
      cv::imwrite(saving_path, combined_img);
    }
  }
  if (feed_type_ == 0) {
    return 0;
  }

  // if (!combined_img.empty()) {
  //   sensor_msgs::msg::Image visual_img_msg;
  //   auto img_bridge = cv_bridge::CvImage(pub_data->header,
  //                                        "bgr8", 
  //                                        combined_img.clone()).toImageMsg();
  //   img_publisher_->publish(*img_bridge);
  // }

  // 3. 创建用于发布的AI消息
  if (!msg_publisher_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid msg_publisher_");
    return -1;
  }
  
  // 3.1 发布光流AI消息
  auto &flow = det_result->perception.flow;
  if (flow.height != 0 && flow.width != 0) {
    ai_msgs::msg::Capture capture;
    capture.features.swap(flow.data);
    capture.img.height = flow.valid_h;
    capture.img.width = flow.valid_w;

    capture.img.step = static_cast<int>(std::round(static_cast<float>(flow.height) / 
                                        static_cast<float>(flow.valid_h)));

    RCLCPP_INFO(rclcpp::get_logger("mono_pwcnet"),
                "features size: %d, width: %d, height: %d, step: %d",
                capture.features.size(),
                capture.img.width,
                capture.img.height,
                capture.img.step);

    ai_msgs::msg::Target target;
    target.set__type("optical_flow");
    
    ai_msgs::msg::Attribute attribute;
    attribute.set__type("optical_flow_attribute");
    target.attributes.emplace_back(std::move(attribute));

    target.captures.emplace_back(std::move(capture));
    pub_data->targets.emplace_back(std::move(target));
  }

  pub_data->header.set__stamp(parser_output->msg_header->stamp);
  pub_data->header.set__frame_id(parser_output->msg_header->frame_id);

  // 填充perf性能统计信息
  // 前处理统计
  ai_msgs::msg::Perf perf_preprocess = std::move(parser_output->perf_preprocess);
  perf_preprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_preprocess.stamp_start, perf_preprocess.stamp_end));
  pub_data->perfs.push_back(perf_preprocess);

  // predict
  if (parser_output->rt_stat) {
    ai_msgs::msg::Perf perf;
    perf.set__type(model_name_ + "_predict_infer");
    perf.set__stamp_start(
        ConvertToRosTime(parser_output->rt_stat->infer_timespec_start));
    perf.set__stamp_end(
        ConvertToRosTime(parser_output->rt_stat->infer_timespec_end));
    perf.set__time_ms_duration(parser_output->rt_stat->infer_time_ms);
    pub_data->perfs.push_back(perf);
  }

  ai_msgs::msg::Perf perf_postprocess;
  perf_postprocess.set__type(model_name_ + "_postprocess");
  perf_postprocess.set__stamp_start(ConvertToRosTime(time_now));
  clock_gettime(CLOCK_REALTIME, &time_now);
  perf_postprocess.set__stamp_end(ConvertToRosTime(time_now));
  perf_postprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_postprocess.stamp_start, perf_postprocess.stamp_end));
  pub_data->perfs.emplace_back(perf_postprocess);

  // 如果当前帧有更新统计信息，输出统计信息
  if (parser_output->rt_stat->fps_updated) {
      RCLCPP_WARN(rclcpp::get_logger("mono_pwcnet"),
                  "pre process time ms: %d, "
                  "infer time ms: %d, "
                  "post process time ms: %d",
                  static_cast<int>(perf_preprocess.time_ms_duration),
                  parser_output->rt_stat->infer_time_ms,
                  static_cast<int>(perf_postprocess.time_ms_duration));
  }
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

void PwcNetNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg) {
    RCLCPP_DEBUG(rclcpp::get_logger("mono_pwcnet"), "Get img failed");
    return;
  }

  if (!rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "_"
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("mono_pwcnet"), "%s", ss.str().c_str());

  auto dnn_output = std::make_shared<PwcNetOutput>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;

  // // 1. 将图片处理成模型输入数据类型DNNTensor
  auto model = GetModel();
  hbDNNTensorProperties tensor_properties;
  model->GetInputTensorProperties(tensor_properties, 0);
  if ("nv12" ==
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
    cv::Mat cur_img;
    hobot::dnn_node::ImageProc::Nv12ToBGR(reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height, img_msg->width, cur_img);
    std::unique_lock<std::mutex> img_l(mtx_img_);
    if (cache_img_.size() > cache_img_limit_) {
      cache_img_.pop();
    }
    cache_img_.push(cur_img);
    img_l.unlock();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"),
                 "Unsupported img encoding: %s, only nv12 img encoding is "
                 "supported for shared mem.",
                 img_msg->encoding.data());
    return;
  }

  std::shared_ptr<DNNTensor> input_tensor = nullptr;
  std::unique_lock<std::mutex> img_l(mtx_img_);
  cv::Mat img1, img2;
  if (cache_img_.size() > 1) {
    img1 = cache_img_.front();
    cache_img_.pop();
    img2 = cache_img_.front();
    input_tensor = ImagePreprocesss(
        img1, 
        img2, 
        dnn_output->resized_h,
        dnn_output->resized_w,
        model_input_height_,
        model_input_width_,
        tensor_properties);
  }
  img_l.unlock();

  if (!input_tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), "Get Tensor fail");
    return;
  }

  dnn_output->bgr_mat = std::move(img1);

  // // 2. 创建推理输出数据
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;
  dnn_output->resized_h = dnn_output->bgr_mat.rows / dnn_output->ratio;
  dnn_output->resized_w = dnn_output->bgr_mat.cols / dnn_output->ratio;

  // 3. 将准备好的输入输出数据存进缓存
  std::unique_lock<std::mutex> lg(mtx_task_);
  if (cache_task_.size() > cache_task_limit_) {
    CacheTaskType img_msg = cache_task_.front();
    cache_task_.pop();
    auto drop_dnn_output = img_msg.first;
    std::string ts =
        std::to_string(drop_dnn_output->msg_header->stamp.sec) + "." +
        std::to_string(drop_dnn_output->msg_header->stamp.nanosec);
    RCLCPP_INFO(rclcpp::get_logger("mono_pwcnet"),
                "drop cache_task_ ts %s",
                ts.c_str());
  }
  CacheTaskType cache_task = std::make_pair<std::shared_ptr<PwcNetOutput>,
                                          std::shared_ptr<DNNTensor>>(
      std::move(dnn_output), std::move(input_tensor));
  cache_task_.push(cache_task);
  cv_img_.notify_one();
  lg.unlock();
}


#ifdef SHARED_MEM_ENABLED
void PwcNetNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step << ", index: " << img_msg->index
     << ", stamp: " << img_msg->time_stamp.sec << "_"
     << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(rclcpp::get_logger("mono_pwcnet"), "%s", ss.str().c_str());

  auto tp_start = std::chrono::system_clock::now();

  auto dnn_output = std::make_shared<PwcNetOutput>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;

  // 1. 将图片处理成模型输入数据类型DNNTensor
  auto model = GetModel();
  hbDNNTensorProperties tensor_properties;
  model->GetInputTensorProperties(tensor_properties, 0);
  // if ("nv12" ==
  //     std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
  //   hobot::dnn_node::ImageProc::Nv12ToBGR(reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height, img_msg->width, cur_img);
  //   if (pre_img.empty()) {
  //     pre_img = cur_img.clone();
  //     input_tensor = ImagePreprocesss(
  //         pre_img, 
  //         cur_img, 
  //         dnn_output->resized_h,
  //         dnn_output->resized_w,
  //         model_input_height_,
  //         model_input_width_,
  //         tensor_properties);
  //   } else {
  //     input_tensor = ImagePreprocesss(
  //         pre_img, 
  //         cur_img, 
  //         dnn_output->resized_h,
  //         dnn_output->resized_w,
  //         model_input_height_,
  //         model_input_width_,
  //         tensor_properties);
  //   }
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"),
  //                "Unsupported img encoding: %s, only nv12 img encoding is "
  //                "supported for shared mem.",
  //                img_msg->encoding.data());
  //   return;
  // }
  if ("nv12" ==
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
    cv::Mat cur_img;
    hobot::dnn_node::ImageProc::Nv12ToBGR(reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height, img_msg->width, cur_img);
    std::unique_lock<std::mutex> img_l(mtx_img_);
    if (cache_img_.size() > cache_img_limit_) {
      cache_img_.pop();
    }
    cache_img_.push(cur_img);
    img_l.unlock();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"),
                 "Unsupported img encoding: %s, only nv12 img encoding is "
                 "supported for shared mem.",
                 img_msg->encoding.data());
    return;
  }

  std::shared_ptr<DNNTensor> input_tensor = nullptr;
  std::unique_lock<std::mutex> img_l(mtx_img_);
  cv::Mat img1, img2;
  if (cache_img_.size() > 1) {
    img1 = cache_img_.front();
    cache_img_.pop();
    img2 = cache_img_.front();
    input_tensor = ImagePreprocesss(
        img1, 
        img2, 
        dnn_output->resized_h,
        dnn_output->resized_w,
        model_input_height_,
        model_input_width_,
        tensor_properties);
  }
  img_l.unlock();

  if (!input_tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), "Get Tensor fail");
    return;
  }

  dnn_output->bgr_mat = std::move(img1);

  // // 2. 创建推理输出数据
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->msg_header->set__stamp(img_msg->time_stamp);
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;
  dnn_output->resized_h = dnn_output->bgr_mat.rows / dnn_output->ratio;
  dnn_output->resized_w = dnn_output->bgr_mat.cols / dnn_output->ratio;

  // auto inputs = std::vector<std::shared_ptr<DNNTensor>>{input_tensor};

  // // 3. 开始预测
  // if (Run(inputs, dnn_output, true) != 0) {
  //   RCLCPP_ERROR(rclcpp::get_logger("example"), "Run predict failed!");
  //   return;
  // }

  // {
  //   auto tp_now = std::chrono::system_clock::now();
  //   auto interval =
  //       std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
  //           .count();
  //   RCLCPP_DEBUG(
  //       rclcpp::get_logger("example"), "after Predict cost ms: %d", interval);
  // }

  // 3. 将准备好的输入输出数据存进缓存
  std::unique_lock<std::mutex> lg(mtx_task_);
  if (cache_task_.size() > cache_task_limit_) {
    CacheTaskType img_msg = cache_task_.front();
    cache_task_.pop();
    auto drop_dnn_output = img_msg.first;
    std::string ts =
        std::to_string(drop_dnn_output->msg_header->stamp.sec) + "." +
        std::to_string(drop_dnn_output->msg_header->stamp.nanosec);
    RCLCPP_INFO(rclcpp::get_logger("mono_pwcnet"),
                "drop cache_task_ ts %s",
                ts.c_str());
  }
  CacheTaskType cache_task = std::make_pair<std::shared_ptr<PwcNetOutput>,
                                          std::shared_ptr<DNNTensor>>(
      std::move(dnn_output), std::move(input_tensor));
  cache_task_.push(cache_task);
  cv_img_.notify_one();
  lg.unlock();
}
#endif

int PwcNetNode::FeedFromLocal() {

  // RCLCPP_ERROR(rclcpp::get_logger("MyDebug"), 
  //     "file1: %s or file2: %s", 
  //     image_file_[0].c_str(), image_file_[1].c_str());

  if (access(image_file_[0].c_str(), R_OK) == -1 ||  access(image_file_[1].c_str(), R_OK) == -1) {
    RCLCPP_ERROR(
        rclcpp::get_logger("mono_pwcnet"), 
                           "Image: %s or Image: %s not exist!", 
                           image_file_[0].c_str(), image_file_[1].c_str());
    return -1;
  }

  auto model = GetModel();
  hbDNNTensorProperties tensor_properties;
  model->GetInputTensorProperties(tensor_properties, 0);

  auto dnn_output = std::make_shared<PwcNetOutput>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id("feedback");

  std::shared_ptr<DNNTensor> input_tensor = nullptr;

  if (static_cast<int>(ImageType::BGR) == image_type_) {
    if (image_file_.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"),
                   "Invalid file size: %d, the size should be 2", image_file_.size());
      return -1;
    }
    if (access(image_file_[0].c_str(), R_OK) == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), 
                   "Image: %s not exist!", 
                   image_file_[0].c_str());
      return -1;
    }
    if (access(image_file_[1].c_str(), R_OK) == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), 
                   "Image: %s not exist!", 
                   image_file_[1].c_str());
      return -1;
    }

    cv::Mat bgr_mat1 = cv::imread(image_file_[0], cv::IMREAD_COLOR);
    if (bgr_mat1.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"),
                "Get tensor fail with image: %s",
                image_file_[0].c_str());
      return -1;
    }

    cv::Mat bgr_mat2 = cv::imread(image_file_[1], cv::IMREAD_COLOR);
    if (bgr_mat2.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"),
                "Get tensor fail with image: %s",
                image_file_[1].c_str());
      return -1;
    }

    input_tensor = ImagePreprocesss(
          bgr_mat1, 
          bgr_mat2, 
          dnn_output->resized_h,
          dnn_output->resized_w,
          model_input_height_,
          model_input_width_,
          tensor_properties);
    if (!input_tensor) {
      RCLCPP_ERROR(rclcpp::get_logger("pwcnet"),
                   "Get tensor fail with image");
      return -1;
    }

    dnn_output->bgr_mat = std::move(bgr_mat1);

  } else if (static_cast<int>(ImageType::BIN) == image_type_) {
    // TODO
  } else {
    RCLCPP_ERROR(
        this->get_logger(), "Invalid image type: %d", image_type_);
    return -1;
  }

  // 2. 创建DNNTensor对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNTensor>>{input_tensor};
  dnn_output->model_w = model_input_width_;
  dnn_output->model_h = model_input_height_;
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id("feedback");

  // 3. 开始预测
  if (Run(inputs, dnn_output, true) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_pwcnet"), "Run Model Error");
    return -1;
  }

  return 0;
}



int PwcNetNode::Debug() {

  std::vector<std::vector<float>> boxes =   
      {{69.9, 166.20001, 264.30002, 203.70001}};

  const float* mask;

  readbinary("/mnt/ai_toolchain/models/pwcnet/orlresult", mask);
  auto parser = std::make_shared<PwcNetOutputParser>(96, 96, 5, 0.1);
  auto det_result = std::make_shared<hobot::dnn_node::output_parser::DnnParserResult>();

  Perception perception;
  float ratio = 10.0 / 3;
  // parser->GenMask(mask, 288, 384, 384, 384, perception);

  cv::Mat bgr_mat = cv::imread(image_file_[0], cv::IMREAD_COLOR);

  std::string saving_path = "/mnt/testdata/render_pwcnet_0.jpeg";
  // RenderSeg(bgr_mat, perception.flow, saving_path);

  return 0;
}

// void PwcNetNode::AiMsgProcess(
//     const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
//   if (!msg || !rclcpp::ok() || !ai_msg_manage_) {
//     return;
//   }

//   std::stringstream ss;
//   ss << "Recved ai msg"
//      << ", frame_id: " << msg->header.frame_id
//      << ", stamp: " << msg->header.stamp.sec << "_"
//      << msg->header.stamp.nanosec;
//   RCLCPP_INFO(
//       rclcpp::get_logger("mono_pwcnet"), "%s", ss.str().c_str());

//   ai_msg_manage_->Feed(msg);
// }

void PwcNetNode::RunPredict() {
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lg(mtx_task_);
    cv_img_.wait(lg, [this]() { return !cache_task_.empty() || !rclcpp::ok(); });
    if (cache_task_.empty()) {
      continue;
    }
    if (!rclcpp::ok()) {
      break;
    }
    CacheTaskType img_msg = cache_task_.front();
    cache_task_.pop();
    lg.unlock();

    // 1. 获取相应的 Tensor
    auto dnn_output = img_msg.first;
    auto input_tensor = img_msg.second;

    std::string ts =
        std::to_string(dnn_output->msg_header->stamp.sec) + "." +
        std::to_string(dnn_output->msg_header->stamp.nanosec);

    auto inputs = std::vector<std::shared_ptr<DNNTensor>>{input_tensor};

    // 2. 开始预测
    if (Run(inputs, dnn_output, true) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("example"), "Run predict failed!");
      return;
    }

  }
}


std::shared_ptr<DNNTensor> PwcNetNode::ImagePreprocesss(
    cv::Mat &bgr_mat1,
    cv::Mat &bgr_mat2,
    int &resized_img_height,
    int &resized_img_width,
    int model_img_height,
    int model_img_width,
    hbDNNTensorProperties tensor_properties) {

  // if (bgr_mat1.rows != bgr_mat2.rows || bgr_mat1.cols != bgr_mat2.cols) {
  //   RCLCPP_ERROR(rclcpp::get_logger("pwcnet"), 
  //                "The sizes of the two imgs should be consistent!");
  //   return nullptr;
  // }
  int raw_img_width = bgr_mat2.cols;
  int raw_img_height = bgr_mat2.rows;

  auto w_stride = ALIGN_16(model_img_width);
  cv::Mat pad_frame1, pad_frame2;

  if (static_cast<uint32_t>(raw_img_width) != w_stride ||
      raw_img_height != model_img_height) {
    pad_frame1 = cv::Mat(model_img_height, w_stride, CV_8UC3, cv::Scalar::all(0));
    pad_frame2 = cv::Mat(model_img_height, w_stride, CV_8UC3, cv::Scalar::all(0));
    auto [resized_height, resized_width] = 
          hobot::dnn_node::GetResizedImgShape(raw_img_height, 
                                              raw_img_width,
                                              model_img_height,
                                              w_stride);
    resized_img_height = resized_height;
    resized_img_width = resized_width;
    cv::resize(bgr_mat1, bgr_mat1, cv::Size(resized_width, resized_height));
    cv::resize(bgr_mat2, bgr_mat2, cv::Size(resized_width, resized_height));
    // 按长宽固定比例resize后复制到目标图像左上角
    bgr_mat1.copyTo(pad_frame1(cv::Rect(0,
                                        0,
                                        bgr_mat1.cols,
                                        bgr_mat1.rows)));
    bgr_mat2.copyTo(pad_frame2(cv::Rect(0,
                                        0,
                                        bgr_mat2.cols,
                                        bgr_mat2.rows)));
  } else {
    resized_img_height = raw_img_height;
    resized_img_width = raw_img_width;
    pad_frame1 = bgr_mat1;
    pad_frame2 = bgr_mat2;
  }

  cv::Mat mat1_yuv444, mat2_yuv444;

  auto ret1 = hobot::dnn_node::ImageProc::BGRToYUV444(pad_frame1, mat1_yuv444, -128);
  auto ret2 = hobot::dnn_node::ImageProc::BGRToYUV444(pad_frame2, mat2_yuv444, -128);
  if (ret1 || ret2) {
    RCLCPP_ERROR(rclcpp::get_logger("pwcnet"), 
                 "get yuv444 image failed");
    return nullptr;
  }
  int src_elem_size = 1;
  switch (tensor_properties.tensorType)
  {
    case HB_DNN_TENSOR_TYPE_S8:
    case HB_DNN_TENSOR_TYPE_U8: src_elem_size = 1; break;
    case HB_DNN_TENSOR_TYPE_F16:
    case HB_DNN_TENSOR_TYPE_S16: 
    case HB_DNN_TENSOR_TYPE_U16: src_elem_size = 2; break;
    case HB_DNN_TENSOR_TYPE_F32:
    case HB_DNN_TENSOR_TYPE_S32:
    case HB_DNN_TENSOR_TYPE_U32: src_elem_size = 4; break;
    case HB_DNN_TENSOR_TYPE_F64:
    case HB_DNN_TENSOR_TYPE_S64: 
    case HB_DNN_TENSOR_TYPE_U64: src_elem_size = 8; break;
    default: RCLCPP_ERROR(rclcpp::get_logger("image_proc"), 
        "Tensor Type %d is not support", tensor_properties.tensorType); break;
  }

  auto *mem = new hbSysMem;
  uint32_t size = model_img_height * w_stride * 3 * src_elem_size * 2;
  hbSysAllocCachedMem(mem, size);
  //内存初始化
  memset(mem->virAddr, 0, size);
  auto *img1_src = mat1_yuv444.ptr<int8_t>();
  auto *img2_src = mat2_yuv444.ptr<int8_t>();
  auto *mem_addr = reinterpret_cast<int8_t *>(mem->virAddr);
  memcpy(mem_addr, img1_src, size / 2);

  mem_addr += size / 2;
  memcpy(mem_addr, img2_src, size / 2);
  hbSysFlushMem(mem, HB_SYS_MEM_CACHE_CLEAN);
  auto input_tensor = new DNNTensor;

  input_tensor->properties = tensor_properties;
  input_tensor->sysMem[0].virAddr = reinterpret_cast<void *>(mem->virAddr);
  input_tensor->sysMem[0].phyAddr = mem->phyAddr;
  input_tensor->sysMem[0].memSize = size;
  return std::shared_ptr<DNNTensor>(
      input_tensor, [mem](DNNTensor *input_tensor) {
        // 销毁时释放空间
        hbSysFreeMem(mem);
        delete mem;
        delete input_tensor;
      });
}

