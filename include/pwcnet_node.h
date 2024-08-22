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

#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/grasp_group.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "cv_bridge/cv_bridge.h"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "dnn_node/util/output_parser/perception_common.h"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// #include "include/ai_msg_manage.h"
// #include "include/data_preprocess.h"
#include "include/pwcnet_output_parser.h"

#ifndef PWCNET_NODE_H_
#define PWCNET_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::Model;

using ai_msgs::msg::PerceptionTargets;

class ThreadPool {
public:
    ThreadPool() : stop(false) {
        worker = std::thread(&ThreadPool::workerThread, this);
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        if (worker.joinable()) {
            worker.join();
        }
    }

    void enqueue(std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            tasks.push(task);
        }
        condition.notify_one();
    }

private:
    void workerThread() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                condition.wait(lock, [this] { return stop || !tasks.empty(); });
                if (stop && tasks.empty())
                    return;
                if (!tasks.empty()) {
                    task = std::move(tasks.front());
                    tasks.pop();
                }
            }
            if (task) {
                task();
            }
        }
    }

    std::thread worker;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};


struct PwcNetOutput : public DnnNodeOutput {
  // resize参数，用于算法检测结果的映射
  float ratio = 1.0;  //缩放比例系数，无需缩放为1

  ai_msgs::msg::Perf perf_preprocess;

  int model_w = 0; // 模型输入的w
  int model_h = 0; // 模型输入的h

  int resized_w = 0; // 经过resize后图像的w
  int resized_h = 0; // 经过resize后图像的w

//   std::vector<OpticalFlow> flow;

  cv::Mat bgr_mat;
};

class PwcNetNode : public DnnNode {
 public:
  PwcNetNode(const std::string &node_name,
                 const NodeOptions &options = NodeOptions());
  ~PwcNetNode() override;

 protected:
  int SetNodePara() override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  int Debug();

  int FeedFromLocal();

  int LoadModels();
  
  int Infer(std::vector<hbDNNTensor>& inputs,
            std::vector<hbDNNTensor>& outputs,
            int idx);

  int RunMulti(std::vector<std::shared_ptr<DNNTensor>>& inputs,
        const std::shared_ptr<DnnNodeOutput> &output = nullptr,
        const bool is_sync_mode = false);

  std::shared_ptr<DNNTensor> ImagePreprocesss(
    cv::Mat &bgr_mat1,
    cv::Mat &bgr_mat2,
    int &resized_img_height,
    int &resized_img_width,
    int model_img_height,
    int model_img_width,
    hbDNNTensorProperties tensor_properties);

#ifdef SHARED_MEM_ENABLED
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

  std::string ai_msg_pub_topic_name_ = "/hobot_pwcnet";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;

  std::string flow_img_pub_topic_name_ = "/pwcnet_img";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_ =
      nullptr;

//   std::string ai_msg_sub_topic_name_ = "/hobot_detection";
//   rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
//       ai_msg_subscription_ = nullptr;
//   void AiMsgProcess(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

  // 目前只支持订阅深度图原图
  std::string ros_img_sub_topic_name_ = "/image";
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);


  int dump_render_img_ = 0;
  // 用于预测的图片来源, 0：本地彩色图, 1： 订阅到的image msg
  int feed_type_ = 0;
  // 回灌图片的格式，0: bgr, 1: bin
  enum class ImageType { BGR = 0, BIN = 1};
  int image_type_ = 0;

  std::vector<std::string> image_file_ = {"config/img1.jpg", "config/img2.jpg"};
//   std::string image_file1_ = "config/img1.jpg";
//   std::string image_file2_ = "config/img2.jpg";
  // 加载模型后，查询出模型输入分辨率
  int model_input_width_ = 512;
  int model_input_height_ = 384;

  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 1;
  int is_sync_mode_ = 0;

//   cv::Mat pre_img;
//   cv::Mat cur_img;
  std::mutex mtx_img_;
  int cache_img_limit_ = 11;
  std::queue<cv::Mat> cache_img_;

  // 算法推理的任务数
  int task_num_ = 4;

  // 模型推理相关
  std::string model_file_name_ = "config/model.hbm";
  std::string model_name_ = "";

  std::vector<Model *> models_;
  std::vector<hbPackedDNNHandle_t> packed_dnn_handles_;
  ThreadPool threadPool;

//   std::shared_ptr<AiMsgManage> ai_msg_manage_ = nullptr;

  std::shared_ptr<PwcNetOutputParser> output_parser_ = nullptr;

  // 将订阅到的图片数据转成pym之后缓存
  // 在线程中执行推理，避免阻塞订阅IO通道，导致AI msg消息丢失
  int cache_task_limit_ = 8;
  std::mutex mtx_task_;
  std::condition_variable cv_img_;
  using CacheTaskType = std::pair<std::shared_ptr<PwcNetOutput>,
                                 std::shared_ptr<DNNTensor>>;
  std::queue<CacheTaskType> cache_task_;
  void RunPredict();
  std::shared_ptr<std::thread> predict_task_ = nullptr;
};

#endif  // PWCNET_NODE_H_
