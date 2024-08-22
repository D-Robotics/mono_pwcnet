[English](./README.md) | 简体中文

Getting Started with mono mobilesam
=======

# 功能介绍

mono_pwcnet package是基于 PwcNet 量化部署的使用示例。图像数据来源于本地图片回灌和订阅到的image msg。PwcNet 需要提供两张图片，并计算第一张图片的光流。

本示例中, 我们提供了两种部署展示方式:
- 本地图片光流计算：采用本地的两张图片进行回灌，计算第一张图片的光流。
- 订阅图片光流计算：订阅图片信息，不断计算连续两帧图片中的第一帧图片的光流。

# 开发环境

- 编程语言: C/C++
- 开发平台: X5/X86
- 系统版本：Ubuntu 22.04
- 编译工具链: Linux GCC 11.4.0

# 编译

- X5版本：支持在X5 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

- X86版本：支持在X86 Ubuntu系统上编译一种方式。

同时支持通过编译选项控制编译pkg的依赖和pkg的功能。

## 依赖库

- opencv:3.4.5

ros package：

- ai_msgs
- cv_bridge
- dnn node
- hbm_img_msgs
- sensor_msgs

hbm_img_msgs为自定义的图片消息格式, 用于shared mem场景下的图片传输, hbm_img_msgs pkg定义在hobot_msgs中, 因此如果使用shared mem进行图片传输, 需要依赖此pkg。


## 编译选项

1、SHARED_MEM

- shared mem（共享内存传输）使能开关, 默认打开（ON）, 编译时使用-DSHARED_MEM=OFF命令关闭。
- 如果打开, 编译和运行会依赖hbm_img_msgs pkg, 并且需要使用tros进行编译。
- 如果关闭, 编译和运行不依赖hbm_img_msgs pkg, 支持使用原生ros和tros进行编译。
- 对于shared mem通信方式, 当前只支持订阅nv12格式图片。

## X5 Ubuntu系统上编译

1、编译环境确认

- 板端已安装X5 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon, 需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select mono_pwcnet`

## docker交叉编译 X5版本

1、编译环境确认

- 在docker中编译, 并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译dnn node package
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令：

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s mono_pwcnet
  ```

- 编译选项中默认打开了shared mem通信方式。


## X86 Ubuntu系统上编译 X86版本

1、编译环境确认

  x86 ubuntu版本: ubuntu22.04
  
2、编译

- 编译命令：

  ```shell
  colcon build --packages-select mono_pwcnet \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DPLATFORM_X86=ON \
     -DTHIRD_PARTY=`pwd`/../sysroot_docker
  ```

## 注意事项


# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- usb_cam package：发布图片msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名             | 解释                                  | 是否必须             | 默认值              | 备注                                                                    |
| ------------------ | ------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| cache_img_limit          | 设置缓存的图片buffer长度            | 否                   | 11                   |                                                                         |
| cache_task_limit          | 设置缓存的任务buffer长度            | 否                   | 8                   |                                                                         |
| feed_type          | 图片来源, 0：本地；1：订阅            | 否                   | 0                   |                                                                         |
| image_file_              | 本地图片地址                          | 否                   | {"config/img1.jpg", "config/img2.jpg"}     |                                                                         |
| is_shared_mem_sub  | 使用shared mem通信方式订阅图片        | 否                   | 0                   |                                                                         |
| dump_render_img    | 是否进行渲染，0：否；1：是            | 否                   | 0                   |                                                                         |
| ai_msg_pub_topic_name | 发布智能结果的topicname,用于web端展示 | 否                   | /hobot_pwcnet | |
| ros_img_sub_topic_name | 接收ros图片话题名 | 否                   | /image | |
| ros_string_sub_topic_name | 接收string消息话题名改变检测类别 | 否                   | /target_words | |


## 运行

- mono_pwcnet 使用到的模型在安装包'config'路径下。

- 编译成功后, 将生成的install路径拷贝到地平线RDK上（如果是在RDK上编译, 忽略拷贝步骤）, 并执行如下命令运行。

## X5 Ubuntu系统上运行

运行方式1, 使用可执行文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash
# config中为示例使用的模型, 回灌使用的本地图片
# 根据实际安装路径进行拷贝（docker中的安装路径为install/lib/mono_pwcnet/config/, 拷贝命令为cp -r install/lib/mono_pwcnet/config/ .）。
cp -r install/lib/mono_mobilesam/config/ .

# 运行模式1：
# 使用本地jpg格式图片进行回灌预测
ros2 run mono_pwcnet mono_pwcnet --ros-args -p image_file:=["config/img001.jpg","config/img002.jpg"] -p dump_render_img:=1

```

运行方式2, 使用launch文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型, 根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项）, 拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., 其中PKG_NAME为具体的package名。
cp -r install/lib/mono_pwcnet/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 运行模式1：启动launch文件, 启动 pwcnet 节点
ros2 launch mono_pwcnet pwcnet.launch_test.py pwcnet_dump_render_img:=0

```

## X86 Ubuntu系统上运行

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型, 根据实际安装路径进行拷贝
cp -r ./install/lib/mono_pwcnet/config/ .

export CAM_TYPE=fb

# 运行模式1：启动launch文件, 启动 pwcnet 节点
ros2 launch mono_pwcnet pwcnet.launch_test.py

```

# 结果分析

## X5结果展示

log：

运行命令：`ros2 run mono_pwcnet mono_pwcnet --ros-args -p image_file:=["config/img001.jpg","config/img002.jpg"] -p dump_render_img:=1`

```shell
[WARN] [0000000297.873178433] [mono_pwcnet]: Parameter:
 cache_img_limit: 11
 cache_task_limit: 8
 dump_render_img: 1
 feed_type(0:local, 1:sub): 0
 image_size: 2
 is_shared_mem_sub: 1
 is_sync_mode: 0
 ai_msg_pub_topic_name: /hobot_pwcnet
 ros_img_sub_topic_name: /image
 flow_img_pub_topic_name_: /pwcnet_img
[INFO] [0000000297.873456225] [hobot_yolo_world]: Set node para.
[WARN] [0000000297.873504933] [mono_pwcnet]: model_file_name_: config/model.hbm, task_num: 4
[INFO] [0000000297.873555391] [dnn]: Node init.
[INFO] [0000000297.873584641] [hobot_yolo_world]: Set node para.
[WARN] [0000000297.873612475] [mono_pwcnet]: model_file_name_: config/model.hbm, task_num: 4
[INFO] [0000000297.873659350] [dnn]: Model init.
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.49.0
[DNN] Runtime version = 1.23.8_(3.15.49 HBRT)
[W][DNN]bpu_model_info.cpp:491][Version](1970-01-01,00:04:58.68.583) Model: pwcnet_pwcnetneck_flyingchairs. Inconsistency between the hbrt library version 3.15.49.0 and the model build version 3.15.54.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[INFO] [0000000298.069318683] [dnn]: The model input 0 width is 512 and height is 384
[INFO] [0000000298.069524058] [dnn]:
Model Info:
name: pwcnet_pwcnetneck_flyingchairs.
[input]
 - (0) Layout: NCHW, Shape: [1, 6, 384, 512], Type: HB_DNN_TENSOR_TYPE_S8.
[output]
 - (0) Layout: NCHW, Shape: [1, 2, 96, 128], Type: HB_DNN_TENSOR_TYPE_S32.

[INFO] [0000000298.069600891] [dnn]: Task init.
[INFO] [0000000298.071799891] [dnn]: Set task_num [4]
[WARN] [0000000298.071860266] [hobot_yolo_world]: Get model name: pwcnet_pwcnetneck_flyingchairs from load model.
[INFO] [0000000298.071901308] [pwcnet_node]: The model input width is 512 and height is 384
[INFO] [0000000298.174975975] [mono_pwcnet]: [channel]: 2;  [height]: 96; [width]: 128; [quantiType]: 2; scale[0]: 0.00027313; scale[1]: 0.000285034
[INFO] [0000000298.184596600] [MobileSam]: Draw result to file: render_pwcnet_feedback_0_0.jpeg
^C[INFO] [0000000443.604610044] [rclcpp]: signal_handler(signum=2)
^C^C[ros2run]: Interrupt
root@buildroot:/userdata/pwcnet# cp config4/img001.jpg config/img001.jpg
root@buildroot:/userdata/pwcnet# cp config4/img002.jpg config/img002.jpg
root@buildroot:/userdata/pwcnet# ros2 run mono_pwcnet mono_pwcnet --ros-args -p image_file:=["config/img001.jpg","config/img002.jpg"] -p dump_render_img:=1
[WARN] [0000000471.826425808] [mono_pwcnet]: Parameter:
 cache_img_limit: 11
 cache_task_limit: 8
 dump_render_img: 1
 feed_type(0:local, 1:sub): 0
 image_size: 2
 is_shared_mem_sub: 1
 is_sync_mode: 0
 ai_msg_pub_topic_name: /hobot_pwcnet
 ros_img_sub_topic_name: /image
 flow_img_pub_topic_name_: /pwcnet_img
[INFO] [0000000471.826698349] [hobot_yolo_world]: Set node para.
[WARN] [0000000471.826762058] [mono_pwcnet]: model_file_name_: config/model.hbm, task_num: 4
[INFO] [0000000471.826820474] [dnn]: Node init.
[INFO] [0000000471.826853516] [hobot_yolo_world]: Set node para.
[WARN] [0000000471.826882641] [mono_pwcnet]: model_file_name_: config/model.hbm, task_num: 4
[INFO] [0000000471.826932558] [dnn]: Model init.
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.49.0
[DNN] Runtime version = 1.23.8_(3.15.49 HBRT)
[W][DNN]bpu_model_info.cpp:491][Version](1970-01-01,00:07:52.20.736) Model: pwcnet_pwcnetneck_flyingchairs. Inconsistency between the hbrt library version 3.15.49.0 and the model build version 3.15.54.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[INFO] [0000000472.021419808] [dnn]: The model input 0 width is 512 and height is 384
[INFO] [0000000472.021605391] [dnn]:
Model Info:
name: pwcnet_pwcnetneck_flyingchairs.
[input]
 - (0) Layout: NCHW, Shape: [1, 6, 384, 512], Type: HB_DNN_TENSOR_TYPE_S8.
[output]
 - (0) Layout: NCHW, Shape: [1, 2, 96, 128], Type: HB_DNN_TENSOR_TYPE_S32.

[INFO] [0000000472.021669183] [dnn]: Task init.
[INFO] [0000000472.023770558] [dnn]: Set task_num [4]
[WARN] [0000000472.023827099] [hobot_yolo_world]: Get model name: pwcnet_pwcnetneck_flyingchairs from load model.
[INFO] [0000000472.023866808] [pwcnet_node]: The model input width is 512 and height is 384
[INFO] [0000000472.126077183] [mono_pwcnet]: [channel]: 2;  [height]: 96; [width]: 128; [quantiType]: 2; scale[0]: 0.00027313; scale[1]: 0.000285034
[INFO] [0000000472.134030183] [MobileSam]: Draw result to file: render_pwcnet_feedback_0_0.jpeg

```

## 渲染结果
![image](img/render_pwcnet_feedback_0_0.jpeg)

说明：前处理对图片进行缩放和补全处理。