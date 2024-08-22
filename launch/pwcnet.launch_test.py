# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    # 拷贝config中文件
    # dnn_node_example_path = os.path.join(
    #     get_package_prefix('dnn_node_example'),
    #     "lib/dnn_node_example")
    # print("dnn_node_example_path is ", dnn_node_example_path)
    # cp_cmd = "cp -r " + dnn_node_example_path + "/config ."
    # print("cp_cmd is ", cp_cmd)
    # os.system(cp_cmd)

    # args that can be set from the command line or a default will be used
    img_file_launch_arg = DeclareLaunchArgument(
        "img_config_file", default_value=TextSubstitution(text="['config/img1.jpg','config/img2.jpg']")
    )
    dump_render_launch_arg = DeclareLaunchArgument(
        "pwcnet_dump_render_img", default_value=TextSubstitution(text="0")
    )
    image_width_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_width", default_value=TextSubstitution(text="512")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_height", default_value=TextSubstitution(text="384")
    )
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "pwcnet_msg_pub_topic_name", default_value=TextSubstitution(text="/pwcnet_msg")
    )
    img_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "pwcnet_img_pub_topic_name", default_value=TextSubstitution(text="/pwcnet_img")
    )

    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)
    
    cam_node = None
    camera_type_mipi = None
    camera_device_arg = None

    if camera_type == "usb":
        # usb cam图片发布pkg
        usb_cam_device_arg = DeclareLaunchArgument(
            'device',
            default_value='/dev/video0',
            description='usb camera device')

        usb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_usb_cam'),
                    'launch/hobot_usb_cam.launch.py')),
            launch_arguments={
                'usb_image_width': '640',
                'usb_image_height': '480',
                'usb_video_device': LaunchConfiguration('device')
            }.items()
        )
        print("using usb cam")
        cam_node = usb_node
        camera_type_mipi = False
        camera_device_arg = usb_cam_device_arg

    elif camera_type == "fb":
        # 本地图片发布
        feedback_picture_arg = DeclareLaunchArgument(
            'publish_image_source',
            # default_value='/root/config/target.jpg',
            # default_value='/root/config/bus.jpg',
            default_value='/userdata/pwcnet/config3/',
            description='feedback picture')

        fb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_image_publisher'),
                    'launch/hobot_image_publisher.launch.py')),
            launch_arguments={
                'publish_image_source': LaunchConfiguration('publish_image_source'),
                'publish_output_image_w': LaunchConfiguration('dnn_example_image_width'),
                'publish_output_image_h': LaunchConfiguration('dnn_example_image_height'),
                'publish_image_format': 'jpg',
                'publish_message_topic_name': '/hbmem_img',
                'publish_fps': '5'
            }.items()
        )

        print("using feedback")
        cam_node = fb_node
        camera_type_mipi = True
        camera_device_arg = feedback_picture_arg

    else:
        if camera_type == "mipi":
            print("using mipi cam")
        else:
            print("invalid camera_type ", camera_type,
                ", which is set with export CAM_TYPE=usb/mipi/fb, using default mipi cam")
        # mipi cam图片发布pkg
        mipi_cam_device_arg = DeclareLaunchArgument(
            'device',
            default_value='F37',
            description='mipi camera device')
        mipi_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mipi_cam'),
                    'launch/mipi_cam.launch.py')),
            launch_arguments={
                'mipi_image_width': LaunchConfiguration('dnn_example_image_width'),
                'mipi_image_height': LaunchConfiguration('dnn_example_image_height'),
                'mipi_io_method': 'shared_mem',
                'mipi_frame_ts_type': 'realtime',
                'mipi_video_device': LaunchConfiguration('device')
            }.items()
        )

        cam_node = mipi_node
        camera_type_mipi = True
        camera_device_arg = mipi_cam_device_arg

    # jpeg图片编码&发布pkg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image'
        }.items()
    )

    # nv12图片解码&发布pkg
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )
    
    
    pwcnet_jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_sub_topic': LaunchConfiguration("pwcnet_img_pub_topic_name"),
            'codec_in_format': 'bgr8',
            'codec_pub_topic': '/image',
            'codec_out_format': 'jpeg'
        }.items()
    )

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            # 'websocket_image_topic': LaunchConfiguration("pwcnet_img_pub_topic_name"),
            'websocket_image_type': 'mjpeg',
            'websocket_only_show_image': 'false',
            'websocket_smart_topic': LaunchConfiguration("pwcnet_msg_pub_topic_name")
        }.items()
    )

    # 算法pkg
    pwcnet_node = Node(
        package='mono_pwcnet',
        executable='mono_pwcnet',
        output='screen',
        parameters=[
            {"image_file_": LaunchConfiguration('img_config_file')},
            {"dump_render_img": LaunchConfiguration(
                'pwcnet_dump_render_img')},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"ai_msg_pub_topic_name": LaunchConfiguration(
                "pwcnet_msg_pub_topic_name")},
            {"flow_img_pub_topic_name": LaunchConfiguration(
                "pwcnet_img_pub_topic_name")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    shared_mem_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hobot_shm'),
                        'launch/hobot_shm.launch.py'))
            )

    if camera_type_mipi:
        return LaunchDescription([
            camera_device_arg,
            img_file_launch_arg,
            dump_render_launch_arg,
            image_width_launch_arg,
            image_height_launch_arg,
            msg_pub_topic_name_launch_arg,
            img_pub_topic_name_launch_arg,
            # 启动零拷贝环境配置node
            shared_mem_node,
            # 图片发布pkg
            cam_node,
            # 图片编解码&发布pkg
            jpeg_codec_node,
            # 启动example pkg
            pwcnet_node,
            # pwcnet_jpeg_codec_node,
            # 启动web展示pkg
            web_node
        ])
    else:
        return LaunchDescription([
            camera_device_arg,
            img_file_launch_arg,
            dump_render_launch_arg,
            image_width_launch_arg,
            image_height_launch_arg,
            msg_pub_topic_name_launch_arg,
            img_pub_topic_name_launch_arg,
            # 启动零拷贝环境配置node
            shared_mem_node,
            # 图片发布pkg
            cam_node,
            # 图片编解码&发布pkg
            nv12_codec_node,
            # 启动example pkg
            pwcnet_node,
            # 启动web展示pkg
            web_node
        ])
