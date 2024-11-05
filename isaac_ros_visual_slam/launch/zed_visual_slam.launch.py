import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    # The zed camera mode name. zed, zed2, zed2i, zedm, zedx or zedxm
    camera_model = 'zed'

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': camera_model+'_camera_center',
                    'input_imu_frame': camera_model+'_imu_link',
                    'enable_imu_fusion': False,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 35.00
                    }],
        remappings=[('visual_slam/image_0', '/zed/zed_node/left/image_rect_color'),    # rgb
                    ('visual_slam/camera_info_0', '/zed/zed_node/left/camera_info'),
                    ('visual_slam/image_1', '/zed/zed_node/right/image_rect_color'),  # rgb
                    ('visual_slam/camera_info_1', '/zed/zed_node/right/camera_info'),
                    ('visual_slam/imu', '/zed/zed_node/imu/data')]
    )
    
    image_format_converter_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[{
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image_raw', '/zed/zed_node/left/image_rect_color'),
            ('image', '/zed/zed_node/rgb/image_rect_color')]
    )

    image_format_converter_node_right = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_right',
        parameters=[{
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image_raw', '/zed/zed_node/right/image_rect_color'),
            ('image', '/zed/zed_node/rgb/image_rect_color')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            image_format_converter_node_left,
            image_format_converter_node_right,
            visual_slam_node
        ],
        output='screen'
    )

    # URDF/xacro file to be loaded by the Robot State Publisher node
    # xacro_path = os.path.join(
    #     get_package_share_directory('zed_wrapper'),
    #     'urdf', 'zed_descr.urdf.xacro'
    # )

    # ZED Configurations to be loaded by ZED Node
    # config_common = os.path.join(
    #     get_package_share_directory('isaac_ros_visual_slam'),
    #     'config',
    #     'zed.yaml'
    # )

    # Robot State Publisher node
    # rsp_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='zed_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': Command(
    #             [
    #                 'xacro', ' ', xacro_path, ' ',
    #                 'camera_name:=', camera_model, ' ',
    #                 'camera_model:=', camera_model
    #             ])
    #     }]
    # )

    # ZED node using manual composition
    # zed_node = Node(
    #     package='zed_wrapper',
    #     executable='zed_wrapper',
    #     output='screen',
    #     parameters=[
    #         config_common,  # Common parameters
    #         {'general.camera_model': camera_model,
    #          'general.camera_name': camera_model}
    #     ]
    # )

    # Adding delay because isaac_ros_visual_slam requires
    # tf from rsp_node at start
    return launch.LaunchDescription([
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[visual_slam_launch_container]
        ),
        # rsp_node,
        # zed_node
        ])

