import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

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
                    # 'base_frame': 'zed_camera_link',
                    'base_frame': 'base_link',
                    'input_imu_frame': 'zed'+'_imu_link',
                    'camera_optical_frames': ['zed'+'_left_camera_optical_frame',
                                              'zed'+'_right_camera_optical_frame'],
                    'enable_imu_fusion': False,      
                    'gyro_noise_density': 0.0028,           # 제드 imu 기준 -> 추정치
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.0314,          # 제드 imu 기준 -> 추정치
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 400.0,         # 제드 imu 기준 -> 추정치
                    'image_jitter_threshold_ms': 70.00,  
                    'enable_ground_constraint_in_odometry': True,
                    'enable_ground_constraint_in_slam': True,
                    'multicam_mode': 2,                      # Multicamera mode: moderate (0), performance (1) or precision (2).   
                    'enable_localization_n_mapping': True,    # True면 맵 생성+localization, False면 localization만            
                    'publish_map_to_odom_tf': True,
                    'publish_odom_to_base_tf': True,
                    }],
        remappings=[('/visual_slam/image_0', '/zed/zed_node/left_gray/image_rect_gray'),
                    ('/visual_slam/camera_info_0', '/zed/zed_node/left_gray/camera_info'),
                    ('/visual_slam/image_1', '/zed/zed_node/right_gray/image_rect_gray'),
                    ('/visual_slam/camera_info_1', '/zed/zed_node/right_gray/camera_info'),
                    ('visual_slam/imu', '/zed/zed_node/imu/data')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container])
