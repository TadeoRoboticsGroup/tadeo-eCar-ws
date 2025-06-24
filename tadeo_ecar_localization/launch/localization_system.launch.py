#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration files
    robot_config = os.path.join(
        FindPackageShare('tadeo_ecar_config').find('tadeo_ecar_config'),
        'config', 'robot_params.yaml'
    )
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_ekf_arg = DeclareLaunchArgument(
        'enable_ekf',
        default_value='true',
        description='Enable EKF localization'
    )
    
    enable_ukf_arg = DeclareLaunchArgument(
        'enable_ukf',
        default_value='true',
        description='Enable UKF localization'
    )
    
    enable_fusion_arg = DeclareLaunchArgument(
        'enable_fusion',
        default_value='true',
        description='Enable sensor fusion'
    )
    
    enable_map_matching_arg = DeclareLaunchArgument(
        'enable_map_matching',
        default_value='false',
        description='Enable map matching correction'
    )
    
    # Localization nodes group
    localization_nodes = GroupAction([
        # EKF Localization Node
        Node(
            package='tadeo_ecar_localization',
            executable='ekf_localization_node',
            name='ekf_localization',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_ekf').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'filter_frequency': 50.0,
                    'publish_tf': False,  # Fusion node will publish TF
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_link_frame': 'base_link',
                    'use_imu': True,
                    'use_odom': True,
                    'initial_x': 0.0,
                    'initial_y': 0.0,
                    'initial_yaw': 0.0
                }
            ],
            remappings=[
                ('odom', 'odom'),
                ('imu/data', 'imu/data'),
                ('initialpose', 'initialpose'),
                ('odometry/filtered', 'odometry/ekf_filtered'),
                ('pose', 'pose/ekf'),
            ],
        ),
        
        # UKF Localization Node
        Node(
            package='tadeo_ecar_localization',
            executable='ukf_localization_node',
            name='ukf_localization',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_ukf').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'filter_frequency': 50.0,
                    'publish_tf': False,  # Fusion node will publish TF
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_link_frame': 'base_link',
                    'use_imu': True,
                    'use_odom': True,
                    'initial_x': 0.0,
                    'initial_y': 0.0,
                    'initial_yaw': 0.0
                }
            ],
            remappings=[
                ('odom', 'odom'),
                ('imu/data', 'imu/data'),
                ('initialpose', 'initialpose'),
                ('odometry/ukf_filtered', 'odometry/ukf_filtered'),
                ('ukf_pose', 'pose/ukf'),
            ],
        ),
        
        # Sensor Fusion Localization Node
        Node(
            package='tadeo_ecar_localization',
            executable='sensor_fusion_localization_node',
            name='sensor_fusion_localization',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_fusion').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'fusion_frequency': 20.0,
                    'ekf_weight': 0.6,
                    'ukf_weight': 0.4,
                    'gps_weight': 0.3,
                    'use_gps': False
                }
            ],
            remappings=[
                ('odometry/filtered', 'odometry/ekf_filtered'),
                ('odometry/ukf_filtered', 'odometry/ukf_filtered'),
                ('gps/fix', 'gps/fix'),
                ('odometry/fused', 'odometry/fused'),
                ('pose/fused', 'pose/fused'),
            ],
        ),
        
        # Map Matcher Node
        Node(
            package='tadeo_ecar_localization',
            executable='map_matcher_node',
            name='map_matcher',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_map_matching').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'matching_frequency': 10.0,
                    'max_correction_distance': 0.5,
                    'min_scan_match_score': 0.7,
                    'enable_correction': True
                }
            ],
            remappings=[
                ('pose/fused', 'pose/fused'),
                ('map', 'map'),
                ('scan', 'scan'),
                ('pose/corrected', 'pose/corrected'),
            ],
        ),
    ])
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_ekf_arg,
        enable_ukf_arg,
        enable_fusion_arg,
        enable_map_matching_arg,
        localization_nodes,
    ])