#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    slam_package_dir = get_package_share_directory('tadeo_ecar_slam')
    config_package_dir = get_package_share_directory('tadeo_ecar_config')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true')
    
    enable_grid_slam_arg = DeclareLaunchArgument(
        'enable_grid_slam',
        default_value='true',
        description='Enable grid-based SLAM')
    
    enable_graph_slam_arg = DeclareLaunchArgument(
        'enable_graph_slam',
        default_value='true',
        description='Enable graph-based SLAM')
    
    enable_map_manager_arg = DeclareLaunchArgument(
        'enable_map_manager',
        default_value='true',
        description='Enable map manager')
    
    enable_loop_detector_arg = DeclareLaunchArgument(
        'enable_loop_detector',
        default_value='true',
        description='Enable loop closure detection')
    
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='online',
        choices=['online', 'mapping', 'localization'],
        description='SLAM operation mode')
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to existing map file for localization mode')
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tadeo_ecar_config'),
            'config',
            'slam_params.yaml'
        ]),
        description='Path to SLAM configuration file')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_grid_slam = LaunchConfiguration('enable_grid_slam')
    enable_graph_slam = LaunchConfiguration('enable_graph_slam')
    enable_map_manager = LaunchConfiguration('enable_map_manager')
    enable_loop_detector = LaunchConfiguration('enable_loop_detector')
    slam_mode = LaunchConfiguration('slam_mode')
    map_file = LaunchConfiguration('map_file')
    robot_namespace = LaunchConfiguration('robot_namespace')
    config_file = LaunchConfiguration('config_file')
    
    # Set use_sim_time parameter
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # Grid SLAM Node
    grid_slam_node = Node(
        package='tadeo_ecar_slam',
        executable='grid_slam_node',
        name='grid_slam_node',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'odom'),
            ('initialpose', 'initialpose'),
            ('map', 'grid_map'),
            ('slam_pose', 'grid_slam_pose'),
            ('slam_odom', 'grid_slam_odom'),
            ('slam/grid_health', 'slam/grid_health')
        ],
        condition=IfCondition(enable_grid_slam),
        output='screen'
    )
    
    # Graph SLAM Node
    graph_slam_node = Node(
        package='tadeo_ecar_slam',
        executable='graph_slam_node',
        name='graph_slam_node',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', 'scan'),
            ('pointcloud', 'pointcloud'),
            ('odom', 'odom'),
            ('initialpose', 'initialpose'),
            ('graph_slam_pose', 'graph_slam_pose'),
            ('graph_slam_odom', 'graph_slam_odom'),
            ('pose_graph', 'pose_graph'),
            ('slam/graph_health', 'slam/graph_health')
        ],
        condition=IfCondition(enable_graph_slam),
        output='screen'
    )
    
    # Map Manager Node
    map_manager_node = Node(
        package='tadeo_ecar_slam',
        executable='map_manager_node',
        name='map_manager_node',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('map', 'grid_map'),
            ('slam_pose', 'grid_slam_pose'),
            ('scan', 'scan'),
            ('merged_map', 'map'),
            ('map_updates', 'map_updates'),
            ('slam/map_manager_health', 'slam/map_manager_health')
        ],
        condition=IfCondition(enable_map_manager),
        output='screen'
    )
    
    # Loop Detector Node
    loop_detector_node = Node(
        package='tadeo_ecar_slam',
        executable='loop_detector_node',
        name='loop_detector_node',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', 'scan'),
            ('pointcloud', 'pointcloud'),
            ('slam_pose', 'graph_slam_pose'),
            ('keyframe_trigger', 'keyframe_trigger'),
            ('loop_detections', 'loop_detections'),
            ('loop_closures', 'loop_closures'),
            ('slam/loop_detector_health', 'slam/loop_detector_health')
        ],
        condition=IfCondition(enable_loop_detector),
        output='screen'
    )
    
    # Static Transform Publishers for SLAM coordinate frames
    map_to_odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        namespace=robot_namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(enable_grid_slam),  # Only if SLAM is not providing this transform
        output='screen'
    )
    
    # SLAM System Health Monitor
    slam_health_monitor = Node(
        package='tadeo_ecar_slam',
        executable='slam_health_monitor',
        name='slam_health_monitor',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('slam/grid_health', 'slam/grid_health'),
            ('slam/graph_health', 'slam/graph_health'),
            ('slam/map_manager_health', 'slam/map_manager_health'),
            ('slam/loop_detector_health', 'slam/loop_detector_health'),
            ('slam/system_health', 'slam/system_health')
        ],
        output='screen'
    )
    
    # RViz Configuration (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('tadeo_ecar_slam'),
        'rviz',
        'slam_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='slam_rviz',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('launch_rviz', default='false')),
        output='screen'
    )
    
    # Launch RViz argument
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz for SLAM visualization')
    
    # Group all SLAM nodes
    slam_group = GroupAction([
        set_use_sim_time,
        grid_slam_node,
        graph_slam_node,
        map_manager_node,
        loop_detector_node,
        map_to_odom_static_tf,
        slam_health_monitor,
        rviz_node
    ])
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_grid_slam_arg,
        enable_graph_slam_arg,
        enable_map_manager_arg,
        enable_loop_detector_arg,
        slam_mode_arg,
        map_file_arg,
        robot_namespace_arg,
        config_file_arg,
        launch_rviz_arg,
        
        # SLAM system
        slam_group,
    ])