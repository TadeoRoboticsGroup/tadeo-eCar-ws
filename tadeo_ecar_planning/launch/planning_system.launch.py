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
    planning_package_dir = get_package_share_directory('tadeo_ecar_planning')
    config_package_dir = get_package_share_directory('tadeo_ecar_config')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true')
    
    enable_global_planner_arg = DeclareLaunchArgument(
        'enable_global_planner',
        default_value='true',
        description='Enable global path planner')
    
    enable_local_planner_arg = DeclareLaunchArgument(
        'enable_local_planner',
        default_value='true',
        description='Enable local path planner')
    
    enable_trajectory_optimizer_arg = DeclareLaunchArgument(
        'enable_trajectory_optimizer',
        default_value='true',
        description='Enable trajectory optimizer')
    
    enable_path_manager_arg = DeclareLaunchArgument(
        'enable_path_manager',
        default_value='true',
        description='Enable path manager')
    
    planner_type_arg = DeclareLaunchArgument(
        'planner_type',
        default_value='RRTstar',
        choices=['RRTstar', 'PRM', 'EST'],
        description='Global planner algorithm')
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tadeo_ecar_config'),
            'config',
            'planning_params.yaml'
        ]),
        description='Path to planning configuration file')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_global_planner = LaunchConfiguration('enable_global_planner')
    enable_local_planner = LaunchConfiguration('enable_local_planner')
    enable_trajectory_optimizer = LaunchConfiguration('enable_trajectory_optimizer')
    enable_path_manager = LaunchConfiguration('enable_path_manager')
    planner_type = LaunchConfiguration('planner_type')
    robot_namespace = LaunchConfiguration('robot_namespace')
    config_file = LaunchConfiguration('config_file')
    
    # Set use_sim_time parameter
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # Global Path Planner Node
    global_planner_node = Node(
        package='tadeo_ecar_planning',
        executable='global_path_planner_node',
        name='global_path_planner_node',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'planner_type': planner_type
            }
        ],
        remappings=[
            ('map', 'map'),
            ('move_base_simple/goal', 'move_base_simple/goal'),
            ('amcl_pose', 'amcl_pose'),
            ('global_path', 'global_path'),
            ('global_path_markers', 'global_path_markers'),
            ('planning/global_health', 'planning/global_health')
        ],
        condition=IfCondition(enable_global_planner),
        output='screen'
    )
    
    # Local Path Planner Node
    local_planner_node = Node(
        package='tadeo_ecar_planning',
        executable='local_path_planner_node',
        name='local_path_planner_node',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('global_path', 'global_path'),
            ('odom', 'odom'),
            ('scan', 'scan'),
            ('cmd_vel', 'cmd_vel'),
            ('local_path', 'local_path'),
            ('local_trajectory_markers', 'local_trajectory_markers'),
            ('planning/local_health', 'planning/local_health')
        ],
        condition=IfCondition(enable_local_planner),
        output='screen'
    )
    
    # Trajectory Optimizer Node
    trajectory_optimizer_node = Node(
        package='tadeo_ecar_planning',
        executable='trajectory_optimizer_node',
        name='trajectory_optimizer_node',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('raw_path', 'global_path'),
            ('optimized_path', 'optimized_path'),
            ('velocity_profile', 'velocity_profile'),
            ('optimized_trajectory_markers', 'optimized_trajectory_markers'),
            ('planning/optimizer_health', 'planning/optimizer_health')
        ],
        condition=IfCondition(enable_trajectory_optimizer),
        output='screen'
    )
    
    # Path Manager Node
    path_manager_node = Node(
        package='tadeo_ecar_planning',
        executable='path_manager_node',
        name='path_manager_node',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('global_path', 'global_path'),
            ('optimized_path', 'optimized_path'),
            ('local_path', 'local_path'),
            ('odom', 'odom'),
            ('move_base_simple/goal', 'move_base_simple/goal'),
            ('final_path', 'final_path'),
            ('navigation_status', 'navigation_status'),
            ('path_progress', 'path_progress'),
            ('planning_visualization', 'planning_visualization'),
            ('planning/manager_health', 'planning/manager_health')
        ],
        condition=IfCondition(enable_path_manager),
        output='screen'
    )
    
    # Planning System Health Monitor
    planning_health_monitor = Node(
        package='tadeo_ecar_planning',
        executable='planning_health_monitor',
        name='planning_health_monitor',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('planning/global_health', 'planning/global_health'),
            ('planning/local_health', 'planning/local_health'),
            ('planning/optimizer_health', 'planning/optimizer_health'),
            ('planning/manager_health', 'planning/manager_health'),
            ('planning/system_health', 'planning/system_health')
        ],
        output='screen'
    )
    
    # RViz Configuration (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('tadeo_ecar_planning'),
        'rviz',
        'planning_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='planning_rviz',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('launch_rviz', default='false')),
        output='screen'
    )
    
    # Launch RViz argument
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz for planning visualization')
    
    # Group all planning nodes
    planning_group = GroupAction([
        set_use_sim_time,
        global_planner_node,
        local_planner_node,
        trajectory_optimizer_node,
        path_manager_node,
        planning_health_monitor,
        rviz_node
    ])
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_global_planner_arg,
        enable_local_planner_arg,
        enable_trajectory_optimizer_arg,
        enable_path_manager_arg,
        planner_type_arg,
        robot_namespace_arg,
        config_file_arg,
        launch_rviz_arg,
        
        # Planning system
        planning_group,
    ])