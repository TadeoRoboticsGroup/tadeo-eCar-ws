import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Paquetes
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_tadeo_ecar = FindPackageShare(package='tadeo_ecar_description').find('tadeo_ecar_description')
    
    # Archivos
    urdf_file = os.path.join(pkg_tadeo_ecar, 'urdf', 'tadeo_ecar.urdf.xacro')
    world_file = os.path.join(pkg_tadeo_ecar, 'worlds', 'empty_world.world')
    
    # Gazebo con mundo vacío
    gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # Spawn robot
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tadeo_ecar',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tadeo_ecar',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.0',  # 1 metro sobre el suelo
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_cmd,
        robot_state_publisher_cmd,
        spawn_robot_cmd
    ])