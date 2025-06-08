import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='tadeo_ecar_description').find('tadeo_ecar_description')
    default_model_path = os.path.join(pkg_share, 'urdf/tadeo_ecar.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/tadeo_ecar.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                            description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])