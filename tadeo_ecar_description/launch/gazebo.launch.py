import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # AGREGAR ESTA LÍNEA

def generate_launch_description():
    
    # Paquetes
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_tadeo_ecar = FindPackageShare(package='tadeo_ecar_description').find('tadeo_ecar_description')
    
    # Archivos
    urdf_file = os.path.join(pkg_tadeo_ecar, 'urdf', 'tadeo_ecar.urdf.xacro')
    world_file = os.path.join(pkg_tadeo_ecar, 'worlds', 'empty_world.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')
    
    # Robot description como ParameterValue
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file,
                         'verbose': 'true'}.items()
    )
    
    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robot State Publisher - CORREGIDO
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description  # USAR LA VARIABLE CORREGIDA
        }]
    )
    
    # Spawn robot en Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tadeo_ecar',
        arguments=['-topic', 'robot_description',
                  '-entity', robot_name,
                  '-x', x_pose,
                  '-y', y_pose,
                  '-z', z_pose,
                  '-R', roll,
                  '-P', pitch,
                  '-Y', yaw],
        output='screen'
    )

    # Joint State Publisher (opcional, para mover joints manualmente)
    joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_tadeo_ecar, 'rviz', 'tadeo_ecar.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('gui', default_value='false',
                            description='Set to "true" to run joint_state_publisher_gui'),
        DeclareLaunchArgument('rviz', default_value='true',
                            description='Set to "true" to run rviz'),
        DeclareLaunchArgument('x_pose', default_value='0.0',
                            description='x position of robot'),
        DeclareLaunchArgument('y_pose', default_value='0.0',
                            description='y position of robot'),
        DeclareLaunchArgument('z_pose', default_value='0.5',  # CAMBIÉ A 0.5 PARA QUE SE VEA MEJOR
                            description='z position of robot'),
        DeclareLaunchArgument('roll', default_value='0.0',
                            description='roll orientation of robot'),
        DeclareLaunchArgument('pitch', default_value='0.0',
                            description='pitch orientation of robot'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                            description='yaw orientation of robot'),
        DeclareLaunchArgument('robot_name', default_value='tadeo_ecar',
                            description='name of robot'),
        
        # Nodos
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_robot_cmd,
        joint_state_publisher_gui_cmd,
        rviz_cmd
    ])