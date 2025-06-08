import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Paquetes
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_tadeo_ecar = FindPackageShare(package='tadeo_ecar_description').find('tadeo_ecar_description')
    
    # Archivos
    urdf_file = os.path.join(pkg_tadeo_ecar, 'urdf', 'tadeo_ecar.urdf.xacro')
    world_file = os.path.join(pkg_tadeo_ecar, 'worlds', 'empty_world_ignition.sdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' pkg_path:=', pkg_tadeo_ecar]),
        value_type=str
    )
    
    # Gazebo Ignition server
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Gazebo Ignition client
    gz_sim_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-g']
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Joint State Publisher - AGREGADO para manejar joints de suspensión
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )
    
    # Spawn robot en Ignition Gazebo
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_tadeo_ecar',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    # Bridge mejorado para conectar Ignition con ROS 2
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Comandos y odometría
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            
            # Sensores
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Transformaciones - MEJORADO
            f'/{robot_name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            
            # Clock y Joint States - AGREGADO
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            f'/{robot_name}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Bridge adicional específico para joint states - NUEVO
    joint_states_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_states_bridge',
        arguments=[
            f'/world/empty_world/model/{robot_name}/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            (f'/world/empty_world/model/{robot_name}/joint_state', '/joint_states')
        ],
        output='screen'
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
        # Variables de entorno para Ignition
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', 
                              f'{pkg_tadeo_ecar}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', 
                              f'{pkg_tadeo_ecar}:{os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")}'),
        
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('rviz', default_value='true',
                            description='Set to "true" to run rviz'),
        DeclareLaunchArgument('x_pose', default_value='0.0',
                            description='x position of robot'),
        DeclareLaunchArgument('y_pose', default_value='0.0',
                            description='y position of robot'),
        DeclareLaunchArgument('z_pose', default_value='0.5',
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
        gz_sim_server,
        gz_sim_client,
        robot_state_publisher_cmd,
        joint_state_publisher_cmd,  # AGREGADO
        spawn_robot_cmd,
        bridge_cmd,
        joint_states_bridge_cmd,    # AGREGADO
        rviz_cmd
    ])