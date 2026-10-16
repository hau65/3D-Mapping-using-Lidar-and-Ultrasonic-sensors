import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)

    # ====================== PATH ======================
    default_model_path = os.path.join(pkg_share, 'urdf', 'mobile_robot_v3.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'sim_room.world')
    ekf_config_path    = os.path.join(pkg_share, 'config', 'ekf.yaml')
    filter_config      = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    bridge_config      = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    # ====================== Launch Config ======================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_model   = LaunchConfiguration('model', default=default_model_path)
    world_file   = LaunchConfiguration('world', default=default_world_path)
    gz_args      = LaunchConfiguration('gz_args', default=['-r ', world_file])

    # ====================== Gazebo Sim (Harmonic) ======================
    # gz_sim.launch.py reads the <gazebo_ros gazebo_model_path> export from
    # package.xml and populates GZ_SIM_RESOURCE_PATH, so model:// URIs resolve.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args
        }.items()
    )

    # ====================== ROS <-> GZ BRIDGE ======================
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': use_sim_time}]
    )

    # ====================== Robot State Publisher ======================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model])
        }]
    )

    # ====================== EKF Node ======================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # ====================== LASER FILTER ======================
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[filter_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ],
    )

    # ====================== ULTRASONIC FUSION ======================
    ultrasonic_fusion = Node(
        package='mobile_robot',
        executable='ultrasonic_fusion_node_v3.py',
        name='ultrasonic_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================== UlTRASONIC 3D MAP ======================
    ultrasonic_3d_map = Node(
        package='mobile_robot',
        executable='ultrasonic_3d_map_node.py',
        name='ultrasonic_3d_map_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================== OCTOMAP SERVER ======================
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{'frame_id': 'map', 'use_sim_time': use_sim_time}],
        remappings=[('/cloud_in', '/ultrasonic_pointcloud')]
    )

    # ====================== Spawn Entity (ros_gz_sim create) ======================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mobile_robot',
            '-z', '0.05'
        ]
    )

    delayed_spawn_entity      = TimerAction(period=5.0, actions=[spawn_entity])
    delayed_filter            = TimerAction(period=6.0, actions=[scan_filter])
    delayed_ultrasonic_fusion = TimerAction(period=7.0, actions=[ultrasonic_fusion])
    delayed_ultrasonic_3d_map = TimerAction(period=9.0, actions=[ultrasonic_3d_map])
    delayed_octomap_server    = TimerAction(period=10.0, actions=[octomap_server])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',        default_value=default_model_path),
        DeclareLaunchArgument('world',        default_value=default_world_path),
        DeclareLaunchArgument('gz_args',      default_value=['-r ', world_file]),

        gazebo,
        ros_gz_bridge,
        robot_state_publisher,
        ekf_node,

        delayed_spawn_entity,
        delayed_filter,
        delayed_ultrasonic_fusion,
        delayed_ultrasonic_3d_map,
        delayed_octomap_server
    ])